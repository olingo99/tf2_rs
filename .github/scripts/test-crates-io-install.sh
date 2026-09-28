#!/bin/bash
# Follows the "From crates.io" instructions in README.md: creates a new
# ament_cargo package from the Cargo.toml and package.xml snippets in the
# README, adds every Rust example of the Usage section as a binary, builds it
# with colcon and runs the examples that terminate on their own.
#
# By default tf2_rs comes from `cargo package`, i.e. exactly the files that
# would be uploaded to crates.io. With --from-crates-io the published crate is
# downloaded instead.
#
# Runs inside the image built from docker/Dockerfile.
set -eo pipefail

repo=/workspace/src/tf2_rs
ws=/tmp/consumer_ws
pkg="$ws/src/tf2_rs_consumer"

. "/opt/ros/$ROS_DISTRO/setup.sh"

patch_path=""
version=$(sed -n 's/^version = "\(.*\)"/\1/p' "$repo/Cargo.toml" | head -n1)
test -n "$version"
if [ "${1:-}" != "--from-crates-io" ]; then
    (cd "$repo" && cargo package --locked --quiet)
    patch_path="$repo/target/package/tf2_rs-$version"
fi

rm -rf "$ws"
mkdir -p "$pkg/src/bin"
python3 - "$repo/README.md" "$pkg" "$patch_path" "$version" <<'EOF'
import re
import sys
from pathlib import Path

readme = Path(sys.argv[1]).read_text()
pkg, patch_path, version = Path(sys.argv[2]), sys.argv[3], sys.argv[4]

def section(title):
    start = readme.index(title)
    level = title.split(" ")[0]
    rest = readme[start + len(title):]
    end = re.search(rf"^{level} ", rest, re.M)
    return rest[: end.start()] if end else rest

def blocks(text, lang):
    return re.findall(rf"^```{lang}\n(.*?)^```", text, re.M | re.S)

install = section("### From crates.io")
dependencies = blocks(install, "toml")[0]
depends = blocks(install, "xml")[0]
examples = blocks(section("## Usage"), "rust")
assert examples, "no Rust examples found in the Usage section"

# A registry verification must install the tag being released, not merely the
# newest version allowed by a possibly stale README requirement. The ordinary
# CI path leaves the README requirement intact and verifies below that its
# local patch resolves to the candidate version.
if not patch_path:
    dependencies, replacements = re.subn(
        r'(?m)^tf2_rs\s*=\s*"[^"]+"\s*$',
        f'tf2_rs = "={version}"',
        dependencies,
    )
    assert replacements == 1, "expected one simple tf2_rs dependency in the README"

cargo_toml = f"""[package]
name = "tf2_rs_consumer"
version = "0.0.0"
edition = "2021"

{dependencies}"""
if patch_path:
    cargo_toml += f'\n[patch.crates-io]\ntf2_rs = {{ path = "{patch_path}" }}\n'
(pkg / "Cargo.toml").write_text(cargo_toml)

(pkg / "package.xml").write_text(f"""<package format="3">
  <name>tf2_rs_consumer</name>
  <version>0.0.0</version>
  <description>Checks the README installation instructions</description>
  <maintainer email="ci@example.com">CI</maintainer>
  <license>MIT</license>

{depends}
  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
""")

for i, code in enumerate(examples, 1):
    (pkg / "src" / "bin" / f"example_{i}.rs").write_text(code)
print(f"generated tf2_rs_consumer with {len(examples)} README examples")
EOF

cd "$ws"
rosdep update -q
rosdep install --from-paths src "$repo" --ignore-src -y --simulate
colcon build --event-handlers console_direct+

# Guard against testing an older registry release when Cargo.toml was bumped
# but the README was not. A [patch] only applies when its package version also
# satisfies the dependency requirement, so a mismatched README could otherwise
# make this test pass against an already-published version.
python3 - "$pkg/Cargo.toml" "$version" "$patch_path" <<'EOF'
import json
import subprocess
import sys
from pathlib import Path

manifest, expected_version, patch_path = sys.argv[1:]
metadata = json.loads(subprocess.check_output([
    "cargo", "metadata", "--locked", "--format-version", "1",
    "--manifest-path", manifest,
]))
packages = [package for package in metadata["packages"] if package["name"] == "tf2_rs"]
assert len(packages) == 1, f"expected one resolved tf2_rs package, got {len(packages)}"
package = packages[0]
assert package["version"] == expected_version, (
    f"resolved tf2_rs {package['version']}, expected {expected_version}; "
    "update the README dependency when bumping the crate version"
)
if patch_path:
    assert package["source"] is None, "candidate tf2_rs package did not resolve from the local patch"
    assert Path(package["manifest_path"]).is_relative_to(Path(patch_path)), (
        "resolved local tf2_rs package came from an unexpected path"
    )
else:
    assert package["source"] and package["source"].startswith("registry+"), (
        "published tf2_rs package did not resolve from a registry"
    )
print(f"verified tf2_rs {expected_version} from {package['source'] or patch_path}")
EOF

bin="$ws/install/tf2_rs_consumer/lib/tf2_rs_consumer"

# Example 1 fills a buffer by hand and exits.
"$bin/example_1"

# Example 2 listens on /tf and transforms clouds from /cloud_in into "map".
# Publish a map -> lidar transform and a cloud in "lidar", and check it arrives.
"$bin/example_2" > /tmp/example_2.log 2>&1 &
example_pid=$!
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id lidar \
    > /dev/null 2>&1 &
tf_pid=$!
sleep 2
# One point (1, 2, 3) as little-endian float32 x, y, z.
cloud="{header: {frame_id: lidar}, height: 1, width: 1,
  fields: [{name: x, offset: 0, datatype: 7, count: 1},
           {name: y, offset: 4, datatype: 7, count: 1},
           {name: z, offset: 8, datatype: 7, count: 1}],
  is_bigendian: false, point_step: 12, row_step: 12, is_dense: true,
  data: [0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64]}"
timeout 30 ros2 topic pub --times 5 --wait-matching-subscriptions 1 \
    /cloud_in sensor_msgs/msg/PointCloud2 "$cloud" > /dev/null
kill "$example_pid" "$tf_pid"
cat /tmp/example_2.log
grep -q "transformed cloud into map" /tmp/example_2.log
