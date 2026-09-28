# tf2_rs

[![CI](https://github.com/olingo99/tf2_rs/actions/workflows/ci.yml/badge.svg)](https://github.com/olingo99/tf2_rs/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/tf2_rs.svg)](https://crates.io/crates/tf2_rs)
[![docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://olingo99.github.io/tf2_rs/)

`tf2_rs` provides Rust bindings for a focused subset of ROS 2 TF2. It wraps
`tf2::BufferCore` through `cxx`, integrates with `rclrs`, and exposes Rust-first
types for transform lookup, listening, broadcasting, and `PointCloud2`
transforms.

## Status

- Validated against ROS 2 Humble, Jazzy, Kilted, Lyrical, and Rolling. Iron is
  not supported because it is end-of-life and current `rclrs` no longer ships
  Iron bindings.
- Recommended build path: `ament_cargo` inside a sourced ROS 2 workspace.
- Currently implemented `Transformable` support: `sensor_msgs::msg::PointCloud2`.
- This crate is not a full `tf2_ros` replacement yet; it covers the core pieces
  needed for TF lookup and a small set of transform operations from Rust.

## Public API

The current crate surface is centered on these types:

- `BufferCore`: store transforms, query availability, look them up, and apply
  them to supported message types.
- `TransformListener`: subscribe to `/tf` and `/tf_static` and keep a shared
  `BufferCore` updated.
- `TransformBroadcaster` and `StaticTransformBroadcaster`: publish transforms
  from Rust nodes.
- `TransformStamped`: owned Rust representation of a TF transform, with
  conversion to and from `geometry_msgs::msg::TransformStamped`.
- `LookupTime` and `TimeSpec`: choose `Latest`, a specific timestamp, or a
  timestamp taken from a message header.
- `TransformAvailability`: richer result for preflight checks via
  `BufferCore::check_transform`.
- `Transformable` and `HasHeader`: traits used by supported transformable
  messages.
- `Tf2Error`: Rust error enum for TF lookup, connectivity, extrapolation, and
  argument failures.

Frame semantics follow TF2: `lookup_transform(target, source, when)` returns the
transform `target <- source`, which is the transform you use to express data
from `source` in `target`.

## Requirements

- ROS 2 installed locally and sourceable from `/opt/ros/<distro>/setup.bash`
- Rust 1.88 or newer
- `cargo`, `colcon`, `rosdep`, and a C++17-capable compiler
- `rosidl_generator_rs` and the generated Rust interfaces from the selected
  ROS installation
- `colcon-cargo` and `colcon-ros-cargo`

`build.rs` reads `ROS_DISTRO` and `AMENT_PREFIX_PATH`. If your shell is not
sourced before building, the crate will fail to compile.

## Installation

### From crates.io (recommended)

In your own `ament_cargo` package, add `tf2_rs` next to the `rclrs` and
`ros-env` versions it is built against:

```toml
[dependencies]
tf2_rs = "0.1"
rclrs = "0.8"
ros-env = "0.3"
```

`tf2_rs` compiles a small C++ wrapper against the TF2 headers of your ROS
installation, so your `package.xml` also needs the packages it uses:

```xml
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>std_msgs</depend>
<depend>tf2</depend>
<depend>tf2_geometry_msgs</depend>
<depend>tf2_msgs</depend>
<depend>tf2_sensor_msgs</depend>
```

Then build your workspace as usual from a sourced shell:

```bash
source /opt/ros/<distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

API documentation is published at <https://olingo99.github.io/tf2_rs/>.
docs.rs cannot build this crate because it needs a ROS installation.

### From source

Current ROS binary installations from Humble through Rolling contain the Rust
generator and generated interfaces needed by `ros-env`, so no source message
repositories or `geometry2` checkout are required. For example, on Jazzy:

```bash
mkdir -p ~/tf2_rs_ws/src
cd ~/tf2_rs_ws/src
git clone https://github.com/olingo99/tf2_rs.git tf2_rs

cd ~/tf2_rs_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to tf2_rs
source install/setup.bash
```

### With Docker

`docker/Dockerfile` builds the repository against any supported distribution
without installing ROS or Rust locally. From the repository root:

```bash
docker build -f docker/Dockerfile --build-arg ROS_DISTRO=jazzy -t tf2_rs:jazzy .
docker run --rm tf2_rs:jazzy \
  bash -c '. /opt/ros/$ROS_DISTRO/setup.sh && cd src/tf2_rs && cargo test'
```

## Usage

### Manual transform insertion and lookup

```rust
use std::time::Duration;

use tf2_rs::{BufferCore, LookupTime, Tf2Error, TransformStamped};

fn main() -> Result<(), Tf2Error> {
    let mut buffer = BufferCore::new(Duration::from_secs(10));

    buffer.set_transform(
        &TransformStamped {
            stamp_sec: 1,
            stamp_nanosec: 0,
            parent_frame: "map".to_string(),
            child_frame: "base_link".to_string(),
            translation: [1.0, 2.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0], // x, y, z, w
        },
        "manual",
        true,
    )?;

    let tf = buffer.lookup_transform("map", "base_link", LookupTime::Latest)?;
    println!("{tf:?}");

    Ok(())
}
```

### Listen on `/tf` and transform a `PointCloud2`

`TransformListener` owns the subscriptions that populate the buffer, so it must
be kept alive for as long as you want TF updates.

```rust
use std::time::Duration;

use rclrs::{Context, CreateBasicExecutor, RclrsError, RclrsErrorFilter, SpinOptions};
use ros_env::sensor_msgs::msg::PointCloud2;
use tf2_rs::{BufferCore, TimeSpec, TransformListener};

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("tf2_rs_cloud_example")?;

    let buffer = BufferCore::new(Duration::from_secs(10));
    let _listener = TransformListener::new(&node, buffer.clone())?;

    let buffer_cb = buffer.clone();
    let _sub = node.create_subscription::<PointCloud2, _>(
        "/cloud_in",
        move |msg: PointCloud2| {
            match buffer_cb.transform(&msg, "map", TimeSpec::FromMsg) {
                Ok(out) => println!("transformed cloud into {}", out.header.frame_id),
                Err(err) => eprintln!("transform failed: {err}"),
            }
        },
    )?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
```

If you want lower-level control, call `lookup_transform(...)` yourself and then
use `msg.apply_transform(&tf)` on any type that implements `Transformable`.

## Citation

The paper describing this work, [*tf2_rs: Bringing tf2 to
Rust*](https://ceur-ws.org/Vol-4243/paper4.pdf), was published in the
[Proceedings of the 8th International Workshop on Robotics Software Engineering
(RoSE 2026)](https://ceur-ws.org/Vol-4243/). If you use `tf2_rs` in academic
work, please cite:

```bibtex
@inproceedings{engels2026tf2rs,
  author    = {Theo Engels and Antonio Paolillo and Ken Hasselmann},
  title     = {{tf2\_rs}: Bringing tf2 to Rust},
  booktitle = {Proceedings of the 8th International Workshop on Robotics Software Engineering (RoSE 2026)},
  series    = {CEUR Workshop Proceedings},
  volume    = {4243},
  publisher = {CEUR-WS.org},
  year      = {2026},
  url       = {https://ceur-ws.org/Vol-4243/paper4.pdf}
}
```

## Contributing

Issues and PRs are welcome.

CI builds and tests every push and pull request on Humble, Jazzy, Kilted,
Lyrical and Rolling. Releases are described in [`RELEASING.md`](RELEASING.md).

## License

MIT. See `LICENSE`.
