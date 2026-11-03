# tf2_rs

[![CI](https://github.com/olingo99/tf2_rs/actions/workflows/ci.yml/badge.svg)](https://github.com/olingo99/tf2_rs/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/tf2_rs.svg)](https://crates.io/crates/tf2_rs)
[![docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://olingo99.github.io/tf2_rs/)

`tf2_rs` brings ROS 2 tf2 to Rust. It wraps `tf2::BufferCore` through `cxx`,
integrates with `rclrs`, and transforms the ROS messages tf2 supports in native
Rust, checked against upstream C++.

## Status

- Validated against ROS 2 Humble, Jazzy, Kilted, Lyrical, and Rolling.
- Results are bit-exact with C++ tf2 on x86-64,[^x86] and equal up to rounding
  on arm64. See [Parity with C++ tf2](#parity-with-c-tf2).
- This crate is not a full `tf2_ros` replacement yet.

## What it provides

- `BufferCore`: store, look up and apply transforms.
- `TransformListener`: keep a `BufferCore` updated from `/tf` and `/tf_static`.
- `TransformBroadcaster` and `StaticTransformBroadcaster`: publish transforms.
- `do_transform` and `BufferCore::transform`: transform any supported message.

Supported messages, from `geometry_msgs` and `sensor_msgs`:

| Can use | Messages |
|---|---|
| `do_transform` and `BufferCore::transform` | `PointStamped`, `PolygonStamped`, `PoseStamped`, `PoseWithCovarianceStamped`, `QuaternionStamped`, `TransformStamped`, `Vector3Stamped`, `WrenchStamped`, `Imu`, `MagneticField`, `PointCloud2` |
| `do_transform` only | `Point`, `Point32`, `Polygon`, `Vector3`, `Quaternion`, `Pose`, `PoseWithCovariance`, `Transform`, `Wrench`, `VelocityStamped` |

Frame semantics follow tf2: `lookup_transform(target, source, when)` returns the
transform that expresses data from `source` in `target`.

The [API documentation](https://olingo99.github.io/tf2_rs/) describes every type
and function. It is on GitHub Pages because docs.rs cannot build this crate
without a ROS installation.

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

`tf2_rs` builds against the tf2 headers of your ROS installation, so your
`package.xml` also needs the packages it uses:

```xml
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>std_msgs</depend>
<depend>tf2</depend>
<depend>tf2_geometry_msgs</depend>
<depend>tf2_msgs</depend>
```

Then build your workspace as usual from a sourced shell:

```bash
source /opt/ros/<distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### Requirements

- ROS 2 Humble, Jazzy, Kilted, Lyrical or Rolling. Iron is end-of-life and
  current `rclrs` no longer ships Iron bindings.
- Rust 1.88 or newer, and a C++17 compiler.
- `colcon-cargo` and `colcon-ros-cargo`.

`build.rs` reads `ROS_DISTRO` and `AMENT_PREFIX_PATH`, so the crate fails to
compile if your shell is not sourced. It also reads the installed
`tf2_geometry_msgs` header to follow that version's
[behaviour](#behaviour-by-distribution).

### From source

ROS binary installations from Humble through Rolling contain the Rust generator
and generated interfaces, so no other source repositories are needed. On Jazzy:

```bash
mkdir -p ~/tf2_rs_ws/src && cd ~/tf2_rs_ws/src
git clone https://github.com/olingo99/tf2_rs.git
cd ~/tf2_rs_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to tf2_rs
```

### With Docker

`docker/Dockerfile` builds the repository for any supported distribution without
installing ROS or Rust locally. From the repository root:

```bash
docker build -f docker/Dockerfile --build-arg ROS_DISTRO=jazzy -t tf2_rs:jazzy .
docker run --rm tf2_rs:jazzy \
  bash -c '. /opt/ros/$ROS_DISTRO/setup.sh && cd src/tf2_rs && cargo test'
```

## Usage

### Look up a transform and apply it

```rust
use std::time::Duration;

use ros_env::geometry_msgs::msg::Point;
use tf2_rs::{BufferCore, LookupTime, Tf2Error, TransformStamped, do_transform};

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
    let point = Point { x: 1.0, y: 0.0, z: 0.0 };
    println!("{:?}", do_transform(&point, &tf)?);

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

## More on transforming

### Choosing how to transform

- `buffer.transform(&msg, "map", time)` finds the transform from the message's
  own frame. `time` is a `TimeSpec`: `Latest`, `Stamp { sec, nanosec }`, or
  `FromMsg` to use the stamp in the message's header.
- `do_transform(&msg, &tf)`, or `msg.apply_transform(&tf)`, uses the transform
  you pass. It is the only way for messages without a frame, as in C++ tf2.

`VelocityStamped` has a header but can only use `do_transform`, as upstream has
no `getFrameId` for it.

To check a transform exists first, use `buffer.can_transform(...)`, or
`buffer.check_transform(...)` for a diagnostic when it doesn't.

### Your own message types

Implement `Transformable` to use `do_transform`, and `HasHeader` as well to use
`buffer.transform`:

```rust
use ros_env::{geometry_msgs::msg::Point, std_msgs::msg::Header};
use tf2_rs::{Tf2Error, Transformable, TransformStamped, do_transform};

struct Landmark {
    header: Header,
    position: Point,
    label: String,
}

// Provides frame_id() and stamp() for a type with a `header` field.
tf2_rs::impl_has_header_for_ros2_msg!(Landmark);

impl Transformable for Landmark {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        Ok(Self {
            header: Header {
                stamp: ros_env::builtin_interfaces::msg::Time {
                    sec: tf.stamp_sec,
                    nanosec: tf.stamp_nanosec,
                },
                frame_id: tf.parent_frame.clone(),
            },
            position: do_transform(&self.position, tf)?,
            label: self.label.clone(),
        })
    }
}
```

Rust only lets you implement these traits for types defined in your crate. To
transform a ROS message that `tf2_rs` does not support, such as `PoseArray`,
wrap it in a struct of your own (`struct Poses(PoseArray);`) and implement them
on that.

### Broadcasting

`send_transform` accepts a `TransformStamped` or a
`geometry_msgs::msg::TransformStamped`, and `send_transforms` takes several.
`StaticTransformBroadcaster` works the same way on `/tf_static`.

```rust
use rclrs::{Context, CreateBasicExecutor, RclrsError};
use tf2_rs::{TransformBroadcaster, TransformStamped};

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("tf2_rs_broadcast_example")?;

    let broadcaster = TransformBroadcaster::new(&node)?;
    broadcaster.send_transform(TransformStamped {
        stamp_sec: 1,
        stamp_nanosec: 0,
        parent_frame: "map".to_string(),
        child_frame: "base_link".to_string(),
        translation: [1.0, 2.0, 0.0],
        rotation: [0.0, 0.0, 0.0, 1.0], // x, y, z, w
    })?;
    Ok(())
}
```

## Parity with C++ tf2

`tf2_rs` looks up transforms through the real `tf2::BufferCore`, and applies them
in native Rust. The test-only [`oracle/`](oracle/) crate calls the installed
`tf2::doTransform` for every supported message, on the same inputs as Rust
(including NaN, infinities, zero and non-unit quaternions), and compares whole
messages, headers included. CI runs it on every supported distribution:

```bash
cargo test -p tf2_rs_oracle -- --nocapture
```

### Accuracy

- **x86-64:** bit-exact.[^x86]
- **arm64:** equal up to rounding. C++ compilers there fuse `a * b + c` into one
  operation that rounds once where Rust rounds twice, so the last bits can
  differ. C++ even disagrees with itself between optimisation levels, so no
  single Rust version could match it everywhere.

We measured every message type on arm64 (Jazzy and Rolling, under emulation):
48–99% of values are bit-identical, and the largest difference is about 1e-14
relative for `f64` values and 2e-7 for the `f32` values of `PointCloud2`, or
0.02 mm at 100 m. Edge inputs give a few larger differences, all harmless:

- a 180° rotation can come out as `-q` instead of `q`, which is the same
  rotation;
- an infinite input can give NaN instead of infinity;
- in `VelocityStamped`, inputs of 1e300 that cancel out leave a result around
  1e284 that differs from upstream's, though by far less than the inputs.

The arm64 CI job runs the parity tests with a tolerance, but does not allow
these edge cases yet, so it is expected to fail and does not block a PR.

### Behaviour by distribution

`tf2_rs` follows what upstream does on your distribution, so one version of the
crate can give different results on two distributions. `build.rs` detects this
from the installed headers, not the distribution's name, so it also follows fixes
backported upstream.

| Message | Humble, Jazzy, Kilted | Lyrical, Rolling |
|---|---|---|
| `TransformStamped` | `child_frame_id` is left empty | `child_frame_id` is copied from the input |
| `VelocityStamped` | The input velocity is added to the result, and the input's header is kept | The result is the transformed velocity, with the transform's header |

Upstream has no `Imu` or `MagneticField` transform on Humble. `tf2_rs` provides
them there with the same code as elsewhere, but cannot compare them.

### Behaviour that may surprise you

These match upstream unless stated otherwise.

- **Headers:** a transformed stamped message takes the transform's stamp and
  parent frame, not its own stamp.
- **Non-unit quaternions** also scale, except in pose covariance and
  `VelocityStamped`, which normalise.
- **Signs:** a `Quaternion` keeps the sign of the product, while a `Pose`
  rebuilds its orientation from a matrix, so the same rotation can come out with
  opposite signs.
- **`PointCloud2`:** the point count is `data.len() / point_step`, not
  `width * height`, and `x`, `y`, `z` and `normal_*` are read as native-endian
  `f32` whatever datatype the fields declare.
- **`PointCloud2`, unlike upstream:** an empty cloud returns an empty cloud,
  where upstream is undefined behaviour. A field past `point_step`, or data that
  isn't a whole number of points, returns `InvalidArgument`, where upstream reads
  out of bounds.

[^x86]: With default compiler flags. C++ built with `-mfma`, or with
    `-march` for a CPU with FMA, rounds like arm64 instead.

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

Issues and PRs are welcome. CI builds and tests every push and pull request on
every supported distribution, including the [parity tests](#parity-with-c-tf2).
Releases are described in [`RELEASING.md`](RELEASING.md).

## License

MIT. See `LICENSE`.
