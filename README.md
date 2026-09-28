# tf2_rs

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
- Rust 1.85 or newer
- `cargo`, `colcon`, `rosdep`, and a C++17-capable compiler
- `rosidl_generator_rs` and the generated Rust interfaces from the selected
  ROS installation
- `colcon-cargo` and `colcon-ros-cargo`

`build.rs` reads `ROS_DISTRO` and `AMENT_PREFIX_PATH`. If your shell is not
sourced before building, the crate will fail to compile.

## Installation

### Recommended workflow

Current ROS binary installations from Humble through Rolling contain the Rust
generator and generated interfaces needed by `ros-env`. Add this package under
`src/`; no source message repositories or `geometry2` checkout are required.

```bash
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install --packages-up-to tf2_rs
source install/setup.bash
```

### Minimal workspace

For example, on Jazzy:

```bash
mkdir -p ~/tf2_rs_ws/src
cd ~/tf2_rs_ws/src

git clone https://github.com/olingo99/tf2_rs.git tf2_rs
```

Install system dependencies and build the crate:

```bash
cd ~/tf2_rs_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to tf2_rs
source install/setup.bash
```

A Docker build for any supported distribution is available from the repository
root:

```bash
docker build -f docker/Dockerfile --build-arg ROS_DISTRO=jazzy .
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

use rclrs::{Context, RclrsError, SpinOptions};
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

## License

MIT. See `LICENSE`.
