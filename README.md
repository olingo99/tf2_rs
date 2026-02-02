## Disclaimer

This is a very early draft only built on ros2 jazzy and very breifly tested. Further work and tests may be done in the future.

# tf2_rs: ROS 2 TF2 bindings for Rust

`tf2_rs` bridges ROS 2 TF2 functionality into Rust via the `cxx` crate. It provides a safe, Rust-first API over TF2's `BufferCore` and select `doTransform` utilities.

## Requirements

- ROS 2 sourced in your shell
- Rust toolchain (stable)
- `cxx` and a C++17-capable compiler

Make sure your environment is set before building:

```bash
source /opt/ros/<distro>/setup.bash
```

## Build

Follow ros2 rust instalation insctructions for your distro. Then:


```bash
mkdir -p ws/src && cd ws
git clone -b jazzy https://github.com/ros2/common_interfaces.git src/common_interfaces
git clone -b jazzy https://github.com/ros2/example_interfaces.git src/example_interfaces
git clone -b jazzy https://github.com/ros2/rcl_interfaces.git src/rcl_interfaces
git clone -b jazzy https://github.com/ros2/rosidl_core.git src/rosidl_core
git clone -b jazzy https://github.com/ros2/rosidl_defaults.git src/rosidl_defaults
git clone -b jazzy https://github.com/ros2/unique_identifier_msgs.git src/unique_identifier_msgs
git clone https://github.com/ros2-rust/rosidl_rust.git src/rosidl_rust
git clone -b jazzy https://github.com/ros2/geometry2 src/geometry2

colcon build
```

## Usage

Basic TF2 lookup:

```rust
use tf2_rs::{BufferCore, LookupTime, Tf2Error, TransformStamped};

fn main() -> Result<(), Tf2Error> {
    let buffer = BufferCore::new(std::time::Duration::new(10, 0));
    let _ = buffer.set_transform(&TransformStamped{
        stamp_nanosec:1,
        stamp_sec:1,
        parent_frame:"map".to_string(),
        child_frame:"base_link".to_string(),
        translation: [1.0,1.0,1.0],
        rotation: [1.0,1.0,1.0,1.0]
    }, "manual", true);

    let tf = buffer.lookup_transform("map", "base_link", LookupTime::Latest)?;
    println!("transform: {tf:?}");

    let tf = buffer.lookup_transform("map", "odom", LookupTime::Latest)?;
    println!("transform: {tf:?}");


    Ok(())
}
```

Transform a pointcloud:
Listening to `/tf` and `/tf_static` with `rclrs` and transforming with `tf2_rs`.
Requires an external tf publisher.

```rust
use rclrs::*;
use tf2_rs::*;
type Pointcloud = sensor_msgs::msg::PointCloud2;

struct PointcloudTfNode{
    node: Node,
    publisher: Publisher<Pointcloud>,
    sub: Subscription<Pointcloud>,
    listener: TransformListener,
    buffer: BufferCore
}

impl PointcloudTfNode {
    fn new(executor: &Executor) -> Result<Self, RclrsError>{

        let node = executor.create_node("tf_example")?;

        let buffer = BufferCore::new(std::time::Duration::new(10, 0));
        let listener = TransformListener::new(&node, buffer.clone())?;
        let publisher = node.create_publisher("/cloud_out")?;

        let buffer_cb = buffer.clone();
        let pub_cb = publisher.clone();
        let logger_cb = node.logger().clone();
        let sub = node.create_subscription("cloud_in", move |msg: Pointcloud| Self::cb(msg, &buffer_cb, &pub_cb, &logger_cb))?;


        Ok(PointcloudTfNode
        {
            node,
            publisher,
            sub,
            listener,
            buffer
        })
    }

    fn cb(msg: Pointcloud, buffer: &BufferCore,  publisher: &Publisher<Pointcloud>, logger: &Logger){
        let target_frame = "base_link";

        let tf = match buffer.lookup_transform(target_frame, &msg.header.frame_id, LookupTime::Latest){
            Ok(t) => t,
            Err(e) => {log_error!(logger, "Error in lookup transform: {}", e); return}
        };



        let out = match msg.apply_transform(&tf) {
            Ok(o) => {println!("got good transform"); o},
            Err(e) => {
                eprintln!("transform_pointcloud2 failed: {:?}", e);
                return;
            }
        };

        if let Err(e) = publisher.publish(out.clone()) {
            eprintln!("publish failed: {:?}", e);
        }
    }
}



fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let minimal_sub = PointcloudTfNode::new(&executor)?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
```

## Contributing

Issues and PRs are welcome. Please include ROS 2 distro and Rust version in bug reports.

## License

MIT. See `LICENSE`.
