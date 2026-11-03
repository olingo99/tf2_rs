//! ROS 2 tf2 for Rust.
//!
//! `tf2_rs` keeps track of coordinate frames over time and transforms ROS
//! messages between them, like `tf2` and `tf2_ros` in C++:
//!
//! - [`BufferCore`] stores transforms and looks them up. It wraps the C++
//!   `tf2::BufferCore`, so lookups, interpolation and errors are upstream's.
//! - [`TransformListener`] fills a [`BufferCore`] from `/tf` and `/tf_static`.
//! - [`TransformBroadcaster`] and [`StaticTransformBroadcaster`] publish
//!   transforms.
//! - [`do_transform`] and [`BufferCore::transform`] apply a transform to a
//!   message. This runs in native Rust and is checked against upstream
//!   `tf2::doTransform`.
//!
//! # Example
//!
//! ```
//! use std::time::Duration;
//!
//! use ros_env::geometry_msgs::msg::Point;
//! use tf2_rs::{BufferCore, LookupTime, TransformStamped, do_transform};
//!
//! let mut buffer = BufferCore::new(Duration::from_secs(10));
//! buffer.set_transform(
//!     &TransformStamped {
//!         stamp_sec: 1,
//!         stamp_nanosec: 0,
//!         parent_frame: "map".to_string(),
//!         child_frame: "base_link".to_string(),
//!         translation: [1.0, 2.0, 0.0],
//!         rotation: [0.0, 0.0, 0.0, 1.0],
//!     },
//!     "example",
//!     true,
//! )?;
//!
//! // Expresses data from base_link in map.
//! let tf = buffer.lookup_transform("map", "base_link", LookupTime::Latest)?;
//! let p = do_transform(&Point { x: 1.0, y: 0.0, z: 0.0 }, &tf)?;
//! assert_eq!((p.x, p.y, p.z), (2.0, 2.0, 0.0));
//! # Ok::<(), tf2_rs::Tf2Error>(())
//! ```
//!
//! # Supported messages
//!
//! [`Transformable`] is implemented for these `geometry_msgs` and `sensor_msgs`
//! messages. Those that also implement [`HasHeader`] can use
//! [`BufferCore::transform`], which finds their transform; the others need
//! [`do_transform`] with a transform you looked up, as in C++ tf2.
//!
//! | Can use | Messages |
//! |---|---|
//! | [`do_transform`] and [`BufferCore::transform`] | `PointStamped`, `PolygonStamped`, `PoseStamped`, `PoseWithCovarianceStamped`, `QuaternionStamped`, `TransformStamped`, `Vector3Stamped`, `WrenchStamped`, `Imu`, `MagneticField`, `PointCloud2` |
//! | [`do_transform`] only | `Point`, `Point32`, `Polygon`, `Vector3`, `Quaternion`, `Pose`, `PoseWithCovariance`, `Transform`, `Wrench`, `VelocityStamped` |
//!
//! `VelocityStamped` has a header, but upstream has no `getFrameId` for it, so
//! it is transformed with [`do_transform`] only.
//!
//! Results follow the tf2 version installed with your ROS distribution, which
//! `build.rs` reads from its headers. See the
//! [README](https://github.com/olingo99/tf2_rs#parity-with-c-tf2) for how they
//! are checked and what differs between distributions.

#![warn(missing_docs)]

pub mod broadcaster;
pub mod buffer;
pub mod listener;
pub mod transform_stamped;

mod error;
mod ffi;
mod ffi_utils;
mod time;
mod transform;

pub use broadcaster::{StaticTransformBroadcaster, TransformBroadcaster};
pub use buffer::{BufferCore, TransformAvailability};
pub use error::Tf2Error;
pub use listener::TransformListener;
pub use time::{LookupTime, TimeSpec};
pub use transform::{HasHeader, Transformable, do_transform};
pub use transform_stamped::TransformStamped;
