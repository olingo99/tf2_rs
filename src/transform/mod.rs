pub mod geometry_msgs;
pub mod sensor_msgs;

use crate::{Tf2Error, TransformStamped};

pub trait HasHeader {
    fn frame_id(&self) -> &str;
    fn stamp(&self) -> (i32, u32);
}

pub trait Transformable: HasHeader + Sized {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error>;
}

#[macro_export]
macro_rules! impl_has_header_for_ros2_msg {
    ($ty:ty) => {
        impl $crate::transform::HasHeader for $ty {
            fn frame_id(&self) -> &str {
                &self.header.frame_id
            }
            fn stamp(&self) -> (i32, u32) {
                (self.header.stamp.sec, self.header.stamp.nanosec)
            }
        }
    };
}
