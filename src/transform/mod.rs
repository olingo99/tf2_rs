pub mod geometry_msgs;
pub mod sensor_msgs;

use crate::{Tf2Error, TransformStamped};

pub trait HasHeader {
    fn frame_id(&self) -> &str;
    fn stamp(&self) -> (i32, u32);
}

pub trait Transformable: Sized {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error>;
}

pub fn do_transform<T: Transformable>(input: &T, tf: &TransformStamped) -> Result<T, Tf2Error> {
    input.apply_transform(tf)
}

#[macro_export]
macro_rules! impl_has_header_for_ros2_msg {
    ($ty:ty) => {
        impl $crate::HasHeader for $ty {
            fn frame_id(&self) -> &str {
                &self.header.frame_id
            }
            fn stamp(&self) -> (i32, u32) {
                (self.header.stamp.sec, self.header.stamp.nanosec)
            }
        }
    };
}
