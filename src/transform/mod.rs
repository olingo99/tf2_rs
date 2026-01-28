pub mod geometry_msgs;
pub mod sensor_msgs;

use crate::ffi::ffi;
use crate::{Tf2Error, TransformStamped};

pub trait HasHeader {
    fn frame_id(&self) -> &str;
    fn stamp(&self) -> (i32, u32);
}

pub trait Transformable: HasHeader + Sized {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error>;
}

pub(crate) fn header_to_ffi(h: &std_msgs::msg::Header) -> ffi::Tf2Header {
    ffi::Tf2Header {
        stamp: ffi::Tf2Time {
            sec: h.stamp.sec,
            nanosec: h.stamp.nanosec,
        },
        frame_id: h.frame_id.clone(),
    }
}

pub(crate) fn header_from_ffi(h: ffi::Tf2Header) -> std_msgs::msg::Header {
    let mut out = std_msgs::msg::Header::default();
    out.stamp.sec = h.stamp.sec;
    out.stamp.nanosec = h.stamp.nanosec;
    out.frame_id = h.frame_id;
    out
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
