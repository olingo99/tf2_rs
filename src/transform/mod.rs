/// Implements `HasHeader` and `Transformable` for a stamped message made of a
/// header and one transformable field: the field is transformed and the header
/// comes from the transform. Defined above the modules that use it, because a
/// `macro_rules!` macro is only visible to the code after it.
macro_rules! impl_stamped {
    ($stamped:ty, $field:ident) => {
        $crate::impl_has_header_for_ros2_msg!($stamped);

        impl $crate::Transformable for $stamped {
            fn apply_transform(
                &self,
                tf: &$crate::TransformStamped,
            ) -> Result<Self, $crate::Tf2Error> {
                Ok(Self {
                    header: $crate::transform::header_from_transform(tf),
                    $field: $crate::Transformable::apply_transform(&self.$field, tf)?,
                })
            }
        }
    };
}

mod geometry_msgs;
mod math;
mod sensor_msgs;
#[cfg(test)]
mod test_support;

use crate::{Tf2Error, TransformStamped};
use ros_env::{builtin_interfaces, std_msgs};

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

/// Header of a transformed stamped message. As in tf2, it takes the stamp of the
/// transform and its parent frame.
pub(crate) fn header_from_transform(tf: &TransformStamped) -> std_msgs::msg::Header {
    // Built directly: the generated `Header::default()` goes through the C message
    // init and conversion, which costs more than the transform itself.
    std_msgs::msg::Header {
        stamp: builtin_interfaces::msg::Time {
            sec: tf.stamp_sec,
            nanosec: tf.stamp_nanosec,
        },
        frame_id: tf.parent_frame.clone(),
    }
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
