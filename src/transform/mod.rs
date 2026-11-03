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

/// A message that knows its frame and time, so
/// [`BufferCore::transform`](crate::BufferCore::transform) can look up its
/// transform.
///
/// For a type with a `std_msgs/Header` field named `header`,
/// [`impl_has_header_for_ros2_msg!`](crate::impl_has_header_for_ros2_msg)
/// implements it.
pub trait HasHeader {
    /// The frame the data is expressed in.
    fn frame_id(&self) -> &str;
    /// The time of the data, as `(sec, nanosec)`.
    fn stamp(&self) -> (i32, u32);
}

/// A message that a transform can be applied to.
///
/// Implemented for every message that `tf2_geometry_msgs` and `tf2_sensor_msgs`
/// transform; see the [crate documentation](crate#supported-messages). Implement
/// it for your own types to use [`do_transform`], and [`HasHeader`] as well to
/// use [`BufferCore::transform`](crate::BufferCore::transform):
///
/// ```
/// use ros_env::{geometry_msgs::msg::Point, std_msgs::msg::Header};
/// use tf2_rs::{Tf2Error, Transformable, TransformStamped, do_transform};
///
/// struct Landmark {
///     header: Header,
///     position: Point,
/// }
///
/// tf2_rs::impl_has_header_for_ros2_msg!(Landmark);
///
/// impl Transformable for Landmark {
///     fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
///         Ok(Self {
///             // As in tf2: the frame and stamp of the transform.
///             header: Header {
///                 stamp: ros_env::builtin_interfaces::msg::Time {
///                     sec: tf.stamp_sec,
///                     nanosec: tf.stamp_nanosec,
///                 },
///                 frame_id: tf.parent_frame.clone(),
///             },
///             position: do_transform(&self.position, tf)?,
///         })
///     }
/// }
/// ```
///
/// Rust only lets you implement it for a type defined in your crate. For a ROS
/// message, such as one `tf2_rs` does not support, wrap it in a struct of your
/// own and implement it on that.
pub trait Transformable: Sized {
    /// Applies `tf` to `self`. As in tf2, a message with a header takes the
    /// stamp and parent frame of `tf`, except `VelocityStamped` on Humble to
    /// Kilted, which keeps its own as upstream does there.
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error>;
}

/// Applies `tf` to `input`, like `tf2::doTransform`. `tf` usually comes from
/// [`BufferCore::lookup_transform`](crate::BufferCore::lookup_transform).
///
/// Use it for messages without a header, or to apply one transform to several
/// messages.
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

/// Implements [`HasHeader`](crate::HasHeader) for a type with a
/// `std_msgs/Header` field named `header`.
///
/// ```
/// use ros_env::std_msgs::msg::Header;
///
/// struct Detection {
///     header: Header,
///     score: f32,
/// }
///
/// tf2_rs::impl_has_header_for_ros2_msg!(Detection);
/// ```
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
