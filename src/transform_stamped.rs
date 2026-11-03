//! [`TransformStamped`], the transform type of the buffer.

use crate::ffi::ffi;
use ros_env::{builtin_interfaces, geometry_msgs, std_msgs};

/// A transform between two frames at a given time, as stored in and returned by
/// [`BufferCore`](crate::BufferCore).
///
/// It converts to and from `geometry_msgs::msg::TransformStamped`. To transform
/// that message itself, use [`do_transform`](crate::do_transform).
#[derive(Clone, Debug)]
pub struct TransformStamped {
    /// Seconds of the time the transform is valid at.
    pub stamp_sec: i32,
    /// Nanoseconds of the time the transform is valid at.
    pub stamp_nanosec: u32,
    /// The frame the transform expresses data in (`header.frame_id`).
    pub parent_frame: String,
    /// The frame of the data it transforms (`child_frame_id`).
    pub child_frame: String,
    /// Translation `[x, y, z]`.
    pub translation: [f64; 3],
    /// Rotation as a quaternion `[x, y, z, w]`.
    pub rotation: [f64; 4],
}

impl TransformStamped {
    pub(crate) fn to_ffi(&self) -> ffi::Tf2TransformStamped {
        ffi::Tf2TransformStamped {
            stamp: ffi::Tf2Time {
                sec: self.stamp_sec,
                nanosec: self.stamp_nanosec,
            },
            parent_frame: self.parent_frame.clone(),
            child_frame: self.child_frame.clone(),
            tf: self.to_ffi_transform(),
        }
    }

    pub(crate) fn to_ffi_transform(&self) -> ffi::Tf2Transform {
        ffi::Tf2Transform {
            tx: self.translation[0],
            ty: self.translation[1],
            tz: self.translation[2],
            qx: self.rotation[0],
            qy: self.rotation[1],
            qz: self.rotation[2],
            qw: self.rotation[3],
        }
    }

    pub(crate) fn from_ffi(v: ffi::Tf2TransformStamped) -> Self {
        Self {
            stamp_sec: v.stamp.sec,
            stamp_nanosec: v.stamp.nanosec,
            parent_frame: v.parent_frame,
            child_frame: v.child_frame,
            translation: [v.tf.tx, v.tf.ty, v.tf.tz],
            rotation: [v.tf.qx, v.tf.qy, v.tf.qz, v.tf.qw],
        }
    }
}

impl From<&geometry_msgs::msg::TransformStamped> for TransformStamped {
    fn from(t: &geometry_msgs::msg::TransformStamped) -> Self {
        TransformStamped {
            stamp_sec: t.header.stamp.sec,
            stamp_nanosec: t.header.stamp.nanosec,
            parent_frame: t.header.frame_id.clone(),
            child_frame: t.child_frame_id.clone(),
            translation: [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ],
            rotation: [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ],
        }
    }
}

impl From<TransformStamped> for geometry_msgs::msg::TransformStamped {
    fn from(t: TransformStamped) -> Self {
        let [x, y, z] = t.translation;
        let [qx, qy, qz, qw] = t.rotation;
        // Built directly: the generated `default()` goes through the C message init
        // and conversion.
        Self {
            header: std_msgs::msg::Header {
                stamp: builtin_interfaces::msg::Time {
                    sec: t.stamp_sec,
                    nanosec: t.stamp_nanosec,
                },
                frame_id: t.parent_frame,
            },
            child_frame_id: t.child_frame,
            transform: geometry_msgs::msg::Transform {
                translation: geometry_msgs::msg::Vector3 { x, y, z },
                rotation: geometry_msgs::msg::Quaternion {
                    x: qx,
                    y: qy,
                    z: qz,
                    w: qw,
                },
            },
        }
    }
}

impl From<&TransformStamped> for geometry_msgs::msg::TransformStamped {
    fn from(t: &TransformStamped) -> Self {
        t.clone().into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conversions_to_and_from_the_message_keep_every_field() {
        let tf = TransformStamped {
            stamp_sec: 1,
            stamp_nanosec: 2,
            parent_frame: "map".into(),
            child_frame: "base_link".into(),
            translation: [3.0, 4.0, 5.0],
            rotation: [6.0, 7.0, 8.0, 9.0],
        };

        let msg: geometry_msgs::msg::TransformStamped = (&tf).into();
        assert_eq!((msg.header.stamp.sec, msg.header.stamp.nanosec), (1, 2));
        assert_eq!(msg.header.frame_id, "map");
        assert_eq!(msg.child_frame_id, "base_link");
        let (t, q) = (&msg.transform.translation, &msg.transform.rotation);
        assert_eq!([t.x, t.y, t.z], tf.translation);
        assert_eq!([q.x, q.y, q.z, q.w], tf.rotation);

        let back: TransformStamped = (&msg).into();
        assert_eq!((back.stamp_sec, back.stamp_nanosec), (1, 2));
        assert_eq!(back.parent_frame, "map");
        assert_eq!(back.child_frame, "base_link");
        assert_eq!(back.translation, tf.translation);
        assert_eq!(back.rotation, tf.rotation);

        let owned: geometry_msgs::msg::TransformStamped = tf.clone().into();
        assert_eq!(owned.child_frame_id, msg.child_frame_id);
    }
}
