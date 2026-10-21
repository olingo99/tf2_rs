use crate::ffi::ffi;
use ros_env::geometry_msgs;

#[derive(Clone, Debug)]
pub struct TransformStamped {
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
    pub parent_frame: String,
    pub child_frame: String,
    pub translation: [f64; 3],
    pub rotation: [f64; 4], // x,y,z,w
}

impl TransformStamped {
    pub fn to_ffi(&self) -> ffi::Tf2TransformStamped {
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

    pub fn from_ffi(v: ffi::Tf2TransformStamped) -> Self {
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

impl From<TransformStamped> for geometry_msgs::msg::TransformStamped {
    fn from(t: TransformStamped) -> Self {
        let mut msg = geometry_msgs::msg::TransformStamped::default();

        msg.header.stamp.sec = t.stamp_sec;
        msg.header.stamp.nanosec = t.stamp_nanosec;
        msg.header.frame_id = t.parent_frame;

        msg.child_frame_id = t.child_frame;

        msg.transform.translation.x = t.translation[0];
        msg.transform.translation.y = t.translation[1];
        msg.transform.translation.z = t.translation[2];

        msg.transform.rotation.x = t.rotation[0];
        msg.transform.rotation.y = t.rotation[1];
        msg.transform.rotation.z = t.rotation[2];
        msg.transform.rotation.w = t.rotation[3];

        msg
    }
}

impl From<&TransformStamped> for geometry_msgs::msg::TransformStamped {
    fn from(t: &TransformStamped) -> Self {
        let mut msg = geometry_msgs::msg::TransformStamped::default();

        msg.header.stamp.sec = t.stamp_sec;
        msg.header.stamp.nanosec = t.stamp_nanosec;
        msg.header.frame_id = t.parent_frame.clone();

        msg.child_frame_id = t.child_frame.clone();

        msg.transform.translation.x = t.translation[0];
        msg.transform.translation.y = t.translation[1];
        msg.transform.translation.z = t.translation[2];

        msg.transform.rotation.x = t.rotation[0];
        msg.transform.rotation.y = t.rotation[1];
        msg.transform.rotation.z = t.rotation[2];
        msg.transform.rotation.w = t.rotation[3];

        msg
    }
}
