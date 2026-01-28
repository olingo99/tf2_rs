use crate::ffi::ffi;

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
            translation_x: self.translation[0],
            translation_y: self.translation[1],
            translation_z: self.translation[2],
            rotation_x: self.rotation[0],
            rotation_y: self.rotation[1],
            rotation_z: self.rotation[2],
            rotation_w: self.rotation[3],
        }
    }

    pub fn from_ffi(v: ffi::Tf2TransformStamped) -> Self {
        Self {
            stamp_sec: v.stamp.sec,
            stamp_nanosec: v.stamp.nanosec,
            parent_frame: v.parent_frame,
            child_frame: v.child_frame,
            translation: [v.translation_x, v.translation_y, v.translation_z],
            rotation: [v.rotation_x, v.rotation_y, v.rotation_z, v.rotation_w],
        }
    }
}
