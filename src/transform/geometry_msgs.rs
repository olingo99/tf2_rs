use crate::transform_stamped::TransformStamped;

pub fn convert_transform_stamped(t: &geometry_msgs::msg::TransformStamped) -> TransformStamped {
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
