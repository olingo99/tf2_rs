use rclrs::IntoPrimitiveOptions;
use crate::{BufferCore, TransformStamped};

/// NOTE: This assumes you have Rust message crates for:
/// - tf2_msgs/msg/TFMessage
/// - geometry_msgs/msg/TransformStamped
///
/// If you don’t, generate them in your ros2-rust workspace (typical), or use a shim approach.
/// (I can adapt this file to rclrs “dynamic messages” if you want to avoid compile-time msg deps.)
use tf2_msgs::msg::TFMessage;

pub struct TransformListener {
    // Hold subscriptions to keep them alive.
    _tf_sub: rclrs::Subscription<TFMessage>,
    _tf_static_sub: rclrs::Subscription<TFMessage>,
}

impl TransformListener {
    pub fn new(
        node: &rclrs::Node,
        buffer: BufferCore,
    ) -> Result<Self, rclrs::RclrsError> {
        // Match common TF listener QoS:
        // /tf: keep_last(100), reliable, volatile
        // /tf_static: keep_last(100), reliable, transient_local
        // :contentReference[oaicite:5]{index=5}
        let buf_tf = buffer.clone();
        let tf_sub = node.create_subscription(
            "/tf".keep_last(100).reliable(),
            move |msg: TFMessage| {
                for t in msg.transforms {
                    let tf = convert_transform_stamped(&t);
                    let _ = buf_tf.set_transform(&tf, "tf2_rs", false);
                }
            },
        )?;

        let buf_static = buffer.clone();
        let tf_static_sub = node.create_subscription(
            "/tf_static".keep_last(100).reliable().transient_local(),
            move |msg: TFMessage| {
                for t in msg.transforms {
                    let tf = convert_transform_stamped(&t);
                    let _ = buf_static.set_transform(&tf, "tf2_rs", true);
                }
            },
        )?;

        Ok(Self {
            _tf_sub: tf_sub,
            _tf_static_sub: tf_static_sub,
        })
    }
}

fn convert_transform_stamped(t: &geometry_msgs::msg::TransformStamped) -> TransformStamped {
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
