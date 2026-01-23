use crate::buffer::BufferCore;
use crate::transform::geometry_msgs::convert_transform_stamped;
use rclrs::{IntoPrimitiveOptions, Logger, log_error};

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
    pub fn new(node: &rclrs::Node, buffer: BufferCore) -> Result<Self, rclrs::RclrsError> {
        // Match common TF listener QoS:
        // /tf: keep_last(100), reliable, volatile
        // /tf_static: keep_last(100), reliable, transient_local
        // :contentReference[oaicite:5]{index=5}
        let buf_tf = buffer.clone();
        let node_cb = node.clone();
        let tf_sub =
            node.create_subscription("/tf".keep_last(100).reliable(), move |msg: TFMessage| {
                buf_tf.ingest_tf_message(msg, "tf2_rs", false, |e| {
                    log_error!(
                        node_cb.logger(),
                        "Tf2 bindings error on set_transform:  {}",
                        e
                    )
                });
            })?;

        let node_cb = node.clone();

        let buf_static = buffer.clone();
        let tf_static_sub = node.create_subscription(
            "/tf_static".keep_last(100).reliable().transient_local(),
            move |msg: TFMessage| {
                buf_static.ingest_tf_message(msg, "tf2_rs", true, |e| {
                    log_error!(
                        node_cb.logger(),
                        "Tf2 bindings error on set_transform:  {}",
                        e
                    )
                });
            },
        )?;

        Ok(Self {
            _tf_sub: tf_sub,
            _tf_static_sub: tf_static_sub,
        })
    }
}
