use crate::buffer::BufferCore;
use rclrs::{IntoPrimitiveOptions, log_error};
use tf2_msgs::msg::TFMessage;

pub struct TransformListener {
    _tf_sub: rclrs::Subscription<TFMessage>,
    _tf_static_sub: rclrs::Subscription<TFMessage>,
}

impl TransformListener {
    pub fn new(node: &rclrs::Node, buffer: BufferCore) -> Result<Self, rclrs::RclrsError> {
        let buf_tf = buffer.clone();
        let logger_cb = node.logger().clone();
        let tf_sub =
            node.create_subscription("/tf".keep_last(100).reliable(), move |msg: TFMessage| {
                buf_tf.ingest_tf_message(msg, "tf2_rs", false, |e| {
                    log_error!(&logger_cb, "Tf2 bindings error on set_transform:  {}", e)
                });
            })?;

        let logger_cb = node.logger().clone();

        let buf_static = buffer.clone();
        let tf_static_sub = node.create_subscription(
            "/tf_static".keep_last(100).reliable().transient_local(),
            move |msg: TFMessage| {
                buf_static.ingest_tf_message(msg, "tf2_rs", true, |e| {
                    log_error!(&logger_cb, "Tf2 bindings error on set_transform:  {}", e)
                });
            },
        )?;

        Ok(Self {
            _tf_sub: tf_sub,
            _tf_static_sub: tf_static_sub,
        })
    }
}
