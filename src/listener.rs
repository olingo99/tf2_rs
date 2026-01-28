use crate::buffer::BufferCore;
use rclrs::{IntoPrimitiveOptions, WorkerSubscription, log_error};
use tf2_msgs::msg::TFMessage;

pub struct TransformListener {
    _tf_sub: WorkerSubscription<TFMessage, BufferCore>,
    _tf_static_sub: WorkerSubscription<TFMessage, BufferCore>,
}

impl TransformListener {
    pub fn new(node: &rclrs::Node, buffer: BufferCore) -> Result<Self, rclrs::RclrsError> {
        let buf_tf = buffer.clone();

        let worker = node.create_worker(buf_tf);

        let logger_cb = node.logger().clone();
        let tf_sub =
            worker.create_subscription("/tf".keep_last(100).reliable(), move |buf: &mut BufferCore, msg: TFMessage| {
                buf.ingest_tf_message(msg, "tf2_rs", false, |e| {
                    log_error!(&logger_cb, "Tf2 bindings error on set_transform:  {}", e)
                });
            })?;

        let logger_cb = node.logger().clone();

        let tf_static_sub = worker.create_subscription(
            "/tf_static".keep_last(100).reliable().transient_local(),
            move |buf: &mut BufferCore, msg: TFMessage| {
                buf.ingest_tf_message(msg, "tf2_rs", true, |e| {
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
