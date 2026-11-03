//! Filling a buffer from `/tf` and `/tf_static`.

use crate::buffer::BufferCore;
use rclrs::{IntoPrimitiveOptions, WorkerSubscription, log_error};
use ros_env::tf2_msgs::msg::TFMessage;

/// Keeps a [`BufferCore`] up to date from `/tf` and `/tf_static`.
///
/// It subscribes on the node it is given, so that node's executor must spin for
/// transforms to arrive. Updates stop when the listener is dropped: keep it alive
/// as long as you need them. Read the buffer through a clone of the one you
/// passed in, which shares its contents.
///
/// ```no_run
/// use std::time::Duration;
///
/// use rclrs::{Context, CreateBasicExecutor, SpinOptions};
/// use tf2_rs::{BufferCore, TransformListener};
///
/// let context = Context::default_from_env()?;
/// let mut executor = context.create_basic_executor();
/// let node = executor.create_node("listener")?;
///
/// let buffer = BufferCore::new(Duration::from_secs(10));
/// let _listener = TransformListener::new(&node, buffer.clone())?;
/// // Look up transforms in `buffer` from callbacks, then:
/// executor.spin(SpinOptions::default());
/// # Ok::<(), rclrs::RclrsError>(())
/// ```
pub struct TransformListener {
    _tf_sub: WorkerSubscription<TFMessage, BufferCore>,
    _tf_static_sub: WorkerSubscription<TFMessage, BufferCore>,
}

impl TransformListener {
    /// Subscribes to `/tf` and `/tf_static` on `node` and adds what arrives to
    /// `buffer`. Transforms that tf2 rejects are logged on the node's logger.
    pub fn new(node: &rclrs::Node, buffer: BufferCore) -> Result<Self, rclrs::RclrsError> {
        let worker = node.create_worker(buffer);

        let logger_cb = node.logger().clone();
        let tf_sub = worker.create_subscription(
            "/tf".keep_last(100).reliable(),
            move |buf: &mut BufferCore, msg: TFMessage| {
                buf.ingest_tf_message(msg, "tf2_rs", false, |e| {
                    log_error!(&logger_cb, "Tf2 bindings error on set_transform: {}", e)
                });
            },
        )?;

        let logger_cb = node.logger().clone();

        let tf_static_sub = worker.create_subscription(
            "/tf_static".keep_last(100).reliable().transient_local(),
            move |buf: &mut BufferCore, msg: TFMessage| {
                buf.ingest_tf_message(msg, "tf2_rs", true, |e| {
                    log_error!(&logger_cb, "Tf2 bindings error on set_transform: {}", e)
                });
            },
        )?;

        Ok(Self {
            _tf_sub: tf_sub,
            _tf_static_sub: tf_static_sub,
        })
    }
}
