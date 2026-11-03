//! Publishing transforms on `/tf` and `/tf_static`.

use rclrs::{IntoPrimitiveOptions, Publisher};
use ros_env::{geometry_msgs, tf2_msgs::msg::TFMessage};

#[derive(Clone)]
struct TfBroadcasterInner {
    pub_: Publisher<TFMessage>,
}

impl TfBroadcasterInner {
    fn new(pub_: Publisher<TFMessage>) -> Self {
        Self { pub_ }
    }

    fn send_transform<T>(&self, tf: T) -> Result<(), rclrs::RclrsError>
    where
        T: Into<geometry_msgs::msg::TransformStamped>,
    {
        let tf: geometry_msgs::msg::TransformStamped = tf.into();
        self.send_transforms(std::iter::once(tf))
    }

    fn send_transforms<I, T>(&self, tfs: I) -> Result<(), rclrs::RclrsError>
    where
        I: IntoIterator<Item = T>,
        T: Into<geometry_msgs::msg::TransformStamped>,
    {
        let msg = TFMessage {
            transforms: tfs.into_iter().map(Into::into).collect(),
        };
        self.pub_.publish(msg)
    }
}

/// Publishes transforms that change over time on `/tf`.
///
/// Clones publish through the same publisher. Like any `rclrs` publisher, it
/// needs nothing to spin.
///
/// ```no_run
/// use rclrs::{Context, CreateBasicExecutor};
/// use tf2_rs::{TransformBroadcaster, TransformStamped};
///
/// let context = Context::default_from_env()?;
/// let executor = context.create_basic_executor();
/// let node = executor.create_node("broadcaster")?;
///
/// let broadcaster = TransformBroadcaster::new(&node)?;
/// broadcaster.send_transform(TransformStamped {
///     stamp_sec: 1,
///     stamp_nanosec: 0,
///     parent_frame: "map".to_string(),
///     child_frame: "base_link".to_string(),
///     translation: [1.0, 2.0, 0.0],
///     rotation: [0.0, 0.0, 0.0, 1.0],
/// })?;
/// # Ok::<(), rclrs::RclrsError>(())
/// ```
#[derive(Clone)]
pub struct TransformBroadcaster {
    inner: TfBroadcasterInner,
}

impl TransformBroadcaster {
    /// Creates a `/tf` publisher on `node`, with the same quality of service as
    /// `tf2_ros`: reliable, keeping the last 100 messages.
    pub fn new(node: &rclrs::Node) -> Result<Self, rclrs::RclrsError> {
        let pub_ = node.create_publisher("/tf".keep_last(100).reliable())?;
        Ok(Self {
            inner: TfBroadcasterInner::new(pub_),
        })
    }

    /// Publishes one transform. Accepts a [`TransformStamped`](crate::TransformStamped)
    /// or a `geometry_msgs::msg::TransformStamped`.
    pub fn send_transform<T>(&self, tf: T) -> Result<(), rclrs::RclrsError>
    where
        T: Into<geometry_msgs::msg::TransformStamped>,
    {
        self.inner.send_transform(tf)
    }

    /// Publishes several transforms in one message.
    pub fn send_transforms<I, T>(&self, tfs: I) -> Result<(), rclrs::RclrsError>
    where
        I: IntoIterator<Item = T>,
        T: Into<geometry_msgs::msg::TransformStamped>,
    {
        self.inner.send_transforms(tfs)
    }
}

/// Publishes transforms that never change on `/tf_static`.
///
/// The topic is transient-local, so subscribers that join later still receive
/// the last message. Unlike `tf2_ros`, this broadcaster does not yet accumulate
/// what it sent: only the transforms of the most recent call reach late
/// subscribers, so send all static transforms in one
/// [`send_transforms`](Self::send_transforms) call.
#[derive(Clone)]
pub struct StaticTransformBroadcaster {
    inner: TfBroadcasterInner,
}

impl StaticTransformBroadcaster {
    /// Creates a `/tf_static` publisher on `node`: reliable, transient-local,
    /// keeping the last message.
    pub fn new(node: &rclrs::Node) -> Result<Self, rclrs::RclrsError> {
        let pub_ = node.create_publisher("/tf_static".keep_last(1).reliable().transient_local())?;
        Ok(Self {
            inner: TfBroadcasterInner::new(pub_),
        })
    }

    /// Publishes one transform. Accepts a [`TransformStamped`](crate::TransformStamped)
    /// or a `geometry_msgs::msg::TransformStamped`.
    pub fn send_transform<T>(&self, tf: T) -> Result<(), rclrs::RclrsError>
    where
        T: Into<geometry_msgs::msg::TransformStamped>,
    {
        self.inner.send_transform(tf)
    }

    /// Publishes several transforms in one message.
    pub fn send_transforms<I, T>(&self, tfs: I) -> Result<(), rclrs::RclrsError>
    where
        I: IntoIterator<Item = T>,
        T: Into<geometry_msgs::msg::TransformStamped>,
    {
        self.inner.send_transforms(tfs)
    }
}
