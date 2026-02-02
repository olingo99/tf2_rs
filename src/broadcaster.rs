use rclrs::{IntoPrimitiveOptions, Publisher};
use tf2_msgs::msg::TFMessage;

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

#[derive(Clone)]
pub struct TransformBroadcaster {
    inner: TfBroadcasterInner,
}

impl TransformBroadcaster {
    pub fn new(node: &rclrs::Node) -> Result<Self, rclrs::RclrsError> {
        let pub_ = node.create_publisher("/tf".keep_last(100).reliable())?;
        Ok(Self {
            inner: TfBroadcasterInner::new(pub_),
        })
    }

    pub fn send_transform<T>(&self, tf: T) -> Result<(), rclrs::RclrsError>
    where
        T: Into<geometry_msgs::msg::TransformStamped>,
    {
        self.inner.send_transform(tf)
    }

    pub fn send_transforms<I, T>(&self, tfs: I) -> Result<(), rclrs::RclrsError>
    where
        I: IntoIterator<Item = T>,
        T: Into<geometry_msgs::msg::TransformStamped>,
    {
        self.inner.send_transforms(tfs)
    }
}

#[derive(Clone)]
pub struct StaticTransformBroadcaster {
    inner: TfBroadcasterInner,
}

impl StaticTransformBroadcaster {
    pub fn new(node: &rclrs::Node) -> Result<Self, rclrs::RclrsError> {
        let pub_ = node.create_publisher("/tf_static".keep_last(1).reliable().transient_local())?;
        Ok(Self {
            inner: TfBroadcasterInner::new(pub_),
        })
    }

    pub fn send_transform<T>(&self, tf: T) -> Result<(), rclrs::RclrsError>
    where
        T: Into<geometry_msgs::msg::TransformStamped>,
    {
        self.inner.send_transform(tf)
    }

    pub fn send_transforms<I, T>(&self, tfs: I) -> Result<(), rclrs::RclrsError>
    where
        I: IntoIterator<Item = T>,
        T: Into<geometry_msgs::msg::TransformStamped>,
    {
        self.inner.send_transforms(tfs)
    }
}
