//! Publishing transforms on `/tf` and `/tf_static`.

use std::sync::{Arc, Mutex, PoisonError};

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

/// Every static transform sent so far, shared by all clones, like `tf2_ros`'s
/// `net_message_`.
#[derive(Clone)]
struct StaticTfBroadcasterInner {
    pub_: Publisher<TFMessage>,
    net_message: Arc<Mutex<TFMessage>>,
}

impl StaticTfBroadcasterInner {
    fn new(pub_: Publisher<TFMessage>) -> Self {
        Self {
            pub_,
            net_message: Arc::new(Mutex::new(TFMessage {
                transforms: Vec::new(),
            })),
        }
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
        // Convert before locking, so a panicking conversion can't poison the lock.
        let incoming = tfs.into_iter().map(Into::into).collect();
        // The set stays valid after a panic: each replace or push is whole.
        let mut net_message = self
            .net_message
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        insert_or_replace(&mut net_message.transforms, incoming);
        // Publish under the lock, so the last message published, the one late
        // subscribers receive, is always the full set.
        self.pub_.publish(&*net_message)
    }
}

/// Adds `incoming` to `transforms` as `tf2_ros` does: each transform replaces
/// the one with the same `child_frame_id` in place, or is appended if there is
/// none.
fn insert_or_replace(
    transforms: &mut Vec<geometry_msgs::msg::TransformStamped>,
    incoming: Vec<geometry_msgs::msg::TransformStamped>,
) {
    for new_tf in incoming {
        match transforms
            .iter_mut()
            .find(|tf| tf.child_frame_id == new_tf.child_frame_id)
        {
            Some(tf) => *tf = new_tf,
            None => transforms.push(new_tf),
        }
    }
}

/// Publishes transforms that never change on `/tf_static`.
///
/// As in `tf2_ros`, the broadcaster keeps every transform it has sent and
/// publishes all of them on each call. A transform with the same child frame as
/// an earlier one replaces it. The topic is transient-local, so subscribers
/// that join later still receive the whole set. Clones share that set.
#[derive(Clone)]
pub struct StaticTransformBroadcaster {
    inner: StaticTfBroadcasterInner,
}

impl StaticTransformBroadcaster {
    /// Creates a `/tf_static` publisher on `node`: reliable, transient-local,
    /// keeping the last message.
    pub fn new(node: &rclrs::Node) -> Result<Self, rclrs::RclrsError> {
        let pub_ = node.create_publisher("/tf_static".keep_last(1).reliable().transient_local())?;
        Ok(Self {
            inner: StaticTfBroadcasterInner::new(pub_),
        })
    }

    /// Adds one transform, then publishes every transform sent so far. Accepts a
    /// [`TransformStamped`](crate::TransformStamped) or a
    /// `geometry_msgs::msg::TransformStamped`.
    pub fn send_transform<T>(&self, tf: T) -> Result<(), rclrs::RclrsError>
    where
        T: Into<geometry_msgs::msg::TransformStamped>,
    {
        self.inner.send_transform(tf)
    }

    /// Adds several transforms, then publishes every transform sent so far in
    /// one message.
    pub fn send_transforms<I, T>(&self, tfs: I) -> Result<(), rclrs::RclrsError>
    where
        I: IntoIterator<Item = T>,
        T: Into<geometry_msgs::msg::TransformStamped>,
    {
        self.inner.send_transforms(tfs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TransformStamped;

    fn tf(child: &str, x: f64) -> geometry_msgs::msg::TransformStamped {
        TransformStamped {
            stamp_sec: 0,
            stamp_nanosec: 0,
            parent_frame: "map".into(),
            child_frame: child.into(),
            translation: [x, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
        .into()
    }

    fn frames(transforms: &[geometry_msgs::msg::TransformStamped]) -> Vec<(&str, f64)> {
        transforms
            .iter()
            .map(|t| (t.child_frame_id.as_str(), t.transform.translation.x))
            .collect()
    }

    #[test]
    fn insert_or_replace_appends_new_children_and_replaces_in_place() {
        let mut transforms = vec![tf("a", 0.0), tf("b", 0.0)];
        insert_or_replace(&mut transforms, vec![tf("c", 1.0), tf("a", 2.0)]);
        assert_eq!(frames(&transforms), [("a", 2.0), ("b", 0.0), ("c", 1.0)]);
    }

    #[test]
    fn insert_or_replace_keeps_the_last_of_a_repeated_child_in_one_call() {
        let mut transforms = Vec::new();
        insert_or_replace(&mut transforms, vec![tf("a", 1.0), tf("a", 2.0)]);
        assert_eq!(frames(&transforms), [("a", 2.0)]);
    }
}
