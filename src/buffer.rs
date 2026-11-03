//! The transform buffer.

use crate::HasHeader;
use crate::error::Tf2Error;
use crate::ffi::ffi::{self, BufferCoreWrapper};
use crate::ffi_utils::{call_bool, call_out};
use crate::time::{LookupTime, TimeSpec};
use crate::transform::Transformable;
use crate::transform_stamped::TransformStamped;
use ros_env::tf2_msgs;

// SAFETY: the wrapper holds only a `tf2::BufferCore`, which guards its frames
// and caches with its own mutex, so it can be used and shared across threads.
unsafe impl Send for BufferCoreWrapper {}
unsafe impl Sync for BufferCoreWrapper {}

/// Stores transforms between frames and looks them up at a given time.
///
/// It wraps the C++ `tf2::BufferCore`, so lookups, interpolation, the frame
/// graph and error messages are upstream's.
///
/// Clones share the same buffer: a transform set through one is visible through
/// all of them. That is how a [`TransformListener`](crate::TransformListener)
/// fills the buffer you read from. The buffer is safe to use from several
/// threads.
#[derive(Clone)]
pub struct BufferCore {
    inner: cxx::SharedPtr<BufferCoreWrapper>,
}

impl BufferCore {
    /// Creates an empty buffer that keeps each frame's history for `cache_time`.
    /// `tf2_ros` uses 10 seconds.
    pub fn new(cache_time: std::time::Duration) -> Self {
        Self {
            inner: ffi::new_buffer_core(cache_time.as_nanos() as u64),
        }
    }

    fn wrapper(&self) -> &BufferCoreWrapper {
        self.inner.as_ref().expect("BufferCoreWrapper is null")
    }

    /// Removes every transform except static ones.
    pub fn clear(&mut self) {
        self.wrapper().clear();
    }

    /// Adds a transform. `authority` names its source in error messages, and
    /// `is_static` marks it valid at all times.
    ///
    /// Returns [`Tf2Error::InvalidArgument`] if tf2 rejects it: an empty child
    /// frame, a child equal to its parent, a NaN value, or a stamp older than
    /// the cache. tf2 logs the reason.
    pub fn set_transform(
        &mut self,
        tf: &TransformStamped,
        authority: &str,
        is_static: bool,
    ) -> Result<(), Tf2Error> {
        let ok = call_bool(|out_ok| {
            self.wrapper()
                .set_transform(&tf.to_ffi(), authority, is_static, out_ok)
        })?;

        if ok {
            Ok(())
        } else {
            Err(Tf2Error::InvalidArgument(
                "set_transform rejected the transform".to_string(),
            ))
        }
    }

    /// Whether `source_frame` can be transformed into `target_frame` at `when`,
    /// with tf2's explanation when it cannot.
    pub fn check_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
        when: LookupTime,
    ) -> Result<TransformAvailability, Tf2Error> {
        let t = ffi::Tf2Time::from(when);
        let out = call_out(|out| {
            self.wrapper()
                .can_transform(target_frame, source_frame, &t, out)
        })?;

        Ok(if out.available {
            TransformAvailability::Available
        } else {
            TransformAvailability::Unavailable {
                diagnostic: (!out.diagnostic.is_empty()).then_some(out.diagnostic),
            }
        })
    }

    /// Whether `source_frame` can be transformed into `target_frame` at `when`.
    /// [`check_transform`](Self::check_transform) also says why not.
    pub fn can_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
        when: LookupTime,
    ) -> Result<bool, Tf2Error> {
        Ok(self
            .check_transform(target_frame, source_frame, when)?
            .is_available())
    }

    /// The transform that expresses data from `source_frame` in `target_frame`
    /// at `when`, interpolated between stored transforms.
    ///
    /// Fails with [`Tf2Error::Lookup`] for an unknown frame,
    /// [`Tf2Error::Connectivity`] if the frames are not connected, and
    /// [`Tf2Error::Extrapolation`] if `when` is outside the stored history.
    pub fn lookup_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
        when: LookupTime,
    ) -> Result<TransformStamped, Tf2Error> {
        let t = ffi::Tf2Time::from(when);
        let ffi_tf = call_out(|out| {
            self.wrapper()
                .lookup_transform(target_frame, source_frame, &t, out)
        })?;

        Ok(TransformStamped::from_ffi(ffi_tf))
    }

    /// Transforms `msg` from its own frame into `target_frame`: looks up the
    /// transform at the time given by `time`, then applies it with
    /// [`Transformable::apply_transform`].
    ///
    /// ```
    /// # use std::time::Duration;
    /// # use ros_env::{geometry_msgs::msg::{Point, PointStamped}, std_msgs::msg::Header};
    /// # use tf2_rs::{BufferCore, TimeSpec, TransformStamped};
    /// # let mut buffer = BufferCore::new(Duration::from_secs(10));
    /// # buffer.set_transform(&TransformStamped {
    /// #     stamp_sec: 0, stamp_nanosec: 0,
    /// #     parent_frame: "map".into(), child_frame: "lidar".into(),
    /// #     translation: [1.0, 0.0, 0.0], rotation: [0.0, 0.0, 0.0, 1.0],
    /// # }, "doc", true)?;
    /// let msg = PointStamped {
    ///     header: Header { frame_id: "lidar".into(), ..Default::default() },
    ///     point: Point { x: 1.0, y: 2.0, z: 3.0 },
    /// };
    /// let out = buffer.transform(&msg, "map", TimeSpec::Latest)?;
    /// assert_eq!(out.header.frame_id, "map");
    /// assert_eq!(out.point.x, 2.0);
    /// # Ok::<(), tf2_rs::Tf2Error>(())
    /// ```
    pub fn transform<T: Transformable + HasHeader>(
        &self,
        msg: &T,
        target_frame: &str,
        time: TimeSpec,
    ) -> Result<T, Tf2Error> {
        let when = time.resolve(msg);
        let tf = self.lookup_transform(target_frame, msg.frame_id(), when)?;
        msg.apply_transform(&tf)
    }

    /// Adds every transform of a `/tf` or `/tf_static` message, as
    /// [`TransformListener`](crate::TransformListener) does. Transforms that
    /// fail are passed to `on_err` and the others are still added.
    pub fn ingest_tf_message(
        &mut self,
        msg: tf2_msgs::msg::TFMessage,
        authority: &str,
        is_static: bool,
        mut on_err: impl FnMut(Tf2Error),
    ) {
        for t in msg.transforms {
            let tf = (&t).into();
            if let Err(e) = self.set_transform(&tf, authority, is_static) {
                on_err(e);
            }
        }
    }
}

/// The result of [`BufferCore::check_transform`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TransformAvailability {
    /// The transform can be looked up.
    Available,
    /// The transform cannot be looked up.
    Unavailable {
        /// tf2's explanation, if it gave one.
        diagnostic: Option<String>,
    },
}

impl TransformAvailability {
    /// Whether this is [`Available`](Self::Available).
    pub fn is_available(&self) -> bool {
        matches!(self, Self::Available)
    }

    /// tf2's explanation when the transform is unavailable.
    pub fn diagnostic(&self) -> Option<&str> {
        match self {
            Self::Available => None,
            Self::Unavailable { diagnostic } => diagnostic.as_deref(),
        }
    }
}
