use crate::ffi::ffi;
use crate::transform::HasHeader;

/// Which time [`BufferCore::transform`](crate::BufferCore::transform) looks up
/// the transform at.
#[derive(Clone, Copy, Debug)]
pub enum TimeSpec {
    /// The latest time at which the transform is available.
    Latest,
    /// A given time. As in tf2, a zero stamp means [`Latest`](Self::Latest).
    Stamp {
        /// Seconds.
        sec: i32,
        /// Nanoseconds.
        nanosec: u32,
    },
    /// The stamp in the message's header. A zero stamp means
    /// [`Latest`](Self::Latest).
    FromMsg,
}

impl TimeSpec {
    /// The [`LookupTime`] this gives for `msg`.
    pub fn resolve<T: HasHeader + ?Sized>(self, msg: &T) -> LookupTime {
        match self {
            TimeSpec::Latest => LookupTime::Latest,
            TimeSpec::Stamp { sec, nanosec } => LookupTime::Time { sec, nanosec },
            TimeSpec::FromMsg => LookupTime::from_msg(msg),
        }
    }
}

/// The time at which [`BufferCore`](crate::BufferCore) looks up a transform.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LookupTime {
    /// The latest time at which the transform is available.
    Latest,
    /// A given time. As in tf2, a zero time means [`Latest`](Self::Latest).
    Time {
        /// Seconds.
        sec: i32,
        /// Nanoseconds.
        nanosec: u32,
    },
}

impl LookupTime {
    /// The stamp in `msg`'s header.
    pub fn from_msg<M: HasHeader + ?Sized>(msg: &M) -> Self {
        let (sec, nanosec) = msg.stamp();
        LookupTime::Time { sec, nanosec }
    }
}

impl From<LookupTime> for ffi::Tf2Time {
    fn from(v: LookupTime) -> Self {
        match v {
            LookupTime::Latest => Self { sec: 0, nanosec: 0 },
            LookupTime::Time { sec, nanosec } => Self { sec, nanosec },
        }
    }
}
