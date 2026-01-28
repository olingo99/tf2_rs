use crate::ffi::ffi;
use crate::transform::HasHeader;

#[derive(Clone, Copy, Debug)]
pub enum TimeSpec {
    Latest,
    Stamp { sec: i32, nanosec: u32 },
    FromMsg,
}

impl TimeSpec {
    pub fn resolve<T: HasHeader + ?Sized>(self, msg: &T) -> LookupTime {
        match self {
            TimeSpec::Latest => LookupTime::Latest,
            TimeSpec::Stamp { sec, nanosec } => LookupTime::Time { sec, nanosec },
            TimeSpec::FromMsg => {
                let (sec, nanosec) = msg.stamp();
                LookupTime::Time { sec, nanosec }
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LookupTime {
    Latest,
    Time { sec: i32, nanosec: u32 },
}

impl LookupTime {
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
