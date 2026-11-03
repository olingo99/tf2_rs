use crate::ffi::ffi::{Tf2Errc, Tf2Status};
use thiserror::Error;

/// An error from tf2 or from transforming a message. Each variant holds the
/// message, from tf2 where it comes from tf2.
#[derive(Error, Debug)]
pub enum Tf2Error {
    /// A frame is unknown (`tf2::LookupException`).
    #[error("tf2 lookup error: {0}")]
    Lookup(String),
    /// The frames are not connected (`tf2::ConnectivityException`).
    #[error("tf2 connectivity error: {0}")]
    Connectivity(String),
    /// The time is outside the stored history (`tf2::ExtrapolationException`).
    #[error("tf2 extrapolation error: {0}")]
    Extrapolation(String),
    /// An input was rejected: a transform by
    /// [`BufferCore::set_transform`](crate::BufferCore::set_transform), or a
    /// malformed message such as a `PointCloud2` whose fields do not fit its
    /// points.
    #[error("tf2 invalid argument: {0}")]
    InvalidArgument(String),
    /// Any other tf2 error.
    #[error("tf2 error: {0}")]
    Other(String),
}

pub fn check_status(st: Tf2Status) -> Result<(), Tf2Error> {
    match st.code {
        Tf2Errc::Ok => Ok(()),
        Tf2Errc::Lookup => Err(Tf2Error::Lookup(st.message)),
        Tf2Errc::Connectivity => Err(Tf2Error::Connectivity(st.message)),
        Tf2Errc::Extrapolation => Err(Tf2Error::Extrapolation(st.message)),
        Tf2Errc::InvalidArgument => Err(Tf2Error::InvalidArgument(st.message)),
        Tf2Errc::Other => Err(Tf2Error::Other(st.message)),
        _ => Err(Tf2Error::Other(st.message)),
    }
}
