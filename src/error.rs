use thiserror::Error;
use crate::ffi::ffi::{Tf2Status,Tf2Errc};

#[derive(Error, Debug)]
pub enum Tf2Error {
    #[error("tf2 lookup error: {0}")]
    Lookup(String),
    #[error("tf2 connectivity error: {0}")]
    Connectivity(String),
    #[error("tf2 extrapolation error: {0}")]
    Extrapolation(String),
    #[error("tf2 invalid argument: {0}")]
    InvalidArgument(String),
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