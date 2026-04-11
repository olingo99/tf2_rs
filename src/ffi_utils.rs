use crate::error::{Tf2Error, check_status};
use crate::ffi::ffi;

pub fn call_out<T: Default>(f: impl FnOnce(&mut T) -> ffi::Tf2Status) -> Result<T, Tf2Error> {
    let mut out = T::default();
    let st = f(&mut out);
    check_status(st)?;
    Ok(out)
}

pub fn call_bool(f: impl FnOnce(&mut bool) -> ffi::Tf2Status) -> Result<bool, Tf2Error> {
    let mut out = false;
    let st = f(&mut out);
    check_status(st)?;
    Ok(out)
}