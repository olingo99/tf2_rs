use crate::ffi::ffi;
use crate::error::{Tf2Error, check_status};


pub fn call_out<T: Default>(
    f: impl FnOnce(&mut T) -> ffi::Tf2Status
) -> Result<T, Tf2Error> {
    let mut out = T::default();
    let st = f(&mut out);
    check_status(st)?;
    Ok(out)
}

pub fn call_bool(
    f: impl FnOnce(&mut bool) -> ffi::Tf2Status
) -> Result<bool, Tf2Error> {
    let mut out = false;
    let st = f(&mut out);
    check_status(st)?;
    Ok(out)
}

pub(crate) fn call_bool_with_diag(
    f: impl FnOnce(&mut bool) -> ffi::Tf2Status
) -> Result<(bool, Option<String>), Tf2Error> {
    let mut out = false;
    let st = f(&mut out);

    let diag = if matches!(st.code, ffi::Tf2Errc::Ok) && !st.message.is_empty() {
        Some(st.message.clone())
    } else {
        None
    };

    check_status(st)?;
    Ok((out, if out { None } else { diag }))
}