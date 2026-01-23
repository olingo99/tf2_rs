use crate::error::{Tf2Error, check_status};
use crate::ffi::ffi::{self, BufferCoreWrapper};
use crate::ffi_utils::{call_bool, call_out, call_bool_with_diag};
use crate::time::{LookupTime, TimeSpec};
use crate::transform::Transformable;
use crate::transform_stamped::TransformStamped;
use crate::ffi::ffi::{Tf2Errc, Tf2Status};

#[derive(Clone)]
pub struct BufferCore {
    inner: cxx::SharedPtr<BufferCoreWrapper>,
}

impl BufferCore {
    pub fn new(cache_time_ns: u64) -> Self {
        Self {
            inner: ffi::new_buffer_core(cache_time_ns),
        }
    }

    fn wrapper(&self) -> &BufferCoreWrapper {
        self.inner.as_ref().expect("BufferCoreWrapper is null")
    }

    pub fn clear(&self) {
        self.wrapper().clear();
    }

    pub fn set_transform(
        &self,
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
                "buffer_set_transform rejected transform".to_string(),
            ))
        }
    }

    pub fn can_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
        when: LookupTime,
    ) -> Result<bool, Tf2Error> {
        let t = when.to_ffi();
        call_bool(|out_ok| {
            self.wrapper()
                .can_transform(target_frame, source_frame, &t, out_ok)
        })
    }

    pub fn can_transform_with_diagnostic(
        &self,
        target_frame: &str,
        source_frame: &str,
        when: LookupTime,
    ) -> Result<(bool, Option<String>), Tf2Error> {
        let t = when.to_ffi();
        call_bool_with_diag(|out_ok| self.wrapper().can_transform(target_frame, source_frame, &t, out_ok))
    }

    pub fn lookup_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
        when: LookupTime,
    ) -> Result<TransformStamped, Tf2Error> {
        let t = when.to_ffi();
        let ffi_tf = call_out(|out| {
            self.wrapper()
                .lookup_transform(target_frame, source_frame, &t, out)
        })?;

        Ok(TransformStamped::from_ffi(ffi_tf))
    }

    pub fn transform<T: Transformable>(
        &self,
        msg: &T,
        target_frame: &str,
        time: TimeSpec,
    ) -> Result<T, Tf2Error> {
        let when = time.resolve(msg);
        let tf = self.lookup_transform(target_frame, msg.frame_id(), when)?;
        msg.apply_transform(&tf)
    }

    pub fn ingest_tf_message(
        &self,
        msg: tf2_msgs::msg::TFMessage,
        authority: &str,
        is_static: bool,
        mut on_err: impl FnMut(Tf2Error),
    ) {
        for t in msg.transforms {
            let tf = crate::transform::geometry_msgs::convert_transform_stamped(&t);
            if let Err(e) = self.set_transform(&tf, authority, is_static) {
                on_err(e);
            }
        }
    }
}

unsafe impl Send for BufferCore {}
unsafe impl Sync for BufferCore {}
