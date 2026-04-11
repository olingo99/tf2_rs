use crate::error::Tf2Error;
use crate::ffi::ffi::{self, BufferCoreWrapper};
use crate::ffi_utils::{call_bool, call_out};
use crate::time::{LookupTime, TimeSpec};
use crate::transform::Transformable;
use crate::transform_stamped::TransformStamped;


unsafe impl Send for BufferCoreWrapper {}
unsafe impl Sync for BufferCoreWrapper {}

#[derive(Clone)]
pub struct BufferCore {
    inner: cxx::SharedPtr<BufferCoreWrapper>,
}

impl BufferCore {
    pub fn new(cache_time_ns: std::time::Duration) -> Self {
        Self {
            inner: ffi::new_buffer_core(cache_time_ns.as_nanos() as u64),
        }
    }

    fn wrapper(&self) -> &BufferCoreWrapper {
        self.inner.as_ref().expect("BufferCoreWrapper is null")
    }

    pub fn clear(&mut self) {
        self.wrapper().clear();
    }

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
                "buffer_set_transform rejected transform".to_string(),
            ))
        }
    }

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

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TransformAvailability {
    Available,
    Unavailable { diagnostic: Option<String> },
}

impl TransformAvailability {
    pub fn is_available(&self) -> bool {
        matches!(self, Self::Available)
    }

    pub fn diagnostic(&self) -> Option<&str> {
        match self {
            Self::Available => None,
            Self::Unavailable { diagnostic } => diagnostic.as_deref(),
        }
    }
}

