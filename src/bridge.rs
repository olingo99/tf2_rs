use std::pin::Pin;
use thiserror::Error;
use std::sync::{Arc, Mutex};

#[cxx::bridge]
mod ffi {
    #[derive(Clone, Debug)]
    struct Tf2Time {
        sec: i32,
        nanosec: u32,
    }

    #[derive(Clone, Debug)]
    struct Tf2Header {
        stamp: Tf2Time,
        frame_id: String,
    }

    #[derive(Clone, Debug)]
    struct Tf2TransformStamped {
        stamp: Tf2Time,
        parent_frame: String,
        child_frame: String,
        translation_x: f64,
        translation_y: f64,
        translation_z: f64,
        rotation_x: f64,
        rotation_y: f64,
        rotation_z: f64,
        rotation_w: f64,
    }

    #[derive(Clone, Debug)]
    struct Tf2PointStamped {
        header: Tf2Header,
        x: f64,
        y: f64,
        z: f64,
    }

    #[derive(Clone, Debug)]
    struct Tf2PoseStamped {
        header: Tf2Header,
        position_x: f64,
        position_y: f64,
        position_z: f64,
        orientation_x: f64,
        orientation_y: f64,
        orientation_z: f64,
        orientation_w: f64,
    }

    // sensor_msgs/msg/PointField
    #[derive(Clone, Debug)]
    struct Tf2PointField {
        name: String,
        offset: u32,
        datatype: u8,
        count: u32,
    }

    // sensor_msgs/msg/PointCloud2
    #[derive(Clone, Debug)]
    struct Tf2PointCloud2 {
        header: Tf2Header,
        height: u32,
        width: u32,
        fields: Vec<Tf2PointField>,
        is_bigendian: bool,
        point_step: u32,
        row_step: u32,
        data: Vec<u8>,
        is_dense: bool,
    }
    unsafe extern "C++" {
        include!("tf2_wrapper.h");

        type BufferCoreWrapper;

        fn new_buffer_core(cache_time_ns: u64) -> UniquePtr<BufferCoreWrapper>;

        fn clear(self: Pin<&mut BufferCoreWrapper>);

        fn set_transform(
            self: Pin<&mut BufferCoreWrapper>,
            tf: &Tf2TransformStamped,
            authority: &str,
            is_static: bool,
        ) -> Result<bool>;

        fn can_transform(
            self: &BufferCoreWrapper,
            target_frame: &str,
            source_frame: &str,
            time: &Tf2Time,
        ) -> Result<bool>;

        fn lookup_transform(
            self: &BufferCoreWrapper,
            target_frame: &str,
            source_frame: &str,
            time: &Tf2Time,
        ) -> Result<Tf2TransformStamped>;

        fn do_transform_point_stamped(
            input: &Tf2PointStamped,
            tf: &Tf2TransformStamped,
        ) -> Result<Tf2PointStamped>;

        fn do_transform_pose_stamped(
            input: &Tf2PoseStamped,
            tf: &Tf2TransformStamped,
        ) -> Result<Tf2PoseStamped>;

        fn do_transform_pointcloud2(
            input: &Tf2PointCloud2,
            tf: &Tf2TransformStamped,
        ) -> Result<Tf2PointCloud2>;
    }


}

unsafe impl Send for ffi::BufferCoreWrapper {}


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

fn map_cxx_err(e: cxx::Exception) -> Tf2Error {
    // We format C++ exceptions as: "[<type>]: <what>"
    let what = e.what().to_string();
    if let Some(end) = what.find(']') {
        let ty = &what[1..end];
        return match ty {
            "tf2::LookupException" => Tf2Error::Lookup(what),
            "tf2::ConnectivityException" => Tf2Error::Connectivity(what),
            "tf2::ExtrapolationException" => Tf2Error::Extrapolation(what),
            "tf2::InvalidArgumentException" => Tf2Error::InvalidArgument(what),
            _ => Tf2Error::Other(what),
        };
    }
    Tf2Error::Other(what)
}

#[derive(Clone, Debug)]
pub struct TransformStamped {
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
    pub parent_frame: String,
    pub child_frame: String,
    pub translation: [f64; 3],
    pub rotation: [f64; 4], // x,y,z,w
}

impl TransformStamped {
    fn to_ffi(&self) -> ffi::Tf2TransformStamped {
        ffi::Tf2TransformStamped {
            stamp: ffi::Tf2Time {
                sec: self.stamp_sec,
                nanosec: self.stamp_nanosec,
            },
            parent_frame: self.parent_frame.clone(),
            child_frame: self.child_frame.clone(),
            translation_x: self.translation[0],
            translation_y: self.translation[1],
            translation_z: self.translation[2],
            rotation_x: self.rotation[0],
            rotation_y: self.rotation[1],
            rotation_z: self.rotation[2],
            rotation_w: self.rotation[3],
        }
    }

    fn from_ffi(v: ffi::Tf2TransformStamped) -> Self {
        Self {
            stamp_sec: v.stamp.sec,
            stamp_nanosec: v.stamp.nanosec,
            parent_frame: v.parent_frame,
            child_frame: v.child_frame,
            translation: [v.translation_x, v.translation_y, v.translation_z],
            rotation: [v.rotation_x, v.rotation_y, v.rotation_z, v.rotation_w],
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum LookupTime {
    /// TF2 “time zero” = latest available transform.
    Latest,
    /// Query at an explicit (sec, nanosec).
    Time { sec: i32, nanosec: u32 },
}

impl LookupTime {
    fn to_ffi(self) -> ffi::Tf2Time {
        match self {
            LookupTime::Latest => ffi::Tf2Time { sec: 0, nanosec: 0 },
            LookupTime::Time { sec, nanosec } => ffi::Tf2Time { sec, nanosec },
        }
    }
}

#[derive(Clone)]
pub struct BufferCore {
    inner: Arc<Mutex<cxx::UniquePtr<ffi::BufferCoreWrapper>>>,
}


impl BufferCore {
    pub fn new(cache_time_ns: u64) -> Self {
        Self {
            inner: Arc::new(Mutex::new(ffi::new_buffer_core(cache_time_ns))),
        }
    }

    pub fn clear(&self) {
        let mut guard = self.inner.lock().unwrap();
        guard.pin_mut().clear();
    }

    pub fn set_transform(
        &self,
        tf: &TransformStamped,
        authority: &str,
        is_static: bool,
    ) -> Result<(), Tf2Error> {
        let mut guard = self.inner.lock().unwrap();
        guard
            .pin_mut()
            .set_transform(&tf.to_ffi(), authority, is_static)
            .map_err(map_cxx_err)
            .and_then(|ok| {
                if ok {
                    Ok(())
                } else {
                    Err(Tf2Error::InvalidArgument(
                        "buffer_set_transform rejected transform".to_string(),
                    ))
                }
            })
    }

    pub fn can_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
        when: LookupTime,
    ) -> Result<bool, Tf2Error> {
        let t = when.to_ffi();
        let guard = self.inner.lock().unwrap();
        let wrapper = guard.as_ref().expect("BufferCoreWrapper is null");
        wrapper
            .can_transform(target_frame, source_frame, &t)
            .map_err(map_cxx_err)
    }

    pub fn lookup_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
        when: LookupTime,
    ) -> Result<TransformStamped, Tf2Error> {
        let t = when.to_ffi();
        let guard = self.inner.lock().unwrap();
        let wrapper = guard.as_ref().expect("BufferCoreWrapper is null");
        let out = wrapper
            .lookup_transform(target_frame, source_frame, &t)
            .map_err(map_cxx_err)?;
        Ok(TransformStamped::from_ffi(out))
    }

    pub fn lookup_latest_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
    ) -> Result<TransformStamped, Tf2Error> {
        self.lookup_transform(target_frame, source_frame, LookupTime::Latest)
    }
}

fn header_to_ffi(h: &std_msgs::msg::Header) -> ffi::Tf2Header {
    ffi::Tf2Header {
        stamp: ffi::Tf2Time { sec: h.stamp.sec, nanosec: h.stamp.nanosec },
        frame_id: h.frame_id.clone(),
    }
}

fn header_from_ffi(h: ffi::Tf2Header) -> std_msgs::msg::Header {
    let mut out = std_msgs::msg::Header::default();
    out.stamp.sec = h.stamp.sec;
    out.stamp.nanosec = h.stamp.nanosec;
    out.frame_id = h.frame_id;
    out
}

pub fn transform_point_stamped_builtin(
    buffer: &BufferCore,
    input: &geometry_msgs::msg::PointStamped,
    target_frame: &str,
    when: LookupTime,
) -> Result<geometry_msgs::msg::PointStamped, Tf2Error> {
    // Typically you use the message stamp (unless you explicitly choose Latest).
    let tf = buffer.lookup_transform(target_frame, &input.header.frame_id, when)?;

    let ffi_in = ffi::Tf2PointStamped {
        header: header_to_ffi(&input.header),
        x: input.point.x,
        y: input.point.y,
        z: input.point.z,
    };

    let ffi_out = ffi::do_transform_point_stamped(&ffi_in, &tf.to_ffi()).map_err(map_cxx_err)?;

    let mut out = geometry_msgs::msg::PointStamped::default();
    out.header = header_from_ffi(ffi_out.header);
    out.point.x = ffi_out.x;
    out.point.y = ffi_out.y;
    out.point.z = ffi_out.z;
    Ok(out)
}

fn pc2_to_ffi(pc: &sensor_msgs::msg::PointCloud2) -> ffi::Tf2PointCloud2 {
    ffi::Tf2PointCloud2 {
        header: ffi::Tf2Header {
            stamp: ffi::Tf2Time { sec: pc.header.stamp.sec, nanosec: pc.header.stamp.nanosec },
            frame_id: pc.header.frame_id.clone(),
        },
        height: pc.height,
        width: pc.width,
        fields: pc.fields.iter().map(|f| ffi::Tf2PointField {
            name: f.name.clone(),
            offset: f.offset,
            datatype: f.datatype,
            count: f.count,
        }).collect(),
        is_bigendian: pc.is_bigendian,
        point_step: pc.point_step,
        row_step: pc.row_step,
        data: pc.data.clone(),
        is_dense: pc.is_dense,
    }
}

fn pc2_from_ffi(pc: ffi::Tf2PointCloud2) -> sensor_msgs::msg::PointCloud2 {
    let mut out = sensor_msgs::msg::PointCloud2::default();
    out.header.stamp.sec = pc.header.stamp.sec;
    out.header.stamp.nanosec = pc.header.stamp.nanosec;
    out.header.frame_id = pc.header.frame_id;

    out.height = pc.height;
    out.width = pc.width;
    out.fields = pc.fields.into_iter().map(|f| {
        let mut pf = sensor_msgs::msg::PointField::default();
        pf.name = f.name;
        pf.offset = f.offset;
        pf.datatype = f.datatype;
        pf.count = f.count;
        pf
    }).collect();

    out.is_bigendian = pc.is_bigendian;
    out.point_step = pc.point_step;
    out.row_step = pc.row_step;
    out.data = pc.data;
    out.is_dense = pc.is_dense;
    out
}

pub fn transform_pointcloud2(
    buffer: &BufferCore,
    cloud: &sensor_msgs::msg::PointCloud2,
    target_frame: &str,
) -> Result<sensor_msgs::msg::PointCloud2, Tf2Error> {
    // let time = LookupTime::Time { sec: (cloud.header.stamp.sec), nanosec: (cloud.header.stamp.nanosec) };
    let time = LookupTime::Time { sec: (0), nanosec: (0) };

    let tf = buffer.lookup_transform(target_frame, &cloud.header.frame_id, time)?;

    let ffi_in = pc2_to_ffi(cloud);
    let ffi_out = ffi::do_transform_pointcloud2(&ffi_in, &tf.to_ffi()).map_err(map_cxx_err)?;
    Ok(pc2_from_ffi(ffi_out))
}
