use cxx::SharedPtr;

#[cxx::bridge]
pub mod ffi {

    #[derive(Clone, Debug, Default)]
    #[repr(i32)]
    pub enum Tf2Errc {
        #[default]
        Ok = 0,
        Lookup = 1,
        Connectivity = 2,
        Extrapolation = 3,
        InvalidArgument = 4,
        Other = 100,
    }

    #[derive(Clone, Debug, Default)]
    struct Tf2Status {
        code: Tf2Errc,
        message: String,
    }

    #[derive(Clone, Debug, Default)]
    struct Tf2Time {
        sec: i32,
        nanosec: u32,
    }

    #[derive(Clone, Debug, Default)]
    struct Tf2Header {
        stamp: Tf2Time,
        frame_id: String,
    }

    #[derive(Clone, Debug, Default)]
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

    #[derive(Clone, Debug, Default)]
    struct Tf2PointStamped {
        header: Tf2Header,
        x: f64,
        y: f64,
        z: f64,
    }

    #[derive(Clone, Debug, Default)]
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
    #[derive(Clone, Debug, Default)]
    struct Tf2PointField {
        name: String,
        offset: u32,
        datatype: u8,
        count: u32,
    }

    // sensor_msgs/msg/PointCloud2
    #[derive(Clone, Debug, Default)]
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

        fn new_buffer_core(cache_time_ns: u64) -> SharedPtr<BufferCoreWrapper>;

        fn clear(self: &BufferCoreWrapper);

        fn lookup_transform(
            self: &BufferCoreWrapper,
            target_frame: &str,
            source_frame: &str,
            time: &Tf2Time,
            out: &mut Tf2TransformStamped,
        ) -> Tf2Status;

        fn can_transform(
            self: &BufferCoreWrapper,
            target_frame: &str,
            source_frame: &str,
            time: &Tf2Time,
            out_ok: &mut bool,
        ) -> Tf2Status;

        fn set_transform(
            self: &BufferCoreWrapper,
            tf: &Tf2TransformStamped,
            authority: &str,
            is_static: bool,
            out_ok: &mut bool,
        ) -> Tf2Status;

        fn do_transform_pointcloud2(
            input: &Tf2PointCloud2,
            tf: &Tf2TransformStamped,
            out: &mut Tf2PointCloud2,
        ) -> Tf2Status;
    }


}
