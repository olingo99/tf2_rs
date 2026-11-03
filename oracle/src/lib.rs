//! C++ reference ("oracle") for the tf2_rs `Transformable` impls. Test-only and
//! not published.
//!
//! Each function runs the upstream `tf2::doTransform` specialisation for one
//! message type, so the parity tests in `tests/` can check the Rust impls against
//! tf2.
//!
//! Whole messages cross the bridge, in the shared structs below, which have `From`
//! conversions here and matching ones in `oracle.cpp`. Every output field, headers
//! and frame ids included, comes from the C++ side, so the tests can compare all of
//! what a C++ user gets.

use ros_env::builtin_interfaces;
use ros_env::geometry_msgs::msg::{
    Point, Point32, PointStamped, Polygon, PolygonStamped, Pose, PoseStamped, PoseWithCovariance,
    PoseWithCovarianceStamped, Quaternion, QuaternionStamped, Transform,
    TransformStamped as TransformStampedMsg, Twist, Vector3, Vector3Stamped, VelocityStamped,
    Wrench, WrenchStamped,
};
use ros_env::sensor_msgs::msg::{Imu, MagneticField, PointCloud2, PointField};
use ros_env::std_msgs::msg::Header;
use tf2_rs::TransformStamped;

#[cxx::bridge(namespace = "tf2_rs_oracle")]
pub mod ffi {
    #[derive(Default)]
    struct HeaderData {
        sec: i32,
        nanosec: u32,
        frame_id: String,
    }

    #[derive(Default)]
    struct Vec3Data {
        x: f64,
        y: f64,
        z: f64,
    }

    #[derive(Default)]
    struct QuatData {
        x: f64,
        y: f64,
        z: f64,
        w: f64,
    }

    struct PoseData {
        position: Vec3Data,
        orientation: QuatData,
    }

    struct PoseWithCovarianceData {
        pose: PoseData,
        covariance: [f64; 36],
    }

    struct Vector3StampedData {
        header: HeaderData,
        vector: Vec3Data,
    }

    struct PoseWithCovarianceStampedData {
        header: HeaderData,
        pose: PoseWithCovarianceData,
    }

    /// geometry_msgs/msg/TransformStamped.
    struct TfData {
        header: HeaderData,
        child_frame_id: String,
        translation: Vec3Data,
        rotation: QuatData,
    }

    #[derive(Default)]
    struct ImuData {
        header: HeaderData,
        orientation: QuatData,
        orientation_covariance: [f64; 9],
        angular_velocity: Vec3Data,
        angular_velocity_covariance: [f64; 9],
        linear_acceleration: Vec3Data,
        linear_acceleration_covariance: [f64; 9],
    }

    struct PointStampedData {
        header: HeaderData,
        point: Vec3Data,
    }

    struct Point32Data {
        x: f32,
        y: f32,
        z: f32,
    }

    struct PolygonData {
        points: Vec<Point32Data>,
    }

    struct PolygonStampedData {
        header: HeaderData,
        polygon: PolygonData,
    }

    struct QuaternionStampedData {
        header: HeaderData,
        quaternion: QuatData,
    }

    struct PoseStampedData {
        header: HeaderData,
        pose: PoseData,
    }

    struct TransformMsgData {
        translation: Vec3Data,
        rotation: QuatData,
    }

    struct WrenchData {
        force: Vec3Data,
        torque: Vec3Data,
    }

    struct WrenchStampedData {
        header: HeaderData,
        wrench: WrenchData,
    }

    struct TwistData {
        linear: Vec3Data,
        angular: Vec3Data,
    }

    struct VelocityStampedData {
        header: HeaderData,
        body_frame_id: String,
        reference_frame_id: String,
        velocity: TwistData,
    }

    #[derive(Default)]
    struct MagneticFieldData {
        header: HeaderData,
        magnetic_field: Vec3Data,
        magnetic_field_covariance: [f64; 9],
    }

    /// sensor_msgs/msg/PointField.
    struct PointFieldData {
        name: String,
        offset: u32,
        datatype: u8,
        count: u32,
    }

    /// sensor_msgs/msg/PointCloud2.
    struct CloudData {
        header: HeaderData,
        height: u32,
        width: u32,
        fields: Vec<PointFieldData>,
        is_bigendian: bool,
        point_step: u32,
        row_step: u32,
        data: Vec<u8>,
        is_dense: bool,
    }

    unsafe extern "C++" {
        include!("tf2_oracle.h");

        fn do_transform_point(p: &Vec3Data, tf: &TfData) -> Vec3Data;
        fn do_transform_point_stamped(p: &PointStampedData, tf: &TfData) -> PointStampedData;
        fn do_transform_point32(p: &Point32Data, tf: &TfData) -> Point32Data;
        fn do_transform_polygon(p: &PolygonData, tf: &TfData) -> PolygonData;
        fn do_transform_polygon_stamped(p: &PolygonStampedData, tf: &TfData) -> PolygonStampedData;
        fn do_transform_vector3(v: &Vec3Data, tf: &TfData) -> Vec3Data;
        fn do_transform_vector3_stamped(v: &Vector3StampedData, tf: &TfData) -> Vector3StampedData;
        fn do_transform_quaternion(q: &QuatData, tf: &TfData) -> QuatData;
        fn do_transform_quaternion_stamped(
            q: &QuaternionStampedData,
            tf: &TfData,
        ) -> QuaternionStampedData;
        fn do_transform_pose(pose: &PoseData, tf: &TfData) -> PoseData;
        fn do_transform_pose_stamped(pose: &PoseStampedData, tf: &TfData) -> PoseStampedData;
        fn do_transform_pose_with_covariance(
            pose: &PoseWithCovarianceData,
            tf: &TfData,
        ) -> PoseWithCovarianceData;
        fn do_transform_pose_with_covariance_stamped(
            pose: &PoseWithCovarianceStampedData,
            tf: &TfData,
        ) -> PoseWithCovarianceStampedData;
        fn do_transform_transform(t: &TransformMsgData, tf: &TfData) -> TransformMsgData;
        fn do_transform_transform_stamped(t: &TfData, tf: &TfData) -> TfData;
        fn do_transform_wrench(w: &WrenchData, tf: &TfData) -> WrenchData;
        fn do_transform_wrench_stamped(w: &WrenchStampedData, tf: &TfData) -> WrenchStampedData;
        fn do_transform_velocity_stamped(
            v: &VelocityStampedData,
            tf: &TfData,
        ) -> VelocityStampedData;
        /// Returns false where tf2_sensor_msgs has no Imu transform (Humble).
        fn do_transform_imu(imu: &ImuData, tf: &TfData, out: &mut ImuData) -> bool;
        /// Returns false where tf2_sensor_msgs has no MagneticField transform (Humble).
        fn do_transform_magnetic_field(
            mag: &MagneticFieldData,
            tf: &TfData,
            out: &mut MagneticFieldData,
        ) -> bool;

        /// `cloud.data` must not be empty: upstream is undefined there.
        fn do_transform_pointcloud2(cloud: &CloudData, tf: &TfData) -> Result<CloudData>;
    }
}

impl From<&Header> for ffi::HeaderData {
    fn from(h: &Header) -> Self {
        Self {
            sec: h.stamp.sec,
            nanosec: h.stamp.nanosec,
            frame_id: h.frame_id.clone(),
        }
    }
}

impl From<ffi::HeaderData> for Header {
    fn from(h: ffi::HeaderData) -> Self {
        Self {
            stamp: builtin_interfaces::msg::Time {
                sec: h.sec,
                nanosec: h.nanosec,
            },
            frame_id: h.frame_id,
        }
    }
}

impl From<&Point> for ffi::Vec3Data {
    fn from(p: &Point) -> Self {
        Self {
            x: p.x,
            y: p.y,
            z: p.z,
        }
    }
}

impl From<ffi::Vec3Data> for Point {
    fn from(v: ffi::Vec3Data) -> Self {
        Self {
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }
}

impl From<&Vector3> for ffi::Vec3Data {
    fn from(v: &Vector3) -> Self {
        Self {
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }
}

impl From<ffi::Vec3Data> for Vector3 {
    fn from(v: ffi::Vec3Data) -> Self {
        Self {
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }
}

impl From<&Quaternion> for ffi::QuatData {
    fn from(q: &Quaternion) -> Self {
        Self {
            x: q.x,
            y: q.y,
            z: q.z,
            w: q.w,
        }
    }
}

impl From<ffi::QuatData> for Quaternion {
    fn from(q: ffi::QuatData) -> Self {
        Self {
            x: q.x,
            y: q.y,
            z: q.z,
            w: q.w,
        }
    }
}

impl From<&Pose> for ffi::PoseData {
    fn from(p: &Pose) -> Self {
        Self {
            position: (&p.position).into(),
            orientation: (&p.orientation).into(),
        }
    }
}

impl From<ffi::PoseData> for Pose {
    fn from(p: ffi::PoseData) -> Self {
        Self {
            position: p.position.into(),
            orientation: p.orientation.into(),
        }
    }
}

impl From<&PoseWithCovariance> for ffi::PoseWithCovarianceData {
    fn from(p: &PoseWithCovariance) -> Self {
        Self {
            pose: (&p.pose).into(),
            covariance: p.covariance,
        }
    }
}

impl From<ffi::PoseWithCovarianceData> for PoseWithCovariance {
    fn from(p: ffi::PoseWithCovarianceData) -> Self {
        Self {
            pose: p.pose.into(),
            covariance: p.covariance,
        }
    }
}

impl From<&Vector3Stamped> for ffi::Vector3StampedData {
    fn from(v: &Vector3Stamped) -> Self {
        Self {
            header: (&v.header).into(),
            vector: (&v.vector).into(),
        }
    }
}

impl From<ffi::Vector3StampedData> for Vector3Stamped {
    fn from(v: ffi::Vector3StampedData) -> Self {
        Self {
            header: v.header.into(),
            vector: v.vector.into(),
        }
    }
}

impl From<&PoseWithCovarianceStamped> for ffi::PoseWithCovarianceStampedData {
    fn from(p: &PoseWithCovarianceStamped) -> Self {
        Self {
            header: (&p.header).into(),
            pose: (&p.pose).into(),
        }
    }
}

impl From<ffi::PoseWithCovarianceStampedData> for PoseWithCovarianceStamped {
    fn from(p: ffi::PoseWithCovarianceStampedData) -> Self {
        Self {
            header: p.header.into(),
            pose: p.pose.into(),
        }
    }
}

impl From<&TransformStamped> for ffi::TfData {
    fn from(tf: &TransformStamped) -> Self {
        let [x, y, z] = tf.translation;
        let [qx, qy, qz, qw] = tf.rotation;
        Self {
            header: ffi::HeaderData {
                sec: tf.stamp_sec,
                nanosec: tf.stamp_nanosec,
                frame_id: tf.parent_frame.clone(),
            },
            child_frame_id: tf.child_frame.clone(),
            translation: ffi::Vec3Data { x, y, z },
            rotation: ffi::QuatData {
                x: qx,
                y: qy,
                z: qz,
                w: qw,
            },
        }
    }
}

impl From<&Imu> for ffi::ImuData {
    fn from(m: &Imu) -> Self {
        Self {
            header: (&m.header).into(),
            orientation: (&m.orientation).into(),
            orientation_covariance: m.orientation_covariance,
            angular_velocity: (&m.angular_velocity).into(),
            angular_velocity_covariance: m.angular_velocity_covariance,
            linear_acceleration: (&m.linear_acceleration).into(),
            linear_acceleration_covariance: m.linear_acceleration_covariance,
        }
    }
}

impl From<ffi::ImuData> for Imu {
    fn from(m: ffi::ImuData) -> Self {
        Self {
            header: m.header.into(),
            orientation: m.orientation.into(),
            orientation_covariance: m.orientation_covariance,
            angular_velocity: m.angular_velocity.into(),
            angular_velocity_covariance: m.angular_velocity_covariance,
            linear_acceleration: m.linear_acceleration.into(),
            linear_acceleration_covariance: m.linear_acceleration_covariance,
        }
    }
}

impl From<&MagneticField> for ffi::MagneticFieldData {
    fn from(m: &MagneticField) -> Self {
        Self {
            header: (&m.header).into(),
            magnetic_field: (&m.magnetic_field).into(),
            magnetic_field_covariance: m.magnetic_field_covariance,
        }
    }
}

impl From<ffi::MagneticFieldData> for MagneticField {
    fn from(m: ffi::MagneticFieldData) -> Self {
        Self {
            header: m.header.into(),
            magnetic_field: m.magnetic_field.into(),
            magnetic_field_covariance: m.magnetic_field_covariance,
        }
    }
}

impl From<&PointStamped> for ffi::PointStampedData {
    fn from(p: &PointStamped) -> Self {
        Self {
            header: (&p.header).into(),
            point: (&p.point).into(),
        }
    }
}

impl From<ffi::PointStampedData> for PointStamped {
    fn from(p: ffi::PointStampedData) -> Self {
        Self {
            header: p.header.into(),
            point: p.point.into(),
        }
    }
}

impl From<&Point32> for ffi::Point32Data {
    fn from(p: &Point32) -> Self {
        Self {
            x: p.x,
            y: p.y,
            z: p.z,
        }
    }
}

impl From<ffi::Point32Data> for Point32 {
    fn from(p: ffi::Point32Data) -> Self {
        Self {
            x: p.x,
            y: p.y,
            z: p.z,
        }
    }
}

impl From<&Polygon> for ffi::PolygonData {
    fn from(p: &Polygon) -> Self {
        Self {
            points: p.points.iter().map(Into::into).collect(),
        }
    }
}

impl From<ffi::PolygonData> for Polygon {
    fn from(p: ffi::PolygonData) -> Self {
        Self {
            points: p.points.into_iter().map(Into::into).collect(),
        }
    }
}

impl From<&PolygonStamped> for ffi::PolygonStampedData {
    fn from(p: &PolygonStamped) -> Self {
        Self {
            header: (&p.header).into(),
            polygon: (&p.polygon).into(),
        }
    }
}

impl From<ffi::PolygonStampedData> for PolygonStamped {
    fn from(p: ffi::PolygonStampedData) -> Self {
        Self {
            header: p.header.into(),
            polygon: p.polygon.into(),
        }
    }
}

impl From<&QuaternionStamped> for ffi::QuaternionStampedData {
    fn from(q: &QuaternionStamped) -> Self {
        Self {
            header: (&q.header).into(),
            quaternion: (&q.quaternion).into(),
        }
    }
}

impl From<ffi::QuaternionStampedData> for QuaternionStamped {
    fn from(q: ffi::QuaternionStampedData) -> Self {
        Self {
            header: q.header.into(),
            quaternion: q.quaternion.into(),
        }
    }
}

impl From<&PoseStamped> for ffi::PoseStampedData {
    fn from(p: &PoseStamped) -> Self {
        Self {
            header: (&p.header).into(),
            pose: (&p.pose).into(),
        }
    }
}

impl From<ffi::PoseStampedData> for PoseStamped {
    fn from(p: ffi::PoseStampedData) -> Self {
        Self {
            header: p.header.into(),
            pose: p.pose.into(),
        }
    }
}

impl From<&Transform> for ffi::TransformMsgData {
    fn from(t: &Transform) -> Self {
        Self {
            translation: (&t.translation).into(),
            rotation: (&t.rotation).into(),
        }
    }
}

impl From<ffi::TransformMsgData> for Transform {
    fn from(t: ffi::TransformMsgData) -> Self {
        Self {
            translation: t.translation.into(),
            rotation: t.rotation.into(),
        }
    }
}

impl From<&TransformStampedMsg> for ffi::TfData {
    fn from(t: &TransformStampedMsg) -> Self {
        Self {
            header: (&t.header).into(),
            child_frame_id: t.child_frame_id.clone(),
            translation: (&t.transform.translation).into(),
            rotation: (&t.transform.rotation).into(),
        }
    }
}

impl From<ffi::TfData> for TransformStampedMsg {
    fn from(t: ffi::TfData) -> Self {
        Self {
            header: t.header.into(),
            child_frame_id: t.child_frame_id,
            transform: Transform {
                translation: t.translation.into(),
                rotation: t.rotation.into(),
            },
        }
    }
}

impl From<&Wrench> for ffi::WrenchData {
    fn from(w: &Wrench) -> Self {
        Self {
            force: (&w.force).into(),
            torque: (&w.torque).into(),
        }
    }
}

impl From<ffi::WrenchData> for Wrench {
    fn from(w: ffi::WrenchData) -> Self {
        Self {
            force: w.force.into(),
            torque: w.torque.into(),
        }
    }
}

impl From<&WrenchStamped> for ffi::WrenchStampedData {
    fn from(w: &WrenchStamped) -> Self {
        Self {
            header: (&w.header).into(),
            wrench: (&w.wrench).into(),
        }
    }
}

impl From<ffi::WrenchStampedData> for WrenchStamped {
    fn from(w: ffi::WrenchStampedData) -> Self {
        Self {
            header: w.header.into(),
            wrench: w.wrench.into(),
        }
    }
}

impl From<&VelocityStamped> for ffi::VelocityStampedData {
    fn from(v: &VelocityStamped) -> Self {
        Self {
            header: (&v.header).into(),
            body_frame_id: v.body_frame_id.clone(),
            reference_frame_id: v.reference_frame_id.clone(),
            velocity: ffi::TwistData {
                linear: (&v.velocity.linear).into(),
                angular: (&v.velocity.angular).into(),
            },
        }
    }
}

impl From<ffi::VelocityStampedData> for VelocityStamped {
    fn from(v: ffi::VelocityStampedData) -> Self {
        Self {
            header: v.header.into(),
            body_frame_id: v.body_frame_id,
            reference_frame_id: v.reference_frame_id,
            velocity: Twist {
                linear: v.velocity.linear.into(),
                angular: v.velocity.angular.into(),
            },
        }
    }
}

impl From<&PointCloud2> for ffi::CloudData {
    fn from(c: &PointCloud2) -> Self {
        Self {
            header: (&c.header).into(),
            height: c.height,
            width: c.width,
            fields: c
                .fields
                .iter()
                .map(|f| ffi::PointFieldData {
                    name: f.name.clone(),
                    offset: f.offset,
                    datatype: f.datatype,
                    count: f.count,
                })
                .collect(),
            is_bigendian: c.is_bigendian,
            point_step: c.point_step,
            row_step: c.row_step,
            data: c.data.clone(),
            is_dense: c.is_dense,
        }
    }
}

impl From<ffi::CloudData> for PointCloud2 {
    fn from(c: ffi::CloudData) -> Self {
        Self {
            header: c.header.into(),
            height: c.height,
            width: c.width,
            fields: c
                .fields
                .into_iter()
                .map(|f| PointField {
                    name: f.name,
                    offset: f.offset,
                    datatype: f.datatype,
                    count: f.count,
                })
                .collect(),
            is_bigendian: c.is_bigendian,
            point_step: c.point_step,
            row_step: c.row_step,
            data: c.data,
            is_dense: c.is_dense,
        }
    }
}

pub fn point(p: &Point, tf: &TransformStamped) -> Point {
    ffi::do_transform_point(&p.into(), &tf.into()).into()
}

pub fn point_stamped(p: &PointStamped, tf: &TransformStamped) -> PointStamped {
    ffi::do_transform_point_stamped(&p.into(), &tf.into()).into()
}

pub fn point32(p: &Point32, tf: &TransformStamped) -> Point32 {
    ffi::do_transform_point32(&p.into(), &tf.into()).into()
}

pub fn polygon(p: &Polygon, tf: &TransformStamped) -> Polygon {
    ffi::do_transform_polygon(&p.into(), &tf.into()).into()
}

pub fn polygon_stamped(p: &PolygonStamped, tf: &TransformStamped) -> PolygonStamped {
    ffi::do_transform_polygon_stamped(&p.into(), &tf.into()).into()
}

pub fn vector3(v: &Vector3, tf: &TransformStamped) -> Vector3 {
    ffi::do_transform_vector3(&v.into(), &tf.into()).into()
}

pub fn quaternion(q: &Quaternion, tf: &TransformStamped) -> Quaternion {
    ffi::do_transform_quaternion(&q.into(), &tf.into()).into()
}

pub fn quaternion_stamped(q: &QuaternionStamped, tf: &TransformStamped) -> QuaternionStamped {
    ffi::do_transform_quaternion_stamped(&q.into(), &tf.into()).into()
}

pub fn pose_stamped(p: &PoseStamped, tf: &TransformStamped) -> PoseStamped {
    ffi::do_transform_pose_stamped(&p.into(), &tf.into()).into()
}

pub fn pose_with_covariance(p: &PoseWithCovariance, tf: &TransformStamped) -> PoseWithCovariance {
    ffi::do_transform_pose_with_covariance(&p.into(), &tf.into()).into()
}

pub fn transform(t: &Transform, tf: &TransformStamped) -> Transform {
    ffi::do_transform_transform(&t.into(), &tf.into()).into()
}

pub fn transform_stamped(t: &TransformStampedMsg, tf: &TransformStamped) -> TransformStampedMsg {
    ffi::do_transform_transform_stamped(&t.into(), &tf.into()).into()
}

pub fn wrench(w: &Wrench, tf: &TransformStamped) -> Wrench {
    ffi::do_transform_wrench(&w.into(), &tf.into()).into()
}

pub fn wrench_stamped(w: &WrenchStamped, tf: &TransformStamped) -> WrenchStamped {
    ffi::do_transform_wrench_stamped(&w.into(), &tf.into()).into()
}

pub fn velocity_stamped(v: &VelocityStamped, tf: &TransformStamped) -> VelocityStamped {
    ffi::do_transform_velocity_stamped(&v.into(), &tf.into()).into()
}

/// `None` where the distro's tf2_sensor_msgs has no MagneticField transform (Humble).
pub fn magnetic_field(m: &MagneticField, tf: &TransformStamped) -> Option<MagneticField> {
    let mut out = ffi::MagneticFieldData::default();
    ffi::do_transform_magnetic_field(&m.into(), &tf.into(), &mut out).then(|| out.into())
}

pub fn vector3_stamped(v: &Vector3Stamped, tf: &TransformStamped) -> Vector3Stamped {
    ffi::do_transform_vector3_stamped(&v.into(), &tf.into()).into()
}

pub fn pose(p: &Pose, tf: &TransformStamped) -> Pose {
    ffi::do_transform_pose(&p.into(), &tf.into()).into()
}

pub fn pose_with_covariance_stamped(
    p: &PoseWithCovarianceStamped,
    tf: &TransformStamped,
) -> PoseWithCovarianceStamped {
    ffi::do_transform_pose_with_covariance_stamped(&p.into(), &tf.into()).into()
}

/// `None` where the distro's tf2_sensor_msgs has no Imu transform (Humble).
pub fn imu(imu: &Imu, tf: &TransformStamped) -> Option<Imu> {
    let mut out = ffi::ImuData::default();
    ffi::do_transform_imu(&imu.into(), &tf.into(), &mut out).then(|| out.into())
}

/// Upstream tf2::doTransform with separate input and output messages. An error for
/// a layout upstream rejects, and for a cloud without data, where it is undefined.
pub fn pointcloud2(cloud: &PointCloud2, tf: &TransformStamped) -> Result<PointCloud2, String> {
    ffi::do_transform_pointcloud2(&cloud.into(), &tf.into())
        .map(Into::into)
        .map_err(|e| e.what().to_string())
}
