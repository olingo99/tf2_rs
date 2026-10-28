//! C++ reference ("oracle") for the tf2_rs `Transformable` impls. Test-only and
//! not published.
//!
//! Each function runs the upstream `tf2::doTransform` specialisation for one
//! message type, so the parity tests in `tests/parity.rs` can check the Rust
//! impls against tf2.
//!
//! Only numbers cross the bridge: a transform is `[tx, ty, tz, qx, qy, qz, qw]` and
//! a pose `[px, py, pz, qx, qy, qz, qw]`. Headers are therefore set on the Rust side,
//! the same way the C++ code sets them (stamp and parent frame of the transform).
//! PointCloud2 is the exception: its layout, fields and bytes cross as they are.

use ros_env::geometry_msgs::msg::{
    Point, Pose, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion, Vector3, Vector3Stamped,
};
use ros_env::sensor_msgs::msg::{Imu, PointCloud2};
use ros_env::{builtin_interfaces, std_msgs};
use tf2_rs::TransformStamped;

#[cxx::bridge(namespace = "tf2_rs_oracle")]
pub mod ffi {
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct ImuData {
        orientation: [f64; 4],
        orientation_covariance: [f64; 9],
        angular_velocity: [f64; 3],
        angular_velocity_covariance: [f64; 9],
        linear_acceleration: [f64; 3],
        linear_acceleration_covariance: [f64; 9],
    }

    /// sensor_msgs/msg/PointField, borrowing its name from the Rust message.
    struct PointField<'a> {
        name: &'a str,
        offset: u32,
        datatype: u8,
        count: u32,
    }

    /// sensor_msgs/msg/PointCloud2 without its header, fields and data.
    struct CloudLayout {
        height: u32,
        width: u32,
        is_bigendian: bool,
        point_step: u32,
        row_step: u32,
        is_dense: bool,
    }

    unsafe extern "C++" {
        include!("tf2_oracle.h");

        fn do_transform_point(p: &[f64; 3], tf: &[f64; 7], out: &mut [f64; 3]);
        fn do_transform_vector3_stamped(v: &[f64; 3], tf: &[f64; 7], out: &mut [f64; 3]);
        fn do_transform_pose(pose: &[f64; 7], tf: &[f64; 7], out: &mut [f64; 7]);
        fn do_transform_pose_with_covariance_stamped(
            pose: &[f64; 7],
            covariance: &[f64; 36],
            tf: &[f64; 7],
            out_pose: &mut [f64; 7],
            out_covariance: &mut [f64; 36],
        );
        /// Returns false where tf2_sensor_msgs has no Imu transform (Humble).
        fn do_transform_imu(imu: &ImuData, tf: &[f64; 7], out: &mut ImuData) -> bool;

        /// `out` must have the same length as `data`, and `data` must not be empty:
        /// upstream is undefined there.
        fn do_transform_pointcloud2(
            layout: &CloudLayout,
            fields: &[PointField],
            data: &[u8],
            tf: &[f64; 7],
            out: &mut [u8],
        ) -> Result<()>;

    }
}

/// As tf2_rs and tf2: the stamp of the transform and its parent frame.
fn header_from_transform(tf: &TransformStamped) -> std_msgs::msg::Header {
    std_msgs::msg::Header {
        stamp: builtin_interfaces::msg::Time {
            sec: tf.stamp_sec,
            nanosec: tf.stamp_nanosec,
        },
        frame_id: tf.parent_frame.clone(),
    }
}

pub fn tf_array(tf: &TransformStamped) -> [f64; 7] {
    let [tx, ty, tz] = tf.translation;
    let [qx, qy, qz, qw] = tf.rotation;
    [tx, ty, tz, qx, qy, qz, qw]
}

fn pose_array(p: &Pose) -> [f64; 7] {
    let (pos, o) = (&p.position, &p.orientation);
    [pos.x, pos.y, pos.z, o.x, o.y, o.z, o.w]
}

fn pose_from_array(a: [f64; 7]) -> Pose {
    Pose {
        position: Point {
            x: a[0],
            y: a[1],
            z: a[2],
        },
        orientation: Quaternion {
            x: a[3],
            y: a[4],
            z: a[5],
            w: a[6],
        },
    }
}

fn vector(v: [f64; 3]) -> Vector3 {
    Vector3 {
        x: v[0],
        y: v[1],
        z: v[2],
    }
}

pub fn point(p: &Point, tf: &TransformStamped) -> Point {
    let mut out = [0.0; 3];
    ffi::do_transform_point(&[p.x, p.y, p.z], &tf_array(tf), &mut out);
    Point {
        x: out[0],
        y: out[1],
        z: out[2],
    }
}

pub fn vector3_stamped(v: &Vector3Stamped, tf: &TransformStamped) -> Vector3Stamped {
    let mut out = [0.0; 3];
    let input = [v.vector.x, v.vector.y, v.vector.z];
    ffi::do_transform_vector3_stamped(&input, &tf_array(tf), &mut out);
    Vector3Stamped {
        header: header_from_transform(tf),
        vector: vector(out),
    }
}

pub fn pose(p: &Pose, tf: &TransformStamped) -> Pose {
    let mut out = [0.0; 7];
    ffi::do_transform_pose(&pose_array(p), &tf_array(tf), &mut out);
    pose_from_array(out)
}

pub fn pose_with_covariance_stamped(
    p: &PoseWithCovarianceStamped,
    tf: &TransformStamped,
) -> PoseWithCovarianceStamped {
    let (mut out_pose, mut out_covariance) = ([0.0; 7], [0.0; 36]);
    ffi::do_transform_pose_with_covariance_stamped(
        &pose_array(&p.pose.pose),
        &p.pose.covariance,
        &tf_array(tf),
        &mut out_pose,
        &mut out_covariance,
    );
    PoseWithCovarianceStamped {
        header: header_from_transform(tf),
        pose: PoseWithCovariance {
            pose: pose_from_array(out_pose),
            covariance: out_covariance,
        },
    }
}

/// `None` where the distro's tf2_sensor_msgs has no Imu transform (Humble).
pub fn imu(imu: &Imu, tf: &TransformStamped) -> Option<Imu> {
    let v = |v: &Vector3| [v.x, v.y, v.z];
    let o = &imu.orientation;
    let input = ffi::ImuData {
        orientation: [o.x, o.y, o.z, o.w],
        orientation_covariance: imu.orientation_covariance,
        angular_velocity: v(&imu.angular_velocity),
        angular_velocity_covariance: imu.angular_velocity_covariance,
        linear_acceleration: v(&imu.linear_acceleration),
        linear_acceleration_covariance: imu.linear_acceleration_covariance,
    };
    let mut out = ffi::ImuData::default();
    if !ffi::do_transform_imu(&input, &tf_array(tf), &mut out) {
        return None;
    }
    let [x, y, z, w] = out.orientation;
    Some(Imu {
        header: header_from_transform(tf),
        orientation: Quaternion { x, y, z, w },
        orientation_covariance: out.orientation_covariance,
        angular_velocity: vector(out.angular_velocity),
        angular_velocity_covariance: out.angular_velocity_covariance,
        linear_acceleration: vector(out.linear_acceleration),
        linear_acceleration_covariance: out.linear_acceleration_covariance,
    })
}

/// Upstream tf2::doTransform with separate input and output messages.
pub fn pointcloud2(cloud: &PointCloud2, tf: &TransformStamped) -> Result<PointCloud2, String> {
    let layout = ffi::CloudLayout {
        height: cloud.height,
        width: cloud.width,
        is_bigendian: cloud.is_bigendian,
        point_step: cloud.point_step,
        row_step: cloud.row_step,
        is_dense: cloud.is_dense,
    };
    let fields: Vec<ffi::PointField> = cloud
        .fields
        .iter()
        .map(|f| ffi::PointField {
            name: &f.name,
            offset: f.offset,
            datatype: f.datatype,
            count: f.count,
        })
        .collect();
    let mut data = vec![0u8; cloud.data.len()];
    ffi::do_transform_pointcloud2(&layout, &fields, &cloud.data, &tf_array(tf), &mut data)
        .map_err(|e| e.what().to_string())?;
    Ok(PointCloud2 {
        header: header_from_transform(tf),
        height: cloud.height,
        width: cloud.width,
        fields: cloud.fields.clone(),
        is_bigendian: cloud.is_bigendian,
        point_step: cloud.point_step,
        row_step: cloud.row_step,
        data,
        is_dense: cloud.is_dense,
    })
}
