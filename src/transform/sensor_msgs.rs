use crate::Tf2Error;
use crate::TransformStamped;
use crate::error::check_status;
use crate::ffi::ffi;
use crate::transform::{Transformable, header_from_transform, math};
use ros_env::geometry_msgs::msg::{Quaternion, Vector3};
use ros_env::sensor_msgs;

crate::impl_has_header_for_ros2_msg!(sensor_msgs::msg::PointCloud2);
crate::impl_has_header_for_ros2_msg!(sensor_msgs::msg::Imu);

impl Transformable for sensor_msgs::msg::PointCloud2 {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        let layout = ffi::Tf2CloudLayout {
            height: self.height,
            width: self.width,
            is_bigendian: self.is_bigendian,
            point_step: self.point_step,
            row_step: self.row_step,
            is_dense: self.is_dense,
        };
        let fields: Vec<ffi::Tf2PointField> = self
            .fields
            .iter()
            .map(|f| ffi::Tf2PointField {
                name: &f.name,
                offset: f.offset,
                datatype: f.datatype,
                count: f.count,
            })
            .collect();

        // C++ writes the transformed points straight into the output buffer.
        let mut data = vec![0u8; self.data.len()];
        check_status(ffi::do_transform_pointcloud2(
            &layout,
            &fields,
            &self.data,
            &tf.to_ffi_transform(),
            &mut data,
        ))?;

        // tf2_sensor_msgs sets the output header to the transform's header.
        let mut out = sensor_msgs::msg::PointCloud2::default();
        out.header.stamp.sec = tf.stamp_sec;
        out.header.stamp.nanosec = tf.stamp_nanosec;
        out.header.frame_id = tf.parent_frame.clone();

        out.height = self.height;
        out.width = self.width;
        out.fields = self.fields.clone();
        out.is_bigendian = self.is_bigendian;
        out.point_step = self.point_step;
        out.row_step = self.row_step;
        out.data = data;
        out.is_dense = self.is_dense;
        Ok(out)
    }
}

/// Follows tf2_sensor_msgs (Jazzy and later; Humble's C++ has no Imu transform).
/// Only the rotation is used. Angular velocity, linear acceleration and their
/// covariances are rotated into the new frame. The orientation describes the
/// sensor in a fixed world frame, so it is composed with the inverse rotation,
/// and its covariance is unchanged.
impl Transformable for sensor_msgs::msg::Imu {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        let r = tf.rotation;
        let m = math::eigen_rotation(r);
        let rotate = |v: &Vector3| {
            let [x, y, z] = math::mat_vec(&m, [v.x, v.y, v.z]);
            Vector3 { x, y, z }
        };
        let o = &self.orientation;
        let [x, y, z, w] = math::eigen_quat_mul([o.x, o.y, o.z, o.w], math::eigen_inverse(r));
        Ok(sensor_msgs::msg::Imu {
            header: header_from_transform(tf),
            orientation: Quaternion { x, y, z, w },
            orientation_covariance: self.orientation_covariance,
            angular_velocity: rotate(&self.angular_velocity),
            angular_velocity_covariance: math::rotate_covariance3(
                &self.angular_velocity_covariance,
                r,
            ),
            linear_acceleration: rotate(&self.linear_acceleration),
            linear_acceleration_covariance: math::rotate_covariance3(
                &self.linear_acceleration_covariance,
                r,
            ),
        })
    }
}

#[cfg(test)]
mod imu_tests {
    use super::*;
    use crate::HasHeader;
    use crate::transform::test_support::{Rng, assert_close, make_tf, rotate_by_quat};

    const S: f64 = std::f64::consts::FRAC_1_SQRT_2;

    fn vec3(v: &Vector3) -> [f64; 3] {
        [v.x, v.y, v.z]
    }

    #[test]
    fn imu_is_rotated_without_translation() {
        let tf = make_tf([10.0, 20.0, 30.0], [0.0, 0.0, S, S]); // 90 deg yaw
        let mut imu = sensor_msgs::msg::Imu::default();
        imu.header.frame_id = "imu_link".into();
        imu.orientation.w = 1.0;
        imu.orientation_covariance = [0.1, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.3];
        imu.angular_velocity = Vector3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
        imu.angular_velocity_covariance = [1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0];
        imu.linear_acceleration = Vector3 {
            x: 0.0,
            y: 0.0,
            z: 9.81,
        };

        let out = imu.apply_transform(&tf).unwrap();
        assert_close(
            &vec3(&out.angular_velocity),
            &[0.0, 1.0, 0.0],
            1e-12,
            "angular velocity",
        );
        assert_close(
            &vec3(&out.linear_acceleration),
            &[0.0, 0.0, 9.81],
            1e-12,
            "acceleration",
        );
        let o = &out.orientation;
        assert_close(
            &[o.x, o.y, o.z, o.w],
            &[0.0, 0.0, -S, S],
            1e-12,
            "orientation",
        );
        let c = out.angular_velocity_covariance;
        assert_close(
            &[c[0], c[4], c[8]],
            &[2.0, 1.0, 3.0],
            1e-12,
            "covariance diagonal",
        );
        assert_eq!(out.orientation_covariance, imu.orientation_covariance);
        assert_eq!(out.frame_id(), "map");
    }

    #[test]
    fn random_imu_transforms_match_independent_math() {
        let mut rng = Rng::new(2);
        for _ in 0..2000 {
            let tf = make_tf(rng.vec3(100.0), rng.unit_quat());
            let mut imu = sensor_msgs::msg::Imu::default();
            let q = rng.unit_quat();
            imu.orientation = Quaternion {
                x: q[0],
                y: q[1],
                z: q[2],
                w: q[3],
            };
            let [x, y, z] = rng.vec3(10.0);
            imu.angular_velocity = Vector3 { x, y, z };
            let [x, y, z] = rng.vec3(10.0);
            imu.linear_acceleration = Vector3 { x, y, z };
            for i in 0..3 {
                for j in i..3 {
                    let v = rng.range(-1.0, 1.0);
                    imu.linear_acceleration_covariance[i * 3 + j] = v;
                    imu.linear_acceleration_covariance[j * 3 + i] = v;
                }
            }

            let out = imu.apply_transform(&tf).unwrap();
            let expected = rotate_by_quat(tf.rotation, vec3(&imu.angular_velocity));
            assert_close(
                &vec3(&out.angular_velocity),
                &expected,
                1e-9,
                "angular velocity",
            );
            let expected = rotate_by_quat(tf.rotation, vec3(&imu.linear_acceleration));
            assert_close(
                &vec3(&out.linear_acceleration),
                &expected,
                1e-9,
                "acceleration",
            );
            // Composing the result with the rotation again gives the input orientation.
            let o = &out.orientation;
            let back = math::quat_mul([o.x, o.y, o.z, o.w], tf.rotation);
            assert_close(&back, &q, 1e-9, "orientation");
            let trace = |c: &[f64; 9]| c[0] + c[4] + c[8];
            let (c_in, c_out) = (
                &imu.linear_acceleration_covariance,
                &out.linear_acceleration_covariance,
            );
            assert!(
                (trace(c_in) - trace(c_out)).abs() < 1e-9,
                "covariance trace"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ROS PointField datatype enum: FLOAT32 == 7.
    const PF_FLOAT32: u8 = 7;

    fn make_tf(
        parent_frame: &str, // target frame (tf header.frame_id)
        child_frame: &str,  // source frame (tf child_frame_id)
        translation: [f64; 3],
        rotation_xyzw: [f64; 4],
        stamp: (i32, u32),
    ) -> crate::TransformStamped {
        crate::TransformStamped {
            stamp_sec: stamp.0,
            stamp_nanosec: stamp.1,
            parent_frame: parent_frame.to_string(),
            child_frame: child_frame.to_string(),
            translation,
            rotation: rotation_xyzw,
        }
    }

    fn make_xyz_cloud(frame: &str, points: &[[f32; 3]]) -> sensor_msgs::msg::PointCloud2 {
        let mut cloud = sensor_msgs::msg::PointCloud2::default();

        cloud.header.frame_id = frame.to_string();
        cloud.header.stamp.sec = 0;
        cloud.header.stamp.nanosec = 0;

        cloud.height = 1;
        cloud.width = points.len() as u32;
        cloud.is_bigendian = false;
        cloud.is_dense = true;

        let x = sensor_msgs::msg::PointField {
            name: "x".to_string(),
            offset: 0,
            datatype: PF_FLOAT32,
            count: 1,
        };

        let y = sensor_msgs::msg::PointField {
            name: "y".to_string(),
            offset: 4,
            datatype: PF_FLOAT32,
            count: 1,
        };

        let z = sensor_msgs::msg::PointField {
            name: "z".to_string(),
            offset: 8,
            datatype: PF_FLOAT32,
            count: 1,
        };

        cloud.fields = vec![x, y, z];

        cloud.point_step = 12;
        cloud.row_step = cloud.point_step * cloud.width;

        cloud.data = Vec::with_capacity(cloud.row_step as usize);
        for p in points {
            cloud.data.extend_from_slice(&p[0].to_le_bytes());
            cloud.data.extend_from_slice(&p[1].to_le_bytes());
            cloud.data.extend_from_slice(&p[2].to_le_bytes());
        }

        cloud
    }

    fn read_xyz_cloud(cloud: &sensor_msgs::msg::PointCloud2) -> Vec<[f32; 3]> {
        assert_eq!(cloud.point_step, 12, "test helper assumes point_step=12");
        cloud
            .data
            .chunks_exact(12)
            .map(|c| {
                let x = f32::from_le_bytes(c[0..4].try_into().unwrap());
                let y = f32::from_le_bytes(c[4..8].try_into().unwrap());
                let z = f32::from_le_bytes(c[8..12].try_into().unwrap());
                [x, y, z]
            })
            .collect()
    }

    fn assert_near(a: f32, b: f32, eps: f32, label: &str) {
        if (a - b).abs() > eps {
            panic!("{label}: {a} vs {b} (eps={eps})");
        }
    }

    #[test]
    fn transform_pointcloud2_translation_only() {
        // Transform from "lidar" (source) into "map" (target)
        let tf = make_tf(
            "map",
            "lidar",
            [10.0, -2.0, 0.5],
            [0.0, 0.0, 0.0, 1.0], // identity quaternion
            (123, 456),
        );

        let cloud_in = make_xyz_cloud("lidar", &[[1.0, 2.0, 3.0]]);
        let cloud_out = cloud_in
            .apply_transform(&tf)
            .expect("transform should succeed");

        // tf2_sensor_msgs does: p_out = p_in; p_out.header = t_in.header;
        // So output header should match the transform header. :contentReference[oaicite:2]{index=2}
        assert_eq!(cloud_out.header.frame_id, "map");
        assert_eq!(cloud_out.header.stamp.sec, 123);
        assert_eq!(cloud_out.header.stamp.nanosec, 456);

        let pts = read_xyz_cloud(&cloud_out);
        assert_eq!(pts.len(), 1);
        assert_near(pts[0][0], 11.0, 1e-5, "x");
        assert_near(pts[0][1], 0.0, 1e-5, "y");
        assert_near(pts[0][2], 3.5, 1e-5, "z");
    }

    #[test]
    fn transform_pointcloud2_rotate_z_90deg() {
        // 90 deg yaw about +Z: q = [0,0,sin(pi/4),cos(pi/4)]
        let s = (0.5f64).sqrt();
        let tf = make_tf("map", "lidar", [0.0, 0.0, 0.0], [0.0, 0.0, s, s], (0, 0));

        let cloud_in = make_xyz_cloud("lidar", &[[1.0, 0.0, 0.0]]);
        let cloud_out = cloud_in
            .apply_transform(&tf)
            .expect("transform should succeed");

        let pts = read_xyz_cloud(&cloud_out);
        assert_eq!(pts.len(), 1);

        // (1,0,0) rotated +90deg about Z -> (0,1,0)
        assert_near(pts[0][0], 0.0, 1e-5, "x");
        assert_near(pts[0][1], 1.0, 1e-5, "y");
        assert_near(pts[0][2], 0.0, 1e-5, "z");
    }

    // Layout with normals and an extra field, to check that the in-place C++ call
    // rotates normals without translating them and leaves other fields alone.
    #[test]
    fn transform_pointcloud2_normals_and_extra_fields() {
        let names = [
            "x",
            "y",
            "z",
            "normal_x",
            "normal_y",
            "normal_z",
            "intensity",
        ];
        let points: [[f32; 7]; 2] = [
            [1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 42.0],
            [0.0, 2.0, 5.0, 0.0, 0.0, 1.0, 7.0],
        ];

        let mut cloud = sensor_msgs::msg::PointCloud2::default();
        cloud.header.frame_id = "lidar".to_string();
        cloud.height = 1;
        cloud.width = points.len() as u32;
        cloud.is_dense = true;
        cloud.fields = names
            .iter()
            .enumerate()
            .map(|(i, name)| sensor_msgs::msg::PointField {
                name: name.to_string(),
                offset: 4 * i as u32,
                datatype: PF_FLOAT32,
                count: 1,
            })
            .collect();
        cloud.point_step = 28;
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.data = points
            .iter()
            .flatten()
            .flat_map(|v| v.to_le_bytes())
            .collect();
        let data_in = cloud.data.clone();

        // 90 deg yaw about +Z plus a translation.
        let s = (0.5f64).sqrt();
        let tf = make_tf("map", "lidar", [10.0, 20.0, 30.0], [0.0, 0.0, s, s], (5, 6));
        let out = cloud
            .apply_transform(&tf)
            .expect("transform should succeed");

        assert_eq!(cloud.data, data_in, "input cloud must not be modified");
        assert_eq!(out.header.frame_id, "map");
        assert_eq!(out.fields.len(), names.len());
        assert_eq!(out.point_step, 28);
        assert_eq!(out.data.len(), cloud.data.len());

        let got: Vec<f32> = out
            .data
            .chunks_exact(4)
            .map(|c| f32::from_le_bytes(c.try_into().unwrap()))
            .collect();
        let expected: [f32; 14] = [
            10.0, 21.0, 30.0, 0.0, 1.0, 0.0, 42.0, // point 0
            8.0, 20.0, 35.0, 0.0, 0.0, 1.0, 7.0, // point 1
        ];
        for (i, (g, e)) in got.iter().zip(expected).enumerate() {
            assert_near(*g, e, 1e-4, &format!("value {i}"));
        }
    }

    #[test]
    fn transform_pointcloud2_missing_xyz_fields_returns_err() {
        let tf = make_tf(
            "map",
            "lidar",
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            (0, 0),
        );

        // Malformed cloud: no fields at all, but has 12 bytes of data.
        let mut cloud = sensor_msgs::msg::PointCloud2::default();
        cloud.header.frame_id = "lidar".to_string();
        cloud.height = 1;
        cloud.width = 1;
        cloud.point_step = 12;
        cloud.row_step = 12;
        cloud.is_bigendian = false;
        cloud.is_dense = true;
        cloud.data = vec![0u8; 12];

        let res = cloud.apply_transform(&tf);
        assert!(
            matches!(
                res,
                Err(crate::Tf2Error::InvalidArgument(_)) | Err(crate::Tf2Error::Other(_))
            ),
            "expected an error for malformed PointCloud2, got: {res:?}"
        );
    }
}
