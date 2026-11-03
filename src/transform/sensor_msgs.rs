use crate::transform::{Transformable, header_from_transform, math};
use crate::{Tf2Error, TransformStamped};
use ros_env::geometry_msgs::msg::{Quaternion, Vector3};
use ros_env::sensor_msgs::msg::{Imu, MagneticField, PointCloud2};

crate::impl_has_header_for_ros2_msg!(PointCloud2);
crate::impl_has_header_for_ros2_msg!(Imu);
crate::impl_has_header_for_ros2_msg!(MagneticField);

/// Follows tf2_sensor_msgs. The fields named `x`, `y`, `z` (and `normal_x`,
/// `normal_y`, `normal_z` when `normal_x` exists) are read as native-endian `f32`
/// at their offsets, whatever their declared datatype, as the C++ iterators do.
/// Points get the full transform and normals only the rotation, both in `f32` as
/// Eigen computes them. All other bytes are copied unchanged, and the point count
/// is `data.len() / point_step`, not `width * height`.
///
/// Where the C++ code would read out of bounds (a field that does not fit in
/// `point_step`, or `data` not a whole number of points), this returns
/// `InvalidArgument` instead.
impl Transformable for PointCloud2 {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        let step = self.point_step as usize;
        let offset = |name: &str| {
            self.fields
                .iter()
                .find(|f| f.name == name)
                .map(|f| f.offset as usize)
                .ok_or_else(|| Tf2Error::InvalidArgument(format!("Field {name} does not exist")))
        };
        // Same lookup order as the C++ iterators, so the same field is reported missing.
        let xyz = [offset("x")?, offset("y")?, offset("z")?];
        let normals = if self.fields.iter().any(|f| f.name == "normal_x") {
            Some([
                offset("normal_x")?,
                offset("normal_y")?,
                offset("normal_z")?,
            ])
        } else {
            None
        };

        if !self.data.is_empty() {
            if step == 0 || !self.data.len().is_multiple_of(step) {
                return Err(Tf2Error::InvalidArgument(format!(
                    "PointCloud2 data length {} is not a multiple of point_step {step}",
                    self.data.len()
                )));
            }
            if let Some(&off) = xyz
                .iter()
                .chain(normals.iter().flatten())
                .find(|&&o| o + 4 > step)
            {
                return Err(Tf2Error::InvalidArgument(format!(
                    "PointCloud2 field at offset {off} does not fit in point_step {step}"
                )));
            }
        }

        let r = math::eigen_rotation_f32(tf.rotation.map(|v| v as f32));
        let t = tf.translation.map(|v| v as f32);
        // Read from the input and write to the copy, as C++ reads p_in and writes
        // p_out: overlapping fields then give the same result.
        let mut data = self.data.clone();
        if step > 0 {
            for (src, dst) in self
                .data
                .chunks_exact(step)
                .zip(data.chunks_exact_mut(step))
            {
                write_f32x3(
                    dst,
                    xyz,
                    math::eigen_affine_f32(&r, t, read_f32x3(src, xyz)),
                );
                if let Some(n) = normals {
                    write_f32x3(
                        dst,
                        n,
                        math::eigen_affine_f32(&r, [0.0; 3], read_f32x3(src, n)),
                    );
                }
            }
        }

        Ok(PointCloud2 {
            header: header_from_transform(tf),
            height: self.height,
            width: self.width,
            fields: self.fields.clone(),
            is_bigendian: self.is_bigendian,
            point_step: self.point_step,
            row_step: self.row_step,
            data,
            is_dense: self.is_dense,
        })
    }
}

// Each value goes through one 4-byte slice: indexing the four bytes separately
// made the PointCloud2 loop about five times slower.
fn read_f32x3(p: &[u8], [a, b, c]: [usize; 3]) -> [f32; 3] {
    let at = |o: usize| f32::from_ne_bytes(p[o..o + 4].try_into().unwrap());
    [at(a), at(b), at(c)]
}

fn write_f32x3(p: &mut [u8], [a, b, c]: [usize; 3], v: [f32; 3]) {
    p[a..a + 4].copy_from_slice(&v[0].to_ne_bytes());
    p[b..b + 4].copy_from_slice(&v[1].to_ne_bytes());
    p[c..c + 4].copy_from_slice(&v[2].to_ne_bytes());
}

/// Follows tf2_sensor_msgs (Jazzy and later; Humble's C++ has no Imu transform).
/// Only the rotation is used. Angular velocity, linear acceleration and their
/// covariances are rotated into the new frame. The orientation describes the
/// sensor in a fixed world frame, so it is composed with the inverse rotation,
/// and its covariance is unchanged.
impl Transformable for Imu {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        let r = tf.rotation;
        let m = math::eigen_rotation(r);
        let rotate = |v: &Vector3| {
            let [x, y, z] = math::eigen_rotate(&m, [v.x, v.y, v.z]);
            Vector3 { x, y, z }
        };
        let o = &self.orientation;
        let [x, y, z, w] = math::eigen_quat_mul([o.x, o.y, o.z, o.w], math::eigen_inverse(r));
        Ok(Imu {
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

/// Follows tf2_sensor_msgs (Jazzy and later; Humble's C++ has no MagneticField
/// transform). Only the rotation is used: the field and its covariance are rotated
/// into the new frame.
impl Transformable for MagneticField {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        let m = &self.magnetic_field;
        let [x, y, z] = math::eigen_rotate(&math::eigen_rotation(tf.rotation), [m.x, m.y, m.z]);
        Ok(MagneticField {
            header: header_from_transform(tf),
            magnetic_field: Vector3 { x, y, z },
            magnetic_field_covariance: math::rotate_covariance3(
                &self.magnetic_field_covariance,
                tf.rotation,
            ),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::HasHeader;
    use crate::transform::test_support::{Rng, assert_close, make_tf, rotate_by_quat};
    use ros_env::sensor_msgs::msg::PointField;

    const S: f64 = std::f64::consts::FRAC_1_SQRT_2;
    // ROS PointField datatype enum: FLOAT32 == 7.
    const PF_FLOAT32: u8 = 7;

    fn vec3(v: &Vector3) -> [f64; 3] {
        [v.x, v.y, v.z]
    }

    #[test]
    fn imu_is_rotated_without_translation() {
        let tf = make_tf([10.0, 20.0, 30.0], [0.0, 0.0, S, S]); // 90 deg yaw
        let mut imu = Imu::default();
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
    fn magnetic_field_is_rotated_without_translation() {
        let mut tf = make_tf([10.0, 20.0, 30.0], [0.0, 0.0, S, S]); // 90 deg yaw
        tf.stamp_sec = 7;
        tf.stamp_nanosec = 8;
        let msg = MagneticField {
            magnetic_field: Vector3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            magnetic_field_covariance: [1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0],
            ..Default::default()
        };
        let out = msg.apply_transform(&tf).unwrap();
        assert_close(
            &vec3(&out.magnetic_field),
            &[0.0, 1.0, 0.0],
            1e-12,
            "magnetic field",
        );
        let c = out.magnetic_field_covariance;
        assert_close(
            &[c[0], c[4], c[8]],
            &[2.0, 1.0, 3.0],
            1e-12,
            "covariance diagonal",
        );
        assert_eq!(out.frame_id(), "map");
        assert_eq!(out.stamp(), (7, 8));
    }

    #[test]
    fn random_imu_transforms_match_independent_math() {
        let mut rng = Rng::new(2);
        for _ in 0..2000 {
            let tf = make_tf(rng.vec3(100.0), rng.unit_quat());
            let mut imu = Imu::default();
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

    fn make_xyz_cloud(frame: &str, points: &[[f32; 3]]) -> PointCloud2 {
        let mut cloud = PointCloud2::default();

        cloud.header.frame_id = frame.to_string();
        cloud.header.stamp.sec = 0;
        cloud.header.stamp.nanosec = 0;

        cloud.height = 1;
        cloud.width = points.len() as u32;
        cloud.is_bigendian = false;
        cloud.is_dense = true;

        let x = PointField {
            name: "x".to_string(),
            offset: 0,
            datatype: PF_FLOAT32,
            count: 1,
        };

        let y = PointField {
            name: "y".to_string(),
            offset: 4,
            datatype: PF_FLOAT32,
            count: 1,
        };

        let z = PointField {
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

    fn read_xyz_cloud(cloud: &PointCloud2) -> Vec<[f32; 3]> {
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

    #[test]
    fn transform_pointcloud2_translation_only() {
        let mut tf = make_tf([10.0, -2.0, 0.5], [0.0, 0.0, 0.0, 1.0]); // identity quaternion
        tf.stamp_sec = 123;
        tf.stamp_nanosec = 456;

        let cloud_in = make_xyz_cloud("lidar", &[[1.0, 2.0, 3.0]]);
        let cloud_out = cloud_in
            .apply_transform(&tf)
            .expect("transform should succeed");

        // tf2_sensor_msgs does: p_out = p_in; p_out.header = t_in.header;
        // So output header should match the transform header.
        assert_eq!(cloud_out.header.frame_id, "map");
        assert_eq!(cloud_out.header.stamp.sec, 123);
        assert_eq!(cloud_out.header.stamp.nanosec, 456);

        let pts = read_xyz_cloud(&cloud_out);
        assert_eq!(pts.len(), 1);
        assert_close(&pts[0].map(f64::from), &[11.0, 0.0, 3.5], 1e-5, "point");
    }

    #[test]
    fn transform_pointcloud2_rotate_z_90deg() {
        // 90 deg yaw about +Z: q = [0,0,sin(pi/4),cos(pi/4)]
        let tf = make_tf([0.0, 0.0, 0.0], [0.0, 0.0, S, S]);

        let cloud_in = make_xyz_cloud("lidar", &[[1.0, 0.0, 0.0]]);
        let cloud_out = cloud_in
            .apply_transform(&tf)
            .expect("transform should succeed");

        let pts = read_xyz_cloud(&cloud_out);
        assert_eq!(pts.len(), 1);

        // (1,0,0) rotated +90deg about Z -> (0,1,0)
        assert_close(&pts[0].map(f64::from), &[0.0, 1.0, 0.0], 1e-5, "point");
    }

    // Layout with normals and an extra field, to check that normals are rotated
    // without being translated and other fields are left alone.
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

        let mut cloud = PointCloud2::default();
        cloud.header.frame_id = "lidar".to_string();
        cloud.height = 1;
        cloud.width = points.len() as u32;
        cloud.is_dense = true;
        cloud.fields = names
            .iter()
            .enumerate()
            .map(|(i, name)| PointField {
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
        let tf = make_tf([10.0, 20.0, 30.0], [0.0, 0.0, S, S]);
        let out = cloud
            .apply_transform(&tf)
            .expect("transform should succeed");

        assert_eq!(cloud.data, data_in, "input cloud must not be modified");
        assert_eq!(out.header.frame_id, "map");
        assert_eq!(out.fields.len(), names.len());
        assert_eq!(out.point_step, 28);
        assert_eq!(out.data.len(), cloud.data.len());

        let got: Vec<f64> = out
            .data
            .chunks_exact(4)
            .map(|c| f32::from_le_bytes(c.try_into().unwrap()).into())
            .collect();
        let expected = [
            10.0, 21.0, 30.0, 0.0, 1.0, 0.0, 42.0, // point 0
            8.0, 20.0, 35.0, 0.0, 0.0, 1.0, 7.0, // point 1
        ];
        assert_close(&got, &expected, 1e-4, "values");
    }

    #[test]
    fn transform_pointcloud2_missing_xyz_fields_returns_err() {
        let tf = make_tf([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);

        // Malformed cloud: no fields at all, but has 12 bytes of data.
        let mut cloud = PointCloud2::default();
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
                Err(Tf2Error::InvalidArgument(_)) | Err(Tf2Error::Other(_))
            ),
            "expected an error for malformed PointCloud2, got: {res:?}"
        );
    }
}
