use super::math;
use crate::Tf2Error;
use crate::transform::{Transformable, header_from_transform};
use crate::transform_stamped::TransformStamped;
use ros_env::geometry_msgs;
use ros_env::geometry_msgs::msg::{
    Point, Pose, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion, Vector3, Vector3Stamped,
};

impl From<&geometry_msgs::msg::TransformStamped> for TransformStamped {
    fn from(t: &geometry_msgs::msg::TransformStamped) -> Self {
        TransformStamped {
            stamp_sec: t.header.stamp.sec,
            stamp_nanosec: t.header.stamp.nanosec,
            parent_frame: t.header.frame_id.clone(),
            child_frame: t.child_frame_id.clone(),
            translation: [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ],
            rotation: [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ],
        }
    }
}

crate::impl_has_header_for_ros2_msg!(Vector3Stamped);
crate::impl_has_header_for_ros2_msg!(PoseWithCovarianceStamped);

// The geometry_msgs conversions follow tf2_geometry_msgs, which applies the
// transform as a KDL frame. Covariances are rotated with tf2's Matrix3x3, which
// normalises the quaternion where KDL does not; see `math`.

fn transform_point(tf: &TransformStamped, p: math::Vec3) -> math::Vec3 {
    let [x, y, z] = math::mat_vec(&math::kdl_rotation(tf.rotation), p);
    [
        x + tf.translation[0],
        y + tf.translation[1],
        z + tf.translation[2],
    ]
}

fn transform_pose(tf: &TransformStamped, pose: &Pose) -> Pose {
    let r = math::kdl_rotation(tf.rotation);
    let [px, py, pz] = math::mat_vec(&r, [pose.position.x, pose.position.y, pose.position.z]);
    let o = &pose.orientation;
    let rotation = math::mat_mul(&r, &math::kdl_rotation([o.x, o.y, o.z, o.w]));
    let [x, y, z, w] = math::kdl_quaternion(&rotation);
    Pose {
        position: Point {
            x: px + tf.translation[0],
            y: py + tf.translation[1],
            z: pz + tf.translation[2],
        },
        orientation: Quaternion { x, y, z, w },
    }
}

/// Rotated and translated.
impl Transformable for Point {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        let [x, y, z] = transform_point(tf, [self.x, self.y, self.z]);
        Ok(Point { x, y, z })
    }
}

/// Rotated only: a direction is not affected by the translation.
impl Transformable for Vector3Stamped {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        let v = &self.vector;
        let [x, y, z] = math::mat_vec(&math::kdl_rotation(tf.rotation), [v.x, v.y, v.z]);
        Ok(Vector3Stamped {
            header: header_from_transform(tf),
            vector: Vector3 { x, y, z },
        })
    }
}

/// The transform is composed with the pose: `tf * pose`.
impl Transformable for Pose {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        Ok(transform_pose(tf, self))
    }
}

/// The pose is composed with the transform and each 3x3 block of the covariance
/// is rotated into the new frame.
impl Transformable for PoseWithCovarianceStamped {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        Ok(PoseWithCovarianceStamped {
            header: header_from_transform(tf),
            pose: PoseWithCovariance {
                pose: transform_pose(tf, &self.pose.pose),
                covariance: math::rotate_covariance6(
                    &self.pose.covariance,
                    &math::tf2_rotation(tf.rotation),
                ),
            },
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transform::test_support::{Rng, assert_close, make_tf, rotate_by_quat};
    use crate::{BufferCore, HasHeader, LookupTime, do_transform};

    const S: f64 = std::f64::consts::FRAC_1_SQRT_2;

    fn pose(p: [f64; 3], q: [f64; 4]) -> Pose {
        Pose {
            position: Point {
                x: p[0],
                y: p[1],
                z: p[2],
            },
            orientation: Quaternion {
                x: q[0],
                y: q[1],
                z: q[2],
                w: q[3],
            },
        }
    }

    fn quat(q: &Quaternion) -> [f64; 4] {
        [q.x, q.y, q.z, q.w]
    }

    #[test]
    fn point_is_rotated_then_translated() {
        let tf = make_tf([1.0, 2.0, 3.0], [0.0, 0.0, S, S]); // 90 deg yaw
        let p = Point {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        }
        .apply_transform(&tf)
        .unwrap();
        assert_close(&[p.x, p.y, p.z], &[1.0, 3.0, 3.0], 1e-12, "point");
    }

    #[test]
    fn vector3_stamped_ignores_translation_and_takes_transform_header() {
        let mut tf = make_tf([10.0, 20.0, 30.0], [0.0, 0.0, S, S]);
        tf.stamp_sec = 7;
        tf.stamp_nanosec = 8;
        let mut v = Vector3Stamped::default();
        v.header.frame_id = "sensor".into();
        v.vector = Vector3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
        let out = v.apply_transform(&tf).unwrap();
        assert_close(
            &[out.vector.x, out.vector.y, out.vector.z],
            &[0.0, 1.0, 0.0],
            1e-12,
            "vector",
        );
        assert_eq!(out.frame_id(), "map");
        assert_eq!(out.stamp(), (7, 8));
    }

    #[test]
    fn pose_composes_rotations_with_kdl_sign_convention() {
        // 90 + 90 deg yaw = 180 deg: KDL's last branch returns +z, w = 0.
        let tf = make_tf([1.0, 0.0, 0.0], [0.0, 0.0, S, S]);
        let out = pose([1.0, 0.0, 0.0], [0.0, 0.0, S, S])
            .apply_transform(&tf)
            .unwrap();
        let p = &out.position;
        assert_close(&[p.x, p.y, p.z], &[1.0, 1.0, 0.0], 1e-12, "position");
        assert_close(
            &quat(&out.orientation),
            &[0.0, 0.0, 1.0, 0.0],
            1e-12,
            "orientation",
        );
    }

    #[test]
    fn pose_covariance_blocks_are_rotated() {
        let tf = make_tf([5.0, 6.0, 7.0], [0.0, 0.0, S, S]);
        let mut msg = PoseWithCovarianceStamped::default();
        msg.pose.pose.orientation.w = 1.0;
        for (i, v) in [1.0, 2.0, 3.0, 4.0, 5.0, 6.0].into_iter().enumerate() {
            msg.pose.covariance[i * 6 + i] = v;
        }
        let out = msg.apply_transform(&tf).unwrap();
        // A 90 deg yaw swaps the x and y variances in both blocks.
        let diag: Vec<f64> = (0..6).map(|i| out.pose.covariance[i * 6 + i]).collect();
        assert_close(&diag, &[2.0, 1.0, 3.0, 5.0, 4.0, 6.0], 1e-12, "diagonal");
        let off_diagonal = (0..36)
            .filter(|i| i % 7 != 0)
            .map(|i| out.pose.covariance[i]);
        assert!(off_diagonal.into_iter().all(|v| v.abs() < 1e-12));
        assert_eq!(out.frame_id(), "map");
    }

    // Properties checked against independent formulas rather than hand values.
    #[test]
    fn random_transforms_match_independent_math() {
        let mut rng = Rng::new(1);
        for _ in 0..2000 {
            let tf = make_tf(rng.vec3(100.0), rng.unit_quat());
            let p = rng.vec3(100.0);
            let q_in = rng.unit_quat();

            // Point: q v q* + t, then back through the inverse transform.
            let out = Point {
                x: p[0],
                y: p[1],
                z: p[2],
            }
            .apply_transform(&tf)
            .unwrap();
            let r = rotate_by_quat(tf.rotation, p);
            let expected = [
                r[0] + tf.translation[0],
                r[1] + tf.translation[1],
                r[2] + tf.translation[2],
            ];
            assert_close(
                &[out.x, out.y, out.z],
                &expected,
                1e-9,
                "point vs quaternion rotation",
            );
            let [qx, qy, qz, qw] = tf.rotation;
            let inv_rot = [-qx, -qy, -qz, qw];
            let inv_t = rotate_by_quat(inv_rot, tf.translation).map(|v| -v);
            let back = out.apply_transform(&make_tf(inv_t, inv_rot)).unwrap();
            assert_close(&[back.x, back.y, back.z], &p, 1e-9, "roundtrip");

            // Pose: same position as Point, orientation = ±(q_tf * q_in), unit length.
            let out = pose(p, q_in).apply_transform(&tf).unwrap();
            let pos = &out.position;
            assert_close(&[pos.x, pos.y, pos.z], &expected, 1e-9, "pose position");
            let o = quat(&out.orientation);
            let product = math::quat_mul(tf.rotation, q_in);
            let sign = if o.iter().zip(product).map(|(a, b)| a * b).sum::<f64>() < 0.0 {
                -1.0
            } else {
                1.0
            };
            assert_close(&o, &product.map(|v| v * sign), 1e-9, "pose orientation");

            // Covariance: symmetry, and each block's trace and norm, are preserved.
            let mut cov = [0.0; 36];
            for i in 0..6 {
                for j in i..6 {
                    let v = rng.range(-10.0, 10.0);
                    cov[i * 6 + j] = v;
                    cov[j * 6 + i] = v;
                }
            }
            let mut msg = PoseWithCovarianceStamped::default();
            msg.pose.pose = pose(p, q_in);
            msg.pose.covariance = cov;
            let c = msg.apply_transform(&tf).unwrap().pose.covariance;
            for i in 0..6 {
                for j in 0..6 {
                    assert!((c[i * 6 + j] - c[j * 6 + i]).abs() < 1e-9, "symmetry");
                }
            }
            for (r0, c0) in [(0, 0), (0, 3), (3, 0), (3, 3)] {
                let block = |m: &[f64; 36], i: usize, j: usize| m[(r0 + i) * 6 + c0 + j];
                let trace = |m: &[f64; 36]| (0..3).map(|i| block(m, i, i)).sum::<f64>();
                let norm =
                    |m: &[f64; 36]| (0..9).map(|k| block(m, k / 3, k % 3).powi(2)).sum::<f64>();
                assert!((trace(&c) - trace(&cov)).abs() < 1e-9, "block trace");
                assert!((norm(&c) - norm(&cov)).abs() < 1e-8, "block norm");
            }
        }
    }

    // Issue #8: look the transform up, then apply it to a header-less Pose.
    #[test]
    fn lookup_then_apply_to_headerless_pose() {
        let mut buffer = BufferCore::new(std::time::Duration::from_secs(10));
        let mut tf = make_tf([1.0, 2.0, 3.0], [0.0, 0.0, S, S]);
        tf.child_frame = "base_link".into();
        buffer.set_transform(&tf, "test", true).unwrap();

        let tf = buffer
            .lookup_transform("map", "base_link", LookupTime::Latest)
            .unwrap();
        let out = do_transform(&pose([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]), &tf).unwrap();
        let p = &out.position;
        assert_close(&[p.x, p.y, p.z], &[1.0, 3.0, 3.0], 1e-12, "position");
        assert_close(
            &quat(&out.orientation),
            &[0.0, 0.0, S, S],
            1e-12,
            "orientation",
        );
    }
}
