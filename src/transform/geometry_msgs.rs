use crate::transform::{Transformable, header_from_transform, math};
use crate::{Tf2Error, TransformStamped};
use ros_env::geometry_msgs::msg::{
    Point, Point32, PointStamped, Polygon, PolygonStamped, Pose, PoseStamped, PoseWithCovariance,
    PoseWithCovarianceStamped, Quaternion, QuaternionStamped, Transform,
    TransformStamped as TransformStampedMsg, Twist, Vector3, Vector3Stamped, VelocityStamped,
    Wrench, WrenchStamped,
};

// The geometry_msgs conversions follow tf2_geometry_msgs, which applies the
// transform as a KDL frame. Covariances and velocities use tf2's Matrix3x3, which
// normalises the quaternion where KDL does not, and quaternions use tf2's product;
// see `math`.

fn vec3(v: &Vector3) -> math::Vec3 {
    [v.x, v.y, v.z]
}

fn vector3([x, y, z]: math::Vec3) -> Vector3 {
    Vector3 { x, y, z }
}

fn quat(q: &Quaternion) -> math::Quat {
    [q.x, q.y, q.z, q.w]
}

fn quaternion([x, y, z, w]: math::Quat) -> Quaternion {
    Quaternion { x, y, z, w }
}

fn transform_point32(r: &math::Mat3, t: math::Vec3, p: &Point32) -> Point32 {
    let [x, y, z] = math::kdl_frame_vec(r, t, [p.x.into(), p.y.into(), p.z.into()]);
    Point32 {
        x: x as f32,
        y: y as f32,
        z: z as f32,
    }
}

/// Rotated and translated.
impl Transformable for Point {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        let r = math::kdl_rotation(tf.rotation);
        let [x, y, z] = math::kdl_frame_vec(&r, tf.translation, [self.x, self.y, self.z]);
        Ok(Point { x, y, z })
    }
}

impl_stamped!(PointStamped, point);

/// Computed in `f64`, as KDL does, then cast back to `f32`.
impl Transformable for Point32 {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        let r = math::kdl_rotation(tf.rotation);
        Ok(transform_point32(&r, tf.translation, self))
    }
}

/// Each point is transformed as a `Point32`.
impl Transformable for Polygon {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        let r = math::kdl_rotation(tf.rotation);
        Ok(Polygon {
            points: self
                .points
                .iter()
                .map(|p| transform_point32(&r, tf.translation, p))
                .collect(),
        })
    }
}

impl_stamped!(PolygonStamped, polygon);

/// Rotated only: a direction is not affected by the translation.
impl Transformable for Vector3 {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        Ok(vector3(math::mat_vec(
            &math::kdl_rotation(tf.rotation),
            vec3(self),
        )))
    }
}

impl_stamped!(Vector3Stamped, vector);

/// `tf * q`, with tf2's quaternion product. Not normalised. Unlike a pose's
/// orientation, which is rebuilt from a matrix, this keeps the sign the product
/// gives, so the same rotation can come out with the opposite sign.
impl Transformable for Quaternion {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        Ok(quaternion(math::quat_mul(tf.rotation, quat(self))))
    }
}

impl_stamped!(QuaternionStamped, quaternion);

/// The transform is composed with the pose: `tf * pose`. The orientation is rebuilt
/// from the composed matrix, as KDL does, so its sign does not follow the input's:
/// `q` and `-q` give the same result, and the sign can jump as a rotation passes 120
/// degrees.
impl Transformable for Pose {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        let p = &self.position;
        let ([x, y, z], rotation) = math::kdl_frame_mul(
            tf.rotation,
            tf.translation,
            [p.x, p.y, p.z],
            quat(&self.orientation),
        );
        Ok(Pose {
            position: Point { x, y, z },
            orientation: quaternion(rotation),
        })
    }
}

impl_stamped!(PoseStamped, pose);

/// The pose as `Pose`, and each 3x3 block of the covariance is rotated into the new
/// frame, normalising the quaternion where the pose does not.
impl Transformable for PoseWithCovariance {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        Ok(PoseWithCovariance {
            pose: self.pose.apply_transform(tf)?,
            covariance: math::rotate_covariance6(
                &self.covariance,
                &math::tf2_rotation(tf.rotation),
            ),
        })
    }
}

impl_stamped!(PoseWithCovarianceStamped, pose);

/// The transform is composed with this one, as for `Pose`.
impl Transformable for Transform {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        let (translation, rotation) = math::kdl_frame_mul(
            tf.rotation,
            tf.translation,
            vec3(&self.translation),
            quat(&self.rotation),
        );
        Ok(Transform {
            translation: vector3(translation),
            rotation: quaternion(rotation),
        })
    }
}

crate::impl_has_header_for_ros2_msg!(TransformStampedMsg);

/// As `Transform`, with the header from the transform. `child_frame_id` follows the
/// installed tf2_geometry_msgs: newer versions copy it from the input, older ones
/// leave it empty.
impl Transformable for TransformStampedMsg {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        Ok(TransformStampedMsg {
            header: header_from_transform(tf),
            child_frame_id: if cfg!(tf2_copies_child_frame_id) {
                self.child_frame_id.clone()
            } else {
                String::new()
            },
            transform: self.transform.apply_transform(tf)?,
        })
    }
}

/// The force is rotated. The torque is rotated, then gains the torque created by
/// moving the force: `translation x force`.
impl Transformable for Wrench {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        let r = math::kdl_rotation(tf.rotation);
        let force = math::mat_vec(&r, vec3(&self.force));
        let torque = math::mat_vec(&r, vec3(&self.torque));
        Ok(Wrench {
            force: vector3(force),
            torque: vector3(math::add(torque, math::tf2_cross(tf.translation, force))),
        })
    }
}

impl_stamped!(WrenchStamped, wrench);

/// `angular` is rotated, and `linear` is rotated and gains
/// `translation x rotated angular`. The rotation is normalised. `body_frame_id` and
/// `reference_frame_id` are left empty.
///
/// **Follows the installed tf2_geometry_msgs, and the result depends on it.** Newer
/// versions (Lyrical, Rolling) return that result with the header from the
/// transform. Older ones (Humble, Jazzy, Kilted) also add the input velocity to the
/// result and keep the input's header, so the output's header does not name the
/// frame it is expressed in.
///
/// Like upstream, this type has no `HasHeader`, so it cannot go through
/// `BufferCore::transform`: look the transform up and call `do_transform`.
impl Transformable for VelocityStamped {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        let basis = math::tf2_rotation(tf.rotation);
        let (linear_in, angular_in) = (vec3(&self.velocity.linear), vec3(&self.velocity.angular));
        let angular = math::mat_vec(&basis, angular_in);
        let linear = math::add(
            math::mat_vec(&basis, linear_in),
            math::tf2_cross(tf.translation, angular),
        );
        let (header, linear, angular) = if cfg!(tf2_velocity_adds_input) {
            (
                self.header.clone(),
                math::add(linear, linear_in),
                math::add(angular, angular_in),
            )
        } else {
            (header_from_transform(tf), linear, angular)
        };
        Ok(VelocityStamped {
            header,
            body_frame_id: String::new(),
            reference_frame_id: String::new(),
            velocity: Twist {
                linear: vector3(linear),
                angular: vector3(angular),
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

    fn yaw_90() -> TransformStamped {
        let mut tf = make_tf([1.0, 2.0, 3.0], [0.0, 0.0, S, S]);
        tf.stamp_sec = 7;
        tf.stamp_nanosec = 8;
        tf
    }

    fn assert_header_from_transform<T: HasHeader>(out: &T) {
        assert_eq!(out.frame_id(), "map");
        assert_eq!(out.stamp(), (7, 8));
    }

    fn input_header() -> ros_env::std_msgs::msg::Header {
        ros_env::std_msgs::msg::Header {
            stamp: ros_env::builtin_interfaces::msg::Time { sec: 1, nanosec: 2 },
            frame_id: "input".into(),
        }
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
    fn vector3_is_rotated_without_translation() {
        let tf = make_tf([10.0, 20.0, 30.0], [0.0, 0.0, S, S]); // 90 deg yaw
        let v = Vector3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        }
        .apply_transform(&tf)
        .unwrap();
        assert_close(&[v.x, v.y, v.z], &[0.0, 1.0, 0.0], 1e-12, "vector");
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

    #[test]
    fn point_stamped_takes_transform_header() {
        let msg = PointStamped {
            header: input_header(),
            point: Point {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
        };
        let out = msg.apply_transform(&yaw_90()).unwrap();
        let p = &out.point;
        assert_close(&[p.x, p.y, p.z], &[1.0, 3.0, 3.0], 1e-12, "point");
        assert_header_from_transform(&out);
    }

    #[test]
    fn point32_is_rotated_then_translated() {
        let p = Point32 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        }
        .apply_transform(&yaw_90())
        .unwrap();
        assert_eq!([p.x, p.y, p.z], [1.0, 3.0, 3.0]);
    }

    #[test]
    fn polygon_transforms_every_point_and_may_be_empty() {
        let tf = yaw_90();
        let point = |x, y, z| Point32 { x, y, z };
        let polygon = Polygon {
            points: vec![point(1.0, 0.0, 0.0), point(0.0, 1.0, 0.0)],
        };
        let out = polygon.apply_transform(&tf).unwrap();
        let got: Vec<f64> = out
            .points
            .iter()
            .flat_map(|p| [p.x, p.y, p.z])
            .map(f64::from)
            .collect();
        assert_close(&got, &[1.0, 3.0, 3.0, 0.0, 2.0, 3.0], 1e-6, "points");

        let empty = Polygon { points: vec![] };
        assert!(empty.apply_transform(&tf).unwrap().points.is_empty());
    }

    #[test]
    fn polygon_stamped_takes_transform_header() {
        let msg = PolygonStamped {
            header: input_header(),
            polygon: Polygon {
                points: vec![Point32 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                }],
            },
        };
        let out = msg.apply_transform(&yaw_90()).unwrap();
        assert_eq!(out.polygon.points.len(), 1);
        assert_header_from_transform(&out);
    }

    #[test]
    fn quaternion_is_the_product_with_the_transform() {
        // 90 + 90 deg yaw = 180 deg.
        let q = quaternion([0.0, 0.0, S, S])
            .apply_transform(&yaw_90())
            .unwrap();
        assert_close(&quat(&q), &[0.0, 0.0, 1.0, 0.0], 1e-12, "quaternion");
    }

    #[test]
    fn quaternion_stamped_takes_transform_header() {
        let msg = QuaternionStamped {
            header: input_header(),
            quaternion: quaternion([0.0, 0.0, 0.0, 1.0]),
        };
        let out = msg.apply_transform(&yaw_90()).unwrap();
        assert_close(
            &quat(&out.quaternion),
            &[0.0, 0.0, S, S],
            1e-12,
            "quaternion",
        );
        assert_header_from_transform(&out);
    }

    #[test]
    fn pose_stamped_takes_transform_header() {
        let msg = PoseStamped {
            header: input_header(),
            pose: pose([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        };
        let out = msg.apply_transform(&yaw_90()).unwrap();
        let p = &out.pose.position;
        assert_close(&[p.x, p.y, p.z], &[1.0, 3.0, 3.0], 1e-12, "position");
        assert_header_from_transform(&out);
    }

    #[test]
    fn pose_with_covariance_rotates_pose_and_covariance() {
        let mut msg = PoseWithCovariance {
            pose: pose([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            ..Default::default()
        };
        msg.covariance[0] = 1.0; // x
        msg.covariance[7] = 2.0; // y
        let out = msg.apply_transform(&yaw_90()).unwrap();
        let p = &out.pose.position;
        assert_close(&[p.x, p.y, p.z], &[1.0, 3.0, 3.0], 1e-12, "position");
        assert_close(
            &[out.covariance[0], out.covariance[7]],
            &[2.0, 1.0],
            1e-12,
            "covariance",
        );
    }

    #[test]
    fn transform_is_composed_with_the_transform() {
        let msg = Transform {
            translation: vector3([1.0, 0.0, 0.0]),
            rotation: quaternion([0.0, 0.0, S, S]),
        };
        let out = msg.apply_transform(&yaw_90()).unwrap();
        assert_close(
            &vec3(&out.translation),
            &[1.0, 3.0, 3.0],
            1e-12,
            "translation",
        );
        assert_close(
            &quat(&out.rotation),
            &[0.0, 0.0, 1.0, 0.0],
            1e-12,
            "rotation",
        );
    }

    #[test]
    fn transform_stamped_child_frame_follows_the_installed_tf2() {
        let msg = TransformStampedMsg {
            header: input_header(),
            child_frame_id: "child".into(),
            transform: Transform {
                translation: vector3([1.0, 0.0, 0.0]),
                rotation: quaternion([0.0, 0.0, 0.0, 1.0]),
            },
        };
        let out = msg.apply_transform(&yaw_90()).unwrap();
        assert_close(
            &vec3(&out.transform.translation),
            &[1.0, 3.0, 3.0],
            1e-12,
            "translation",
        );
        assert_header_from_transform(&out);
        let expected = if cfg!(tf2_copies_child_frame_id) {
            "child"
        } else {
            ""
        };
        assert_eq!(out.child_frame_id, expected);
    }

    #[test]
    fn wrench_torque_gains_the_moved_force() {
        let msg = Wrench {
            force: vector3([1.0, 0.0, 0.0]),
            torque: vector3([0.0, 0.0, 1.0]),
        };
        let out = msg.apply_transform(&yaw_90()).unwrap();
        // The force becomes (0, 1, 0). (1, 2, 3) x (0, 1, 0) = (-3, 0, 1).
        assert_close(&vec3(&out.force), &[0.0, 1.0, 0.0], 1e-12, "force");
        assert_close(&vec3(&out.torque), &[-3.0, 0.0, 2.0], 1e-12, "torque");
    }

    #[test]
    fn wrench_stamped_takes_transform_header() {
        let msg = WrenchStamped {
            header: input_header(),
            wrench: Wrench {
                force: vector3([1.0, 0.0, 0.0]),
                torque: vector3([0.0, 0.0, 0.0]),
            },
        };
        let out = msg.apply_transform(&yaw_90()).unwrap();
        assert_close(&vec3(&out.wrench.force), &[0.0, 1.0, 0.0], 1e-12, "force");
        assert_header_from_transform(&out);
    }

    #[test]
    fn velocity_stamped_follows_the_installed_tf2() {
        let msg = VelocityStamped {
            header: input_header(),
            body_frame_id: "body".into(),
            reference_frame_id: "reference".into(),
            velocity: Twist {
                linear: vector3([1.0, 0.0, 0.0]),
                angular: vector3([0.0, 0.0, 1.0]),
            },
        };
        let out = msg.apply_transform(&yaw_90()).unwrap();
        // Rotated: linear (0, 1, 0), angular (0, 0, 1). (1, 2, 3) x (0, 0, 1) = (2, -1, 0).
        let (linear, angular, frame, stamp) = if cfg!(tf2_velocity_adds_input) {
            ([3.0, 0.0, 0.0], [0.0, 0.0, 2.0], "input", (1, 2))
        } else {
            ([2.0, 0.0, 0.0], [0.0, 0.0, 1.0], "map", (7, 8))
        };
        assert_close(&vec3(&out.velocity.linear), &linear, 1e-12, "linear");
        assert_close(&vec3(&out.velocity.angular), &angular, 1e-12, "angular");
        assert_eq!(out.header.frame_id, frame);
        assert_eq!((out.header.stamp.sec, out.header.stamp.nanosec), stamp);
        assert!(out.body_frame_id.is_empty() && out.reference_frame_id.is_empty());
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
