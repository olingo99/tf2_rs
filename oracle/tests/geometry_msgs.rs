//! Parity of the geometry_msgs `Transformable` impls with tf2_geometry_msgs, checked
//! against the C++ oracle. Run with `cargo test -p tf2_rs_oracle -- --nocapture` to
//! see the error statistics.

mod common;

use common::{
    Rng, check_parity, covariance, edge_vectors, input_header, orientations, point, point32, pose,
    quaternion, vector3, vectors,
};
use ros_env::geometry_msgs::msg::{
    PointStamped, Polygon, PolygonStamped, Pose, PoseStamped, PoseWithCovariance,
    PoseWithCovarianceStamped, QuaternionStamped, Transform, TransformStamped, Twist,
    Vector3Stamped, VelocityStamped, Wrench, WrenchStamped,
};
use tf2_rs_oracle as oracle;

/// Random positions with the edge orientations, then the edge positions.
fn poses(rng: &mut Rng) -> Vec<Pose> {
    let mut out: Vec<_> = orientations(rng)
        .into_iter()
        .map(|q| pose(rng.vec3(1000.0), q))
        .collect();
    out.extend(
        edge_vectors()
            .into_iter()
            .map(|v| pose(v, [0.0, 0.0, 0.0, 1.0])),
    );
    out
}

fn poses_with_covariance(rng: &mut Rng) -> Vec<PoseWithCovariance> {
    orientations(rng)
        .into_iter()
        .enumerate()
        .map(|(i, q)| PoseWithCovariance {
            pose: pose(rng.vec3(1000.0), q),
            covariance: covariance(rng, 10.0, i),
        })
        .collect()
}

/// An empty polygon, one with the edge values, and random ones of a few sizes.
fn polygons(rng: &mut Rng) -> Vec<Polygon> {
    let mut out = vec![
        Polygon { points: vec![] },
        Polygon {
            points: edge_vectors().into_iter().map(point32).collect(),
        },
    ];
    for n in [1, 3, 8] {
        out.push(Polygon {
            points: (0..n).map(|_| point32(rng.vec3(100.0))).collect(),
        });
    }
    out
}

fn transforms(rng: &mut Rng) -> Vec<Transform> {
    poses(rng)
        .into_iter()
        .map(|p| Transform {
            translation: vector3([p.position.x, p.position.y, p.position.z]),
            rotation: p.orientation,
        })
        .collect()
}

/// Vectors paired two ways, so an edge value meets both a random value and another
/// edge value.
fn wrenches(rng: &mut Rng) -> Vec<Wrench> {
    let (a, b) = (vectors(rng, 37, 100.0), vectors(rng, 37, 100.0));
    a.iter()
        .zip(b.iter().rev())
        .chain(a.iter().zip(&b))
        .map(|(f, t)| Wrench {
            force: vector3(*f),
            torque: vector3(*t),
        })
        .collect()
}

#[test]
fn point_matches_tf2() {
    check_parity(
        "Point",
        10,
        |rng| vectors(rng, 37, 1000.0).into_iter().map(point).collect(),
        oracle::point,
    );
}

#[test]
fn point_stamped_matches_tf2() {
    check_parity(
        "PointStamped",
        17,
        |rng| {
            vectors(rng, 37, 1000.0)
                .into_iter()
                .map(|v| PointStamped {
                    header: input_header(),
                    point: point(v),
                })
                .collect()
        },
        oracle::point_stamped,
    );
}

#[test]
fn point32_matches_tf2() {
    check_parity(
        "Point32",
        18,
        |rng| vectors(rng, 37, 1000.0).into_iter().map(point32).collect(),
        oracle::point32,
    );
}

#[test]
fn polygon_matches_tf2() {
    check_parity("Polygon", 19, polygons, oracle::polygon);
}

#[test]
fn polygon_stamped_matches_tf2() {
    check_parity(
        "PolygonStamped",
        20,
        |rng| {
            polygons(rng)
                .into_iter()
                .map(|polygon| PolygonStamped {
                    header: input_header(),
                    polygon,
                })
                .collect()
        },
        oracle::polygon_stamped,
    );
}

#[test]
fn vector3_matches_tf2() {
    check_parity(
        "Vector3",
        21,
        |rng| vectors(rng, 37, 1000.0).into_iter().map(vector3).collect(),
        oracle::vector3,
    );
}

#[test]
fn vector3_stamped_matches_tf2() {
    check_parity(
        "Vector3Stamped",
        11,
        |rng| {
            vectors(rng, 37, 1000.0)
                .into_iter()
                .map(|v| Vector3Stamped {
                    header: input_header(),
                    vector: vector3(v),
                })
                .collect()
        },
        oracle::vector3_stamped,
    );
}

#[test]
fn quaternion_matches_tf2() {
    check_parity(
        "Quaternion",
        22,
        |rng| orientations(rng).into_iter().map(quaternion).collect(),
        oracle::quaternion,
    );
}

#[test]
fn quaternion_stamped_matches_tf2() {
    check_parity(
        "QuaternionStamped",
        23,
        |rng| {
            orientations(rng)
                .into_iter()
                .map(|q| QuaternionStamped {
                    header: input_header(),
                    quaternion: quaternion(q),
                })
                .collect()
        },
        oracle::quaternion_stamped,
    );
}

#[test]
fn pose_matches_tf2() {
    check_parity("Pose", 12, poses, oracle::pose);
}

#[test]
fn pose_stamped_matches_tf2() {
    check_parity(
        "PoseStamped",
        24,
        |rng| {
            poses(rng)
                .into_iter()
                .map(|pose| PoseStamped {
                    header: input_header(),
                    pose,
                })
                .collect()
        },
        oracle::pose_stamped,
    );
}

#[test]
fn pose_with_covariance_matches_tf2() {
    check_parity(
        "PoseWithCovariance",
        25,
        poses_with_covariance,
        oracle::pose_with_covariance,
    );
}

#[test]
fn pose_with_covariance_stamped_matches_tf2() {
    check_parity(
        "PoseWithCovarianceStamped",
        13,
        |rng| {
            poses_with_covariance(rng)
                .into_iter()
                .map(|pose| PoseWithCovarianceStamped {
                    header: input_header(),
                    pose,
                })
                .collect()
        },
        oracle::pose_with_covariance_stamped,
    );
}

#[test]
fn transform_matches_tf2() {
    check_parity("Transform", 26, transforms, oracle::transform);
}

/// `other` also compares `child_frame_id`, which depends on the distro.
#[test]
fn transform_stamped_matches_tf2() {
    check_parity(
        "TransformStamped",
        27,
        |rng| {
            transforms(rng)
                .into_iter()
                .map(|transform| TransformStamped {
                    header: input_header(),
                    child_frame_id: "input_child".into(),
                    transform,
                })
                .collect()
        },
        oracle::transform_stamped,
    );
}

#[test]
fn wrench_matches_tf2() {
    check_parity("Wrench", 28, wrenches, oracle::wrench);
}

#[test]
fn wrench_stamped_matches_tf2() {
    check_parity(
        "WrenchStamped",
        29,
        |rng| {
            wrenches(rng)
                .into_iter()
                .map(|wrench| WrenchStamped {
                    header: input_header(),
                    wrench,
                })
                .collect()
        },
        oracle::wrench_stamped,
    );
}

/// `other` also compares the header and the frame ids, which depend on the distro.
#[test]
fn velocity_stamped_matches_tf2() {
    check_parity(
        "VelocityStamped",
        30,
        |rng| {
            wrenches(rng)
                .into_iter()
                .map(|w| VelocityStamped {
                    header: input_header(),
                    body_frame_id: "input_body".into(),
                    reference_frame_id: "input_reference".into(),
                    velocity: Twist {
                        linear: w.force,
                        angular: w.torque,
                    },
                })
                .collect()
        },
        oracle::velocity_stamped,
    );
}
