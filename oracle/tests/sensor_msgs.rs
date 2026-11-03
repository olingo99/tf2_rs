//! Parity of the sensor_msgs `Transformable` impls with tf2_sensor_msgs, checked
//! against the C++ oracle. Run with `cargo test -p tf2_rs_oracle -- --nocapture` to
//! see the error statistics.

mod common;

use common::{
    BIT_EXACT, Flatten, Rng, check_parity, check_parity_if_available, covariance, edge_vectors,
    input_header, make_tf, orientations, quaternion, vector3,
};
use ros_env::sensor_msgs::msg::{Imu, MagneticField, PointCloud2, PointField};
use tf2_rs::Transformable;
use tf2_rs_oracle as oracle;

impl Flatten for Imu {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let Imu {
            header: _,
            orientation,
            orientation_covariance,
            angular_velocity: a,
            angular_velocity_covariance,
            linear_acceleration: l,
            linear_acceleration_covariance,
        } = self;
        let mut out = orientation.fields();
        out.extend([
            ("orientation_cov", orientation_covariance.to_vec()),
            ("angular_velocity", vec![a.x, a.y, a.z]),
            ("angular_velocity_cov", angular_velocity_covariance.to_vec()),
            ("linear_acceleration", vec![l.x, l.y, l.z]),
            (
                "linear_acceleration_cov",
                linear_acceleration_covariance.to_vec(),
            ),
        ]);
        out
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        vec![("header", format!("{:?}", self.header))]
    }
}

impl Flatten for MagneticField {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let MagneticField {
            header: _,
            magnetic_field: m,
            magnetic_field_covariance,
        } = self;
        vec![
            ("magnetic_field", vec![m.x, m.y, m.z]),
            ("magnetic_field_cov", magnetic_field_covariance.to_vec()),
        ]
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        vec![("header", format!("{:?}", self.header))]
    }
}

impl Flatten for PointCloud2 {
    /// Every 4-byte word, transformed fields and padding alike, as f32.
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let words = self
            .data
            .chunks_exact(4)
            .map(|w| f32::from_ne_bytes([w[0], w[1], w[2], w[3]]) as f64)
            .collect();
        vec![("data", words)]
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        let PointCloud2 {
            header,
            height,
            width,
            fields,
            is_bigendian,
            point_step,
            row_step,
            data,
            is_dense,
        } = self;
        let layout = (height, width, is_bigendian, point_step, row_step, is_dense);
        let mut out = vec![
            ("header", format!("{header:?}")),
            ("layout", format!("{layout:?}")),
            ("point fields", format!("{fields:?}")),
        ];
        // The words above are widened to f64, which can change a signalling NaN.
        if BIT_EXACT {
            out.push(("data bytes", format!("{data:?}")));
        }
        out
    }
}

#[test]
fn imu_matches_tf2() {
    check_parity_if_available(
        "Imu",
        14,
        |rng| {
            let edges = edge_vectors();
            orientations(rng)
                .into_iter()
                .enumerate()
                .map(|(i, q)| {
                    // Turns through the edge values, then random ones.
                    let mut pick = |offset: usize| match edges.get((i + offset) % 40) {
                        Some(v) => vector3(*v),
                        None => vector3(rng.vec3(50.0)),
                    };
                    let angular_velocity = pick(0);
                    let linear_acceleration = pick(3);
                    Imu {
                        header: input_header(),
                        orientation: quaternion(q),
                        orientation_covariance: covariance(rng, 1.0, i),
                        angular_velocity,
                        angular_velocity_covariance: covariance(rng, 1.0, i + 1),
                        linear_acceleration,
                        linear_acceleration_covariance: covariance(rng, 1.0, i + 2),
                    }
                })
                .collect()
        },
        oracle::imu,
    );
}

#[test]
fn magnetic_field_matches_tf2() {
    check_parity_if_available(
        "MagneticField",
        31,
        |rng| {
            let mut fields = edge_vectors();
            fields.extend((0..31).map(|_| rng.vec3(50.0)));
            fields
                .into_iter()
                .enumerate()
                .map(|(i, v)| MagneticField {
                    header: input_header(),
                    magnetic_field: vector3(v),
                    magnetic_field_covariance: covariance(rng, 1.0, i),
                })
                .collect()
        },
        oracle::magnetic_field,
    );
}

/// Cloud with the given f32 fields at 4-byte offsets, in that order, plus 4
/// padding bytes per point that are not a field.
fn random_cloud(rng: &mut Rng, names: &[&str], points: usize) -> PointCloud2 {
    let step = 4 * names.len() + 4;
    let fields = names
        .iter()
        .enumerate()
        .map(|(i, name)| PointField {
            name: name.to_string(),
            offset: 4 * i as u32,
            datatype: 7, // FLOAT32
            count: 1,
        })
        .collect();
    let mut data = Vec::with_capacity(step * points);
    for p in 0..points {
        for _ in names {
            // Lidar clouds that are not dense carry NaN points.
            let v = if p % 17 == 3 {
                f32::NAN
            } else if p % 17 == 5 {
                0.0
            } else if p % 17 == 7 {
                -0.0
            } else if p % 17 == 9 {
                f32::INFINITY
            } else {
                rng.range(-100.0, 100.0) as f32
            };
            data.extend_from_slice(&v.to_ne_bytes());
        }
        data.extend_from_slice(&[0xAB, 0xCD, 0xEF, p as u8]);
    }
    PointCloud2 {
        header: input_header(),
        height: 1,
        width: points as u32,
        fields,
        is_bigendian: false,
        point_step: step as u32,
        row_step: (step * points) as u32,
        data,
        is_dense: false,
    }
}

#[test]
fn pointcloud2_matches_tf2() {
    let layouts: [&[&str]; 3] = [
        &["x", "y", "z"],
        &["intensity", "z", "x", "y"],
        &[
            "x",
            "y",
            "z",
            "normal_x",
            "normal_y",
            "normal_z",
            "curvature",
        ],
    ];
    check_parity(
        "PointCloud2",
        15,
        |rng| {
            layouts
                .iter()
                .map(|names| random_cloud(rng, names, 40))
                .collect()
        },
        |cloud, tf| oracle::pointcloud2(cloud, tf).unwrap(),
    );
}

#[test]
fn pointcloud2_errors_match_tf2() {
    let tf = make_tf([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]);
    let mut rng = Rng::new(16);
    for names in [
        &["x", "y"][..],
        &["y", "z"],
        &["x", "y", "z", "normal_x"],
        &["x", "y", "z", "normal_x", "normal_y"],
    ] {
        let cloud = random_cloud(&mut rng, names, 3);
        let got = cloud.apply_transform(&tf);
        let want = oracle::pointcloud2(&cloud, &tf);
        assert!(got.is_err(), "Rust accepted fields {names:?}");
        assert!(want.is_err(), "C++ accepted fields {names:?}");
    }
    // No points: Rust returns an empty cloud. Upstream is undefined there (its
    // iterators take `&data.front()`), so the oracle refuses the cloud.
    let empty = random_cloud(&mut rng, &["x", "y", "z"], 0);
    assert!(empty.apply_transform(&tf).unwrap().data.is_empty());
    assert!(oracle::pointcloud2(&empty, &tf).is_err());
}
