//! Parity of the Rust `Transformable` impls with upstream tf2, checked against the
//! C++ oracle. Run with `cargo test -p tf2_rs_oracle -- --nocapture` to see the
//! error statistics.

use ros_env::geometry_msgs::msg::{
    Point, Pose, PoseWithCovarianceStamped, Quaternion, Vector3, Vector3Stamped,
};
use ros_env::sensor_msgs::msg::Imu;
use tf2_rs::{TransformStamped, Transformable};
use tf2_rs_oracle as oracle;

/// Largest accepted difference, relative to the largest magnitude in the same
/// field (or 1 if that is smaller).
const TOL: f64 = 1e-12;

struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * ((self.next() >> 11) as f64 / (1u64 << 53) as f64)
    }
    fn vec3(&mut self, scale: f64) -> [f64; 3] {
        [0, 1, 2].map(|_| self.range(-scale, scale))
    }
    fn unit_quat(&mut self) -> [f64; 4] {
        let q = [0, 1, 2, 3].map(|_| self.range(-1.0, 1.0));
        let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        q.map(|v| v / n)
    }
}

/// Rotations that exercise every branch of KDL's matrix-to-quaternion
/// conversion, both signs of the same rotation, and the inputs where KDL, tf2 and
/// Eigen differ: quaternions that are not unit length, and zero.
fn edge_rotations(rng: &mut Rng) -> Vec<[f64; 4]> {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let t = 1.0 / 3f64.sqrt();
    let near_pi = (std::f64::consts::PI - 1e-7) / 2.0;
    let r = rng.unit_quat();
    vec![
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [s, s, 0.0, 0.0],
        [t, t, t, 0.0],
        [0.0, 0.0, s, s],
        [0.0, 0.0, -s, s],
        [near_pi.sin(), 0.0, 0.0, near_pi.cos()],
        [0.0, 0.0, (5e-10f64).sin(), (5e-10f64).cos()],
        r.map(|v| -v),
        r.map(|v| v * 0.5),
        r.map(|v| v * 3.0),
        [0.0, 0.0, 0.0, 0.0],
    ]
}

fn make_tf(translation: [f64; 3], rotation: [f64; 4]) -> TransformStamped {
    TransformStamped {
        stamp_sec: 1,
        stamp_nanosec: 2,
        parent_frame: "map".into(),
        child_frame: "sensor".into(),
        translation,
        rotation,
    }
}

/// Transforms: the edge rotations plus random ones, with random translations.
fn transforms(rng: &mut Rng) -> Vec<TransformStamped> {
    let mut out: Vec<_> = edge_rotations(rng)
        .into_iter()
        .map(|q| make_tf(rng.vec3(100.0), q))
        .collect();
    out.extend((0..300).map(|_| make_tf(rng.vec3(100.0), rng.unit_quat())));
    out
}

/// Orientations for the input messages: the edge rotations plus random ones.
fn orientations(rng: &mut Rng) -> Vec<[f64; 4]> {
    let mut out = edge_rotations(rng);
    out.extend((0..20).map(|_| rng.unit_quat()));
    out
}

#[derive(Default)]
struct Stats {
    cases: usize,
    values: usize,
    bit_exact: usize,
    max_rel: f64,
    /// Fields with at least one value that is not bit-identical to C++.
    inexact_fields: std::collections::BTreeSet<String>,
}

impl Stats {
    /// Normwise comparison: each difference is taken relative to the largest
    /// magnitude in the field, so near-zero entries next to large ones do not count
    /// as large relative errors. NaN matches NaN.
    fn check(&mut self, field: &str, got: &[f64], want: &[f64], case: &str) {
        let scale = want
            .iter()
            .filter(|v| v.is_finite())
            .fold(1.0f64, |m, v| m.max(v.abs()));
        for (i, (&g, &w)) in got.iter().zip(want).enumerate() {
            self.values += 1;
            if g.to_bits() == w.to_bits() || (g.is_nan() && w.is_nan()) {
                self.bit_exact += 1;
                continue;
            }
            let rel = (g - w).abs() / scale;
            assert!(
                rel <= TOL,
                "{field}[{i}]: rust {g} vs C++ {w} (rel {rel:e}) for {case}"
            );
            self.max_rel = self.max_rel.max(rel);
            self.inexact_fields.insert(field.to_string());
        }
    }

    fn report(&self, name: &str) {
        println!(
            "{name:<27} cases {:>6}  values {:>7}  bit-exact {:>6.2}%  max normwise rel err {:.1e}",
            self.cases,
            self.values,
            100.0 * self.bit_exact as f64 / self.values as f64,
            self.max_rel
        );
        if !self.inexact_fields.is_empty() {
            println!("{:<27} not bit-exact: {:?}", "", self.inexact_fields);
        }
    }
}

fn pose(p: [f64; 3], q: [f64; 4]) -> Pose {
    let mut out = Pose::default();
    (out.position.x, out.position.y, out.position.z) = (p[0], p[1], p[2]);
    out.orientation = Quaternion {
        x: q[0],
        y: q[1],
        z: q[2],
        w: q[3],
    };
    out
}

fn pose_values(p: &Pose) -> [f64; 7] {
    let (a, o) = (&p.position, &p.orientation);
    [a.x, a.y, a.z, o.x, o.y, o.z, o.w]
}

fn v3(v: &Vector3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

#[test]
fn point_matches_tf2() {
    let mut rng = Rng::new(10);
    let mut stats = Stats::default();
    for tf in transforms(&mut rng) {
        for _ in 0..37 {
            let [x, y, z] = rng.vec3(1000.0);
            let p = Point { x, y, z };
            let (got, want) = (p.apply_transform(&tf).unwrap(), oracle::point(&p, &tf));
            stats.cases += 1;
            stats.check(
                "point",
                &[got.x, got.y, got.z],
                &[want.x, want.y, want.z],
                &format!("{p:?} {tf:?}"),
            );
        }
    }
    stats.report("Point");
}

#[test]
fn vector3_stamped_matches_tf2() {
    let mut rng = Rng::new(11);
    let mut stats = Stats::default();
    for tf in transforms(&mut rng) {
        for _ in 0..37 {
            let mut v = Vector3Stamped::default();
            let [x, y, z] = rng.vec3(1000.0);
            v.vector = Vector3 { x, y, z };
            let (got, want) = (
                v.apply_transform(&tf).unwrap(),
                oracle::vector3_stamped(&v, &tf),
            );
            stats.cases += 1;
            stats.check(
                "vector",
                &v3(&got.vector),
                &v3(&want.vector),
                &format!("{v:?} {tf:?}"),
            );
        }
    }
    stats.report("Vector3Stamped");
}

#[test]
fn pose_matches_tf2() {
    let mut rng = Rng::new(12);
    let mut stats = Stats::default();
    for tf in transforms(&mut rng) {
        for q in orientations(&mut rng) {
            let p = pose(rng.vec3(1000.0), q);
            let (got, want) = (p.apply_transform(&tf).unwrap(), oracle::pose(&p, &tf));
            stats.cases += 1;
            let case = format!("{p:?} {tf:?}");
            let (got, want) = (pose_values(&got), pose_values(&want));
            stats.check("position", &got[..3], &want[..3], &case);
            stats.check("orientation", &got[3..], &want[3..], &case);
        }
    }
    stats.report("Pose");
}

#[test]
fn pose_with_covariance_stamped_matches_tf2() {
    let mut rng = Rng::new(13);
    let mut stats = Stats::default();
    for tf in transforms(&mut rng) {
        for q in orientations(&mut rng) {
            let mut p = PoseWithCovarianceStamped::default();
            p.pose.pose = pose(rng.vec3(1000.0), q);
            p.pose.covariance = std::array::from_fn(|_| rng.range(-10.0, 10.0));
            let (got, want) = (
                p.apply_transform(&tf).unwrap(),
                oracle::pose_with_covariance_stamped(&p, &tf),
            );
            stats.cases += 1;
            let case = format!("{p:?} {tf:?}");
            let (gp, wp) = (pose_values(&got.pose.pose), pose_values(&want.pose.pose));
            stats.check("position", &gp[..3], &wp[..3], &case);
            stats.check("orientation", &gp[3..], &wp[3..], &case);
            stats.check(
                "covariance",
                &got.pose.covariance,
                &want.pose.covariance,
                &case,
            );
        }
    }
    stats.report("PoseWithCovarianceStamped");
}

#[test]
fn imu_matches_tf2() {
    let mut rng = Rng::new(14);
    let mut stats = Stats::default();
    for tf in transforms(&mut rng) {
        for q in orientations(&mut rng) {
            let [ax, ay, az] = rng.vec3(50.0);
            let [lx, ly, lz] = rng.vec3(50.0);
            let imu = Imu {
                orientation: Quaternion {
                    x: q[0],
                    y: q[1],
                    z: q[2],
                    w: q[3],
                },
                orientation_covariance: std::array::from_fn(|_| rng.range(-1.0, 1.0)),
                angular_velocity: Vector3 {
                    x: ax,
                    y: ay,
                    z: az,
                },
                angular_velocity_covariance: std::array::from_fn(|_| rng.range(-1.0, 1.0)),
                linear_acceleration: Vector3 {
                    x: lx,
                    y: ly,
                    z: lz,
                },
                linear_acceleration_covariance: std::array::from_fn(|_| rng.range(-1.0, 1.0)),
                ..Default::default()
            };

            let Some(want) = oracle::imu(&imu, &tf) else {
                println!("Imu: skipped, this distro's tf2_sensor_msgs has no Imu transform");
                return;
            };
            let got = imu.apply_transform(&tf).unwrap();
            stats.cases += 1;
            let case = format!("{imu:?} {tf:?}");
            let o = |q: &Quaternion| [q.x, q.y, q.z, q.w];
            stats.check(
                "orientation",
                &o(&got.orientation),
                &o(&want.orientation),
                &case,
            );
            stats.check(
                "orientation_cov",
                &got.orientation_covariance,
                &want.orientation_covariance,
                &case,
            );
            stats.check(
                "angular_velocity",
                &v3(&got.angular_velocity),
                &v3(&want.angular_velocity),
                &case,
            );
            stats.check(
                "angular_velocity_cov",
                &got.angular_velocity_covariance,
                &want.angular_velocity_covariance,
                &case,
            );
            stats.check(
                "linear_acceleration",
                &v3(&got.linear_acceleration),
                &v3(&want.linear_acceleration),
                &case,
            );
            stats.check(
                "linear_acceleration_cov",
                &got.linear_acceleration_covariance,
                &want.linear_acceleration_covariance,
                &case,
            );
        }
    }
    stats.report("Imu");
}
