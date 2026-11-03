//! Shared by the parity tests: inputs, the comparison with the oracle and the
//! statistics. Each test binary uses only part of it.
#![allow(dead_code)]

use ros_env::builtin_interfaces::msg::Time;
use ros_env::geometry_msgs::msg::{
    Point, Point32, PointStamped, Polygon, PolygonStamped, Pose, PoseStamped, PoseWithCovariance,
    PoseWithCovarianceStamped, Quaternion, QuaternionStamped, Transform,
    TransformStamped as TransformStampedMsg, Twist, Vector3, Vector3Stamped, VelocityStamped,
    Wrench, WrenchStamped,
};
use ros_env::std_msgs::msg::Header;
use std::fmt::Debug;
use tf2_rs::{TransformStamped, Transformable};

/// Whether the C++ results are expected to match to the last bit. Only with the
/// x86-64 baseline: where GCC merges a multiply and an add into one FMA instruction
/// (the aarch64 default, or `-mfma` / `-march=native` here), upstream rounds
/// differently, in a way that depends on the compiler.
pub const BIT_EXACT: bool = cfg!(target_arch = "x86_64");

/// Without `BIT_EXACT`, the largest accepted difference, relative to the largest
/// magnitude in the same field (or 1 if that is smaller). It allows for FMA in `f32`.
const TOL: f64 = 1e-6;

pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * ((self.next() >> 11) as f64 / (1u64 << 53) as f64)
    }
    pub fn vec3(&mut self, scale: f64) -> [f64; 3] {
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
pub fn edge_rotations(rng: &mut Rng) -> Vec<[f64; 4]> {
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

pub fn make_tf(translation: [f64; 3], rotation: [f64; 4]) -> TransformStamped {
    TransformStamped {
        stamp_sec: 1,
        stamp_nanosec: 2,
        parent_frame: "map".into(),
        child_frame: "sensor".into(),
        translation,
        rotation,
    }
}

/// Transforms: the edge rotations, with a random translation and with none as in many
/// static frames, plus random ones.
pub fn transforms(rng: &mut Rng) -> Vec<TransformStamped> {
    let mut out: Vec<_> = edge_rotations(rng)
        .into_iter()
        .flat_map(|q| [make_tf(rng.vec3(100.0), q), make_tf([0.0; 3], q)])
        .collect();
    out.extend((0..300).map(|_| make_tf(rng.vec3(100.0), rng.unit_quat())));
    out
}

/// Orientations for the input messages: the edge rotations plus random ones.
pub fn orientations(rng: &mut Rng) -> Vec<[f64; 4]> {
    let mut out = edge_rotations(rng);
    out.extend((0..20).map(|_| rng.unit_quat()));
    out
}

/// Exact values random inputs never reach, and real sensors do report.
pub fn edge_vectors() -> Vec<[f64; 3]> {
    vec![
        [0.0, 0.0, 0.0],
        [-0.0, -0.0, -0.0],
        [0.0, -0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [1e-310, -1e-310, 0.0],
        [1e300, -1e300, 1.0],
        [f64::INFINITY, 1.0, 2.0],
        [f64::NAN, 1.0, 2.0],
    ]
}

/// `n` random vectors, then the edge ones.
pub fn vectors(rng: &mut Rng, n: usize, scale: f64) -> Vec<[f64; 3]> {
    let mut out: Vec<_> = (0..n).map(|_| rng.vec3(scale)).collect();
    out.extend(edge_vectors());
    out
}

/// Random entries, or the values drivers fill in when they have no estimate.
pub fn covariance<const N: usize>(rng: &mut Rng, scale: f64, turn: usize) -> [f64; N] {
    match turn % 8 {
        0 => [0.0; N],
        1 => [-1.0; N],
        _ => std::array::from_fn(|_| rng.range(-scale, scale)),
    }
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
    /// Compares the bits of every value, NaN included. Without `BIT_EXACT`, the
    /// comparison is normwise: each difference is taken relative to the largest
    /// magnitude in the field, so near-zero entries next to large ones do not count
    /// as large relative errors, and NaN matches NaN.
    fn check(&mut self, field: &str, got: &[f64], want: &[f64], case: &str) {
        let scale = want
            .iter()
            .filter(|v| v.is_finite())
            .fold(1.0f64, |m, v| m.max(v.abs()));
        for (i, (&g, &w)) in got.iter().zip(want).enumerate() {
            self.values += 1;
            if g.to_bits() == w.to_bits() {
                self.bit_exact += 1;
                continue;
            }
            if BIT_EXACT {
                panic!(
                    "{field}[{i}]: rust {g} ({:#x}) vs C++ {w} ({:#x}) for {case}",
                    g.to_bits(),
                    w.to_bits()
                );
            }
            if g.is_nan() && w.is_nan() {
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

/// A whole message in comparable form. The impls destructure the message, so a
/// field added to it does not compile until it is compared too.
pub trait Flatten {
    /// The floating-point fields, compared bit by bit.
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)>;
    /// Everything else, compared as text.
    fn other(&self) -> Vec<(&'static str, String)>;
}

pub fn input_header() -> Header {
    Header {
        stamp: Time { sec: 7, nanosec: 8 },
        frame_id: "input".into(),
    }
}

impl Flatten for Point {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let Point { x, y, z } = self;
        vec![("point", vec![*x, *y, *z])]
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        vec![]
    }
}

impl Flatten for Vector3 {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let Vector3 { x, y, z } = self;
        vec![("vector", vec![*x, *y, *z])]
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        vec![]
    }
}

impl Flatten for Vector3Stamped {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let Vector3Stamped { header: _, vector } = self;
        vector.fields()
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        let Vector3Stamped { header, vector: _ } = self;
        vec![("header", format!("{header:?}"))]
    }
}

impl Flatten for Quaternion {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let Quaternion { x, y, z, w } = self;
        vec![("orientation", vec![*x, *y, *z, *w])]
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        vec![]
    }
}

impl Flatten for Pose {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let Pose {
            position,
            orientation,
        } = self;
        let Point { x, y, z } = position;
        let mut out = vec![("position", vec![*x, *y, *z])];
        out.extend(orientation.fields());
        out
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        vec![]
    }
}

impl Flatten for PoseWithCovariance {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let PoseWithCovariance { pose, covariance } = self;
        let mut out = pose.fields();
        out.push(("covariance", covariance.to_vec()));
        out
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        vec![]
    }
}

impl Flatten for PoseWithCovarianceStamped {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let PoseWithCovarianceStamped { header: _, pose } = self;
        pose.fields()
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        let PoseWithCovarianceStamped { header, pose: _ } = self;
        vec![("header", format!("{header:?}"))]
    }
}

impl Flatten for PointStamped {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let PointStamped { header: _, point } = self;
        point.fields()
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        let PointStamped { header, point: _ } = self;
        vec![("header", format!("{header:?}"))]
    }
}

impl Flatten for Point32 {
    /// Widened to f64, which keeps every value apart.
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let Point32 { x, y, z } = self;
        vec![("point32", vec![*x as f64, *y as f64, *z as f64])]
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        vec![]
    }
}

impl Flatten for Polygon {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let Polygon { points } = self;
        let coordinates = points
            .iter()
            .flat_map(|p| p.fields())
            .flat_map(|(_, values)| values)
            .collect();
        vec![("points", coordinates)]
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        vec![]
    }
}

impl Flatten for PolygonStamped {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let PolygonStamped { header: _, polygon } = self;
        polygon.fields()
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        let PolygonStamped { header, polygon: _ } = self;
        vec![("header", format!("{header:?}"))]
    }
}

impl Flatten for QuaternionStamped {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let QuaternionStamped {
            header: _,
            quaternion,
        } = self;
        quaternion.fields()
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        let QuaternionStamped {
            header,
            quaternion: _,
        } = self;
        vec![("header", format!("{header:?}"))]
    }
}

impl Flatten for PoseStamped {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let PoseStamped { header: _, pose } = self;
        pose.fields()
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        let PoseStamped { header, pose: _ } = self;
        vec![("header", format!("{header:?}"))]
    }
}

impl Flatten for Transform {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let Transform {
            translation,
            rotation,
        } = self;
        let mut out = vec![("translation", vector3_values(translation))];
        out.extend(rotation.fields());
        out
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        vec![]
    }
}

impl Flatten for TransformStampedMsg {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let TransformStampedMsg {
            header: _,
            child_frame_id: _,
            transform,
        } = self;
        transform.fields()
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        let TransformStampedMsg {
            header,
            child_frame_id,
            transform: _,
        } = self;
        vec![
            ("header", format!("{header:?}")),
            ("child_frame_id", child_frame_id.clone()),
        ]
    }
}

impl Flatten for Wrench {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let Wrench { force, torque } = self;
        vec![
            ("force", vector3_values(force)),
            ("torque", vector3_values(torque)),
        ]
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        vec![]
    }
}

impl Flatten for WrenchStamped {
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let WrenchStamped { header: _, wrench } = self;
        wrench.fields()
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        let WrenchStamped { header, wrench: _ } = self;
        vec![("header", format!("{header:?}"))]
    }
}

impl Flatten for VelocityStamped {
    /// All NaNs compare equal here, whatever their sign. A zero quaternion gives a
    /// NaN matrix (tf2 divides by |q|²), and when it meets a NaN input the sign of
    /// the result depends on how upstream is compiled: -O0, -O1 and -O2 all differ.
    fn fields(&self) -> Vec<(&'static str, Vec<f64>)> {
        let Twist { linear, angular } = &self.velocity;
        let unsigned_nan = |v: &Vector3| {
            let mut values = vector3_values(v);
            for value in values.iter_mut().filter(|value| value.is_nan()) {
                *value = f64::NAN;
            }
            values
        };
        vec![
            ("linear", unsigned_nan(linear)),
            ("angular", unsigned_nan(angular)),
        ]
    }
    fn other(&self) -> Vec<(&'static str, String)> {
        let VelocityStamped {
            header,
            body_frame_id,
            reference_frame_id,
            velocity: _,
        } = self;
        vec![
            ("header", format!("{header:?}")),
            ("body_frame_id", body_frame_id.clone()),
            ("reference_frame_id", reference_frame_id.clone()),
        ]
    }
}

/// Compares the Rust impl with `oracle` for every transform of `transforms()` and
/// every input `inputs` returns for it, then prints the statistics.
pub fn check_parity<T: Transformable + Flatten + Debug>(
    name: &str,
    seed: u64,
    inputs: impl FnMut(&mut Rng) -> Vec<T>,
    oracle: impl Fn(&T, &TransformStamped) -> T,
) {
    check_parity_if_available(name, seed, inputs, |msg, tf| Some(oracle(msg, tf)));
}

/// `check_parity` for an oracle that returns `None` where the distro has no such
/// transform upstream. The test is then skipped.
pub fn check_parity_if_available<T: Transformable + Flatten + Debug>(
    name: &str,
    seed: u64,
    mut inputs: impl FnMut(&mut Rng) -> Vec<T>,
    oracle: impl Fn(&T, &TransformStamped) -> Option<T>,
) {
    let mut rng = Rng::new(seed);
    let mut stats = Stats::default();
    for tf in transforms(&mut rng) {
        for input in inputs(&mut rng) {
            let Some(want) = oracle(&input, &tf) else {
                println!("{name}: skipped, this distro has no such transform upstream");
                return;
            };
            let got = input.apply_transform(&tf).unwrap();
            stats.cases += 1;
            let case = format!("{input:?} {tf:?}");
            let (got_fields, want_fields) = (got.fields(), want.fields());
            assert_eq!(got_fields.len(), want_fields.len(), "fields for {case}");
            for ((field, g), (_, w)) in got_fields.iter().zip(&want_fields) {
                assert_eq!(g.len(), w.len(), "length of {field} for {case}");
                stats.check(field, g, w, &case);
            }
            assert_eq!(got.other(), want.other(), "for {case}");
        }
    }
    stats.report(name);
}

pub fn vector3_values(v: &Vector3) -> Vec<f64> {
    let Vector3 { x, y, z } = v;
    vec![*x, *y, *z]
}

pub fn point([x, y, z]: [f64; 3]) -> Point {
    Point { x, y, z }
}

pub fn point32([x, y, z]: [f64; 3]) -> Point32 {
    Point32 {
        x: x as f32,
        y: y as f32,
        z: z as f32,
    }
}

pub fn vector3([x, y, z]: [f64; 3]) -> Vector3 {
    Vector3 { x, y, z }
}

pub fn quaternion([x, y, z, w]: [f64; 4]) -> Quaternion {
    Quaternion { x, y, z, w }
}

pub fn pose(p: [f64; 3], q: [f64; 4]) -> Pose {
    Pose {
        position: point(p),
        orientation: quaternion(q),
    }
}
