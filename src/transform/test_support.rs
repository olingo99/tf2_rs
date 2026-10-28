//! Helpers shared by the transform unit tests.

use crate::TransformStamped;

/// Small deterministic generator (xorshift64), so failures are reproducible.
pub(crate) struct Rng(u64);

impl Rng {
    pub(crate) fn new(seed: u64) -> Self {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }

    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    pub(crate) fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * ((self.next() >> 11) as f64 / (1u64 << 53) as f64)
    }

    pub(crate) fn vec3(&mut self, scale: f64) -> [f64; 3] {
        [0, 1, 2].map(|_| self.range(-scale, scale))
    }

    pub(crate) fn unit_quat(&mut self) -> [f64; 4] {
        let q = [0, 1, 2, 3].map(|_| self.range(-1.0, 1.0));
        let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        q.map(|v| v / n)
    }
}

pub(crate) fn make_tf(translation: [f64; 3], rotation: [f64; 4]) -> TransformStamped {
    TransformStamped {
        stamp_sec: 0,
        stamp_nanosec: 0,
        parent_frame: "map".to_string(),
        child_frame: "sensor".to_string(),
        translation,
        rotation,
    }
}

/// Rotates `v` by the unit quaternion `q` as `q * (v, 0) * q*`, independently of
/// the matrix formulas in `math`.
pub(crate) fn rotate_by_quat(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let mul = |a: [f64; 4], b: [f64; 4]| {
        [
            a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
            a[3] * b[1] + a[1] * b[3] + a[2] * b[0] - a[0] * b[2],
            a[3] * b[2] + a[2] * b[3] + a[0] * b[1] - a[1] * b[0],
            a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
        ]
    };
    let r = mul(mul(q, [v[0], v[1], v[2], 0.0]), [-q[0], -q[1], -q[2], q[3]]);
    [r[0], r[1], r[2]]
}

pub(crate) fn assert_close(got: &[f64], expected: &[f64], tol: f64, what: &str) {
    assert_eq!(got.len(), expected.len(), "{what}: length");
    for (i, (g, e)) in got.iter().zip(expected).enumerate() {
        assert!(
            (g - e).abs() <= tol,
            "{what}[{i}]: got {g}, expected {e} (tol {tol})"
        );
    }
}
