//! Rotation and covariance math behind the `Transformable` impls.
//!
//! The C++ conversions do not share one implementation: tf2_geometry_msgs moves
//! points and poses with KDL and rotates covariances with tf2's `Matrix3x3`, while
//! tf2_sensor_msgs uses Eigen. The three build the rotation matrix differently when
//! the quaternion is not unit length (KDL and Eigen do not normalise it, tf2 does),
//! so each one is mirrored here and named after the library it follows. The order
//! of operations is kept too, Eigen's vectorised one included: a sum rounded in
//! another order can differ in the last bit.

pub(crate) type Vec3 = [f64; 3];
pub(crate) type Mat3 = [[f64; 3]; 3];
/// Quaternion as `[x, y, z, w]`, the order of `TransformStamped::rotation`.
pub(crate) type Quat = [f64; 4];

pub(crate) fn mat_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    out
}

pub(crate) fn mat_vec(m: &Mat3, v: Vec3) -> Vec3 {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

pub(crate) fn transpose(m: &Mat3) -> Mat3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

/// `KDL::Rotation::Quaternion`. Not normalised: a non-unit quaternion scales the
/// matrix by its squared norm.
pub(crate) fn kdl_rotation(q: Quat) -> Mat3 {
    let [x, y, z, w] = q;
    let (x2, y2, z2, w2) = (x * x, y * y, z * z, w * w);
    [
        [
            w2 + x2 - y2 - z2,
            2.0 * x * y - 2.0 * w * z,
            2.0 * x * z + 2.0 * w * y,
        ],
        [
            2.0 * x * y + 2.0 * w * z,
            w2 - x2 + y2 - z2,
            2.0 * y * z - 2.0 * w * x,
        ],
        [
            2.0 * x * z - 2.0 * w * y,
            2.0 * y * z + 2.0 * w * x,
            w2 - x2 - y2 + z2,
        ],
    ]
}

/// `KDL::Rotation::GetQuaternion`. The branch decides the sign of the result, so
/// it is kept as is rather than replaced by an equivalent formula.
pub(crate) fn kdl_quaternion(m: &Mat3) -> Quat {
    let trace = m[0][0] + m[1][1] + m[2][2];
    if trace > 1e-12 {
        let s = 0.5 / (trace + 1.0).sqrt();
        [
            (m[2][1] - m[1][2]) * s,
            (m[0][2] - m[2][0]) * s,
            (m[1][0] - m[0][1]) * s,
            0.25 / s,
        ]
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = 2.0 * (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt();
        [
            0.25 * s,
            (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[2][1] - m[1][2]) / s,
        ]
    } else if m[1][1] > m[2][2] {
        let s = 2.0 * (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt();
        [
            (m[0][1] + m[1][0]) / s,
            0.25 * s,
            (m[1][2] + m[2][1]) / s,
            (m[0][2] - m[2][0]) / s,
        ]
    } else {
        let s = 2.0 * (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt();
        [
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s,
            (m[1][0] - m[0][1]) / s,
        ]
    }
}

/// `tf2::Matrix3x3::setRotation`. Normalised through `s = 2 / |q|²`.
pub(crate) fn tf2_rotation(q: Quat) -> Mat3 {
    let [x, y, z, w] = q;
    let s = 2.0 / (x * x + y * y + z * z + w * w);
    let (xs, ys, zs) = (x * s, y * s, z * s);
    let (wx, wy, wz) = (w * xs, w * ys, w * zs);
    let (xx, xy, xz) = (x * xs, x * ys, x * zs);
    let (yy, yz, zz) = (y * ys, y * zs, z * zs);
    [
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ]
}

/// `Eigen::Quaternion::toRotationMatrix`. Assumes a unit quaternion and does not
/// normalise.
pub(crate) fn eigen_rotation(q: Quat) -> Mat3 {
    let [x, y, z, w] = q;
    let (tx, ty, tz) = (2.0 * x, 2.0 * y, 2.0 * z);
    let (twx, twy, twz) = (tx * w, ty * w, tz * w);
    let (txx, txy, txz) = (tx * x, ty * x, tz * x);
    let (tyy, tyz, tzz) = (ty * y, tz * y, tz * z);
    [
        [1.0 - (tyy + tzz), txy - twz, txz + twy],
        [txy + twz, 1.0 - (txx + tzz), tyz - twx],
        [txz - twy, tyz + twx, 1.0 - (txx + tyy)],
    ]
}

/// Hamilton product `a * b`, summed left to right as `tf2::Quaternion`'s
/// `operator*` does. Only the tests use it until the Quaternion transform lands.
#[cfg(test)]
pub(crate) fn quat_mul(a: Quat, b: Quat) -> Quat {
    let [ax, ay, az, aw] = a;
    let [bx, by, bz, bw] = b;
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by + ay * bw + az * bx - ax * bz,
        aw * bz + az * bw + ax * by - ay * bx,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

/// `Eigen::Quaterniond` product `a * b`. Eigen computes it two lanes at a time
/// (Geometry_SIMD.h), which groups the terms differently from `quat_mul`.
pub(crate) fn eigen_quat_mul(a: Quat, b: Quat) -> Quat {
    let [ax, ay, az, aw] = a;
    let [bx, by, bz, bw] = b;
    [
        (aw * bx + ay * bz) - (az * by - ax * bw),
        (aw * by + ay * bw) + (az * bx - ax * bz),
        (aw * bz - ay * bx) + (az * bw + ax * by),
        (aw * bw - ay * by) - (az * bz + ax * bx),
    ]
}

/// `Eigen::Quaternion::inverse`: conjugate over the squared norm, or all zeros
/// for a zero quaternion. Eigen sums the squares two lanes at a time, so the
/// norm is `(x² + z²) + (y² + w²)`.
pub(crate) fn eigen_inverse(q: Quat) -> Quat {
    let [x, y, z, w] = q;
    let n2 = (x * x + z * z) + (y * y + w * w);
    if n2 > 0.0 {
        [-x / n2, -y / n2, -z / n2, w / n2]
    } else {
        [0.0; 4]
    }
}

/// Rotates a row-major 6x6 pose covariance as tf2_geometry_msgs'
/// `transformCovariance`: each 3x3 block becomes `R * block * Rᵀ`.
pub(crate) fn rotate_covariance6(cov: &[f64; 36], r: &Mat3) -> [f64; 36] {
    let rt = transpose(r);
    let mut out = [0.0; 36];
    for (row0, col0) in [(0, 0), (0, 3), (3, 0), (3, 3)] {
        let mut block = [[0.0; 3]; 3];
        for (i, row) in block.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                *v = cov[(row0 + i) * 6 + col0 + j];
            }
        }
        let rotated = mat_mul(&mat_mul(r, &block), &rt);
        for (i, row) in rotated.iter().enumerate() {
            for (j, v) in row.iter().enumerate() {
                out[(row0 + i) * 6 + col0 + j] = *v;
            }
        }
    }
    out
}

/// `Eigen::Matrix3d` product `a * b`. Eigen computes rows 0 and 1 as one
/// two-lane packet, summed left to right, and row 2 on its own, summed as
/// `a0 + (a1 + a2)`.
pub(crate) fn eigen_mat_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            let (p0, p1, p2) = (a[i][0] * b[0][j], a[i][1] * b[1][j], a[i][2] * b[2][j]);
            *v = if i < 2 {
                (p0 + p1) + p2
            } else {
                p0 + (p1 + p2)
            };
        }
    }
    out
}

/// Rotates a row-major 3x3 covariance as tf2_sensor_msgs' `transformCovariance`:
/// `r * cov * r.inverse()` with Eigen quaternions.
pub(crate) fn rotate_covariance3(cov: &[f64; 9], r: Quat) -> [f64; 9] {
    let c = [
        [cov[0], cov[1], cov[2]],
        [cov[3], cov[4], cov[5]],
        [cov[6], cov[7], cov[8]],
    ];
    let m = eigen_mat_mul(
        &eigen_mat_mul(&eigen_rotation(r), &c),
        &eigen_rotation(eigen_inverse(r)),
    );
    [
        m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2],
    ]
}
