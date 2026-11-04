//! Fixed PX4 matrix/LatLonAlt operations used by the IMU core.
//! Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause
//! Source mapping and license: docs/provenance.md, tools/px4_msgs/LICENSE.

pub(super) type Vector3 = [f32; 3];
pub(super) type Quaternion = [f32; 4];
pub(super) type Rotation = [[f32; 3]; 3];
pub(super) const GRAVITY: f32 = 9.80665;
pub(super) const IDENTITY: Quaternion = [1.0, 0.0, 0.0, 0.0];

pub(super) const fn square(value: f32) -> f32 {
    value * value
}

pub(super) fn norm(vector: &[f32]) -> f32 {
    libm::sqrtf(vector.iter().fold(0.0, |sum, value| sum + value * value))
}

pub(super) fn cross(a: Vector3, b: Vector3) -> Vector3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub(super) fn normalize(q: Quaternion) -> Quaternion {
    let inverse_length = 1.0 / norm(&q);
    q.map(|value| value * inverse_length)
}

pub(super) fn multiply(q: Quaternion, p: Quaternion) -> Quaternion {
    [
        q[0] * p[0] - q[1] * p[1] - q[2] * p[2] - q[3] * p[3],
        q[1] * p[0] + q[0] * p[1] - q[3] * p[2] + q[2] * p[3],
        q[2] * p[0] + q[3] * p[1] + q[0] * p[2] - q[1] * p[3],
        q[3] * p[0] - q[2] * p[1] + q[1] * p[2] + q[0] * p[3],
    ]
}

pub(super) fn inverse(q: Quaternion) -> Quaternion {
    let norm_squared = q.iter().fold(0.0, |sum, value| sum + value * value);
    [q[0], -q[1], -q[2], -q[3]].map(|value| value / norm_squared)
}

pub(super) fn from_axis_angle(angle: Vector3) -> Quaternion {
    let length = norm(&angle);
    if length < 1e-10 {
        return IDENTITY;
    }
    let magnitude = libm::sinf(length / 2.0);
    let inverse_length = 1.0 / length;
    [
        libm::cosf(length / 2.0),
        (angle[0] * inverse_length) * magnitude,
        (angle[1] * inverse_length) * magnitude,
        (angle[2] * inverse_length) * magnitude,
    ]
}

pub(super) fn to_axis_angle(q: Quaternion) -> Vector3 {
    let imaginary = [q[1], q[2], q[3]];
    let length = norm(&imaginary);
    let scale = if length >= 1e-10 {
        2.0 * libm::atan2f(length, q[0]) / length
    } else {
        2.0 * q[0].signum()
    };
    imaginary.map(|value| value * scale)
}

pub(super) fn align_tilt(accel: Vector3) -> Quaternion {
    let destination = [0.0, 0.0, -1.0];
    let mut axis = cross(accel, destination);
    let dot = -accel[2];
    let scalar = if norm(&axis) < 1e-5 && dot < 0.0 {
        let absolute = accel.map(f32::abs);
        let index = if absolute[0] < absolute[1] {
            if absolute[0] < absolute[2] {
                0
            } else {
                2
            }
        } else if absolute[1] < absolute[2] {
            1
        } else {
            2
        };
        let mut perpendicular = [0.0; 3];
        perpendicular[index] = 1.0;
        axis = cross(accel, perpendicular);
        0.0
    } else {
        dot + norm(&accel)
    };
    normalize([scalar, axis[0], axis[1], axis[2]])
}

pub(super) fn rotation(q: Quaternion) -> Rotation {
    let [a, b, c, d] = q;
    let (ab, ac, ad) = (a * b, a * c, a * d);
    let (bb, bc, bd) = (b * b, b * c, b * d);
    let (cc, cd, dd) = (c * c, c * d, d * d);
    [
        [1.0 - 2.0 * (cc + dd), 2.0 * (bc - ad), 2.0 * (ac + bd)],
        [2.0 * (bc + ad), 1.0 - 2.0 * (bb + dd), 2.0 * (cd - ab)],
        [2.0 * (bd - ac), 2.0 * (ab + cd), 1.0 - 2.0 * (bb + cc)],
    ]
}

pub(super) fn rotate(r: Rotation, v: Vector3) -> Vector3 {
    r.map(|row| row[0] * v[0] + row[1] * v[1] + row[2] * v[2])
}

pub(super) fn rotate_inverse(r: Rotation, v: Vector3) -> Vector3 {
    core::array::from_fn(|i| r[0][i] * v[0] + r[1][i] * v[1] + r[2][i] * v[2])
}

/// PX4 Utilities chooses 321/312 according to tilt to avoid gimbal lock.
pub(super) fn yaw(r: Rotation) -> f32 {
    if r[2][0].abs() < r[2][1].abs() {
        libm::atan2f(r[1][0], r[0][0])
    } else {
        libm::atan2f(-r[0][1], r[1][1])
    }
}

pub(super) fn wrap_pi_f32(value: f32) -> f32 {
    if (-core::f32::consts::PI..core::f32::consts::PI).contains(&value) {
        value
    } else {
        value
            - core::f32::consts::TAU
                * libm::floorf((value + core::f32::consts::PI) * (1.0 / core::f32::consts::TAU))
    }
}

pub(super) fn with_yaw(r: Rotation, yaw: f32) -> Rotation {
    if r[2][0].abs() < r[2][1].abs() {
        let pitch = libm::asinf(-r[2][0]);
        let roll = if (pitch.abs() - core::f32::consts::FRAC_PI_2).abs() < 1e-3 {
            0.0
        } else {
            libm::atan2f(r[2][1], r[2][2])
        };
        let (cp, sp) = (libm::cosf(roll), libm::sinf(roll));
        let (ct, st) = (libm::cosf(pitch), libm::sinf(pitch));
        let (cy, sy) = (libm::cosf(yaw), libm::sinf(yaw));
        [
            [ct * cy, -cp * sy + sp * st * cy, sp * sy + cp * st * cy],
            [ct * sy, cp * cy + sp * st * sy, -sp * cy + cp * st * sy],
            [-st, sp * ct, cp * ct],
        ]
    } else {
        let roll = libm::asinf(r[2][1]);
        let pitch = libm::atan2f(-r[2][0], r[2][2]);
        let (c0, s0) = (libm::cosf(yaw), libm::sinf(yaw));
        let (c1, s1) = (libm::cosf(roll), libm::sinf(roll));
        let (c2, s2) = (libm::cosf(pitch), libm::sinf(pitch));
        [
            [c0 * c2 - s0 * s1 * s2, -c1 * s0, s2 * c0 + c2 * s1 * s0],
            [c2 * s0 + s2 * s1 * c0, c0 * c1, s0 * s2 - s1 * c0 * c2],
            [-s2 * c1, s1, c2 * c1],
        ]
    }
}

pub(super) fn from_rotation(r: Rotation) -> Quaternion {
    let trace = r[0][0] + r[1][1] + r[2][2];
    if trace > 0.0 {
        let t = libm::sqrtf(1.0 + trace);
        let scale = 0.5 / t;
        [
            0.5 * t,
            (r[2][1] - r[1][2]) * scale,
            (r[0][2] - r[2][0]) * scale,
            (r[1][0] - r[0][1]) * scale,
        ]
    } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
        let t = libm::sqrtf(1.0 + r[0][0] - r[1][1] - r[2][2]);
        let scale = 0.5 / t;
        [
            (r[2][1] - r[1][2]) * scale,
            0.5 * t,
            (r[1][0] + r[0][1]) * scale,
            (r[0][2] + r[2][0]) * scale,
        ]
    } else if r[1][1] > r[2][2] {
        let t = libm::sqrtf(1.0 - r[0][0] + r[1][1] - r[2][2]);
        let scale = 0.5 / t;
        [
            (r[0][2] - r[2][0]) * scale,
            (r[1][0] + r[0][1]) * scale,
            0.5 * t,
            (r[2][1] + r[1][2]) * scale,
        ]
    } else {
        let t = libm::sqrtf(1.0 - r[0][0] - r[1][1] + r[2][2]);
        let scale = 0.5 / t;
        [
            (r[1][0] - r[0][1]) * scale,
            (r[0][2] + r[2][0]) * scale,
            (r[2][1] + r[1][2]) * scale,
            0.5 * t,
        ]
    }
}

/// The fixed core accumulates position in WGS84, not nominal state xy.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct GlobalPosition {
    pub latitude_rad: f64,
    pub longitude_rad: f64,
    pub altitude_m: f32,
}

impl GlobalPosition {
    fn radii(self) -> (f64, f64) {
        const RADIUS: f64 = 6_378_137.0;
        const ECCENTRICITY: f64 = 0.0818191908425;
        let tmp = 1.0 - libm::pow(ECCENTRICITY * libm::sin(self.latitude_rad), 2.0);
        let root = libm::sqrt(tmp);
        (
            RADIUS * (1.0 - ECCENTRICITY * ECCENTRICITY) / (tmp * tmp * root),
            RADIUS / root,
        )
    }

    pub fn add_ned(&mut self, delta: Vector3) {
        let (north, east) = self.radii();
        let east_scale = (east + f64::from(self.altitude_m)) * libm::cos(self.latitude_rad);
        self.latitude_rad =
            wrap_pi(self.latitude_rad + f64::from(delta[0]) / (north + f64::from(self.altitude_m)));
        self.longitude_rad = wrap_pi(self.longitude_rad + f64::from(delta[1]) / east_scale);
        self.altitude_m -= delta[2];
    }

    pub fn difference(self, other: Self) -> Vector3 {
        let (north, east) = self.radii();
        let east_scale = (east + f64::from(self.altitude_m)) * libm::cos(self.latitude_rad);
        [
            (wrap_pi(self.latitude_rad - other.latitude_rad) * (north + f64::from(self.altitude_m)))
                as f32,
            (wrap_pi(self.longitude_rad - other.longitude_rad) * east_scale) as f32,
            -(self.altitude_m - other.altitude_m),
        ]
    }

    pub fn transport_rate(self, velocity: Vector3) -> Vector3 {
        let (north, east) = self.radii();
        [
            velocity[1] / (east as f32 + self.altitude_m),
            -velocity[0] / (north as f32 + self.altitude_m),
            -velocity[1] * libm::tanf(self.latitude_rad as f32) / (east as f32 + self.altitude_m),
        ]
    }

    pub fn is_finite(self) -> bool {
        self.latitude_rad.is_finite()
            && self.longitude_rad.is_finite()
            && self.altitude_m.is_finite()
    }
}

fn wrap_pi(value: f64) -> f64 {
    if (-core::f64::consts::PI..core::f64::consts::PI).contains(&value) {
        value
    } else {
        let wraps = libm::floor((value + core::f64::consts::PI) * (1.0 / core::f64::consts::TAU));
        value - core::f64::consts::TAU * wraps
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotations_keep_hamilton_order_and_handle_inverted_gravity() {
        let yaw = from_axis_angle([0.0, 0.0, core::f32::consts::FRAC_PI_2]);
        let roll = from_axis_angle([core::f32::consts::FRAC_PI_2, 0.0, 0.0]);
        let q = normalize(multiply(yaw, roll));
        for value in q {
            assert!((value - 0.5).abs() < 1e-6);
        }
        assert_ne!(multiply(yaw, roll), multiply(roll, yaw));
        let inverted = rotation(align_tilt([0.0, 0.0, GRAVITY]));
        let rotated = rotate(inverted, [0.0, 0.0, GRAVITY]);
        assert!((rotated[2] + GRAVITY).abs() < 1e-5);
        assert_eq!(wrap_pi(core::f64::consts::PI), -core::f64::consts::PI);
        assert_eq!(wrap_pi(-core::f64::consts::PI), -core::f64::consts::PI);
    }
}
