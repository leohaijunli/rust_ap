//! Five-model PX4 EKF-GSF yaw estimator used only by the GNSS extension.
//! Copyright (c) 2020-2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.

use super::{
    gsf_generated,
    math::{self, square},
    ImuDelta,
};

#[derive(Clone, Copy)]
struct Model {
    q: [f32; 4],
    gyro_bias: [f32; 3],
    state: [f32; 3],
    covariance: [[f32; 3]; 3],
    innovation: [f32; 2],
    determinant_inverse: f32,
    nis: f32,
    weight: f32,
}

const EMPTY: Model = Model {
    q: math::IDENTITY,
    gyro_bias: [0.0; 3],
    state: [0.0; 3],
    covariance: [[0.0; 3]; 3],
    innovation: [0.0; 2],
    determinant_inverse: 0.0,
    nis: 0.0,
    weight: 0.0,
};

pub(super) struct GsfYaw {
    models: [Model; 5],
    tilt_aligned: bool,
    active: bool,
    accel: [f32; 3],
    yaw: f32,
    variance: f32,
}

impl GsfYaw {
    pub const fn new() -> Self {
        Self {
            models: [EMPTY; 5],
            tilt_aligned: false,
            active: false,
            accel: [0.0; 3],
            yaw: 0.0,
            variance: f32::INFINITY,
        }
    }
    pub fn reset(&mut self) {
        self.active = false;
        self.variance = f32::INFINITY;
    }
    pub fn yaw(&self) -> f32 {
        self.yaw
    }
    pub fn variance(&self) -> f32 {
        self.variance
    }
    pub fn available(&self) -> bool {
        self.active
            && self.variance > 0.0
            && self.variance < square(0.262)
            && self.variance.is_finite()
    }

    pub fn set_gyro_bias(&mut self, bias: [f32; 3], force: bool) {
        if !self.tilt_aligned || !self.active || force {
            for model in &mut self.models {
                model.gyro_bias = bias;
            }
        }
    }

    fn constrain_covariance(covariance: &mut [[f32; 3]; 3]) {
        covariance[1][0] = covariance[0][1];
        covariance[2][0] = covariance[0][2];
        covariance[2][1] = covariance[1][2];
        for (i, row) in covariance.iter_mut().enumerate() {
            row[i] = row[i].max(1e-6);
        }
    }

    pub fn predict(&mut self, imu: &ImuDelta, in_air: bool) {
        if imu.delta_velocity_dt_s <= 0.001 {
            return;
        }
        let coefficient = (10.0 * imu.delta_velocity_dt_s * 0.2).min(1.0);
        for (i, value) in self.accel.iter_mut().enumerate() {
            *value = *value * (1.0 - coefficient)
                + (imu.delta_velocity_m_s[i] * (1.0 / imu.delta_velocity_dt_s)) * coefficient;
        }
        if !self.tilt_aligned {
            let accel = imu
                .delta_velocity_m_s
                .map(|v| v * (1.0 / imu.delta_velocity_dt_s));
            let norm_squared = |v: [f32; 3]| v.iter().fold(0.0, |sum, x| sum + x * x);
            let low = square(math::GRAVITY * 0.9);
            let high = square(math::GRAVITY * 1.1);
            if !(low..high).contains(&norm_squared(accel))
                || !(low..high).contains(&norm_squared(self.accel))
            {
                return;
            }
            let q = math::align_tilt(imu.delta_velocity_m_s);
            for model in &mut self.models {
                model.q = q;
            }
            self.tilt_aligned = true;
        }
        let accel_norm = math::norm(&self.accel);
        let gain = 0.2
            * square(1.0 - (2.0 * ((accel_norm - math::GRAVITY) / math::GRAVITY).abs()).min(1.0));
        for model in &mut self.models {
            let rate = core::array::from_fn::<_, 3, _>(|i| {
                imu.delta_angle_rad[i] * (1.0 / imu.delta_angle_dt_s.max(0.001))
                    - model.gyro_bias[i]
            });
            let inverse = math::rotation(math::inverse(model.q));
            let gravity = [inverse[0][2], inverse[1][2], inverse[2][2]];
            let correction =
                math::cross(gravity, self.accel).map(|v| (v * gain) * (1.0 / accel_norm));
            if math::norm(&rate) < 10.0_f32.to_radians() {
                for (i, bias) in model.gyro_bias.iter_mut().enumerate() {
                    *bias =
                        (*bias - correction[i] * (0.04 * imu.delta_angle_dt_s)).clamp(-0.05, 0.05);
                }
            }
            let delta = core::array::from_fn(|i| {
                imu.delta_angle_rad[i] + (correction[i] - model.gyro_bias[i]) * imu.delta_angle_dt_s
            });
            model.q = math::normalize(math::multiply(model.q, math::from_axis_angle(delta)));
            if !self.active {
                continue;
            }
            let rotation = math::rotation(model.q);
            model.state[2] = math::yaw(rotation);
            let velocity = math::rotate(rotation, imu.delta_velocity_m_s);
            let (cos, sin) = (libm::cosf(model.state[2]), libm::sinf(model.state[2]));
            let delta_velocity = [
                velocity[0] * cos + velocity[1] * sin,
                -velocity[0] * sin + velocity[1] * cos,
            ];
            let delta_yaw = math::rotate(rotation, imu.delta_angle_rad)[2];
            model.covariance = gsf_generated::predict(
                &model.state,
                &model.covariance,
                delta_velocity,
                square(if in_air { 2.0 } else { 4.0 } * imu.delta_velocity_dt_s),
                delta_yaw,
                square(0.1 * imu.delta_angle_dt_s),
            );
            Self::constrain_covariance(&mut model.covariance);
            model.state[0] += velocity[0];
            model.state[1] += velocity[1];
        }
    }

    pub fn fuse_velocity(&mut self, velocity: [f32; 2], accuracy: f32, in_air: bool) {
        if !self.active {
            self.yaw = 0.0;
            self.variance = square(core::f32::consts::PI / 2.0);
            let increment = 2.0 * core::f32::consts::PI / 5.0;
            for (i, model) in self.models.iter_mut().enumerate() {
                model.state = [
                    velocity[0],
                    velocity[1],
                    -core::f32::consts::PI + 0.5 * increment + i as f32 * increment,
                ];
                model.covariance = [[0.0; 3]; 3];
                model.covariance[0][0] = square(accuracy.max(0.01));
                model.covariance[1][1] = model.covariance[0][0];
                model.covariance[2][2] = square(0.5 * increment);
                model.nis = 0.0;
                model.determinant_inverse = 0.0;
                model.innovation = [0.0; 2];
                model.weight = 1.0 / 5.0;
                model.q = math::from_rotation(math::with_yaw(
                    math::rotation(model.q),
                    math::wrap_pi_f32(model.state[2]),
                ));
            }
            self.active = in_air || math::norm(&velocity) > accuracy;
            return;
        }
        let mut total_weight = 0.0;
        for model in &mut self.models {
            model.innovation = [model.state[0] - velocity[0], model.state[1] - velocity[1]];
            let update =
                gsf_generated::update(&model.covariance, square(accuracy.max(0.01)), f32::EPSILON);
            model.covariance = update.covariance;
            model.determinant_inverse = update.determinant_inverse;
            Self::constrain_covariance(&mut model.covariance);
            let projected = update
                .inverse
                .map(|row| row[0] * model.innovation[0] + row[1] * model.innovation[1]);
            model.nis = model.innovation[0] * projected[0] + model.innovation[1] * projected[1];
            if model.nis > 25.0 {
                let scale = libm::sqrtf(25.0 / model.nis);
                model.innovation = model.innovation.map(|v| v * scale);
                model.nis = 25.0;
            }
            let delta = update
                .gain
                .map(|row| -row[0] * model.innovation[0] - row[1] * model.innovation[1]);
            model.state[0] += delta[0];
            model.state[1] += delta[1];
            model.state[2] = math::wrap_pi_f32(model.state[2] + delta[2]);
            let yaw_delta = [
                libm::cosf(delta[2] / 2.0),
                0.0,
                0.0,
                libm::sinf(delta[2] / 2.0),
            ];
            model.q = math::normalize(math::multiply(yaw_delta, model.q));
            let density = (1.0 / (2.0 * core::f32::consts::PI))
                * libm::sqrtf(model.determinant_inverse)
                * libm::expf(-0.5 * model.nis);
            model.weight = (density * model.weight).max(1e-5);
            total_weight += model.weight;
        }
        let mut vector = [0.0; 2];
        for model in &mut self.models {
            model.weight *= 1.0 / total_weight;
            vector[0] += model.weight * libm::cosf(model.state[2]);
            vector[1] += model.weight * libm::sinf(model.state[2]);
        }
        self.yaw = libm::atan2f(vector[1], vector[0]);
        self.variance = 0.0;
        for model in &self.models {
            let delta = math::wrap_pi_f32(model.state[2] - self.yaw);
            self.variance += model.weight * (model.covariance[2][2] + delta * delta);
        }
        if self.variance <= 0.0 || !self.variance.is_finite() {
            self.reset();
        }
    }
}
