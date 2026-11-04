//! Fixed PX4 full-rate output observer and bounded delayed state histories.
//! Copyright (c) 2022-2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.

use super::{
    imu_ekf::default_float,
    math::{self, GlobalPosition},
    FilterFault, ImuDelta,
};

#[derive(Clone, Copy, Debug)]
pub struct OutputParams {
    pub position_time_constant_s: f32,
    pub velocity_time_constant_s: f32,
    pub imu_offset_body_m: [f32; 3],
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_rate_integrals_and_reset_propagation_preserve_body_outputs() {
        let mut observer = OutputPredictor::new(20, OutputParams::default());
        for time in (2500..=25_000).step_by(2500) {
            observer.predict(&ImuDelta {
                timestamp: time,
                delta_angle_rad: [0.0, 0.0, 0.0025],
                delta_velocity_m_s: [0.0, 0.0, -math::GRAVITY * 0.0025],
                delta_angle_dt_s: 0.0025,
                delta_velocity_dt_s: 0.0025,
                delta_velocity_clipping: [false; 3],
            });
        }
        let angular_velocity = observer.angular_velocity_and_reset();
        assert!((angular_velocity[2] - 1.0).abs() < 3e-6);
        assert_eq!(observer.angular_velocity_and_reset(), [0.0; 3]);
        assert_eq!(observer.velocity_derivative_and_reset(), [0.0; 3]);
        observer.reset_horizontal_velocity([2.0, -1.0]);
        observer.reset_vertical_velocity(0.5);
        observer.reset_altitude(100.0);
        assert_eq!(observer.velocity(), [2.0, -1.0, 0.5]);
        assert_eq!(observer.global_position().altitude_m, 100.0);
        observer.reset_quaternion(math::from_axis_angle([0.0, 0.0, 0.2]));
        assert!(observer.numerics_valid());
        assert!((math::norm(&observer.quaternion()) - 1.0).abs() < 1e-6);
    }
}

impl Default for OutputParams {
    fn default() -> Self {
        Self {
            position_time_constant_s: default_float("EKF2_TAU_POS"),
            velocity_time_constant_s: default_float("EKF2_TAU_VEL"),
            imu_offset_body_m: [0.0; 3],
        }
    }
}

impl OutputParams {
    pub(super) fn validate(self) -> Result<(), FilterFault> {
        if [self.position_time_constant_s, self.velocity_time_constant_s]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.0)
            || self.imu_offset_body_m.iter().any(|v| !v.is_finite())
        {
            return Err(FilterFault::Input);
        }
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct OutputState {
    q: [f32; 4],
    velocity: [f32; 3],
    position: [f32; 3],
}
impl Default for OutputState {
    fn default() -> Self {
        Self {
            q: math::IDENTITY,
            velocity: [0.0; 3],
            position: [0.0; 3],
        }
    }
}
#[derive(Clone, Copy, Default)]
struct VerticalState {
    velocity: f32,
    integral: f32,
    dt: f32,
}

struct History<T: Copy + Default> {
    samples: [T; 64],
    length: usize,
    head: usize,
    tail: usize,
    first: bool,
}
impl<T: Copy + Default> History<T> {
    fn new(length: usize) -> Self {
        Self {
            samples: [T::default(); 64],
            length,
            head: 0,
            tail: 0,
            first: true,
        }
    }
    fn push(&mut self, sample: T) {
        if !self.first {
            self.head = (self.head + 1) % self.length;
        }
        self.samples[self.head] = sample;
        if self.head == self.tail && !self.first {
            self.tail = (self.tail + 1) % self.length;
        } else {
            self.first = false;
        }
    }
    fn newest(&self) -> T {
        self.samples[self.head]
    }
    fn oldest(&self) -> T {
        self.samples[self.tail]
    }
}

pub(super) struct OutputPredictor {
    params: OutputParams,
    output: History<OutputState>,
    vertical: History<VerticalState>,
    new: OutputState,
    new_vertical: VerticalState,
    global_reference: GlobalPosition,
    rotation_now: [[f32; 3]; 3],
    gyro_bias: [f32; 3],
    accel_bias: [f32; 3],
    angle_correction: [f32; 3],
    velocity_error_integral: [f32; 3],
    position_error_integral: [f32; 3],
    tracking_error: [f32; 3],
    imu_relative_velocity: [f32; 3],
    delta_velocity_sum: [f32; 3],
    delta_velocity_time: f32,
    delta_angle_sum: [f32; 4],
    delta_angle_time: f32,
    unaided_yaw: f32,
    update_dt: f32,
    correct_dt: f32,
    last_update: u64,
    last_correction: u64,
}

impl OutputPredictor {
    pub fn new(length: usize, params: OutputParams) -> Self {
        Self {
            params,
            output: History::new(length),
            vertical: History::new(length),
            new: OutputState::default(),
            new_vertical: VerticalState::default(),
            global_reference: GlobalPosition::default(),
            rotation_now: math::rotation(math::IDENTITY),
            gyro_bias: [0.0; 3],
            accel_bias: [0.0; 3],
            angle_correction: [0.0; 3],
            velocity_error_integral: [0.0; 3],
            position_error_integral: [0.0; 3],
            tracking_error: [0.0; 3],
            imu_relative_velocity: [0.0; 3],
            delta_velocity_sum: [0.0; 3],
            delta_velocity_time: 0.0,
            delta_angle_sum: math::IDENTITY,
            delta_angle_time: 0.0,
            unaided_yaw: 0.0,
            update_dt: 0.005,
            correct_dt: 0.01,
            last_update: 0,
            last_correction: 0,
        }
    }

    pub fn align(&mut self, q: [f32; 4], velocity: [f32; 3], global: GlobalPosition) {
        let delayed = self.output.oldest();
        let delta_q = math::normalize(math::multiply(q, math::inverse(delayed.q)));
        let velocity_delta = core::array::from_fn::<_, 3, _>(|i| velocity[i] - delayed.velocity[i]);
        self.global_reference = global;
        for state in &mut self.output.samples[..self.output.length] {
            state.q = math::normalize(math::multiply(delta_q, state.q));
            for (i, velocity_delta) in velocity_delta.iter().enumerate() {
                state.velocity[i] += velocity_delta;
                state.position[i] += -delayed.position[i];
            }
        }
        self.new = self.output.newest();
    }

    pub fn predict(&mut self, imu: &ImuDelta) {
        if self.last_update != 0 {
            let dt = ((imu.timestamp - self.last_update) as f32 * 1e-6).clamp(0.0001, 0.03);
            self.update_dt = 0.8 * self.update_dt + 0.2 * dt;
        }
        self.last_update = imu.timestamp;
        let angle_bias = self.gyro_bias.map(|v| v * imu.delta_angle_dt_s);
        let angle = core::array::from_fn(|i| {
            imu.delta_angle_rad[i] - angle_bias[i] + self.angle_correction[i]
        });
        let velocity = core::array::from_fn(|i| {
            imu.delta_velocity_m_s[i] - self.accel_bias[i] * imu.delta_velocity_dt_s
        });
        let before_q = self.new.q;
        self.new.q = math::normalize(math::multiply(self.new.q, math::from_axis_angle(angle)));
        self.rotation_now = math::rotation(self.new.q);
        let mut delta_earth = math::rotate(self.rotation_now, velocity);
        delta_earth[2] += math::GRAVITY * imu.delta_velocity_dt_s;
        self.delta_velocity_time += imu.delta_velocity_dt_s;
        let previous_velocity = self.new.velocity;
        for i in 0..3 {
            self.delta_velocity_sum[i] += delta_earth[i];
            self.new.velocity[i] += delta_earth[i];
            self.new.position[i] +=
                (self.new.velocity[i] + previous_velocity[i]) * (imu.delta_velocity_dt_s * 0.5);
        }
        self.new_vertical.velocity += delta_earth[2];
        self.new_vertical.integral +=
            (self.new.velocity[2] + previous_velocity[2]) * (imu.delta_velocity_dt_s * 0.5);
        self.new_vertical.dt += imu.delta_velocity_dt_s;
        if imu.delta_angle_dt_s > 0.001 {
            let rate = angle.map(|v| v * (1.0 / imu.delta_angle_dt_s));
            self.imu_relative_velocity = math::rotate(
                self.rotation_now,
                math::cross(rate, self.params.imu_offset_body_m),
            );
        }
        let unaided_delta = math::from_axis_angle(core::array::from_fn(|i| {
            imu.delta_angle_rad[i] - angle_bias[i]
        }));
        let unaided_q = math::multiply(before_q, unaided_delta);
        // OutputPredictor uses Euler 321 yaw, not the 321/312 selection used by the EKF.
        let yaw = |q| {
            let r = math::rotation(q);
            libm::atan2f(r[1][0], r[0][0])
        };
        self.unaided_yaw = math::wrap_pi_f32(self.unaided_yaw + (yaw(unaided_q) - yaw(before_q)));
        self.delta_angle_sum = math::normalize(math::multiply(self.delta_angle_sum, unaided_delta));
        self.delta_angle_time += imu.delta_angle_dt_s;
    }

    pub fn correct(
        &mut self,
        delayed_us: u64,
        q: [f32; 4],
        velocity: [f32; 3],
        global: GlobalPosition,
        gyro_bias: [f32; 3],
        accel_bias: [f32; 3],
    ) {
        if self.last_correction != 0 {
            let dt = ((delayed_us - self.last_correction) as f32 * 1e-6).clamp(0.0001, 0.03);
            self.correct_dt = 0.8 * self.correct_dt + 0.2 * dt;
        }
        self.last_correction = delayed_us;
        self.gyro_bias = gyro_bias;
        self.accel_bias = accel_bias;
        self.output.push(self.new);
        self.vertical.push(self.new_vertical);
        let delayed = self.output.oldest();
        let vertical_delayed = self.vertical.oldest();
        let error = math::normalize(math::multiply(math::inverse(q), delayed.q));
        let scale = if error[0] >= 0.0 { -2.0 } else { 2.0 };
        let angle_error = [scale * error[1], scale * error[2], scale * error[3]];
        let time_delay = ((self.last_update - delayed_us) as f32 * 1e-6).max(self.update_dt);
        let attitude_gain = 0.5 * self.update_dt / time_delay;
        self.angle_correction = angle_error.map(|v| v * attitude_gain);
        self.tracking_error[0] = math::norm(&angle_error);
        let velocity_gain = self.correct_dt
            / self
                .params
                .velocity_time_constant_s
                .clamp(self.correct_dt, 10.0);
        let position_gain = self.correct_dt
            / self
                .params
                .position_time_constant_s
                .clamp(self.correct_dt, 10.0);
        let position = global.difference(self.global_reference);
        let vertical_velocity_error = velocity[2] - vertical_delayed.velocity;
        let vertical_integral_error = position[2] - vertical_delayed.integral;
        let vertical_correction =
            vertical_integral_error * position_gain + vertical_velocity_error * velocity_gain * 1.1;
        self.correct_vertical(vertical_correction, position[2]);
        let velocity_error = core::array::from_fn::<_, 3, _>(|i| velocity[i] - delayed.velocity[i]);
        let position_error = core::array::from_fn::<_, 3, _>(|i| position[i] - delayed.position[i]);
        self.tracking_error[1] = math::norm(&velocity_error);
        self.tracking_error[2] = math::norm(&position_error);
        let mut velocity_correction = [0.0; 3];
        let mut position_correction = [0.0; 3];
        for i in 0..3 {
            self.velocity_error_integral[i] += velocity_error[i];
            self.position_error_integral[i] += position_error[i];
            velocity_correction[i] = velocity_error[i] * velocity_gain
                + (self.velocity_error_integral[i] * math::square(velocity_gain)) * 0.1;
            position_correction[i] = (position_error[i] * position_gain
                + (self.position_error_integral[i] * math::square(position_gain)) * 0.1)
                - position[i];
        }
        for state in &mut self.output.samples[..self.output.length] {
            for i in 0..3 {
                state.velocity[i] += velocity_correction[i];
                state.position[i] += position_correction[i];
            }
        }
        self.new = self.output.newest();
        self.global_reference = global;
    }

    fn correct_vertical(&mut self, correction: f32, reference_change: f32) {
        let mut index = self.vertical.tail;
        for count in 0..self.vertical.length - 1 {
            let next = (index + 1) % self.vertical.length;
            if count == 0 {
                self.vertical.samples[index].velocity += correction;
            }
            self.vertical.samples[next].velocity += correction;
            self.vertical.samples[next].integral = self.vertical.samples[index].integral
                + (self.vertical.samples[index].velocity + self.vertical.samples[next].velocity)
                    * 0.5
                    * self.vertical.samples[next].dt
                - reference_change;
            index = next;
        }
        self.new_vertical = self.vertical.newest();
        self.new_vertical.dt = 0.0;
    }

    pub fn reset_quaternion(&mut self, delta: [f32; 4]) {
        for state in &mut self.output.samples[..self.output.length] {
            state.q = math::multiply(delta, state.q);
        }
        self.new.q = math::multiply(delta, self.new.q);
    }
    pub fn reset_horizontal_velocity(&mut self, delta: [f32; 2]) {
        for state in &mut self.output.samples[..self.output.length] {
            for (i, value) in delta.iter().enumerate() {
                state.velocity[i] += value;
            }
        }
        for (i, value) in delta.iter().enumerate() {
            self.new.velocity[i] += value;
        }
    }
    pub fn reset_vertical_velocity(&mut self, delta: f32) {
        for state in &mut self.output.samples[..self.output.length] {
            state.velocity[2] += delta;
        }
        self.new.velocity[2] += delta;
        for state in &mut self.vertical.samples[..self.vertical.length] {
            state.velocity += delta;
        }
        self.new_vertical.velocity += delta;
    }
    pub fn reset_lat_lon(&mut self, global: GlobalPosition) {
        self.global_reference.latitude_rad = global.latitude_rad;
        self.global_reference.longitude_rad = global.longitude_rad;
    }
    pub fn reset_altitude(&mut self, altitude: f32) {
        self.global_reference.altitude_m = altitude;
    }
    pub fn quaternion(&self) -> [f32; 4] {
        self.new.q
    }
    pub fn velocity(&self) -> [f32; 3] {
        core::array::from_fn(|i| self.new.velocity[i] - self.imu_relative_velocity[i])
    }
    pub fn global_position(&self) -> GlobalPosition {
        let offset = math::rotate(self.rotation_now, self.params.imu_offset_body_m);
        let mut result = self.global_reference;
        result.add_ned(core::array::from_fn(|i| self.new.position[i] - offset[i]));
        result
    }
    pub fn tracking_error(&self) -> [f32; 3] {
        self.tracking_error
    }
    pub fn vertical_derivative(&self) -> f32 {
        self.new_vertical.velocity - self.imu_relative_velocity[2]
    }
    pub fn unaided_yaw(&self) -> f32 {
        self.unaided_yaw
    }
    pub fn angular_velocity_and_reset(&mut self) -> [f32; 3] {
        let value = if self.delta_angle_time > f32::EPSILON {
            math::to_axis_angle(self.delta_angle_sum).map(|v| v * (1.0 / self.delta_angle_time))
        } else {
            [0.0; 3]
        };
        self.delta_angle_sum = math::IDENTITY;
        self.delta_angle_time = 0.0;
        value
    }
    pub fn velocity_derivative_and_reset(&mut self) -> [f32; 3] {
        let value = if self.delta_velocity_time > f32::EPSILON {
            self.delta_velocity_sum
                .map(|v| v * (1.0 / self.delta_velocity_time))
        } else {
            [0.0; 3]
        };
        self.delta_velocity_sum = [0.0; 3];
        self.delta_velocity_time = 0.0;
        value
    }
    pub fn numerics_valid(&self) -> bool {
        self.global_reference.is_finite()
            && self
                .new
                .q
                .iter()
                .chain(&self.new.velocity)
                .chain(&self.new.position)
                .chain(&self.angle_correction)
                .chain(&self.tracking_error)
                .chain(&self.delta_velocity_sum)
                .all(|v| v.is_finite())
            && [
                self.new_vertical.velocity,
                self.new_vertical.integral,
                self.unaided_yaw,
                self.delta_angle_time,
                self.delta_velocity_time,
            ]
            .iter()
            .all(|v| v.is_finite())
            && (math::norm(&self.new.q) - 1.0).abs() < 1e-4
    }
}
