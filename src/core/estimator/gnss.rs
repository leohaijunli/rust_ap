//! GNSS extension of the shared PX4 delayed core. No GNSS types enter the no-GPS kernel.
//! Copyright (c) 2021-2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.

use super::{
    bias_estimator::BiasEstimator,
    gnss_checks::{GnssCheckParams, GnssChecks},
    gsf::GsfYaw,
    imu_ekf::{
        baro_mag::{AidStatus, BaroMagParams, BaroMagSample},
        default_float, default_int,
        range::RangeParams,
        ControlExtension, ImuEkf, ImuEkfParams,
    },
    math::{self, GlobalPosition},
    observation_buffer::ObservationBuffer,
    range_sensor::RangeObservation,
    FilterFault, GnssSample, ImuDelta,
};

const POSITION: u64 = 1 << 2;
const VELOCITY: u64 = 1 << 44;
const HEIGHT: u64 = 1 << 11;
const GNSS_FAULT: u64 = 1 << 45;
const HEIGHT_FAULT: u64 = 1 << 47;

#[derive(Clone, Copy, Debug, Default)]
pub struct GnssObservation {
    pub sample: GnssSample,
    pub pdop: f32,
    pub spoofed: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn antenna_lever_arm_corrects_position_height_and_rotational_velocity() {
        let params = GnssParams {
            offset_body_m: [1.0, 0.0, 0.5],
            ..GnssParams::default()
        };
        let mut aids = GnssAids::new(params, 20).unwrap();
        let mut core = ImuEkf::new(&ImuEkfParams::default(), 0).unwrap();
        let sample = GnssSample {
            timestamp: 1_000_000,
            latitude_deg: 47.0,
            longitude_deg: 8.0,
            altitude_msl_m: 100.0,
            velocity_ned_m_s: [0.0, 1.0, 0.0],
            horizontal_accuracy_m: 0.3,
            vertical_accuracy_m: 0.5,
            speed_accuracy_m_s: 0.1,
            fix_type: 3,
            satellites: 12,
        };
        core.global = GlobalPosition {
            latitude_rad: 47.0_f64.to_radians(),
            longitude_rad: 8.0_f64.to_radians(),
            altitude_m: 100.5,
        };
        aids.last_sample = GnssObservation {
            sample,
            pdop: 1.0,
            spoofed: false,
        };
        let imu = ImuDelta {
            timestamp: 1_000_000,
            delta_angle_rad: [0.0, 0.0, 0.01],
            delta_angle_dt_s: 0.01,
            delta_velocity_dt_s: 0.01,
            delta_velocity_m_s: [0.0; 3],
            delta_velocity_clipping: [false; 3],
        };
        aids.update_observations(&core, &imu);
        assert!((aids.position.innovation[0] - 1.0).abs() < 1e-6);
        assert_eq!(aids.position_observation.altitude_m, 100.5);
        assert_eq!(aids.velocity_observation, [0.0; 3]);
    }

    #[test]
    fn invalid_gnss_input_latches_until_explicit_profile_reset() {
        assert!(GnssEkf::new(
            ImuEkfParams::default(),
            BaroMagParams::default(),
            RangeParams::default(),
            GnssParams {
                mode: 1,
                ..GnssParams::default()
            },
            0
        )
        .is_err());
        let mut filter = GnssEkf::new(
            ImuEkfParams::default(),
            BaroMagParams::default(),
            RangeParams::default(),
            GnssParams::default(),
            0,
        )
        .unwrap();
        let imu = ImuDelta {
            timestamp: 2500,
            delta_angle_rad: [0.0; 3],
            delta_angle_dt_s: 0.0025,
            delta_velocity_dt_s: 0.0025,
            delta_velocity_m_s: [0.0, 0.0, -math::GRAVITY * 0.0025],
            delta_velocity_clipping: [false; 3],
        };
        let sample = GnssSample {
            timestamp: 2500,
            latitude_deg: f64::NAN,
            ..GnssSample::default()
        };
        assert_eq!(
            filter.update(
                &imu,
                None,
                Some(GnssObservation {
                    sample,
                    pdop: 1.0,
                    spoofed: false
                }),
                None
            ),
            Err(FilterFault::Input)
        );
        assert!(!filter.core().health().valid);
        filter.reset(0).unwrap();
        assert!(filter.update(&imu, None, None, None).is_ok());
    }
}

#[derive(Clone, Copy, Debug)]
pub struct GnssParams {
    pub control: i32,
    pub mode: i32,
    pub delay_ms: f32,
    pub position_noise_m: f32,
    pub velocity_noise_m_s: f32,
    pub position_gate: f32,
    pub velocity_gate: f32,
    pub checks: GnssCheckParams,
    /// Antenna-minus-IMU lever arm in body FRD metres.
    pub offset_body_m: [f32; 3],
}

impl Default for GnssParams {
    fn default() -> Self {
        Self {
            control: default_int("EKF2_GPS_CTRL"),
            mode: default_int("EKF2_GPS_MODE"),
            delay_ms: default_float("EKF2_GPS_DELAY"),
            position_noise_m: default_float("EKF2_GPS_P_NOISE"),
            velocity_noise_m_s: default_float("EKF2_GPS_V_NOISE"),
            position_gate: default_float("EKF2_GPS_P_GATE"),
            velocity_gate: default_float("EKF2_GPS_V_GATE"),
            checks: GnssCheckParams::default(),
            offset_body_m: [0.0; 3],
        }
    }
}

/// Main five-sensor instance; sensor sharing is by copied read-only DTO only.
pub struct GnssEkf {
    core: ImuEkf,
    aids: GnssAids,
    params: GnssParams,
    imu_params: ImuEkfParams,
    baro_mag_params: BaroMagParams,
    range_params: RangeParams,
    output_params: super::output_predictor::OutputParams,
}

impl GnssEkf {
    pub fn new(
        imu: ImuEkfParams,
        baro_mag: BaroMagParams,
        range: RangeParams,
        params: GnssParams,
        t0: u64,
    ) -> Result<Self, FilterFault> {
        let mut core = ImuEkf::with_range(&imu, baro_mag, range, t0)?;
        core.initial_horizontal_noise(params.position_noise_m, params.velocity_noise_m_s);
        let length =
            libm::ceilf(imu.delay_max_ms / (imu.predict_us as f32 / 1000.0)).max(2.0) as usize;
        Ok(Self {
            core,
            aids: GnssAids::new(params, length)?,
            params,
            imu_params: imu,
            baro_mag_params: baro_mag,
            range_params: range,
            output_params: super::output_predictor::OutputParams::default(),
        })
    }

    pub fn reset(&mut self, t0: u64) -> Result<(), FilterFault> {
        let output_params = self.output_params;
        *self = Self::new(
            self.imu_params,
            self.baro_mag_params,
            self.range_params,
            self.params,
            t0,
        )?;
        self.configure_output(output_params)?;
        Ok(())
    }

    pub fn core(&self) -> &ImuEkf {
        &self.core
    }
    pub fn configure_output(
        &mut self,
        params: super::output_predictor::OutputParams,
    ) -> Result<(), FilterFault> {
        self.core.configure_output(params)?;
        self.output_params = params;
        Ok(())
    }
    pub fn status(&self) -> [AidStatus; 3] {
        [self.aids.position, self.aids.velocity, self.aids.height]
    }
    pub fn quality_failure_bits(&self) -> u16 {
        self.aids.checks.failure_bits()
    }

    pub fn update(
        &mut self,
        imu: &ImuDelta,
        baro_mag: Option<BaroMagSample>,
        gnss: Option<GnssObservation>,
        range: Option<RangeObservation>,
    ) -> Result<bool, FilterFault> {
        self.aids.pending = gnss;
        self.core
            .update_extended(imu, baro_mag, range, &mut self.aids)
    }
}

struct GnssAids {
    params: GnssParams,
    buffer: ObservationBuffer<GnssObservation>,
    checks: GnssChecks,
    yaw: GsfYaw,
    pending: Option<GnssObservation>,
    last_sample: GnssObservation,
    started: bool,
    data_ready: bool,
    last_input: u64,
    last_push: u64,
    position: AidStatus,
    velocity: AidStatus,
    height: AidStatus,
    position_observation: GlobalPosition,
    velocity_observation: [f32; 3],
    position_variance: f32,
    velocity_variance: [f32; 3],
    last_position_fuse: u64,
    last_velocity_fuse: u64,
    last_height_fuse: u64,
    height_bias: BiasEstimator,
}

impl GnssAids {
    fn new(params: GnssParams, length: usize) -> Result<Self, FilterFault> {
        if params.control & !7 != 0
            || params.mode != 0 // Only the Auto reset policy is implemented in this profile.
            || [
                params.delay_ms,
                params.position_noise_m,
                params.velocity_noise_m_s,
                params.position_gate,
                params.velocity_gate,
            ]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.0)
            || params.offset_body_m.iter().any(|v| !v.is_finite())
        {
            return Err(FilterFault::Input);
        }
        Ok(Self {
            params,
            buffer: ObservationBuffer::new(length),
            checks: GnssChecks::new(params.checks)?,
            yaw: GsfYaw::new(),
            pending: None,
            last_sample: GnssObservation::default(),
            started: false,
            data_ready: false,
            last_input: 0,
            last_push: 0,
            position: AidStatus::default(),
            velocity: AidStatus::default(),
            height: AidStatus::default(),
            position_observation: GlobalPosition::default(),
            velocity_observation: [0.0; 3],
            position_variance: 0.0,
            velocity_variance: [0.0; 3],
            last_position_fuse: 0,
            last_velocity_fuse: 0,
            last_height_fuse: 0,
            height_bias: BiasEstimator::new(0.0, 0.0)?,
        })
    }

    fn status(
        time: u64,
        innovation: [f32; 3],
        variance: [f32; 3],
        gate: f32,
        dimensions: usize,
    ) -> AidStatus {
        let mut ratio = [0.0; 3];
        for i in 0..dimensions {
            ratio[i] = math::square(innovation[i]) / (math::square(gate.max(1.0)) * variance[i]);
        }
        AidStatus {
            timestamp: time,
            innovation,
            variance,
            ratio,
            fused: false,
            rejected: ratio[..dimensions]
                .iter()
                .any(|v| !v.is_finite() || *v > 1.0),
        }
    }

    fn zero(status: &mut AidStatus, variance: [f32; 3]) {
        status.innovation = [0.0; 3];
        status.variance = variance;
        status.ratio = [0.0; 3];
        status.fused = true;
        status.rejected = false;
    }

    fn stop_height(&mut self, filter: &mut ImuEkf) {
        if filter.control & HEIGHT != 0 && filter.height_reference == 1 {
            filter.height_reference = 4;
        }
        filter.control &= !HEIGHT;
    }

    fn stop(&mut self, filter: &mut ImuEkf) {
        if filter.control & (POSITION | VELOCITY) != 0 {
            self.checks.reset();
        }
        filter.control &= !(POSITION | VELOCITY);
        self.stop_height(filter);
        self.yaw.reset();
    }

    fn update_observations(&mut self, filter: &ImuEkf, imu: &ImuDelta) {
        let sample = self.last_sample.sample;
        let rotation = math::rotation(filter.quaternion());
        let offset = math::rotate(rotation, self.params.offset_body_m);
        let mut position = GlobalPosition {
            latitude_rad: sample.latitude_deg.to_radians(),
            longitude_rad: sample.longitude_deg.to_radians(),
            altitude_m: sample.altitude_msl_m,
        };
        position.add_ned(offset.map(|v| -v));
        self.position_observation = position;
        let mut noise = sample
            .horizontal_accuracy_m
            .max(self.params.position_noise_m);
        if filter.control & VELOCITY == 0 {
            noise = noise.min(filter.params.no_aid_noise);
        }
        self.position_variance = math::square(noise).max(0.0001);
        let delta = filter.global.difference(position);
        self.position = Self::status(
            sample.timestamp,
            [delta[0], delta[1], 0.0],
            [
                filter.covariance[6][6] + self.position_variance,
                filter.covariance[7][7] + self.position_variance,
                0.0,
            ],
            self.params.position_gate,
            2,
        );
        let angular_velocity = core::array::from_fn(|i| {
            imu.delta_angle_rad[i] * (1.0 / imu.delta_angle_dt_s) - filter.state[10 + i]
        });
        let velocity_offset = math::rotate(
            rotation,
            math::cross(angular_velocity, self.params.offset_body_m),
        );
        self.velocity_observation =
            core::array::from_fn(|i| sample.velocity_ned_m_s[i] - velocity_offset[i]);
        let variance = math::square(
            sample
                .speed_accuracy_m_s
                .max(self.params.velocity_noise_m_s)
                .max(0.01),
        );
        self.velocity_variance = [variance, variance, variance * math::square(1.5)];
        self.velocity = Self::status(
            sample.timestamp,
            core::array::from_fn(|i| filter.state[4 + i] - self.velocity_observation[i]),
            core::array::from_fn(|i| filter.covariance[3 + i][3 + i] + self.velocity_variance[i]),
            self.params.velocity_gate,
            3,
        );
        if filter.px4_fault & (1 << 10) != 0
            && self.velocity.ratio[2] > 1.0
            && self.velocity.ratio[0] < 1.0
            && self.velocity.ratio[1] < 1.0
            && sample.speed_accuracy_m_s < self.params.checks.maximum_speed_accuracy_m_s
        {
            let limit = self.params.velocity_gate.max(1.0) * libm::sqrtf(self.velocity.variance[2]);
            self.velocity.innovation[2] = self.velocity.innovation[2].clamp(-limit, limit);
            self.velocity.rejected = false;
        }
    }

    fn fuse_velocity(&mut self, filter: &mut ImuEkf) -> bool {
        if self.velocity.rejected {
            self.velocity.fused = false;
            return false;
        }
        for i in 0..3 {
            filter.fuse_direct(
                self.velocity.innovation[i],
                self.velocity.variance[i],
                self.velocity_variance[i],
                3 + i,
                true,
            );
        }
        self.velocity.fused = true;
        self.last_velocity_fuse = filter.delayed_us;
        filter.last_horizontal_velocity_fuse = filter.delayed_us;
        filter.last_vertical_velocity_fuse = filter.delayed_us;
        true
    }

    fn reset_velocity(&mut self, filter: &mut ImuEkf) {
        let previous = [filter.state[4], filter.state[5], filter.state[6]];
        for i in 0..3 {
            filter.state[4 + i] = self.velocity_observation[i];
            filter.reset_variance(3 + i, self.velocity_variance[i].max(0.0001));
        }
        for i in 6..9 {
            filter.reset_variance(i, filter.covariance[i][i]);
        }
        filter.propagate_velocity_reset(previous);
        self.last_velocity_fuse = filter.delayed_us;
        filter.last_horizontal_velocity_fuse = filter.delayed_us;
        filter.last_vertical_velocity_fuse = filter.delayed_us;
        Self::zero(&mut self.velocity, self.velocity_variance);
    }

    fn fuse_position(&mut self, filter: &mut ImuEkf) -> bool {
        if self.position.rejected {
            self.position.fused = false;
            return false;
        }
        for i in 0..2 {
            filter.fuse_direct(
                self.position.innovation[i],
                self.position.variance[i],
                self.position_variance,
                6 + i,
                true,
            );
        }
        self.position.fused = true;
        self.last_position_fuse = filter.delayed_us;
        filter.last_horizontal_position_fuse = filter.delayed_us;
        true
    }

    fn reset_position(&mut self, filter: &mut ImuEkf) {
        let previous = filter.global.project(filter.origin.unwrap_or_default());
        if filter.origin.is_none() {
            filter.origin = Some(self.position_observation);
            filter.origin_timestamp = filter.delayed_us;
        }
        filter.global.latitude_rad = self.position_observation.latitude_rad;
        filter.global.longitude_rad = self.position_observation.longitude_rad;
        filter.propagate_position_reset();
        filter.record_position_reset(previous);
        filter.reset_variance(6, (2.0 * self.position_variance).max(0.0001));
        filter.reset_variance(7, (2.0 * self.position_variance).max(0.0001));
        self.last_position_fuse = filter.delayed_us;
        filter.last_horizontal_position_fuse = filter.delayed_us;
        Self::zero(
            &mut self.position,
            [self.position_variance, self.position_variance, 0.0],
        );
    }

    fn control_velocity(&mut self, filter: &mut ImuEkf, force_reset: bool) {
        let continuing = self.params.control & 4 != 0
            && filter.control & 3 == 3
            && filter.control & (GNSS_FAULT | HEIGHT_FAULT) == 0;
        if filter.control & VELOCITY != 0 {
            if continuing {
                self.fuse_velocity(filter);
                if filter.timed_out(self.last_velocity_fuse, 7_000_000) || force_reset {
                    self.reset_velocity(filter);
                }
            } else {
                filter.control &= !VELOCITY;
                if filter.control & POSITION == 0 {
                    self.checks.reset();
                }
            }
        } else if continuing && self.checks.passed() {
            let do_reset = force_reset || filter.previous_control & 2 == 0;
            let fused = !do_reset
                && self.velocity.ratio[0] + self.velocity.ratio[1] < 0.25
                && self.fuse_velocity(filter);
            if !fused {
                self.reset_velocity(filter);
            }
            filter.control |= VELOCITY;
        }
    }

    fn control_position(&mut self, filter: &mut ImuEkf, force_reset: bool) {
        let enabled = self.params.control & 1 != 0;
        let continuing = enabled && filter.control & 3 == 3 && filter.control & HEIGHT_FAULT == 0;
        if filter.control & POSITION != 0 {
            if continuing {
                self.fuse_position(filter);
                if filter.timed_out(self.last_position_fuse, 7_000_000) || force_reset {
                    self.reset_position(filter);
                }
            } else {
                filter.control &= !POSITION;
                if filter.control & VELOCITY == 0 {
                    self.checks.reset();
                }
            }
        } else if continuing && self.checks.passed() {
            let do_reset = force_reset || filter.previous_control & 2 == 0;
            let fused = filter.origin.is_some()
                && !do_reset
                && self.position.ratio[0] + self.position.ratio[1] < 0.25
                && self.fuse_position(filter);
            if !fused {
                self.reset_position(filter);
            }
            filter.control |= POSITION;
            filter.control &= !GNSS_FAULT;
        } else if enabled && self.checks.passed() && filter.origin.is_none() {
            self.reset_position(filter);
        }
    }
}

impl ControlExtension for GnssAids {
    fn push(&mut self, filter: &ImuEkf) -> Result<(), FilterFault> {
        let Some(observation) = self.pending.take() else {
            return Ok(());
        };
        let sample = observation.sample;
        if sample.timestamp <= self.last_input
            || sample.timestamp > filter.latest_us
            || !sample.latitude_deg.is_finite()
            || sample.latitude_deg.abs() > 90.0
            || !sample.longitude_deg.is_finite()
            || sample.longitude_deg.abs() > 180.0
            || !sample.altitude_msl_m.is_finite()
            || sample.velocity_ned_m_s.iter().any(|v| !v.is_finite())
            || [
                sample.horizontal_accuracy_m,
                sample.vertical_accuracy_m,
                sample.speed_accuracy_m_s,
                observation.pdop,
            ]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.0)
        {
            return Err(FilterFault::Input);
        }
        self.last_input = sample.timestamp;
        let time = sample.timestamp as i64
            - (self.params.delay_ms * 1000.0) as i64
            - (filter.average_dt_s * 5e5) as i64;
        let interval = (filter.latest_us - filter.delayed_us) / (self.buffer.length as u64 - 1);
        self.started = true;
        if time >= (self.buffer.newest_time() + interval) as i64 {
            self.buffer.push(observation, time as u64);
            self.last_push = filter.latest_us;
        }
        Ok(())
    }

    fn control_horizontal(
        &mut self,
        filter: &mut ImuEkf,
        imu: &ImuDelta,
    ) -> Result<(), FilterFault> {
        self.data_ready = false;
        if !self.started || self.params.control == 0 {
            self.stop(filter);
            return Ok(());
        }
        if filter.params.imu_ctrl & 1 != 0 {
            self.yaw.set_gyro_bias(
                [filter.state[10], filter.state[11], filter.state[12]],
                false,
            );
        }
        self.yaw.predict(imu, true);
        if let Some((mut observation, time)) = self.buffer.pop(filter.delayed_us) {
            observation.sample.timestamp = time;
            self.last_sample = observation;
            self.data_ready = self.checks.run(
                &observation.sample,
                observation.pdop,
                observation.spoofed,
                filter.delayed_us,
            )?;
            if !self.data_ready
                && filter.control & (POSITION | VELOCITY) != 0
                && filter.timed_out(self.checks.last_pass_us(), 7_000_000)
            {
                self.stop(filter);
            }
            self.update_observations(filter, imu);
        } else if filter.control & (POSITION | VELOCITY) != 0
            && self.last_push + 7_000_000 <= filter.latest_us
        {
            self.stop(filter);
        }
        if self.data_ready {
            if self.velocity_variance[0] > 0.0
                && self.velocity_variance[0] < self.params.checks.maximum_speed_accuracy_m_s
            {
                // Pinned caller supplies velocity variance as the GSF accuracy;
                // GSF squares it again. Preserve the actual upstream contract.
                self.yaw.fuse_velocity(
                    [self.velocity_observation[0], self.velocity_observation[1]],
                    self.velocity_variance[0],
                    true,
                );
                if filter.control & 1 != 0 && filter.control & 2 == 0 && self.yaw.available() {
                    filter.reset_yaw(self.yaw.yaw(), self.yaw.variance());
                    filter.control |= 2;
                }
            }
            let force_reset = filter.control & GNSS_FAULT == 0
                && filter.control & (POSITION | VELOCITY) != 0
                && self.yaw.available()
                && math::wrap_pi_f32(
                    math::yaw(math::rotation(filter.quaternion())) - self.yaw.yaw(),
                )
                .abs()
                    > 25.0_f32.to_radians()
                && filter.timed_out(filter.last_horizontal_velocity_fuse, 1_000_000)
                && filter.last_horizontal_velocity_fuse != 0;
            if force_reset {
                filter.reset_yaw(self.yaw.yaw(), self.yaw.variance());
                filter.control |= 2;
                if filter.control & ((1 << 4) | (1 << 5)) != 0 {
                    filter.control |= 1 << 17;
                }
            }
            self.control_velocity(filter, force_reset);
            self.control_position(filter, force_reset);
        }
        Ok(())
    }

    fn shift_altitude(&mut self, delta: f32) -> Result<(), FilterFault> {
        self.height_bias.set_bias(self.height_bias.bias() + delta)
    }
    fn vertical_innovations(&self) -> [(f32, f32); 2] {
        [
            (self.height.innovation[0], self.height.variance[0]),
            (self.velocity.innovation[2], self.velocity.variance[2]),
        ]
    }

    fn control_height(&mut self, filter: &mut ImuEkf) -> Result<(), FilterFault> {
        if filter.control & HEIGHT != 0 && filter.height_reference != 1 {
            self.height_bias.predict(filter.average_dt_s)?;
        }
        if !self.data_ready {
            if filter.control & HEIGHT != 0 && self.last_push + 1_000_000 <= filter.latest_us {
                self.stop_height(filter);
            }
            return Ok(());
        }
        let sample = self.last_sample.sample;
        let mut noise = sample
            .vertical_accuracy_m
            .max(1.5 * self.params.position_noise_m);
        if filter.control & ((1 << 9) | (1 << 10)) != 0 {
            noise = noise.min(filter.params.no_aid_noise);
        }
        let measurement = sample.altitude_msl_m
            + math::rotate(
                math::rotation(filter.quaternion()),
                self.params.offset_body_m,
            )[2];
        let measurement_variance = math::square(noise);
        let variance = measurement_variance + self.height_bias.variance();
        let innovation = -filter.global.altitude_m + (measurement - self.height_bias.bias());
        self.height = Self::status(
            sample.timestamp,
            [innovation, 0.0, 0.0],
            [filter.covariance[8][8] + variance, 0.0, 0.0],
            self.params.position_gate,
            1,
        );
        if filter.px4_fault & (1 << 10) != 0 && self.height.rejected {
            let limit = self.params.position_gate.max(1.0) * libm::sqrtf(self.height.variance[0]);
            self.height.innovation[0] = self.height.innovation[0].clamp(-limit, limit);
            self.height.rejected = false;
        }
        let common =
            filter.origin.is_some() && self.checks.passed() && filter.control & GNSS_FAULT == 0;
        let continuing = self.params.control & 2 != 0 && common;
        let starting = continuing && self.last_push + 1_000_000 > filter.latest_us;
        let reference = self.params.control != 0 && filter.aiding_height_reference() == 1;
        if filter.control & HEIGHT != 0 {
            if continuing {
                self.height_bias
                    .set_noise(0.13, libm::sqrtf(measurement_variance))?;
                if filter.height_reference != 1 {
                    self.height_bias.fuse(
                        measurement - filter.global.altitude_m,
                        measurement_variance + filter.covariance[8][8],
                    )?;
                }
                if !self.height.rejected {
                    filter.fuse_direct(
                        self.height.innovation[0],
                        self.height.variance[0],
                        variance,
                        8,
                        true,
                    );
                    self.height.fused = true;
                    self.last_height_fuse = filter.delayed_us;
                    filter.last_height_fuse = filter.delayed_us;
                }
                if filter.height_reset_required() && filter.height_reference == 1 {
                    filter.reset_altitude(measurement, measurement_variance);
                    self.height_bias
                        .set_bias(-filter.global.altitude_m + measurement)?;
                    Self::zero(&mut self.height, [variance, 0.0, 0.0]);
                    self.last_height_fuse = filter.delayed_us;
                } else if filter.timed_out(self.last_height_fuse, 5_000_000) {
                    self.stop_height(filter);
                }
            } else {
                self.stop_height(filter);
            }
        } else if common
            && filter.origin_altitude.is_none()
            && reference
            && self.last_push + 1_000_000 > filter.latest_us
        {
            filter.initialize_altitude(measurement, measurement_variance);
            self.height_bias.reset();
            if starting {
                filter.height_reference = 1;
                Self::zero(&mut self.height, [variance, 0.0, 0.0]);
                self.last_height_fuse = filter.delayed_us;
                filter.control |= HEIGHT;
            }
        } else if starting {
            if reference {
                filter.height_reference = 1;
                filter.reset_altitude(measurement, measurement_variance);
                self.height_bias.reset();
                Self::zero(&mut self.height, [variance, 0.0, 0.0]);
            } else {
                self.height_bias
                    .set_bias(-filter.global.altitude_m + measurement)?;
            }
            self.last_height_fuse = filter.delayed_us;
            filter.control |= HEIGHT;
            filter.control &= !HEIGHT_FAULT;
        }
        Ok(())
    }
}
