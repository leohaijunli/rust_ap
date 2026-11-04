//! PX4 range-height/terrain controller with independently delayed observations.
//! Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.

use super::super::{
    math,
    observation_buffer::ObservationBuffer,
    range_math::hagl_update,
    range_sensor::{RangeObservation, RangeSensorChecks, RangeSensorParams},
    FilterFault,
};
use super::{baro_mag::AidStatus, default_float, default_int, ImuEkf};

pub(super) const RANGE_HEIGHT: u64 = 1 << 10;
pub(super) const RANGE_TERRAIN: u64 = 1 << 39;
const KINEMATIC_CONSISTENT: u64 = 1 << 31;

#[derive(Clone, Copy, Debug)]
pub struct RangeParams {
    pub control: i32,
    pub delay_ms: f32,
    pub noise_m: f32,
    pub gate: f32,
    pub scale_error: f32,
    pub conditional_max_height_m: f32,
    pub conditional_max_speed_m_s: f32,
    pub kinematic_gate: f32,
    pub terrain_noise_m_s: f32,
    pub terrain_gradient: f32,
    pub minimum_ground_clearance_m: f32,
    pub sensor: RangeSensorParams,
    /// Sensor-minus-IMU lever arm in body FRD metres.
    pub offset_body_m: [f32; 3],
}

impl Default for RangeParams {
    fn default() -> Self {
        Self {
            control: default_int("EKF2_RNG_CTRL"),
            delay_ms: default_float("EKF2_RNG_DELAY"),
            noise_m: default_float("EKF2_RNG_NOISE"),
            gate: default_float("EKF2_RNG_GATE"),
            scale_error: default_float("EKF2_RNG_SFE"),
            conditional_max_height_m: default_float("EKF2_RNG_A_HMAX"),
            conditional_max_speed_m_s: default_float("EKF2_RNG_A_VMAX"),
            kinematic_gate: default_float("EKF2_RNG_K_GATE"),
            terrain_noise_m_s: default_float("EKF2_TERR_NOISE"),
            terrain_gradient: default_float("EKF2_TERR_GRAD"),
            minimum_ground_clearance_m: default_float("EKF2_MIN_RNG"),
            sensor: RangeSensorParams {
                minimum_m: 0.1,
                maximum_m: 30.0, // Synthetic host fixture, not hardware limits.
                pitch_offset_rad: default_float("EKF2_RNG_PITCH"),
                minimum_cos_tilt: f32::from_bits(0x3f350481), // Exact pinned 0.7071f, not 1/sqrt(2).
                quality_hysteresis_us: (default_float("EKF2_RNG_QLTY_T") * 1e6) as u64,
                stuck_threshold_m: 0.1,
                maximum_fog_distance_m: default_float("EKF2_RNG_FOG"),
            },
            offset_body_m: [0.0; 3],
        }
    }
}

struct Consistency {
    previous_time: u64,
    previous_distance: f32,
    last_horizontal_motion: u64,
    last_inconsistent: u64,
    signed_ratio: f32,
    consistent: bool,
}

impl Consistency {
    fn new() -> Self {
        Self {
            previous_time: 0,
            previous_distance: 0.0,
            last_horizontal_motion: 0,
            last_inconsistent: 0,
            signed_ratio: 0.0,
            consistent: true,
        }
    }

    fn update(&mut self, distance: f32, variance: f32, filter: &ImuEkf, gate: f32) {
        let horizontal_motion = math::square(filter.state[4]) + math::square(filter.state[5])
            > (filter.covariance[3][3] + filter.covariance[4][4]).max(0.1);
        if horizontal_motion {
            self.last_horizontal_motion = filter.delayed_us;
        }
        let dt = (filter.delayed_us - self.previous_time) as f32 * 1e-6;
        if self.previous_time != 0 && (0.001..=0.5).contains(&dt) {
            let velocity = (distance - self.previous_distance) / dt;
            let innovation = -velocity - filter.state[6];
            let innovation_variance = 2.0 * variance / (dt * dt) + filter.covariance[5][5];
            let ratio = math::square(innovation) / innovation_variance / math::square(gate);
            let sign = if innovation > 0.0 {
                1.0
            } else if innovation < 0.0 {
                -1.0
            } else {
                0.0
            };
            self.signed_ratio += dt / (2.0 + dt) * (sign * ratio - self.signed_ratio);
            if self.signed_ratio.abs() >= 1.0 {
                // The pinned C++ compares microseconds directly with the float
                // tau (2), not 2e6. Preserve it for reference parity.
                if (filter.delayed_us - self.last_horizontal_motion) as f32 > 2.0 {
                    self.consistent = false;
                    self.last_inconsistent = filter.delayed_us;
                }
            } else if filter.state[6].abs() > 0.5
                && ratio < 1.0
                && filter.delayed_us - self.last_inconsistent > 1_000_000
            {
                self.consistent = true;
            }
        }
        self.previous_time = filter.delayed_us;
        self.previous_distance = distance;
    }
}

pub(super) struct RangeAids {
    pub status: AidStatus,
    params: RangeParams,
    buffer: ObservationBuffer<RangeObservation>,
    sensor: RangeSensorChecks,
    consistency: Consistency,
    last_input: u64,
    last_push: u64,
    last_fuse: u64,
    started: bool,
}

impl RangeAids {
    pub fn new(params: RangeParams, length: usize) -> Result<Self, FilterFault> {
        if !(0..=2).contains(&params.control)
            || [
                params.delay_ms,
                params.noise_m,
                params.gate,
                params.scale_error,
                params.conditional_max_height_m,
                params.conditional_max_speed_m_s,
                params.kinematic_gate,
                params.terrain_noise_m_s,
                params.terrain_gradient,
                params.minimum_ground_clearance_m,
            ]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.0)
            || params.kinematic_gate <= 0.0
            || params.offset_body_m.iter().any(|v| !v.is_finite())
        {
            return Err(FilterFault::Input);
        }
        Ok(Self {
            status: AidStatus::default(),
            params,
            buffer: ObservationBuffer::new(length),
            sensor: RangeSensorChecks::new(params.sensor)?,
            consistency: Consistency::new(),
            last_input: 0,
            last_push: 0,
            last_fuse: 0,
            started: false,
        })
    }

    pub fn push(&mut self, sample: RangeObservation, filter: &ImuEkf) -> Result<(), FilterFault> {
        if sample.timestamp <= self.last_input
            || sample.timestamp > filter.latest_us
            || !sample.distance_m.is_finite()
            || !(-1..=100).contains(&sample.quality_percent)
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
            self.buffer.push(sample, time as u64);
            self.last_push = filter.latest_us;
        }
        Ok(())
    }

    fn stop_height(&mut self, filter: &mut ImuEkf) {
        if filter.control & RANGE_HEIGHT != 0 && filter.height_reference == 2 {
            filter.height_reference = 4;
        }
        filter.control &= !RANGE_HEIGHT;
    }

    fn zero_status(&mut self, variance: f32) {
        self.status.innovation = [0.0; 3];
        self.status.variance = [variance, 0.0, 0.0];
        self.status.ratio = [0.0; 3];
        self.status.rejected = false;
        self.status.fused = true;
    }

    fn reset_terrain(&mut self, filter: &mut ImuEkf, variance: f32) {
        let previous = filter.state[24];
        filter.reset_variance(23, 0.0);
        let mut gain = [0.0; 24];
        gain[23] = 1.0;
        let mut h = [0.0; 24];
        h[8] = -1.0;
        h[23] = 1.0;
        filter.measurement_update(gain, h, variance, self.status.innovation[0]);
        filter.record_terrain_reset(filter.state[24] - previous);
        self.last_fuse = filter.delayed_us;
    }

    pub fn control(&mut self, filter: &mut ImuEkf) -> Result<(), FilterFault> {
        if !self.started {
            return Ok(());
        }
        let popped = self.buffer.pop(filter.delayed_us);
        if let Some((mut sample, time)) = popped {
            sample.timestamp = time;
            self.sensor.set_sample(sample)?;
        }
        self.sensor.run(
            filter.delayed_us,
            math::rotation(filter.quaternion()),
            popped.is_some(),
        )?;
        if self.sensor.data_healthy() {
            let offset = math::rotate(
                math::rotation(filter.quaternion()),
                self.params.offset_body_m,
            )[2];
            self.sensor.correct_mounting(offset);
            let distance = self.sensor.distance_vertical_m().unwrap();
            let variance = math::square(self.params.noise_m)
                + math::square(self.params.scale_error * distance);
            self.consistency.update(
                distance,
                variance.max(0.001),
                filter,
                self.params.kinematic_gate,
            );
        }
        if self.consistency.consistent {
            filter.control |= KINEMATIC_CONSISTENT;
        } else {
            filter.control &= !KINEMATIC_CONSISTENT;
        }
        if let Some((_, time)) = popped {
            let measurement = self
                .sensor
                .distance_vertical_m()
                .unwrap()
                .max(self.params.minimum_ground_clearance_m);
            let variance = (filter.covariance[8][8]
                + math::square(self.params.noise_m)
                + math::square(self.params.scale_error * self.sensor.distance_m().unwrap()))
            .max(0.0);
            // PX4 uses global altitude for HAGL; a reset may leave nominal
            // posD cached until the next state correction in this same frame.
            let mut observation_state = filter.state;
            observation_state[9] = -filter.global.altitude_m;
            let update = hagl_update(
                &observation_state,
                &filter.covariance,
                measurement,
                variance,
            )?;
            let ratio = math::square(update.innovation)
                / (math::square(self.params.gate.max(1.0)) * update.innovation_variance);
            self.status = AidStatus {
                timestamp: time,
                innovation: [update.innovation, 0.0, 0.0],
                variance: [update.innovation_variance, 0.0, 0.0],
                ratio: [ratio, 0.0, 0.0],
                fused: false,
                rejected: !ratio.is_finite() || ratio > 1.0,
            };
            if filter.px4_fault & (1 << 10) != 0
                && self.status.rejected
                && self.sensor.data_healthy()
            {
                let limit = self.params.gate.max(1.0) * libm::sqrtf(self.status.variance[0]);
                self.status.innovation[0] = self.status.innovation[0].clamp(-limit, limit);
                self.status.rejected = false;
            }
            let continuing = self.params.control != 0 && filter.control & 1 != 0;
            let starting = continuing
                && self.last_push + 400_000 > filter.latest_us
                && self.sensor.regularly_sending()
                && self.sensor.data_healthy();
            let active = filter.control & (RANGE_HEIGHT | RANGE_TERRAIN) != 0;
            let height_active = filter.control & RANGE_HEIGHT != 0;
            let scale = if height_active { 1.0 } else { 0.7 };
            let stable = ratio < if height_active { 1.0 } else { 0.01 };
            let suitable = filter.state[24] + filter.global.altitude_m
                < scale * self.params.conditional_max_height_m
                && stable
                && (!filter.horizontal_aiding()
                    || math::norm(&filter.state[4..6])
                        <= scale * self.params.conditional_max_speed_m_s);
            let conditional = active && self.params.control == 1 && suitable;
            let enabled = active && self.params.control == 2;
            if height_active && !(conditional || enabled) {
                self.stop_height(filter);
            } else if !height_active && starting {
                let reference = filter
                    .aiding_params
                    .map(|p| p.height_reference)
                    .unwrap_or(4);
                if reference == 2 && conditional {
                    filter.height_reference = 2;
                    filter.control |= RANGE_HEIGHT;
                    filter.control &= !RANGE_TERRAIN;
                    if self.status.rejected {
                        self.reset_terrain(filter, variance);
                        self.zero_status(variance);
                    }
                } else if reference == 2 && enabled {
                    filter.height_reference = 2;
                    filter.reset_altitude(measurement, variance);
                    filter.state[24] = 0.0;
                    self.zero_status(variance);
                    filter.control |= RANGE_HEIGHT;
                    filter.control &= !RANGE_TERRAIN;
                    self.last_fuse = filter.delayed_us;
                } else if reference != 2 && (conditional || enabled) {
                    filter.control |= RANGE_HEIGHT;
                    if self.status.rejected {
                        self.reset_terrain(filter, variance);
                        self.zero_status(variance);
                    }
                }
            }
            if filter.control & (RANGE_HEIGHT | RANGE_TERRAIN) != 0 {
                if continuing {
                    if conditional {
                        filter.height_reference = 2;
                    } else if filter.height_reference == 2 {
                        filter.height_reference = 4;
                    }
                    if self.sensor.data_healthy()
                        && self.consistency.consistent
                        && !self.status.rejected
                    {
                        let mut gain = filter
                            .project_covariance(update.observation)
                            .map(|v| v * (1.0 / self.status.variance[0])); // PX4 Matrix scalar division multiplies the reciprocal.
                        if filter.control & RANGE_TERRAIN == 0 {
                            gain[23] = 0.0;
                        }
                        if filter.control & RANGE_HEIGHT == 0 {
                            let terrain_gain = gain[23];
                            gain.fill(0.0);
                            gain[23] = terrain_gain;
                        }
                        filter.measurement_update(
                            gain,
                            update.observation,
                            variance,
                            self.status.innovation[0],
                        );
                        self.last_fuse = filter.delayed_us;
                        self.status.fused = true;
                        if filter.control & RANGE_TERRAIN != 0 {
                            filter.last_terrain_fuse = filter.delayed_us;
                        }
                    }
                    if filter.height_reset_required()
                        && filter.control & RANGE_HEIGHT != 0
                        && filter.height_reference == 2
                        && starting
                    {
                        filter.reset_altitude(measurement - filter.state[24], f32::NAN);
                        self.zero_status(variance);
                        if !filter.vertical_velocity_aiding() {
                            filter.reset_vertical_velocity(0.0, 10.0);
                        }
                        self.last_fuse = filter.delayed_us;
                    } else if filter.timed_out(self.last_fuse, 5_000_000) && starting {
                        self.reset_terrain(filter, variance);
                        self.zero_status(variance);
                    }
                } else {
                    self.stop_height(filter);
                    filter.control &= !RANGE_TERRAIN;
                }
            } else if starting {
                if self.status.rejected {
                    self.reset_terrain(filter, variance);
                    self.zero_status(variance);
                }
                filter.control |= RANGE_TERRAIN;
            }
        } else if filter.control & (RANGE_HEIGHT | RANGE_TERRAIN) != 0
            && self.last_push + 400_000 <= filter.latest_us
        {
            self.stop_height(filter);
            filter.control &= !RANGE_TERRAIN;
        }
        self.update_validity(filter);
        Ok(())
    }

    fn update_validity(&self, filter: &mut ImuEkf) {
        let hagl = filter.state[24] + filter.global.altitude_m;
        let variance =
            filter.covariance[23][23] - filter.covariance[23][8] - filter.covariance[8][23]
                + filter.covariance[8][8];
        let positive = filter.last_terrain_fuse != 0 && variance > 0.0;
        let relative = variance < math::square((0.1 * hagl).max(0.5));
        if filter.terrain_valid {
            filter.terrain_valid = hagl >= 0.0 && positive && relative;
        } else if hagl >= 0.0
            && positive
            && filter.control & RANGE_TERRAIN != 0
            && !filter.timed_out(self.last_fuse, 5_000_000)
        {
            filter.terrain_valid = true;
        }
    }
}
