//! E2 host aid configuration and fixed-capacity delayed observations.
//! Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.

use super::super::{
    bias_estimator::BiasEstimator,
    mag_generated, math,
    observation_buffer::ObservationBuffer,
    wmm::{self, MagneticField},
    FilterFault, Timestamp,
};
use super::{default_float, default_int, ImuEkf};

pub(super) const MAG: u64 = 1 << 35;
pub(super) const BARO: u64 = 1 << 9;
const YAW: u64 = 1 << 1;
const MAG_HDG: u64 = 1 << 4;
const MAG_3D: u64 = 1 << 5;
const MAG_DEC: u64 = 1 << 6;
const MAG_ALIGNED: u64 = 1 << 23;
const MAG_CONSISTENT: u64 = 1 << 37;
// AlphaFilter is initialized with dt=0.01s, tau=0.09s. Literal 0.1f differs
// by one f32 ULP and can change a height reset by an altitude-sized ULP.
const SENSOR_ALPHA: f32 = 0.01 / (0.09 + 0.01);

/// Already processed pressure altitude and calibrated FRD field, not raw pressure.
#[derive(Clone, Copy, Debug, Default)]
pub struct BaroMagSample {
    pub timestamp: Timestamp,
    pub altitude_m: f32,
    pub field_gauss: [f32; 3],
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::imu_ekf::ImuEkfParams;

    #[test]
    fn mag_stop_uses_one_second_and_revokes_unobservable_yaw() {
        let params = BaroMagParams::default();
        let mut filter = ImuEkf::with_baro_mag(&ImuEkfParams::default(), params, 0).unwrap();
        let mut aids = filter.aiding.take().unwrap();
        aids.last_mag_push = 1_000_000;
        filter.control |= MAG | MAG_3D | MAG_ALIGNED | YAW;
        filter.covariance[15][15] = 0.123;
        filter.latest_us = 1_500_000;
        aids.control_mag(&mut filter);
        assert_ne!(filter.control & MAG, 0);
        filter.latest_us = 2_000_001;
        aids.control_mag(&mut filter);
        assert_eq!(filter.control & (MAG | MAG_3D | MAG_ALIGNED | YAW), 0);
        assert_eq!(filter.covariance[15][15], math::square(params.mag_noise));
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AidStatus {
    pub timestamp: u64,
    pub innovation: [f32; 3],
    pub variance: [f32; 3],
    pub ratio: [f32; 3],
    pub fused: bool,
    pub rejected: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct BaroMagParams {
    pub baro_noise: f32,
    pub baro_gate: f32,
    pub baro_delay_ms: f32,
    pub mag_noise: f32,
    pub mag_gate: f32,
    pub mag_delay_ms: f32,
    pub mag_earth_noise: f32,
    pub mag_bias_noise: f32,
    pub mag_declination_deg: f32,
    pub mag_check: i32,
    pub mag_type: i32,
    pub height_reference: i32,
}

impl Default for BaroMagParams {
    fn default() -> Self {
        Self {
            baro_noise: default_float("EKF2_BARO_NOISE"),
            baro_gate: default_float("EKF2_BARO_GATE"),
            baro_delay_ms: default_float("EKF2_BARO_DELAY"),
            mag_noise: default_float("EKF2_MAG_NOISE"),
            mag_gate: default_float("EKF2_MAG_GATE"),
            mag_delay_ms: default_float("EKF2_MAG_DELAY"),
            mag_earth_noise: default_float("EKF2_MAG_E_NOISE"),
            mag_bias_noise: default_float("EKF2_MAG_B_NOISE"),
            mag_declination_deg: default_float("EKF2_MAG_DECL"),
            mag_check: default_int("EKF2_MAG_CHECK"),
            mag_type: default_int("EKF2_MAG_TYPE"),
            height_reference: default_int("EKF2_HGT_REF"),
        }
    }
}

pub(super) struct Aids {
    pub baro_status: AidStatus,
    pub mag_status: AidStatus,
    pub params: BaroMagParams,
    baro: ObservationBuffer<BaroMagSample>,
    mag: ObservationBuffer<BaroMagSample>,
    last_input: u64,
    last_baro_push: u64,
    last_mag_push: u64,
    baro_started: bool,
    mag_started: bool,
    baro_counter: usize,
    mag_counter: usize,
    baro_lpf: f32,
    mag_lpf: [f32; 3],
    baro_bias: BiasEstimator,
    last_baro_fuse: u64,
    last_mag_fuse: u64,
    last_mag_bad: u64,
    mag_heading_lpf: f32,
    wmm: Option<MagneticField>,
    wmm_earth: [f32; 3],
    last_wmm_check: u64,
}

impl Aids {
    pub fn new(params: BaroMagParams, length: usize) -> Result<Self, FilterFault> {
        if [
            params.baro_noise,
            params.baro_gate,
            params.baro_delay_ms,
            params.mag_noise,
            params.mag_gate,
            params.mag_delay_ms,
            params.mag_earth_noise,
            params.mag_bias_noise,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v < 0.0)
            || !params.mag_declination_deg.is_finite()
            || params.mag_check & !7 != 0
            || ![0, 1, 5, 6].contains(&params.mag_type)
            || !(0..=3).contains(&params.height_reference)
        {
            return Err(FilterFault::Input);
        }
        Ok(Self {
            baro_status: AidStatus::default(),
            mag_status: AidStatus::default(),
            params,
            baro: ObservationBuffer::new(length),
            mag: ObservationBuffer::new(length),
            last_input: 0,
            last_baro_push: 0,
            last_mag_push: 0,
            baro_started: false,
            mag_started: false,
            baro_counter: 0,
            mag_counter: 0,
            baro_lpf: 0.0,
            mag_lpf: [0.0; 3],
            baro_bias: BiasEstimator::new(0.0, 0.0)?,
            last_baro_fuse: 0,
            last_mag_fuse: 0,
            last_mag_bad: 0,
            mag_heading_lpf: 0.0,
            wmm: None,
            wmm_earth: [0.0; 3],
            last_wmm_check: 0,
        })
    }

    pub fn push(&mut self, sample: BaroMagSample, filter: &ImuEkf) -> Result<(), FilterFault> {
        if sample.timestamp <= self.last_input
            || sample.timestamp > filter.latest_us
            || !sample.altitude_m.is_finite()
            || sample.field_gauss.iter().any(|v| !v.is_finite())
        {
            return Err(FilterFault::Input);
        }
        self.last_input = sample.timestamp;
        let minimum_interval =
            (filter.latest_us - filter.delayed_us) / (self.baro.length as u64 - 1);
        for (buffer, delay, latest, started) in [
            (
                &mut self.baro,
                self.params.baro_delay_ms,
                &mut self.last_baro_push,
                &mut self.baro_started,
            ),
            (
                &mut self.mag,
                self.params.mag_delay_ms,
                &mut self.last_mag_push,
                &mut self.mag_started,
            ),
        ] {
            let time = sample.timestamp as i64
                - (delay * 1000.0) as i64
                - (filter.average_dt_s * 5e5) as i64;
            if time >= (buffer.newest_time() + minimum_interval) as i64 {
                buffer.push(sample, time as u64);
                *latest = filter.latest_us;
                *started = true;
            }
        }
        Ok(())
    }

    pub fn shift_altitude(&mut self, delta: f32) -> Result<(), FilterFault> {
        self.baro_bias.set_bias(self.baro_bias.bias() + delta)
    }

    pub fn control_baro(&mut self, filter: &mut ImuEkf) -> Result<(), FilterFault> {
        // Pinned updateGroundEffect runs before pressure-height fusion.
        if filter.control & (1 << 7) != 0 && filter.control & (1 << 8) == 0 {
            if filter.terrain_valid {
                let near_ground =
                    filter.state[24] + filter.global.altitude_m < default_float("EKF2_GND_MAX_HGT");
                if near_ground {
                    filter.control |= 1 << 20;
                } else {
                    filter.control &= !(1 << 20);
                }
            } else if filter.control & (1 << 20) != 0 {
                // This profile has no external ground-effect-on flag input;
                // its zero timeout origin is expired, as in the pinned core.
                filter.control &= !(1 << 20);
            }
        } else {
            filter.control &= !(1 << 20);
        }
        if filter.control & BARO != 0 && filter.height_reference != 0 {
            self.baro_bias.predict(filter.average_dt_s)?;
        }
        if let Some((sample, time)) = self
            .baro_started
            .then(|| self.baro.pop(filter.delayed_us))
            .flatten()
        {
            if self.baro_counter == 0 {
                self.baro_lpf = sample.altitude_m;
            } else {
                self.baro_lpf += SENSOR_ALPHA * (sample.altitude_m - self.baro_lpf);
            }
            self.baro_counter += 1;
            if self.baro_counter <= self.baro.length {
                self.baro_bias
                    .set_bias(-filter.global.altitude_m + self.baro_lpf)?;
            }
            let measurement_variance = math::square(self.params.baro_noise);
            let variance = measurement_variance + self.baro_bias.variance();
            let innovation =
                -filter.global.altitude_m + (sample.altitude_m - self.baro_bias.bias());
            let innovation_variance = filter.covariance[8][8] + variance;
            let ratio = math::square(innovation)
                / (math::square(self.params.baro_gate.max(1.0)) * innovation_variance);
            self.baro_status = AidStatus {
                timestamp: time,
                innovation: [innovation, 0.0, 0.0],
                variance: [innovation_variance, 0.0, 0.0],
                ratio: [ratio, 0.0, 0.0],
                fused: false,
                rejected: !ratio.is_finite() || ratio > 1.0,
            };
            if filter.px4_fault & (1 << 10) != 0 && self.baro_status.rejected {
                let limit = self.params.baro_gate.max(1.0) * libm::sqrtf(innovation_variance);
                self.baro_status.innovation[0] = innovation.clamp(-limit, limit);
                self.baro_status.rejected = false;
            }
            let deadzone = default_float("EKF2_GND_EFF_DZ");
            if filter.control & (1 << 20) != 0
                && deadzone > 0.0
                && self.baro_status.innovation[0] < 0.0
            {
                self.baro_status.innovation[0] = if self.baro_status.innovation[0] <= -deadzone {
                    self.baro_status.innovation[0] + deadzone
                } else {
                    0.0
                };
            }
            self.baro_bias
                .set_noise(0.13, libm::sqrtf(measurement_variance))?;
            if filter.control & BARO != 0 && filter.height_reference != 0 {
                self.baro_bias.fuse(
                    sample.altitude_m - filter.global.altitude_m,
                    measurement_variance + filter.covariance[8][8],
                )?;
            }
            if filter.control & BARO != 0 {
                if !self.baro_status.rejected {
                    filter.fuse_direct(
                        self.baro_status.innovation[0],
                        innovation_variance,
                        variance,
                        8,
                        true,
                    );
                    self.last_baro_fuse = filter.delayed_us;
                    filter.last_height_fuse = filter.delayed_us;
                    self.baro_status.fused = true;
                }
                if filter.height_reset_required() && filter.height_reference == 0 {
                    filter.reset_altitude(
                        self.baro_lpf - self.baro_bias.bias(),
                        measurement_variance,
                    );
                    self.baro_bias
                        .set_bias(-filter.global.altitude_m + self.baro_lpf)?;
                    self.baro_status.innovation = [0.0; 3];
                    self.baro_status.variance = [variance, 0.0, 0.0];
                    self.baro_status.ratio = [0.0; 3];
                    self.baro_status.fused = true;
                    self.baro_status.rejected = false;
                    if !filter.vertical_velocity_aiding() {
                        filter.reset_vertical_velocity(0.0, 10.0);
                    }
                    self.last_baro_fuse = filter.delayed_us;
                } else if filter.timed_out(self.last_baro_fuse, 5_000_000) {
                    filter.control &= !BARO;
                    if !filter.timed_out(filter.last_height_fuse, 5_000_000) {
                        filter.control |= 1 << 43;
                    }
                    if filter.height_reference == 0 {
                        filter.height_reference = 4;
                    }
                }
            } else if self.baro_counter > self.baro.length
                && self.last_baro_push + 400_000 > filter.latest_us
            {
                if self.params.height_reference == 0 {
                    filter.height_reference = 0;
                    filter.initialize_altitude(sample.altitude_m, measurement_variance);
                    self.baro_bias.reset();
                    self.baro_status.innovation[0] = 0.0;
                    self.baro_status.variance[0] = measurement_variance;
                    self.baro_status.ratio[0] = 0.0;
                    self.baro_status.fused = true;
                    self.baro_status.rejected = false;
                } else {
                    self.baro_bias
                        .set_bias(-filter.global.altitude_m + self.baro_lpf)?;
                }
                filter.control |= BARO;
                self.last_baro_fuse = filter.delayed_us;
            }
        } else if filter.control & BARO != 0 && self.last_baro_push + 400_000 <= filter.latest_us {
            filter.control &= !BARO;
            if filter.height_reference == 0 {
                filter.height_reference = 4;
            }
        }
        Ok(())
    }

    pub fn control_mag(&mut self, filter: &mut ImuEkf) {
        if self.params.mag_type == 5 {
            self.stop_mag(filter);
            return;
        }
        let Some((sample, time)) = self
            .mag_started
            .then(|| self.mag.pop(filter.delayed_us))
            .flatten()
        else {
            if filter.control & MAG != 0 && self.last_mag_push + 1_000_000 <= filter.latest_us {
                self.stop_mag(filter);
            }
            return;
        };
        let previous_yaw = filter.control & YAW != 0;
        if self.mag_counter == 0 {
            self.mag_lpf = sample.field_gauss;
            filter.state[19..22].fill(0.0);
            for i in 18..21 {
                filter.reset_variance(i, math::square(self.params.mag_noise));
            }
            self.stop_mag(filter);
        } else {
            for i in 0..3 {
                self.mag_lpf[i] += SENSOR_ALPHA * (sample.field_gauss[i] - self.mag_lpf[i]);
            }
        }
        self.mag_counter += 1;
        let wmm_updated = self.update_wmm(filter);
        let observation_variance = math::square(self.params.mag_noise).max(0.0001);
        let (innovation, variance, h_x) = mag_generated::mag_x(
            &filter.state,
            &filter.covariance,
            sample.field_gauss,
            observation_variance,
        );
        let ratio = core::array::from_fn(|i| {
            math::square(innovation[i])
                / (math::square(self.params.mag_gate.max(1.0)) * variance[i])
        });
        self.mag_status = AidStatus {
            timestamp: time,
            innovation,
            variance,
            ratio,
            fused: false,
            rejected: ratio.iter().any(|value| !value.is_finite() || *value > 1.0),
        };
        let continuing = filter.control & 1 != 0 && math::norm(&sample.field_gauss) > 0.0;
        let starting = continuing
            && !filter.quaternion_reset_this_update()
            && self.check_field(sample.field_gauss, filter)
            && self.mag_counter > 3
            && self.last_mag_push + 500_000 > filter.latest_us;
        self.check_heading(sample.field_gauss, filter);
        let consistent = filter.control & MAG_CONSISTENT != 0 || !filter.horizontal_aiding();
        let active = filter.control & MAG != 0
            && filter.control & ((1 << 16) | (1 << 17)) == 0
            && consistent;
        if active && self.params.mag_type == 0 && filter.control & MAG_ALIGNED != 0 {
            filter.control |= MAG_3D;
        } else {
            filter.control &= !MAG_3D;
        }
        if active
            && (self.params.mag_type == 1
                || (self.params.mag_type == 0 && filter.control & MAG_3D == 0))
        {
            filter.control |= MAG_HDG;
        } else {
            filter.control &= !MAG_HDG;
        }
        if filter.control & MAG != 0 && !filter.horizontal_aiding() {
            filter.control |= MAG_DEC;
        } else {
            filter.control &= !MAG_DEC;
        }
        if filter.control & MAG != 0 {
            if !continuing || !previous_yaw {
                self.stop_mag(filter);
                return;
            }
            let hagl = if filter.terrain_valid {
                filter.state[24] + filter.global.altitude_m
            } else {
                filter.global.altitude_m
            };
            let hagl_reset = filter.range_params.is_some()
                && filter.control & MAG_ALIGNED == 0
                && hagl > 1.5
                && filter.control & (MAG_HDG | MAG_3D) != 0;
            if hagl_reset || (wmm_updated && !filter.horizontal_aiding()) {
                self.reset_mag(filter, filter.control & (MAG_HDG | MAG_3D) != 0);
                self.last_mag_fuse = filter.delayed_us;
                return;
            }
            let rejected = (0..3).any(|i| {
                math::square(innovation[i])
                    / (math::square(self.params.mag_gate.max(1.0)) * variance[i])
                    > 1.0
            });
            let update_all = filter.control & (MAG_3D | MAG_HDG) != 0;
            let update_tilt = filter.control & MAG_3D != 0;
            if !rejected {
                let linearization = filter.state;
                for axis in 0..3 {
                    let (s, h) = if axis == 0 {
                        (variance[0], h_x)
                    } else if axis == 1 {
                        mag_generated::mag_y(
                            &linearization,
                            &filter.covariance,
                            observation_variance,
                        )
                    } else {
                        mag_generated::mag_z(
                            &linearization,
                            &filter.covariance,
                            observation_variance,
                        )
                    };
                    let innov = if axis == 0 {
                        innovation[0]
                    } else {
                        let earth = [filter.state[16], filter.state[17], filter.state[18]];
                        math::rotate_inverse(math::rotation(filter.quaternion()), earth)[axis]
                            + filter.state[19 + axis]
                            - sample.field_gauss[axis]
                    };
                    self.mag_status.variance[axis] = s;
                    self.mag_status.innovation[axis] = innov;
                    if s < observation_variance {
                        return;
                    }
                    let mut gain = filter.project_covariance(h).map(|v| v * (1.0 / s));
                    Self::limit_gains(&mut gain, update_all, update_tilt);
                    filter.measurement_update(gain, h, observation_variance, innov);
                }
                self.last_mag_fuse = filter.delayed_us;
                self.mag_status.fused = true;
                if update_all {
                    filter.last_heading_fuse = filter.delayed_us;
                }
            }
            if filter.control & MAG_DEC != 0 {
                let r_decl = if self.wmm.is_some() { 0.5 } else { 0.25 };
                let (prediction, s, h) = mag_generated::declination(
                    &filter.state,
                    &filter.covariance,
                    r_decl,
                    f32::EPSILON,
                );
                let mut gain = filter.project_covariance(h).map(|v| v * (1.0 / s));
                // With no WMM, a configured saved declination updates the
                // active states; the zero fallback updates magnetic states only.
                let saved = self.params.mag_declination_deg != 0.0 || self.wmm.is_some();
                Self::limit_gains(&mut gain, saved && update_all, saved && update_tilt);
                let observation = if let Some(wmm) = self.wmm {
                    wmm.declination_deg.to_radians()
                } else if saved {
                    self.params.mag_declination_deg.to_radians()
                } else {
                    0.0
                };
                filter.measurement_update(
                    gain,
                    h,
                    r_decl,
                    math::wrap_pi_f32(prediction - observation),
                );
            }
            if filter.timed_out(self.last_mag_fuse, 7_000_000) {
                if !filter.horizontal_aiding() {
                    self.reset_mag(filter, update_all);
                    self.last_mag_fuse = filter.delayed_us;
                } else {
                    self.stop_mag(filter);
                }
            }
        } else if starting {
            let reset = !previous_yaw
                || wmm_updated
                || math::norm(&filter.state[16..19]) == 0.0
                || (15..21).any(|i| filter.covariance[i][i] < 1e-9);
            if reset {
                self.reset_mag(filter, !previous_yaw);
            } else if self.mag_status.rejected {
                return;
            } else {
                // In the pinned start-without-reset path mag is not active
                // yet, so measurementUpdate clears its Kalman gains.
                // Retain the Joseph/state-normalization calls, not a reset.
                for (i, innovation_value) in innovation.iter().enumerate() {
                    let h = if i == 0 {
                        h_x
                    } else if i == 1 {
                        mag_generated::mag_y(
                            &filter.state,
                            &filter.covariance,
                            observation_variance,
                        )
                        .1
                    } else {
                        mag_generated::mag_z(
                            &filter.state,
                            &filter.covariance,
                            observation_variance,
                        )
                        .1
                    };
                    filter.measurement_update(
                        [0.0; 24],
                        h,
                        observation_variance,
                        *innovation_value,
                    );
                }
                self.mag_status.fused = true;
            }
            self.last_mag_fuse = filter.delayed_us;
            filter.control |= MAG | YAW;
            if reset && !previous_yaw {
                self.mag_status.innovation = [0.0; 3];
                self.mag_status.variance = [observation_variance; 3];
                self.mag_status.ratio = [0.0; 3];
                self.mag_status.rejected = false;
                self.mag_status.fused = true;
            }
        }
    }

    fn limit_gains(gain: &mut [f32; 24], all: bool, tilt: bool) {
        if !all {
            gain[..15].fill(0.0);
            gain[21..].fill(0.0);
        } else if !tilt {
            gain[0] = 0.0;
            gain[1] = 0.0;
            gain[9] = 0.0;
            gain[10] = 0.0;
        }
    }
    fn stop_mag(&mut self, filter: &mut ImuEkf) {
        if filter.control & MAG == 0 {
            return;
        }
        for i in 15..21 {
            filter.reset_variance(i, math::square(self.params.mag_noise));
        }
        if filter.control & (MAG_HDG | MAG_3D) != 0 && !filter.horizontal_aiding() {
            filter.control &= !YAW;
        }
        filter.control &= !(MAG | MAG_HDG | MAG_3D | MAG_DEC | MAG_ALIGNED);
        filter.px4_fault &= !0x1f;
    }
    fn check_field(&mut self, field: [f32; 3], filter: &mut ImuEkf) -> bool {
        filter.control &= !(1 << 16);
        if self.params.mag_check == 0 {
            return true;
        }
        let strength = math::norm(&field);
        let mut failing = false;
        if self.params.mag_check & 1 != 0 {
            failing = if let Some(wmm) = self.wmm {
                (strength - wmm.strength_gauss).abs() > default_float("EKF2_MAG_CHK_STR")
            } else {
                self.params.mag_check & 4 != 0 || !(0.05..=0.85).contains(&strength)
            };
        }
        if self.params.mag_check & 2 != 0 {
            if let Some(wmm) = self.wmm {
                let earth = math::rotate(math::rotation(filter.quaternion()), field);
                let inclination = libm::asinf(earth[2] / math::norm(&earth).max(1e-4));
                failing |= math::wrap_pi_f32(inclination - wmm.inclination_deg.to_radians()).abs()
                    > default_float("EKF2_MAG_CHK_INC").to_radians();
            } else {
                failing |= self.params.mag_check & 4 != 0;
            }
        }
        if failing {
            filter.control |= 1 << 16;
        }
        if failing || self.last_mag_bad == 0 {
            self.last_mag_bad = filter.delayed_us;
        }
        filter.delayed_us - self.last_mag_bad > 1_000_000
    }
    fn check_heading(&mut self, field: [f32; 3], filter: &mut ImuEkf) {
        let r = math::rotation(filter.quaternion());
        let bias_good = (18..21).all(|i| {
            filter.covariance[i][i] > 0.0
                && filter.covariance[i][i] <= math::square(self.params.mag_noise)
        });
        let corrected =
            core::array::from_fn(|i| field[i] - if bias_good { filter.state[19 + i] } else { 0.0 });
        let earth = math::rotate(math::with_yaw(r, 0.0), corrected);
        let heading = -libm::atan2f(earth[1], earth[0]) + self.declination(filter);
        if filter.control & YAW != 0 {
            self.mag_heading_lpf +=
                SENSOR_ALPHA * (math::wrap_pi_f32(math::yaw(r) - heading) - self.mag_heading_lpf);
        } else {
            self.mag_heading_lpf = 0.0;
        }
        if self.mag_heading_lpf.abs() >= filter.params.heading_noise {
            filter.control &= !MAG_CONSISTENT;
        } else if filter.horizontal_aiding()
            && math::norm(&filter.horizontal_accel_lpf) > default_float("EKF2_MAG_ACCLIM")
        {
            filter.control |= MAG_CONSISTENT;
        }
    }
    fn declination(&self, filter: &ImuEkf) -> f32 {
        if filter.control & MAG_ALIGNED != 0 {
            libm::atan2f(filter.state[17], filter.state[16])
        } else if let Some(wmm) = self.wmm {
            wmm.declination_deg.to_radians()
        } else {
            self.params.mag_declination_deg.to_radians()
        }
    }
    fn reset_mag(&mut self, filter: &mut ImuEkf, reset_heading: bool) {
        if self.wmm.is_some() {
            let changed = math::norm(&core::array::from_fn::<_, 3, _>(|i| {
                filter.state[16 + i] - self.wmm_earth[i]
            })) > 0.01;
            if changed {
                filter.state[16..19].copy_from_slice(&self.wmm_earth);
                for i in 15..18 {
                    filter.reset_variance(i, math::square(self.params.mag_noise));
                }
            }
            if !reset_heading && filter.control & YAW != 0 {
                if changed {
                    let body =
                        math::rotate_inverse(math::rotation(filter.quaternion()), self.wmm_earth);
                    for (i, value) in body.iter().enumerate() {
                        filter.state[19 + i] = self.mag_lpf[i] - value;
                        filter.reset_variance(18 + i, math::square(self.params.mag_noise));
                    }
                }
                return;
            }
        }
        filter.state[19..22].fill(0.0);
        for i in 18..21 {
            filter.reset_variance(i, math::square(self.params.mag_noise));
        }
        if reset_heading {
            let r = math::with_yaw(math::rotation(filter.quaternion()), 0.0);
            let earth = math::rotate(r, self.mag_lpf);
            let heading = -libm::atan2f(earth[1], earth[0]) + self.declination(filter);
            filter.reset_yaw(
                heading,
                math::square(filter.params.heading_noise).max(0.0001),
            );
            filter.control |= MAG_CONSISTENT | MAG_ALIGNED;
            self.mag_heading_lpf = 0.0;
        }
        let earth = if self.wmm.is_some() {
            self.wmm_earth
        } else {
            math::rotate(math::rotation(filter.quaternion()), self.mag_lpf)
        };
        if math::norm(&core::array::from_fn::<_, 3, _>(|i| {
            filter.state[16 + i] - earth[i]
        })) > 0.01
        {
            filter.state[16..19].copy_from_slice(&earth);
            for i in 15..18 {
                filter.reset_variance(i, math::square(self.params.mag_noise));
            }
        }
    }

    fn update_wmm(&mut self, filter: &ImuEkf) -> bool {
        let Some(origin) = filter.origin else {
            return false;
        };
        let newer = filter.origin_timestamp > self.last_mag_fuse;
        if !newer
            && !(filter.horizontal_aiding() && filter.timed_out(self.last_wmm_check, 10_000_000))
        {
            return false;
        }
        let location = if filter.origin_altitude.is_some() {
            filter.global
        } else {
            origin
        };
        let Ok(field) = wmm::lookup(
            location.latitude_rad.to_degrees() as f32,
            location.longitude_rad.to_degrees() as f32,
        ) else {
            return false;
        };
        if filter.origin_altitude.is_some() {
            self.last_wmm_check = filter.delayed_us;
        }
        let changed = self.wmm.is_none_or(|previous| {
            (field.declination_deg - previous.declination_deg).abs() > 1.0
                || (field.inclination_deg - previous.inclination_deg).abs() > 1.0
                || (field.strength_gauss - previous.strength_gauss).abs() > 0.01
        });
        if changed {
            let (declination, inclination) = (
                field.declination_deg.to_radians(),
                field.inclination_deg.to_radians(),
            );
            self.wmm_earth = [
                libm::cosf(inclination) * libm::cosf(declination) * field.strength_gauss,
                libm::cosf(inclination) * libm::sinf(declination) * field.strength_gauss,
                libm::sinf(inclination) * field.strength_gauss,
            ];
            self.wmm = Some(field);
        }
        changed
    }
}
