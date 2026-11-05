//! Shared delayed EKF core and barometer/magnetometer/range aiding.
//! GNSS lives in a separate extension; the common core has no GNSS input type.
//! Pinned profiles include internal pseudo updates and are not flight-qualified.
//! Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause
//! Source mapping and license: docs/provenance.md, tools/px4_msgs/LICENSE.

use super::{
    covariance_generated::predict_covariance,
    ekf2_parameter,
    imu_downsampler::{DownSampler, ImuBuffer, MAX_IMU_BUFFER},
    math::{self, square, GlobalPosition},
    output_predictor::{OutputParams, OutputPredictor},
    reset_status::{ResetReport, ResetStatus},
    Covariance, FilterFault, FilterHealth, ImuDelta, NavOutput, ParamValue, StateVector, Timestamp,
    STATE_COUNT,
};

#[path = "baro_mag.rs"]
pub mod baro_mag;
use baro_mag::{Aids, BaroMagParams, BaroMagSample};
#[path = "range_fusion.rs"]
pub mod range;
use super::range_sensor::RangeObservation;
use range::{RangeAids, RangeParams};

const TILT_ALIGN: u64 = 1;
const IN_AIR: u64 = 1 << 7;
const DEAD_RECKONING: u64 = 1 << 29;
const FAKE_POS: u64 = 1 << 32;
const FAKE_HGT: u64 = 1 << 33;
const BAD_HEADING: u32 = 1 << 3;
const BAD_ACCEL_CLIPPING: u32 = 1 << 11;
const TILT_VARIANCE_LIMIT: f32 = square(3.0 * (core::f64::consts::PI / 180.0) as f32);

/// Rust boundary watchdog, independent of PX4 aid timeouts. A gap requires reset.
pub const IMU_FRESH_US: u64 = 100_000;

/// Extension phases follow PX4's ordering without making the common kernel
/// depend on GNSS. The no-GPS instance uses the empty implementation.
pub(super) trait ControlExtension {
    fn push(&mut self, _filter: &ImuEkf) -> Result<(), FilterFault> {
        Ok(())
    }
    fn control_horizontal(
        &mut self,
        _filter: &mut ImuEkf,
        _imu: &ImuDelta,
    ) -> Result<(), FilterFault> {
        Ok(())
    }
    fn control_height(&mut self, _filter: &mut ImuEkf) -> Result<(), FilterFault> {
        Ok(())
    }
    fn shift_altitude(&mut self, _delta: f32) -> Result<(), FilterFault> {
        Ok(())
    }
    /// Height and vertical-velocity innovation/variance for the GNSS reference.
    fn vertical_innovations(&self) -> [(f32, f32); 2] {
        [(0.0, 0.0); 2]
    }
}
struct NoExtension;
impl ControlExtension for NoExtension {}

/// Snapshot of implemented IMU parameters from the single generated catalog.
/// Changing it requires a new/reset core; it is not a parameter registry.
#[derive(Clone, Copy, Debug)]
pub struct ImuEkfParams {
    pub predict_us: i32,
    pub delay_max_ms: f32,
    pub imu_ctrl: i32,
    pub velocity_limit: f32,
    pub gyro_noise: f32,
    pub accel_noise: f32,
    pub gyro_bias_noise: f32,
    pub accel_bias_noise: f32,
    pub gyro_bias_init: f32,
    pub accel_bias_init: f32,
    pub angle_error_init: f32,
    pub no_aid_noise: f32,
    pub heading_noise: f32,
    pub gyro_bias_limit: f32,
    pub accel_bias_limit: f32,
    pub accel_bias_accel_limit: f32,
    pub accel_bias_gyro_limit: f32,
    pub accel_bias_time_constant: f32,
}

pub(super) const fn default_float(name: &str) -> f32 {
    match ekf2_parameter(name)
        .expect("implemented parameter in generated catalog")
        .default
    {
        ParamValue::Float(value) => value,
        ParamValue::Int(_) => panic!("float parameter catalog type mismatch"),
    }
}

pub(super) const fn default_int(name: &str) -> i32 {
    match ekf2_parameter(name)
        .expect("implemented parameter in generated catalog")
        .default
    {
        ParamValue::Int(value) => value,
        ParamValue::Float(_) => panic!("integer parameter catalog type mismatch"),
    }
}

impl Default for ImuEkfParams {
    fn default() -> Self {
        Self::defaults()
    }
}

impl ImuEkfParams {
    pub(super) const fn defaults() -> Self {
        Self {
            predict_us: default_int("EKF2_PREDICT_US"),
            delay_max_ms: default_float("EKF2_DELAY_MAX"),
            imu_ctrl: default_int("EKF2_IMU_CTRL"),
            velocity_limit: default_float("EKF2_VEL_LIM"),
            gyro_noise: default_float("EKF2_GYR_NOISE"),
            accel_noise: default_float("EKF2_ACC_NOISE"),
            gyro_bias_noise: default_float("EKF2_GYR_B_NOISE"),
            accel_bias_noise: default_float("EKF2_ACC_B_NOISE"),
            gyro_bias_init: default_float("EKF2_GBIAS_INIT"),
            accel_bias_init: default_float("EKF2_ABIAS_INIT"),
            angle_error_init: default_float("EKF2_ANGERR_INIT"),
            no_aid_noise: default_float("EKF2_NOAID_NOISE"),
            heading_noise: default_float("EKF2_HEAD_NOISE"),
            gyro_bias_limit: default_float("EKF2_GYR_B_LIM"),
            accel_bias_limit: default_float("EKF2_ABL_LIM"),
            accel_bias_accel_limit: default_float("EKF2_ABL_ACCLIM"),
            accel_bias_gyro_limit: default_float("EKF2_ABL_GYRLIM"),
            accel_bias_time_constant: default_float("EKF2_ABL_TAU"),
        }
    }
}

impl ImuEkfParams {
    pub(super) fn buffer_length(&self) -> Result<usize, FilterFault> {
        use core::sync::atomic::Ordering;
        let values = [
            ("EKF2_PREDICT_US", self.predict_us as f32),
            ("EKF2_DELAY_MAX", self.delay_max_ms),
            ("EKF2_IMU_CTRL", self.imu_ctrl as f32),
            ("EKF2_VEL_LIM", self.velocity_limit),
            ("EKF2_GYR_NOISE", self.gyro_noise),
            ("EKF2_ACC_NOISE", self.accel_noise),
            ("EKF2_GYR_B_NOISE", self.gyro_bias_noise),
            ("EKF2_ACC_B_NOISE", self.accel_bias_noise),
            ("EKF2_GBIAS_INIT", self.gyro_bias_init),
            ("EKF2_ABIAS_INIT", self.accel_bias_init),
            ("EKF2_ANGERR_INIT", self.angle_error_init),
            ("EKF2_NOAID_NOISE", self.no_aid_noise),
            ("EKF2_HEAD_NOISE", self.heading_noise),
            ("EKF2_GYR_B_LIM", self.gyro_bias_limit),
            ("EKF2_ABL_LIM", self.accel_bias_limit),
            ("EKF2_ABL_ACCLIM", self.accel_bias_accel_limit),
            ("EKF2_ABL_GYRLIM", self.accel_bias_gyro_limit),
            ("EKF2_ABL_TAU", self.accel_bias_time_constant),
        ];
        for (index, (name, value)) in values.into_iter().enumerate() {
            let Some(definition) = ekf2_parameter(name) else {
                super::LAST_OUTPUT_FAULT.store(40 + index as u32, Ordering::Relaxed);
                return Err(FilterFault::Input);
            };
            if !value.is_finite()
                || value < 0.0
                || definition.min.is_some_and(|min| value < min)
                || definition.max.is_some_and(|max| value > max)
            {
                // Retain the failing triple for the board-side diagnostic readout.
                super::FAILED_VALUE.store(value.to_bits(), Ordering::Relaxed);
                super::FAILED_MIN.store(
                    definition.min.unwrap_or(f32::NAN).to_bits(),
                    Ordering::Relaxed,
                );
                super::FAILED_MAX.store(
                    definition.max.unwrap_or(f32::NAN).to_bits(),
                    Ordering::Relaxed,
                );
                super::LAST_OUTPUT_FAULT.store(50 + index as u32, Ordering::Relaxed);
                return Err(FilterFault::Input);
            }
        }
        if !(1000..=100_000).contains(&self.predict_us) {
            super::LAST_OUTPUT_FAULT.store(70, Ordering::Relaxed);
            return Err(FilterFault::Input);
        }
        if self.imu_ctrl & !7 != 0 {
            super::LAST_OUTPUT_FAULT.store(71, Ordering::Relaxed);
            return Err(FilterFault::Input);
        }
        if self.accel_bias_time_constant <= 0.0 {
            super::LAST_OUTPUT_FAULT.store(72, Ordering::Relaxed);
            return Err(FilterFault::Input);
        }
        let length =
            libm::ceilf(self.delay_max_ms / (self.predict_us as f32 / 1000.0)).max(2.0) as usize;
        if length > MAX_IMU_BUFFER {
            super::LAST_OUTPUT_FAULT.store(73, Ordering::Relaxed);
            return Err(FilterFault::Input);
        }
        Ok(length)
    }
}

/// Bring-up diagnostic: run the baro/mag and range aid constructors without
/// touching filter state. Called only from the estimator module probe.
pub fn constructor_probe(
    aiding: baro_mag::BaroMagParams,
    range: range::RangeParams,
    length: usize,
) -> Result<(), FilterFault> {
    baro_mag::Aids::new(aiding, length)?;
    range::RangeAids::new(range, length)?;
    Ok(())
}

/// Streaming host/MCU-buildable mathematical core; firmware does not instantiate
/// it yet. Raw control bits are PX4 fixture state, not vehicle arming state.
pub struct ImuEkf {
    aiding_params: Option<BaroMagParams>,
    aiding: Option<Aids>,
    pending_baro: Option<baro_mag::BaroAltitude>,
    range_params: Option<RangeParams>,
    range: Option<RangeAids>,
    pub(super) params: ImuEkfParams,
    horizontal_position_noise: f32,
    horizontal_velocity_noise: f32,
    downsampler: DownSampler,
    buffer: ImuBuffer,
    output_params: OutputParams,
    output_predictor: OutputPredictor,
    resets: ResetStatus,
    last_good_vertical_accel: Timestamp,
    last_bad_vertical_accel: Timestamp,
    pub(super) state: StateVector,
    pub(super) covariance: Covariance,
    pub(super) global: GlobalPosition,
    pub(super) origin: Option<GlobalPosition>,
    pub(super) origin_timestamp: Timestamp,
    pub(super) origin_altitude: Option<f32>,
    pub(super) height_reference: i32,
    pub(super) altitude_delta: f32,
    pub(super) previous_control: u64,
    pub(super) last_horizontal_position_fuse: Timestamp,
    pub(super) last_horizontal_velocity_fuse: Timestamp,
    pub(super) last_vertical_velocity_fuse: Timestamp,
    pub(super) last_terrain_fuse: Timestamp,
    pub(super) terrain_valid: bool,
    pub(super) horizontal_accel_lpf: [f32; 2],
    last_known_global: GlobalPosition,
    earth_rate: math::Vector3,
    earth_rate_latitude: f64,
    accel_lpf: math::Vector3,
    gyro_lpf: math::Vector3,
    pub(super) average_dt_s: f32,
    first_sample: bool,
    input_seen: bool,
    initialized: bool,
    pub(super) latest_us: Timestamp,
    pub(super) delayed_us: Timestamp,
    pub(super) control: u64,
    pub(super) px4_fault: u32,
    fault: Option<FilterFault>,
    gyro_bias_inhibit: [bool; 3],
    accel_bias_inhibit: [bool; 3],
    gyro_magnitude: f32,
    accel_magnitude: f32,
    clipping_count: [u16; 3],
    pub(super) last_heading_fuse: Timestamp,
    last_fake_pos_fuse: Timestamp,
    last_fake_hgt_fuse: Timestamp,
    pub(super) last_height_fuse: Timestamp,
    last_horizontal_aiding: Timestamp,
    horizontal_position_dead_reckoning: bool,
    last_vertical_position_aiding: Timestamp,
    vertical_position_dead_reckoning: bool,
    vertical_velocity_dead_reckoning: bool,
    last_vertical_velocity_aiding: Timestamp,
    pub(super) mag_alignment_start: Timestamp,
}

impl ImuEkf {
    pub fn new(params: &ImuEkfParams, t0: Timestamp) -> Result<Self, FilterFault> {
        let mut result = Self::dormant();
        result.initialize(params, t0)?;
        Ok(result)
    }

    /// Reset into an existing allocation. The constant template is copied from
    /// flash directly; no full ImuEkf value crosses an async call boundary.
    #[inline(never)]
    fn initialize(&mut self, params: &ImuEkfParams, t0: Timestamp) -> Result<(), FilterFault> {
        let buffer_length = params.buffer_length()?;
        let mut params = *params;
        // Same parameter clamps as fixed Ekf::updateParameters().
        params.gyro_noise = params.gyro_noise.clamp(0.0, 1.0);
        params.accel_noise = params.accel_noise.clamp(0.0, 1.0);
        params.gyro_bias_noise = params.gyro_bias_noise.clamp(0.0, 1.0);
        params.accel_bias_noise = params.accel_bias_noise.clamp(0.0, 1.0);
        const INITIAL: ImuEkf = ImuEkf::dormant();
        *self = INITIAL;
        self.params = params;
        self.horizontal_position_noise = params.no_aid_noise;
        self.downsampler = DownSampler::new(params.predict_us as u32);
        self.buffer.reset_to(buffer_length);
        self.output_predictor
            .reset_to(buffer_length, self.output_params);
        self.latest_us = t0;
        self.delayed_us = t0;
        self.fault = None;
        Ok(())
    }

    pub(super) const fn dormant() -> Self {
        let params = ImuEkfParams::defaults();
        let mut state = [0.0; 25];
        state[0] = 1.0;
        Self {
            aiding_params: None,
            aiding: None,
            pending_baro: None,
            range_params: None,
            range: None,
            params,
            horizontal_position_noise: params.no_aid_noise,
            horizontal_velocity_noise: 0.5,
            downsampler: DownSampler::dormant(),
            buffer: ImuBuffer::new(2),
            output_params: OutputParams::defaults(),
            output_predictor: OutputPredictor::new(2, OutputParams::defaults()),
            resets: ResetStatus::new(),
            last_good_vertical_accel: 0,
            last_bad_vertical_accel: 0,
            state,
            covariance: [[0.0; 24]; 24],
            global: GlobalPosition::ZERO,
            origin: None,
            origin_timestamp: 0,
            origin_altitude: None,
            height_reference: 4,
            altitude_delta: 0.0,
            previous_control: IN_AIR,
            last_horizontal_position_fuse: 0,
            last_horizontal_velocity_fuse: 0,
            last_vertical_velocity_fuse: 0,
            last_terrain_fuse: 0,
            terrain_valid: false,
            horizontal_accel_lpf: [0.0; 2],
            last_known_global: GlobalPosition::ZERO,
            earth_rate: [0.0; 3],
            earth_rate_latitude: 0.0,
            accel_lpf: [0.0; 3],
            gyro_lpf: [0.0; 3],
            average_dt_s: 0.01,
            first_sample: true,
            input_seen: false,
            initialized: false,
            latest_us: 0,
            delayed_us: 0,
            // Pinned Ekf::reset() defaults to in_air, with no system flag input.
            control: IN_AIR,
            px4_fault: 0,
            fault: Some(FilterFault::Input),
            gyro_bias_inhibit: [false; 3],
            accel_bias_inhibit: [false; 3],
            gyro_magnitude: 0.0,
            accel_magnitude: 0.0,
            clipping_count: [0; 3],
            last_heading_fuse: 0,
            last_fake_pos_fuse: 0,
            last_fake_hgt_fuse: 0,
            last_height_fuse: 0,
            last_horizontal_aiding: 0,
            horizontal_position_dead_reckoning: true,
            last_vertical_position_aiding: 0,
            vertical_position_dead_reckoning: true,
            vertical_velocity_dead_reckoning: true,
            last_vertical_velocity_aiding: 0,
            mag_alignment_start: 0,
        }
    }

    /// Explicit recovery. No automatic acceptance of samples after a fault.
    pub fn reset(&mut self, t0: Timestamp) -> Result<(), FilterFault> {
        let output_params = self.output_params;
        let (params, aids, range) = (self.params, self.aiding_params, self.range_params);
        self.initialize(&params, t0)?;
        if let Some(aids) = aids {
            self.enable_baro_mag(aids)?;
        }
        if let Some(range) = range {
            self.enable_range(range)?;
        }
        self.configure_output(output_params)?;
        Ok(())
    }

    /// Output configuration is a snapshot and may only change before IMU input.
    pub fn configure_output(&mut self, params: OutputParams) -> Result<(), FilterFault> {
        use core::sync::atomic::Ordering;
        if params.validate().is_err() {
            super::LAST_OUTPUT_FAULT.store(1, Ordering::Relaxed);
            return Err(FilterFault::Input);
        }
        if self.input_seen {
            super::LAST_OUTPUT_FAULT.store(2, Ordering::Relaxed);
            return Err(FilterFault::Input);
        }
        self.output_params = params;
        // Validate a stack copy: distinguishes corrupted static memory from a
        // corrupted stack frame when the two disagree.
        let snapshot = self.params;
        let length = match snapshot.buffer_length() {
            Ok(length) => length,
            Err(_) => {
                // Preserve the inner buffer_length diagnostic when it recorded one.
                if super::LAST_OUTPUT_FAULT.load(Ordering::Relaxed) < 40 {
                    super::LAST_OUTPUT_FAULT.store(3, Ordering::Relaxed);
                }
                return Err(FilterFault::Input);
            }
        };
        self.output_predictor.reset_to(length, params);
        Ok(())
    }

    pub fn with_range(
        params: &ImuEkfParams,
        aiding: BaroMagParams,
        range: RangeParams,
        t0: Timestamp,
    ) -> Result<Self, FilterFault> {
        let mut result = Self::dormant();
        result.initialize_range(params, aiding, range, t0)?;
        Ok(result)
    }

    #[inline(never)]
    pub(super) fn initialize_range(
        &mut self,
        params: &ImuEkfParams,
        aiding: BaroMagParams,
        range: RangeParams,
        t0: Timestamp,
    ) -> Result<(), FilterFault> {
        self.initialize(params, t0)?;
        self.enable_baro_mag(aiding)?;
        self.enable_range(range)
    }

    /// Diagnostic variant of `initialize_range` with the same behavior split
    /// across separately reportable steps. Semantics identical on success.
    pub fn initialize_only(
        &mut self,
        params: &ImuEkfParams,
        t0: Timestamp,
    ) -> Result<(), FilterFault> {
        self.initialize(params, t0)
    }

    pub fn enable_baro_mag_probe(&mut self, aiding: BaroMagParams) -> Result<(), FilterFault> {
        self.enable_baro_mag(aiding)
    }

    pub fn enable_range_probe(&mut self, range: RangeParams) -> Result<(), FilterFault> {
        self.enable_range(range)
    }

    #[inline(never)]
    fn enable_baro_mag(&mut self, aiding: BaroMagParams) -> Result<(), FilterFault> {
        self.aiding = Some(Aids::new(aiding, self.params.buffer_length()?)?);
        self.aiding_params = Some(aiding);
        Ok(())
    }

    #[inline(never)]
    fn enable_range(&mut self, range: RangeParams) -> Result<(), FilterFault> {
        self.range = Some(RangeAids::new(range, self.params.buffer_length()?)?);
        self.range_params = Some(range);
        self.state[24] = range.minimum_ground_clearance_m;
        Ok(())
    }

    pub(super) fn initial_horizontal_noise(&mut self, position: f32, velocity: f32) {
        self.horizontal_position_noise = position;
        self.horizontal_velocity_noise = velocity;
    }

    fn push_range(&mut self, sample: Option<RangeObservation>) -> Result<(), FilterFault> {
        if let Some(sample) = sample {
            let Some(mut aids) = self.range.take() else {
                return self.fail(FilterFault::Input);
            };
            let result = aids.push(sample, self);
            self.range = Some(aids);
            if let Err(fault) = result {
                return self.fail(fault);
            }
        }
        Ok(())
    }

    pub fn with_baro_mag(
        params: &ImuEkfParams,
        aiding: BaroMagParams,
        t0: Timestamp,
    ) -> Result<Self, FilterFault> {
        let mut result = Self::new(params, t0)?;
        result.enable_baro_mag(aiding)?;
        Ok(result)
    }

    fn push_aiding(&mut self, sample: Option<BaroMagSample>) -> Result<(), FilterFault> {
        if let Some(sample) = self.pending_baro.take() {
            let Some(mut aiding) = self.aiding.take() else {
                return self.fail(FilterFault::Input);
            };
            let result = aiding.push_baro(sample, self);
            self.aiding = Some(aiding);
            if let Err(fault) = result {
                return self.fail(fault);
            }
        }
        if let Some(sample) = sample {
            let Some(mut aiding) = self.aiding.take() else {
                return self.fail(FilterFault::Input);
            };
            let result = aiding.push(sample, self);
            self.aiding = Some(aiding);
            if let Err(fault) = result {
                return self.fail(fault);
            }
        }
        Ok(())
    }

    pub fn state(&self) -> &StateVector {
        &self.state
    }
    pub fn covariance(&self) -> &Covariance {
        &self.covariance
    }
    pub fn timestamp(&self) -> Timestamp {
        self.delayed_us
    }
    pub fn latest_timestamp(&self) -> Timestamp {
        self.latest_us
    }
    pub fn control_status(&self) -> u64 {
        self.control
    }
    pub fn fault_status(&self) -> u32 {
        self.px4_fault
    }

    pub(super) fn local_position_validity(&self) -> [bool; 3] {
        [
            !self.horizontal_position_dead_reckoning,
            !self.vertical_position_dead_reckoning,
            !self.vertical_velocity_dead_reckoning,
        ]
    }

    pub(super) fn range_height_limits(&self) -> Option<[f32; 2]> {
        if self.control & ((1 << 9) | (1 << 10) | (1 << 11)) != (1 << 10) {
            return None;
        }
        self.range_params
            .map(|p| [p.sensor.minimum_m, 0.9 * p.sensor.maximum_m])
    }

    pub fn aid_status(&self) -> Option<(baro_mag::AidStatus, baro_mag::AidStatus)> {
        self.aiding
            .as_ref()
            .map(|aids| (aids.baro_status, aids.mag_status))
    }

    pub fn range_status(&self) -> Option<baro_mag::AidStatus> {
        self.range.as_ref().map(|aids| aids.status)
    }

    pub fn global_position(&self) -> (f64, f64, f32) {
        (
            self.global.latitude_rad.to_degrees(),
            self.global.longitude_rad.to_degrees(),
            self.global.altitude_m,
        )
    }

    pub fn output(&self) -> NavOutput {
        let global = self.output_predictor.global_position();
        let xy = global.project(self.origin.unwrap_or_default());
        NavOutput {
            timestamp: self.latest_us,
            q_body_to_ned: self.output_predictor.quaternion(),
            velocity_ned_m_s: self.output_predictor.velocity(),
            position_ned_m: [
                xy[0],
                xy[1],
                -(global.altitude_m - self.origin_altitude.unwrap_or(0.0)),
            ],
            height_msl_m: global.altitude_m,
        }
    }
    pub fn output_global_position(&self) -> (f64, f64, f32) {
        let global = self.output_predictor.global_position();
        (
            global.latitude_rad.to_degrees(),
            global.longitude_rad.to_degrees(),
            global.altitude_m,
        )
    }
    pub fn output_tracking_error(&self) -> [f32; 3] {
        self.output_predictor.tracking_error()
    }
    pub fn reset_report(&self) -> ResetReport {
        self.resets.report
    }
    pub(super) fn quaternion_reset_this_update(&self) -> bool {
        self.resets.quaternion_changed()
    }
    pub(super) fn record_position_reset(&mut self, before: [f32; 2]) {
        let after = self.global.project(self.origin.unwrap_or_default());
        self.resets
            .horizontal_position([after[0] - before[0], after[1] - before[1]]);
    }
    pub(super) fn record_terrain_reset(&mut self, delta: f32) {
        self.resets.hagl(delta);
    }
    /// Bring-up diagnostics: the ACTIVE clamped IMU parameter set. Distinguishes
    /// memory corruption from value-dependent validation after a failed reconfigure.
    pub fn debug_params(&self) -> ImuEkfParams {
        self.params
    }

    pub fn debug_input_seen(&self) -> bool {
        self.input_seen
    }

    /// Bring-up diagnostic: detect corruption of the active parameter copy by
    /// comparing every field with the expected clamped input.
    pub fn params_differ(&self, expected: &ImuEkfParams) -> bool {
        macro_rules! differs {
            ($($field:ident),+ $(,)?) => {
                $(self.params.$field != expected.$field)||+
            };
        }
        differs!(
            predict_us,
            delay_max_ms,
            imu_ctrl,
            velocity_limit,
            gyro_noise,
            accel_noise,
            gyro_bias_noise,
            accel_bias_noise,
            gyro_bias_init,
            accel_bias_init,
            angle_error_init,
            no_aid_noise,
            heading_noise,
            gyro_bias_limit,
            accel_bias_limit,
            accel_bias_accel_limit,
            accel_bias_gyro_limit,
            accel_bias_time_constant
        )
    }

    pub fn output_vertical_derivative(&self) -> f32 {
        self.output_predictor.vertical_derivative()
    }
    pub fn output_unaided_yaw(&self) -> f32 {
        self.output_predictor.unaided_yaw()
    }
    pub fn angular_velocity_and_reset(&mut self) -> [f32; 3] {
        self.output_predictor.angular_velocity_and_reset()
    }
    pub fn velocity_derivative_and_reset(&mut self) -> [f32; 3] {
        self.output_predictor.velocity_derivative_and_reset()
    }
    pub(super) fn propagate_position_reset(&mut self) {
        self.output_predictor.reset_lat_lon(self.global);
    }
    pub(super) fn propagate_velocity_reset(&mut self, previous: [f32; 3]) {
        self.output_predictor
            .reset_horizontal_velocity([self.state[4] - previous[0], self.state[5] - previous[1]]);
        self.output_predictor
            .reset_vertical_velocity(self.state[6] - previous[2]);
        self.resets
            .horizontal_velocity([self.state[4] - previous[0], self.state[5] - previous[1]]);
        self.resets.vertical_velocity(self.state[6] - previous[2]);
    }

    pub(super) fn horizontal_aiding(&self) -> bool {
        self.control & ((1 << 2) | (1 << 44)) != 0
    }
    pub(super) fn aiding_height_reference(&self) -> i32 {
        self.aiding_params.map(|p| p.height_reference).unwrap_or(4)
    }
    fn vertical_position_aiding(&self) -> bool {
        self.control & ((1 << 11) | (1 << 9) | (1 << 10)) != 0
    }
    pub(super) fn vertical_velocity_aiding(&self) -> bool {
        self.control & (1 << 44) != 0
    }
    pub(super) fn height_reset_required(&self) -> bool {
        self.timed_out(self.last_good_vertical_accel, 500_000)
            || self.timed_out(self.last_height_fuse, 5_000_000)
    }
    fn check_vertical_acceleration(
        &mut self,
        checks: [(u8, f32, f32); 4],
        clipping_frequently: bool,
    ) {
        let ratios = checks.map(|(_, innovation, variance)| {
            if variance < f32::EPSILON {
                0.0
            } else {
                innovation / libm::sqrtf(variance)
            }
        });
        let medium = ratios.iter().any(|ratio| *ratio > 3.0);
        let high = (0..4).any(|i| {
            (0..4).any(|j| checks[i].0 != checks[j].0 && ratios[i] > 3.0 && ratios[j] > 1.0)
        });
        let bad = high || (clipping_frequently && medium);
        if bad {
            self.last_bad_vertical_accel = self.delayed_us;
        } else {
            self.last_good_vertical_accel = self.delayed_us;
        }
        let active = if self.px4_fault & (1 << 10) != 0 {
            self.last_bad_vertical_accel != 0
                && self.last_bad_vertical_accel.saturating_add(3_000_000) > self.delayed_us
        } else {
            bad
        };
        if active {
            self.px4_fault |= 1 << 10;
        } else {
            self.px4_fault &= !(1 << 10);
        }
    }
    pub(super) fn reset_vertical_velocity(&mut self, velocity: f32, variance: f32) {
        let delta = velocity - self.state[6];
        self.state[6] = velocity;
        if variance.is_finite() {
            self.reset_variance(5, variance.max(0.0001));
        }
        self.reset_variance(8, self.covariance[8][8]);
        self.output_predictor.reset_vertical_velocity(delta);
        self.resets.vertical_velocity(delta);
        self.last_vertical_velocity_fuse = self.delayed_us;
    }

    pub(super) fn reset_altitude(&mut self, altitude: f32, variance: f32) {
        let delta = -(altitude - self.global.altitude_m);
        self.global.altitude_m = altitude;
        if variance.is_finite() {
            self.reset_variance(8, variance.max(0.0001));
        }
        if self.range_params.is_some() {
            self.state[24] += delta;
            self.resets.hagl(delta);
        }
        self.resets.height(delta);
        self.altitude_delta += delta;
        self.output_predictor.reset_altitude(altitude);
        self.last_height_fuse = self.delayed_us;
    }

    pub(super) fn initialize_altitude(&mut self, altitude: f32, variance: f32) {
        if self.origin_altitude.is_none() {
            self.origin_altitude = Some(if !self.vertical_position_dead_reckoning {
                altitude - self.global.altitude_m
            } else {
                altitude
            });
        }
        self.reset_altitude(altitude, variance);
    }

    pub(super) fn reset_yaw(&mut self, yaw: f32, variance: f32) {
        let previous = self.quaternion();
        let q = math::from_rotation(math::with_yaw(math::rotation(self.quaternion()), yaw));
        self.state[..4].copy_from_slice(&q);
        if variance.is_finite() && variance > f32::EPSILON {
            self.reset_variance(2, variance);
        }
        self.last_heading_fuse = self.delayed_us;
        self.output_predictor
            .reset_quaternion(math::normalize(math::multiply(q, math::inverse(previous))));
        self.resets
            .quaternion(math::normalize(math::multiply(q, math::inverse(previous))));
    }

    /// Relative attitude-core validity only; no navigation validity is implied.
    pub fn health(&self) -> FilterHealth {
        FilterHealth {
            valid: self.initialized
                && self.control & TILT_ALIGN != 0
                && self.fault.is_none()
                && self.px4_fault == 0,
            fault: self.fault,
        }
    }

    fn fail<T>(&mut self, fault: FilterFault) -> Result<T, FilterFault> {
        self.fault = Some(fault);
        Err(fault)
    }

    pub fn check_fresh(&mut self, now: Timestamp) -> Result<(), FilterFault> {
        if let Some(fault) = self.fault {
            return Err(fault);
        }
        if now < self.latest_us {
            return self.fail(FilterFault::Input);
        }
        if now - self.latest_us > IMU_FRESH_US {
            return self.fail(FilterFault::Timeout);
        }
        Ok(())
    }

    /// Consume one body-frame integral. `true` means one delayed EKF update,
    /// not one raw IMU sample, and not that absolute navigation is valid.
    pub fn update(&mut self, input: &ImuDelta) -> Result<bool, FilterFault> {
        self.update_aided(input, None)
    }

    pub fn update_aided(
        &mut self,
        input: &ImuDelta,
        aiding: Option<BaroMagSample>,
    ) -> Result<bool, FilterFault> {
        self.update_extended(input, aiding, None, &mut NoExtension)
    }

    pub fn update_range(
        &mut self,
        input: &ImuDelta,
        aiding: Option<BaroMagSample>,
        range: Option<RangeObservation>,
    ) -> Result<bool, FilterFault> {
        self.update_extended(input, aiding, range, &mut NoExtension)
    }

    pub(super) fn pending_baro(&mut self, sample: Option<baro_mag::BaroAltitude>) {
        self.pending_baro = sample;
    }

    pub fn update_baro(
        &mut self,
        input: &ImuDelta,
        baro: Option<baro_mag::BaroAltitude>,
        range: Option<RangeObservation>,
    ) -> Result<bool, FilterFault> {
        self.pending_baro = baro;
        self.update_extended(input, None, range, &mut NoExtension)
    }

    pub(super) fn update_extended<X: ControlExtension>(
        &mut self,
        input: &ImuDelta,
        aiding: Option<BaroMagSample>,
        range: Option<RangeObservation>,
        extension: &mut X,
    ) -> Result<bool, FilterFault> {
        if let Some(fault) = self.fault {
            return Err(fault);
        }
        if self.initialized && !self.numerics_valid() {
            return self.fail(FilterFault::Numerics);
        }
        let dt = [input.delta_angle_dt_s, input.delta_velocity_dt_s];
        if input.timestamp <= self.latest_us
            || dt
                .iter()
                .any(|value| !value.is_finite() || !(1e-6..=1.0).contains(value))
            || input
                .delta_angle_rad
                .iter()
                .chain(&input.delta_velocity_m_s)
                .any(|value| !value.is_finite())
            || dt
                .iter()
                .any(|value| libm::roundf(*value * 1e6) > input.timestamp as f32)
        {
            return self.fail(FilterFault::Input);
        }
        if self.input_seen && input.timestamp - self.latest_us > IMU_FRESH_US {
            return self.fail(FilterFault::Timeout);
        }
        if !self.input_seen {
            // PX4 initialise_interface starts both horizons at the first IMU,
            // before a downsampled sample exists. Later warmup repeats remain.
            self.delayed_us = input.timestamp;
        }
        self.latest_us = input.timestamp;
        self.input_seen = true;
        self.output_predictor.predict(input);
        if !self.output_predictor.numerics_valid() {
            return self.fail(FilterFault::Numerics);
        }
        let downsampled = self.downsampler.update(input);
        if !self.downsampler.numerics_valid() {
            return self.fail(FilterFault::Numerics);
        }
        let Some(mut sample) = downsampled else {
            self.push_aiding(aiding)?;
            self.push_range(range)?;
            if let Err(fault) = extension.push(self) {
                return self.fail(fault);
            }
            return Ok(false);
        };
        if sample
            .delta_angle_rad
            .iter()
            .chain(&sample.delta_velocity_m_s)
            .any(|value| !value.is_finite())
        {
            return self.fail(FilterFault::Numerics);
        }
        let period = self.params.predict_us as f32 * 1e-6;
        sample.delta_angle_dt_s = sample.delta_angle_dt_s.clamp(0.5 * period, 2.0 * period);
        sample.delta_velocity_dt_s = sample.delta_velocity_dt_s.clamp(0.5 * period, 2.0 * period);
        let delayed = self.buffer.push(sample);
        self.delayed_us = delayed.timestamp;
        self.push_aiding(aiding)?;
        self.push_range(range)?;
        if let Err(fault) = extension.push(self) {
            return self.fail(fault);
        }
        let mean_dt = (0.5 * (delayed.delta_angle_dt_s + delayed.delta_velocity_dt_s))
            .clamp(0.5 * period, 2.0 * period);
        let accel = delayed
            .delta_velocity_m_s
            .map(|value| value * (1.0 / delayed.delta_velocity_dt_s));
        let gyro = delayed
            .delta_angle_rad
            .map(|value| value * (1.0 / delayed.delta_angle_dt_s));
        if self.first_sample {
            self.accel_lpf = accel;
            self.gyro_lpf = gyro;
            self.average_dt_s = mean_dt;
            self.first_sample = false;
        } else {
            let alpha = 0.01_f32 / (0.09_f32 + 0.01_f32);
            for axis in 0..3 {
                self.accel_lpf[axis] += alpha * (accel[axis] - self.accel_lpf[axis]);
                self.gyro_lpf[axis] += alpha * (gyro[axis] - self.gyro_lpf[axis]);
            }
            self.average_dt_s = 0.99 * self.average_dt_s + 0.01 * mean_dt;
        }
        if !self.initialized {
            let accel_norm = math::norm(&self.accel_lpf);
            if !(0.8 * math::GRAVITY..=1.2 * math::GRAVITY).contains(&accel_norm)
                || math::norm(&self.gyro_lpf) > 15.0_f32.to_radians()
            {
                return Ok(false);
            }
            let q = math::align_tilt(self.accel_lpf);
            self.state[..4].copy_from_slice(&q);
            self.initialize_covariance();
            self.output_predictor.align(
                q,
                [self.state[4], self.state[5], self.state[6]],
                self.global,
            );
            self.initialized = true;
        }
        self.update_bias_inhibit(&delayed, accel, gyro);
        self.predict_covariance(&delayed, accel);
        self.predict_state(&delayed);
        if let Err(fault) = self.control_fusion(&delayed, extension) {
            return self.fail(fault);
        }
        if !self.numerics_valid() {
            return self.fail(FilterFault::Numerics);
        }
        let q = self.quaternion();
        self.output_predictor.correct(
            self.delayed_us,
            q,
            [self.state[4], self.state[5], self.state[6]],
            self.global,
            [self.state[10], self.state[11], self.state[12]],
            [self.state[13], self.state[14], self.state[15]],
        );
        if !self.output_predictor.numerics_valid() {
            return self.fail(FilterFault::Numerics);
        }
        Ok(true)
    }

    pub(super) fn quaternion(&self) -> math::Quaternion {
        [self.state[0], self.state[1], self.state[2], self.state[3]]
    }

    fn initialize_covariance(&mut self) {
        self.covariance = [[0.0; 24]; 24];
        for (i, value) in [
            square(self.params.angle_error_init.max(0.01)),
            square(self.params.angle_error_init.max(0.01)),
            0.0,
            square(self.horizontal_velocity_noise.max(0.01)),
            square(self.horizontal_velocity_noise.max(0.01)),
            square(1.5) * square(self.horizontal_velocity_noise.max(0.01)),
            square(self.horizontal_position_noise.max(0.01)),
            square(self.horizontal_position_noise.max(0.01)),
            1.0,
            square(self.params.gyro_bias_init),
            square(self.params.gyro_bias_init),
            square(self.params.gyro_bias_init),
            square(self.params.accel_bias_init),
            square(self.params.accel_bias_init),
            square(self.params.accel_bias_init),
        ]
        .into_iter()
        .enumerate()
        {
            self.covariance[i][i] = value;
        }
        if let Some(params) = self.aiding_params {
            self.covariance[8][8] = square(params.baro_noise.max(0.01));
            for i in 15..21 {
                self.covariance[i][i] = square(params.mag_noise);
            }
        }
        if let Some(params) = self.range_params {
            self.covariance[23][23] = square(params.minimum_ground_clearance_m);
        }
    }

    fn update_bias_inhibit(
        &mut self,
        sample: &ImuDelta,
        accel: math::Vector3,
        gyro: math::Vector3,
    ) {
        let gyro_corrected =
            core::array::from_fn::<_, 3, _>(|axis| gyro[axis] - self.state[10 + axis]);
        let accel_corrected =
            core::array::from_fn::<_, 3, _>(|axis| accel[axis] - self.state[13 + axis]);
        let gyro_beta =
            1.0 - (sample.delta_angle_dt_s / self.params.accel_bias_time_constant).clamp(0.0, 1.0);
        let accel_beta = 1.0
            - (sample.delta_velocity_dt_s / self.params.accel_bias_time_constant).clamp(0.0, 1.0);
        self.gyro_magnitude = math::norm(&gyro_corrected).max(gyro_beta * self.gyro_magnitude);
        self.accel_magnitude = math::norm(&accel_corrected).max(accel_beta * self.accel_magnitude);
        let high_manoeuvre = self.gyro_magnitude > self.params.accel_bias_gyro_limit
            || self.accel_magnitude > self.params.accel_bias_accel_limit;
        let rotation = math::rotation(self.quaternion());
        for (axis, vertical_projection) in rotation[2].iter().enumerate() {
            self.gyro_bias_inhibit[axis] = self.params.imu_ctrl & 1 == 0;
            let observable = self.control & FAKE_HGT == 0
                && (self.control & FAKE_POS == 0 || vertical_projection.abs() > 0.966);
            self.accel_bias_inhibit[axis] = self.params.imu_ctrl & 2 == 0
                || high_manoeuvre
                || self.px4_fault & (1 << 10) != 0
                || sample.delta_velocity_clipping[axis]
                || !observable;
        }
    }

    fn predict_covariance(&mut self, sample: &ImuDelta, accel: math::Vector3) {
        let dt = 0.5 * (sample.delta_angle_dt_s + sample.delta_velocity_dt_s);
        let gyro_var = square(self.params.gyro_noise);
        let accel_var = sample.delta_velocity_clipping.map(|clipped| {
            square(if clipped || self.px4_fault & (1 << 10) != 0 {
                4.9
            } else {
                self.params.accel_noise
            })
        });
        self.covariance = predict_covariance(
            &self.state,
            &self.covariance,
            accel,
            accel_var,
            gyro_var,
            dt,
        );
        for (axis, variance) in accel_var.into_iter().enumerate() {
            if self.covariance[9 + axis][9 + axis] < gyro_var {
                self.covariance[9 + axis][9 + axis] += square(dt * self.params.gyro_bias_noise);
            }
            if self.covariance[12 + axis][12 + axis] < variance {
                self.covariance[12 + axis][12 + axis] += square(dt * self.params.accel_bias_noise);
            }
        }
        for row in 0..STATE_COUNT {
            for column in 0..row {
                self.covariance[row][column] = self.covariance[column][row];
            }
        }
        if let Some(params) = self.aiding_params {
            for (start, noise) in [(15, params.mag_earth_noise), (18, params.mag_bias_noise)] {
                for i in start..start + 3 {
                    if self.covariance[i][i] < square(params.mag_noise) {
                        self.covariance[i][i] += square(dt * noise.clamp(0.0, 1.0));
                    }
                }
            }
        }
        if let Some(params) = self.range_params {
            if self.height_reference != 2 {
                let noise = square(sample.delta_velocity_dt_s * params.terrain_noise_m_s)
                    + square(sample.delta_velocity_dt_s * params.terrain_gradient)
                        * (square(self.state[4]) + square(self.state[5]));
                self.covariance[23][23] += noise;
            }
        }
        self.constrain_variances();
    }

    fn predict_state(&mut self, sample: &ImuDelta) {
        if (self.global.latitude_rad - self.earth_rate_latitude).abs() > 1.0_f64.to_radians() {
            self.earth_rate_latitude = self.global.latitude_rad;
            let latitude = self.earth_rate_latitude as f32;
            self.earth_rate = [
                7.292115e-5 * libm::cosf(latitude),
                0.0,
                -7.292115e-5 * libm::sinf(latitude),
            ];
        }
        let earth_body = math::rotate_inverse(math::rotation(self.quaternion()), self.earth_rate);
        let angle = core::array::from_fn(|i| {
            (sample.delta_angle_rad[i] - self.state[10 + i] * sample.delta_angle_dt_s)
                - earth_body[i] * sample.delta_angle_dt_s
        });
        let q = math::normalize(math::multiply(
            self.quaternion(),
            math::from_axis_angle(angle),
        ));
        self.state[..4].copy_from_slice(&q);
        let delta_velocity = core::array::from_fn(|i| {
            sample.delta_velocity_m_s[i] - self.state[13 + i] * sample.delta_velocity_dt_s
        });
        let delta_earth = math::rotate(math::rotation(q), delta_velocity);
        let previous_velocity = [self.state[4], self.state[5], self.state[6]];
        let coriolis = math::cross(self.earth_rate, previous_velocity).map(|v| -2.0 * v);
        let transport = math::cross(
            self.global.transport_rate(previous_velocity),
            previous_velocity,
        )
        .map(|v| -v);
        for i in 0..3 {
            let gravity = if i == 2 { math::GRAVITY } else { 0.0 };
            self.state[4 + i] += delta_earth[i];
            self.state[4 + i] +=
                ((gravity + coriolis[i]) + transport[i]) * sample.delta_velocity_dt_s;
        }
        let delta_position = core::array::from_fn(|i| {
            (previous_velocity[i] + self.state[4 + i]) * sample.delta_velocity_dt_s * 0.5
        });
        self.global.add_ned(delta_position);
        self.state[9] = -self.global.altitude_m;
        for i in 4..7 {
            self.state[i] =
                self.state[i].clamp(-self.params.velocity_limit, self.params.velocity_limit);
        }
        let alpha = sample.delta_velocity_dt_s / (1.0 + sample.delta_velocity_dt_s);
        for (i, value) in self.horizontal_accel_lpf.iter_mut().enumerate() {
            *value += alpha * (delta_earth[i] / sample.delta_velocity_dt_s - *value);
        }
    }

    fn tilt_variance(&self) -> f32 {
        self.covariance[0][0] + self.covariance[1][1]
    }
    pub(super) fn timed_out(&self, last: Timestamp, interval: u64) -> bool {
        last == 0 || last.saturating_add(interval) < self.delayed_us
    }

    fn control_fusion<X: ControlExtension>(
        &mut self,
        sample: &ImuDelta,
        extension: &mut X,
    ) -> Result<(), FilterFault> {
        self.previous_control = self.control;
        self.resets.begin_update();
        self.altitude_delta = 0.0;
        let mut aiding = self.aiding.take();
        let mut range = self.range.take();
        let result = (|| {
            if self.control & TILT_ALIGN == 0 && self.tilt_variance() < TILT_VARIANCE_LIMIT {
                self.control |= TILT_ALIGN;
            }
            if let Some(aiding) = &mut aiding {
                aiding.control_mag(self);
            }
            extension.control_horizontal(self, sample)?;
            let limit = (1.0 / self.average_dt_s) as u16;
            for axis in 0..3 {
                if sample.delta_velocity_clipping[axis] && self.clipping_count[axis] < limit {
                    self.clipping_count[axis] += 1;
                } else if self.clipping_count[axis] > 0 {
                    self.clipping_count[axis] -= 1;
                }
            }
            let warning = self.clipping_count.map(|count| count >= limit / 2);
            let bad_clipping = warning.iter().all(|value| *value)
                || self.clipping_count.iter().any(|count| *count >= limit);
            if bad_clipping {
                self.px4_fault |= BAD_ACCEL_CLIPPING;
            } else {
                self.px4_fault &= !BAD_ACCEL_CLIPPING;
            }
            let gnss = extension.vertical_innovations();
            let baro = aiding.as_ref().map(|a| a.baro_status);
            let range_status = range.as_ref().map(|a| a.status);
            let active = |bit: u64, value: (f32, f32)| {
                if self.control & bit != 0 {
                    value
                } else {
                    (0.0, 0.0)
                }
            };
            let baro = active(
                1 << 9,
                baro.map(|a| (a.innovation[0], a.variance[0]))
                    .unwrap_or_default(),
            );
            let height = active(1 << 11, gnss[0]);
            let velocity = active(1 << 44, gnss[1]);
            let range_check = active(
                1 << 10,
                range_status
                    .map(|a| (-a.innovation[0], a.variance[0]))
                    .unwrap_or_default(),
            );
            self.check_vertical_acceleration(
                [
                    (0, baro.0, baro.1),
                    (1, height.0, height.1),
                    (1, velocity.0, velocity.1),
                    (2, range_check.0, range_check.1),
                ],
                warning[2] || bad_clipping,
            );
            // With all external aids compiled out there is no falling-likelihood
            // evidence, and no at_rest system flag to enable zero gyro/velocity.
            if let Some(aiding) = &mut aiding {
                aiding.control_baro(self)?;
            }
            extension.shift_altitude(self.altitude_delta)?;
            let before = self.altitude_delta;
            extension.control_height(self)?;
            if let Some(aiding) = &mut aiding {
                aiding.shift_altitude(self.altitude_delta - before)?;
            }
            let before = self.altitude_delta;
            if let Some(range) = &mut range {
                range.control(self)?;
            }
            let delta = self.altitude_delta - before;
            if let Some(aiding) = &mut aiding {
                aiding.shift_altitude(delta)?;
            }
            extension.shift_altitude(delta)?;
            if self.height_reference == 4 {
                let order = match self.aiding_params.map(|p| p.height_reference) {
                    Some(2) => [(2, 10), (0, 9), (1, 11)],
                    Some(0) => [(0, 9), (1, 11), (2, 10)],
                    _ => [(1, 11), (0, 9), (2, 10)],
                };
                if let Some((reference, _)) =
                    order.iter().find(|(_, bit)| self.control & (1 << bit) != 0)
                {
                    self.height_reference = *reference;
                }
            }
            if self.control & ((1 << 4) | (1 << 5)) == 0
                && self.timed_out(self.last_heading_fuse, 200_000)
            {
                let variance = if self.control & TILT_ALIGN != 0 {
                    0.25
                } else {
                    0.001
                };
                let innovation_variance = self.covariance[2][2] + variance;
                if self.control & TILT_ALIGN == 0
                    || innovation_variance - variance > square(self.params.heading_noise)
                {
                    if innovation_variance >= variance {
                        let gain = core::array::from_fn(|row| {
                            self.covariance[row][2] * (1.0 / innovation_variance)
                        });
                        let mut observation = [0.0; STATE_COUNT];
                        observation[2] = 1.0;
                        self.measurement_update(gain, observation, variance, 0.0);
                        self.last_heading_fuse = self.delayed_us;
                        self.px4_fault &= !BAD_HEADING;
                    } else {
                        self.px4_fault |= BAD_HEADING;
                        self.initialize_covariance();
                    }
                }
            }
            self.control_fake_position();
            self.control_fake_height();
            if (self.control & (1 << 2) != 0
                && self.last_horizontal_position_fuse != 0
                && self.last_horizontal_position_fuse.saturating_add(1_000_000) > self.delayed_us)
                || (self.control & (1 << 44) != 0
                    && self.last_horizontal_velocity_fuse != 0
                    && self.last_horizontal_velocity_fuse.saturating_add(1_000_000)
                        > self.delayed_us)
            {
                self.control &= !DEAD_RECKONING;
                if self.delayed_us > 1_000_000 {
                    self.last_horizontal_aiding = self.delayed_us - 1_000_000;
                }
                self.horizontal_position_dead_reckoning = false;
            } else {
                self.control |= DEAD_RECKONING;
                if self.timed_out(
                    self.last_horizontal_aiding,
                    default_int("EKF2_NOAID_TOUT") as u64,
                ) {
                    self.horizontal_position_dead_reckoning = true;
                }
            }
            if self.vertical_position_aiding() {
                self.last_vertical_position_aiding = self.last_height_fuse;
                self.vertical_position_dead_reckoning = false;
            } else if self.timed_out(
                self.last_vertical_position_aiding,
                default_int("EKF2_NOAID_TOUT") as u64,
            ) {
                self.vertical_position_dead_reckoning = true;
            }
            if self.control & (1 << 44) != 0 {
                self.last_vertical_velocity_aiding = self.last_vertical_velocity_fuse;
                self.vertical_velocity_dead_reckoning = false;
            } else if self.vertical_position_dead_reckoning
                && self.timed_out(
                    self.last_vertical_velocity_aiding,
                    default_int("EKF2_NOAID_TOUT") as u64,
                )
            {
                self.vertical_velocity_dead_reckoning = true;
            }
            Ok(())
        })();
        self.aiding = aiding;
        self.range = range;
        result
    }

    fn reset_fake_position(&mut self) {
        let previous = [self.state[4], self.state[5]];
        self.last_known_global.latitude_rad = self.global.latitude_rad;
        self.last_known_global.longitude_rad = self.global.longitude_rad;
        for axis in 0..2 {
            self.reset_variance(6 + axis, square(self.params.no_aid_noise).max(square(0.01)));
            self.state[4 + axis] = 0.0;
            self.reset_variance(3 + axis, 25.0);
        }
        self.output_predictor.reset_lat_lon(self.global);
        self.output_predictor
            .reset_horizontal_velocity([self.state[4] - previous[0], self.state[5] - previous[1]]);
        self.resets.horizontal_position([0.0; 2]);
        self.resets
            .horizontal_velocity([self.state[4] - previous[0], self.state[5] - previous[1]]);
        self.last_fake_pos_fuse = self.delayed_us;
    }

    fn control_fake_position(&mut self) {
        if self.horizontal_aiding() {
            self.control &= !FAKE_POS;
            return;
        }
        if !self.timed_out(self.last_fake_pos_fuse, 200_000) {
            return;
        }
        let variance = if self.control & TILT_ALIGN != 0 {
            square(self.params.no_aid_noise.max(1.0))
        } else {
            square(0.5)
        };
        let innovation = self.global.difference(self.last_known_global);
        let innovation_variance = [
            self.covariance[6][6] + variance,
            self.covariance[7][7] + variance,
        ];
        let rejected = (0..2).any(|i| {
            let ratio = square(innovation[i]) / (square(3.0) * innovation_variance[i]);
            !ratio.is_finite() || ratio > 1.0
        });
        let enabled = self.tilt_variance() > TILT_VARIANCE_LIMIT;
        if self.control & FAKE_POS != 0 {
            if !enabled {
                self.control &= !FAKE_POS;
                return;
            }
            if !rejected {
                for i in 0..2 {
                    self.fuse_direct(innovation[i], innovation_variance[i], variance, 6 + i, true);
                }
                self.last_fake_pos_fuse = self.delayed_us;
            }
            if self.timed_out(self.last_fake_pos_fuse, 400_000) {
                self.reset_fake_position();
            }
        } else if enabled {
            self.control |= FAKE_POS;
            self.reset_fake_position();
        }
    }

    fn reset_fake_height(&mut self) {
        let previous = self.state[6];
        self.last_known_global.altitude_m = self.global.altitude_m;
        self.state[6] = 0.0;
        self.reset_variance(5, 10.0);
        self.reset_variance(8, square(self.params.no_aid_noise).max(square(0.01)));
        self.output_predictor
            .reset_vertical_velocity(self.state[6] - previous);
        self.output_predictor.reset_altitude(self.global.altitude_m);
        self.resets.vertical_velocity(self.state[6] - previous);
        self.resets.height(0.0);
        if self.range_params.is_some() {
            self.resets.hagl(0.0);
        }
        self.last_fake_hgt_fuse = self.delayed_us;
    }

    fn control_fake_height(&mut self) {
        if self.vertical_position_aiding() {
            self.control &= !FAKE_HGT;
            return;
        }
        if !self.timed_out(self.last_fake_hgt_fuse, 200_000) {
            return;
        }
        let variance = square(self.params.no_aid_noise);
        let innovation = self.state[9] + self.last_known_global.altitude_m;
        let innovation_variance = self.covariance[8][8] + variance;
        let ratio = square(innovation) / (square(3.0) * innovation_variance);
        if self.control & FAKE_HGT != 0 {
            if ratio.is_finite() && ratio <= 1.0 && ratio < square(100.0 / 3.0) {
                self.fuse_direct(innovation, innovation_variance, variance, 8, true);
                self.last_fake_hgt_fuse = self.delayed_us;
            }
            if self.timed_out(self.last_fake_hgt_fuse, 400_000) {
                self.reset_fake_height();
            }
        } else if self.vertical_position_dead_reckoning && self.vertical_velocity_dead_reckoning {
            self.control |= FAKE_HGT;
            self.reset_fake_height();
        }
    }

    pub(super) fn reset_variance(&mut self, index: usize, variance: f32) {
        for i in 0..STATE_COUNT {
            self.covariance[i][index] = 0.0;
            self.covariance[index][i] = 0.0;
        }
        self.covariance[index][index] = variance;
    }

    pub(super) fn fuse_direct(
        &mut self,
        innovation: f32,
        innovation_variance: f32,
        variance: f32,
        index: usize,
        constrain: bool,
    ) {
        let gain = core::array::from_fn(|row| self.covariance[row][index] / innovation_variance);
        self.update_with_gain(gain, index, variance, innovation, constrain);
    }

    fn update_with_gain(
        &mut self,
        mut gain: [f32; STATE_COUNT],
        index: usize,
        variance: f32,
        innovation: f32,
        constrain: bool,
    ) {
        for axis in 0..3 {
            if self.gyro_bias_inhibit[axis] {
                gain[9 + axis] = 0.0;
            }
            if self.accel_bias_inhibit[axis] {
                gain[12 + axis] = 0.0;
            }
        }
        if self.aiding_params.is_some() && self.control & baro_mag::MAG == 0 {
            gain[15..21].fill(0.0);
        }
        let ph = self.covariance[index];
        for (i, gain_i) in gain.iter().enumerate() {
            for (j, ph_j) in ph.iter().enumerate() {
                self.covariance[i][j] -= gain_i * ph_j;
            }
        }
        let ph: [f32; STATE_COUNT] = core::array::from_fn(|i| self.covariance[i][index]);
        for i in 0..STATE_COUNT {
            for j in 0..=i {
                self.covariance[i][j] =
                    self.covariance[i][j] - ph[i] * gain[j] + gain[i] * variance * gain[j];
                self.covariance[j][i] = self.covariance[i][j];
            }
        }
        if constrain {
            self.constrain_variances();
        }
        self.correct_state(gain, innovation);
    }

    fn correct_state(&mut self, gain: [f32; STATE_COUNT], innovation: f32) {
        let delta = math::from_axis_angle([
            gain[0] * (-innovation),
            gain[1] * (-innovation),
            gain[2] * (-innovation),
        ]);
        let q = math::normalize(math::multiply(delta, self.quaternion()));
        self.state[..4].copy_from_slice(&q);
        let position_delta = core::array::from_fn(|i| gain[6 + i] * (-innovation));
        self.global.add_ned(position_delta);
        self.state[7] = 0.0;
        self.state[8] = 0.0;
        self.state[9] = -self.global.altitude_m;
        for i in 0..3 {
            self.state[4 + i] = (self.state[4 + i] - gain[3 + i] * innovation).clamp(-1e3, 1e3);
            self.state[10 + i] = (self.state[10 + i] - gain[9 + i] * innovation)
                .clamp(-self.params.gyro_bias_limit, self.params.gyro_bias_limit);
            self.state[13 + i] = (self.state[13 + i] - gain[12 + i] * innovation)
                .clamp(-self.params.accel_bias_limit, self.params.accel_bias_limit);
        }
        if self.control & baro_mag::MAG != 0 {
            for i in 0..3 {
                self.state[16 + i] =
                    (self.state[16 + i] - gain[15 + i] * innovation).clamp(-1.0, 1.0);
                self.state[19 + i] =
                    (self.state[19 + i] - gain[18 + i] * innovation).clamp(-0.5, 0.5);
            }
        }
        if self.range_params.is_some() {
            self.state[24] = (self.state[24] - gain[23] * innovation).clamp(-1e4, 1e4);
        }
    }

    /// Fixed PX4 measurementUpdate for a general error-state Jacobian.
    /// The second PH uses the already updated, potentially nonsymmetric P.
    pub(super) fn measurement_update(
        &mut self,
        mut gain: [f32; STATE_COUNT],
        observation: [f32; STATE_COUNT],
        variance: f32,
        innovation: f32,
    ) {
        for axis in 0..3 {
            if self.gyro_bias_inhibit[axis] {
                gain[9 + axis] = 0.0;
            }
            if self.accel_bias_inhibit[axis] {
                gain[12 + axis] = 0.0;
            }
        }
        if self.aiding_params.is_some() && self.control & baro_mag::MAG == 0 {
            gain[15..21].fill(0.0);
        }
        let ph = self.project_covariance(observation);
        for (i, gain_i) in gain.iter().enumerate() {
            for (j, ph_j) in ph.iter().enumerate() {
                self.covariance[i][j] -= gain_i * ph_j;
            }
        }
        let ph = self.project_covariance(observation);
        for i in 0..STATE_COUNT {
            for j in 0..=i {
                self.covariance[i][j] =
                    self.covariance[i][j] - ph[i] * gain[j] + gain[i] * variance * gain[j];
                self.covariance[j][i] = self.covariance[i][j];
            }
        }
        self.constrain_variances();
        self.correct_state(gain, innovation);
    }

    pub(super) fn project_covariance(&self, observation: [f32; STATE_COUNT]) -> [f32; STATE_COUNT] {
        self.covariance.map(|row| {
            row.iter()
                .zip(observation)
                .fold(0.0, |sum, (value, h)| sum + value * h)
        })
    }

    fn constrain_group(&mut self, start: usize, min: f32, max: f32) {
        for i in start..start + 3 {
            if self.covariance[i][i] < min {
                self.covariance[i][i] = min;
            } else if self.covariance[i][i] > max {
                let variance = 10.0 * self.covariance[i][i];
                let innovation_variance = self.covariance[i][i] + variance;
                self.fuse_direct(0.0, innovation_variance, variance, i, false);
            }
        }
    }

    fn constrain_variances(&mut self) {
        self.constrain_group(0, 1e-9, 1.0);
        self.constrain_group(3, 1e-6, 1e6);
        self.constrain_group(6, 1e-6, 1e6);
        for start in [9, 12] {
            let maximum = (start..start + 3)
                .map(|i| self.covariance[i][i])
                .fold(0.0, f32::max)
                .clamp(1e-9, 1.0);
            self.constrain_group(start, (maximum / 1e6).clamp(1e-9, 1.0), maximum);
        }
        if self.control & baro_mag::MAG != 0 {
            for start in [15, 18] {
                let maximum = (start..start + 3)
                    .map(|i| self.covariance[i][i])
                    .fold(0.0, f32::max)
                    .clamp(1e-9, 1.0);
                self.constrain_group(start, (maximum / 1e6).clamp(1e-9, 1.0), maximum);
            }
        }
        if self.range_params.is_some() && self.covariance[23][23] > 1e4 {
            let variance = 10.0 * self.covariance[23][23];
            self.fuse_direct(0.0, self.covariance[23][23] + variance, variance, 23, false);
        }
    }

    fn numerics_valid(&self) -> bool {
        self.output_predictor.numerics_valid()
            && self.global.is_finite()
            && self.state.iter().all(|value| value.is_finite())
            && (math::norm(&self.quaternion()) - 1.0).abs() < 1e-4
            && self
                .covariance
                .iter()
                .flatten()
                .all(|value| value.is_finite())
            && (0..STATE_COUNT).all(|i| self.covariance[i][i] >= 0.0)
            && (0..STATE_COUNT)
                .all(|i| (0..i).all(|j| self.covariance[i][j] == self.covariance[j][i]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resets_decorrelate_covariance_and_record_velocity_height_and_yaw_changes() {
        let mut filter = ImuEkf::new(&ImuEkfParams::default(), 0).unwrap();
        filter.initialize_covariance();
        filter.state[6] = 3.0;
        filter.covariance[8][8] = 4.0;
        filter.covariance[5][8] = 0.2;
        filter.covariance[8][5] = 0.2;
        filter.reset_vertical_velocity(0.0, 10.0);
        assert_eq!(filter.covariance[5][5], 10.0);
        assert_eq!(filter.covariance[8][8], 4.0);
        assert_eq!(filter.covariance[5][8], 0.0);
        assert_eq!(filter.reset_report().velocity_delta[2], -3.0);
        filter.reset_altitude(100.0, 9.0);
        assert_eq!(filter.covariance[8][8], 9.0);
        assert_eq!(filter.global.altitude_m, 100.0);
        assert_eq!(filter.reset_report().position_delta[2], -100.0);
        filter.reset_yaw(0.2, 0.04);
        assert_eq!(filter.covariance[2][2], 0.04);
        assert_eq!(filter.reset_report().counts, [1, 0, 1, 0, 1, 0]);
        assert!(
            (math::to_axis_angle(filter.reset_report().quaternion_delta)[2] - 0.2).abs() < 1e-6
        );
    }

    #[test]
    fn vertical_acceleration_evidence_uses_independent_references_and_strict_probation() {
        let mut filter = ImuEkf::new(&ImuEkfParams::default(), 0).unwrap();
        filter.delayed_us = 1_000_000;
        filter.check_vertical_acceleration(
            [(0, 4.0, 1.0), (1, 2.0, 1.0), (1, 0.0, 0.0), (2, 0.0, 0.0)],
            false,
        );
        assert_ne!(filter.px4_fault & (1 << 10), 0);
        filter.delayed_us = 3_999_999;
        filter.check_vertical_acceleration([(0, 0.0, 0.0); 4], false);
        assert_ne!(filter.px4_fault & (1 << 10), 0);
        filter.delayed_us = 4_000_000;
        filter.check_vertical_acceleration([(0, 0.0, 0.0); 4], false);
        assert_eq!(filter.px4_fault & (1 << 10), 0);
    }

    #[test]
    fn output_configuration_is_explicit_and_reset_preserves_the_snapshot() {
        let mut filter = ImuEkf::new(&ImuEkfParams::default(), 0).unwrap();
        let params = OutputParams {
            imu_offset_body_m: [0.1, 0.2, 0.3],
            ..OutputParams::default()
        };
        filter.configure_output(params).unwrap();
        filter.update(&stationary(2500)).unwrap();
        assert_eq!(filter.output().timestamp, 2500);
        assert_eq!(filter.timestamp(), 2500);
        assert_eq!(
            filter.configure_output(OutputParams::default()),
            Err(FilterFault::Input)
        );
        filter.reset(0).unwrap();
        assert_eq!(
            filter.output_params.imu_offset_body_m,
            params.imu_offset_body_m
        );
        assert_eq!(
            filter.configure_output(OutputParams {
                position_time_constant_s: f32::NAN,
                ..params
            }),
            Err(FilterFault::Input)
        );
    }

    #[test]
    fn recent_aiding_is_strict_at_the_one_second_dead_reckoning_boundary() {
        let mut filter = ImuEkf::new(&ImuEkfParams::default(), 0).unwrap();
        filter.initialize_covariance();
        filter.control = TILT_ALIGN | (1 << 1) | (1 << 2) | (1 << 11) | IN_AIR;
        filter.last_horizontal_position_fuse = 1_000_000;
        filter.delayed_us = 1_999_999;
        filter.latest_us = 2_200_000;
        filter
            .control_fusion(&stationary(1_999_999), &mut NoExtension)
            .unwrap();
        assert_eq!(filter.control & DEAD_RECKONING, 0);
        filter.delayed_us = 2_000_000;
        filter
            .control_fusion(&stationary(2_000_000), &mut NoExtension)
            .unwrap();
        assert_ne!(filter.control & DEAD_RECKONING, 0);
    }

    #[test]
    fn aided_input_rejects_invalid_or_future_measurement_and_reset_preserves_profile() {
        let mut filter =
            ImuEkf::with_baro_mag(&ImuEkfParams::default(), BaroMagParams::default(), 0).unwrap();
        let future = BaroMagSample {
            timestamp: 5000,
            altitude_m: 100.0,
            field_gauss: [0.2, 0.0, 0.4],
        };
        assert_eq!(
            filter.update_aided(&stationary(2500), Some(future)),
            Err(FilterFault::Input)
        );
        assert_eq!(filter.update(&stationary(5000)), Err(FilterFault::Input));
        filter.reset(0).unwrap();
        assert!(filter.aiding_params.is_some());
        let invalid = BaroMagSample {
            timestamp: 2500,
            altitude_m: f32::NAN,
            field_gauss: [0.2, 0.0, 0.4],
        };
        assert_eq!(
            filter.update_aided(&stationary(2500), Some(invalid)),
            Err(FilterFault::Input)
        );
        filter.reset(0).unwrap();
        for timestamp in [2500, 5000, 7500] {
            filter.update(&stationary(timestamp)).unwrap();
        }
        assert!(filter.numerics_valid());
    }

    #[test]
    fn generated_magnetic_y_z_jacobians_match_identity_frame() {
        let mut state = [0.0; 25];
        state[0] = 1.0;
        state[16..19].copy_from_slice(&[0.2, 0.1, 0.4]);
        let mut covariance = [[0.0; 24]; 24];
        for (i, row) in covariance.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        let (variance_y, h_y) = super::super::mag_generated::mag_y(&state, &covariance, 0.01);
        let (variance_z, h_z) = super::super::mag_generated::mag_z(&state, &covariance, 0.01);
        let mut expected_y = [0.0; 24];
        expected_y[0] = 0.4;
        expected_y[2] = -0.2;
        expected_y[16] = 1.0;
        expected_y[19] = 1.0;
        let mut expected_z = [0.0; 24];
        expected_z[0] = -0.1;
        expected_z[1] = 0.2;
        expected_z[17] = 1.0;
        expected_z[20] = 1.0;
        assert_eq!(h_y, expected_y);
        assert_eq!(h_z, expected_z);
        assert!((variance_y - 2.21).abs() < 1e-6);
        assert!((variance_z - 2.06).abs() < 1e-6);
        let (innovation, variance, h_x) =
            super::super::mag_generated::mag_x(&state, &covariance, [0.2, 0.1, 0.4], 0.01);
        assert_eq!(innovation, [0.0; 3]);
        let mut expected_x = [0.0; 24];
        expected_x[1] = -0.4;
        expected_x[2] = 0.1;
        expected_x[15] = 1.0;
        expected_x[18] = 1.0;
        assert_eq!(h_x, expected_x);
        for (actual, expected) in variance.into_iter().zip([2.18, 2.21, 2.06]) {
            assert!((actual - expected).abs() < 1e-6);
        }
        let (prediction, variance, h) =
            super::super::mag_generated::declination(&state, &covariance, 0.01, f32::EPSILON);
        let x = 0.2 + 1.5 * f32::EPSILON;
        let inverse = 1.0 / (x * x + 0.1_f32 * 0.1_f32);
        assert_eq!(prediction, libm::atan2f(0.1, x));
        assert_eq!(h[15], -inverse * 0.1);
        assert_eq!(h[16], inverse * x);
        assert!((variance - (0.01 + h[15] * h[15] + h[16] * h[16])).abs() < 1e-5);
    }

    fn stationary(timestamp: u64) -> ImuDelta {
        ImuDelta {
            timestamp,
            delta_angle_rad: [0.0; 3],
            delta_velocity_m_s: [0.0, 0.0, -math::GRAVITY * 0.0025],
            delta_angle_dt_s: 0.0025,
            delta_velocity_dt_s: 0.0025,
            delta_velocity_clipping: [false; 3],
        }
    }

    #[test]
    fn general_observation_joseph_update_keeps_correlated_velocity_covariance() {
        let mut filter = ImuEkf::new(&ImuEkfParams::default(), 0).unwrap();
        filter.initialize_covariance();
        filter.reset_variance(3, 1.0);
        filter.reset_variance(4, 4.0);
        let mut observation = [0.0; STATE_COUNT];
        observation[3] = 1.0;
        observation[4] = 2.0;
        let gain = filter
            .project_covariance(observation)
            .map(|value| value / 18.0);
        filter.measurement_update(gain, observation, 1.0, -10.0);
        assert!((filter.state[4] - 10.0 / 18.0).abs() < 1e-6);
        assert!((filter.state[5] - 80.0 / 18.0).abs() < 1e-6);
        assert!((filter.covariance[3][3] - 17.0 / 18.0).abs() < 1e-6);
        assert!((filter.covariance[4][4] - 8.0 / 18.0).abs() < 1e-6);
        assert!((filter.covariance[3][4] + 8.0 / 18.0).abs() < 1e-6);
        assert!(filter.numerics_valid());
    }

    #[test]
    fn warmup_repeats_oldest_sample_without_changing_input_time() {
        let mut filter = ImuEkf::new(&ImuEkfParams::default(), 0).unwrap();
        let mut times = std::vec::Vec::new();
        for timestamp in (2500..=250_000).step_by(2500) {
            if filter.update(&stationary(timestamp)).unwrap() {
                times.push((filter.timestamp(), filter.latest_timestamp()));
            }
        }
        assert_eq!(times[0], (7500, 7500));
        assert_eq!(
            times.iter().take_while(|(time, _)| *time == 7500).count(),
            20
        );
        assert!(times.windows(2).all(|pair| pair[0].1 < pair[1].1));
        assert!(times[20..].windows(2).all(|pair| pair[0].0 < pair[1].0));
        assert_eq!(
            filter.control_status() & (FAKE_POS | FAKE_HGT | DEAD_RECKONING),
            FAKE_POS | FAKE_HGT | DEAD_RECKONING
        );
        assert!(!filter.health().valid);
        assert!(filter.numerics_valid());
    }

    #[test]
    fn bad_input_timeout_and_numeric_overflow_require_explicit_reset() {
        let params = ImuEkfParams::default();
        let mut filter = ImuEkf::new(&params, 0).unwrap();
        let before = *filter.state();
        let mut bad = stationary(2500);
        bad.delta_velocity_m_s[0] = f32::NAN;
        assert_eq!(filter.update(&bad), Err(FilterFault::Input));
        assert_eq!(filter.state(), &before);
        assert_eq!(filter.update(&stationary(5000)), Err(FilterFault::Input));
        assert!(!filter.health().valid);
        filter.reset(0).unwrap();
        filter.update(&stationary(2500)).unwrap();
        assert_eq!(filter.update(&stationary(2500)), Err(FilterFault::Input));
        filter.reset(0).unwrap();
        filter.update(&stationary(2500)).unwrap();
        assert_eq!(
            filter.check_fresh(2500 + IMU_FRESH_US + 1),
            Err(FilterFault::Timeout)
        );
        assert_eq!(filter.update(&stationary(5000)), Err(FilterFault::Timeout));
        filter.reset(0).unwrap();
        bad = stationary(2500);
        bad.delta_angle_rad = [1e38; 3];
        assert_eq!(filter.update(&bad), Err(FilterFault::Numerics));
        filter.reset(0).unwrap();
        for timestamp in [2500, 5000, 7500] {
            filter.update(&stationary(timestamp)).unwrap();
        }
        filter.covariance[0][1] = f32::NAN;
        assert_eq!(
            filter.update(&stationary(10_000)),
            Err(FilterFault::Numerics)
        );
        filter.reset(0).unwrap();
        let mut boundary = stationary(2000);
        boundary.delta_angle_dt_s = 0.001;
        boundary.delta_velocity_dt_s = 0.002;
        boundary.delta_velocity_m_s[2] = -math::GRAVITY * 0.002;
        assert!(filter.update(&boundary).is_ok());
    }

    #[test]
    fn parameter_snapshot_rejects_invalid_values_and_unbounded_buffer() {
        let defaults = ImuEkfParams::default();
        assert_eq!(defaults.buffer_length(), Ok(20));
        assert_eq!(defaults.predict_us, 10_000);
        for params in [
            ImuEkfParams {
                predict_us: 0,
                ..defaults
            },
            ImuEkfParams {
                predict_us: 1000,
                delay_max_ms: 200.0,
                ..defaults
            },
            ImuEkfParams {
                gyro_noise: f32::NAN,
                ..defaults
            },
            ImuEkfParams {
                accel_bias_time_constant: 0.0,
                ..defaults
            },
            ImuEkfParams {
                imu_ctrl: 8,
                ..defaults
            },
        ] {
            assert!(ImuEkf::new(&params, 0).is_err());
        }
    }

    #[test]
    fn axis_clipping_increases_noise_and_inhibits_bias_learning() {
        let mut normal = ImuEkf::new(&ImuEkfParams::default(), 0).unwrap();
        let mut clipped = ImuEkf::new(&ImuEkfParams::default(), 0).unwrap();
        for timestamp in (2500..=800_000).step_by(2500) {
            let mut sample = stationary(timestamp);
            normal.update(&sample).unwrap();
            sample.delta_velocity_clipping[1] = timestamp > 250_000;
            clipped.update(&sample).unwrap();
        }
        assert!(clipped.accel_bias_inhibit[1]);
        assert!(clipped.covariance[4][4] > normal.covariance[4][4]);
        assert!(clipped.numerics_valid());
        assert_eq!(clipped.fault_status(), 0);
        for timestamp in (802_500..=2_000_000).step_by(2500) {
            let mut sample = stationary(timestamp);
            sample.delta_velocity_clipping = [true; 3];
            clipped.update(&sample).unwrap();
        }
        assert_ne!(clipped.fault_status() & BAD_ACCEL_CLIPPING, 0);
        assert!(!clipped.health().valid);
        for timestamp in (2_002_500..=3_500_000).step_by(2500) {
            clipped.update(&stationary(timestamp)).unwrap();
        }
        assert_eq!(clipped.fault_status(), 0);
    }

    #[test]
    fn bad_vertical_acceleration_inhibits_every_accel_bias_axis_until_recovery() {
        let mut filter = ImuEkf::new(&ImuEkfParams::default(), 0).unwrap();
        for timestamp in (2500..=800_000).step_by(2500) {
            filter.update(&stationary(timestamp)).unwrap();
        }
        let baseline = filter.accel_bias_inhibit;
        filter.px4_fault |= 1 << 10;
        filter.update(&stationary(802_500)).unwrap();
        assert!(filter.accel_bias_inhibit.iter().all(|inhibit| *inhibit));
        filter.px4_fault &= !(1 << 10);
        for timestamp in (805_000..=1_600_000).step_by(2500) {
            filter.update(&stationary(timestamp)).unwrap();
        }
        assert_eq!(filter.accel_bias_inhibit, baseline);
    }
}
