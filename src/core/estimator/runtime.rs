//! Single-owner, algorithm-independent ingestion, supervision and attitude output.

use crate::icm42688;

use super::{
    mahony::Mahony,
    math,
    standard_output::{AngularVelocity, Attitude},
    AidId, FilterFault, ImuDelta, Measurement, NavFilter, StepReport,
};

/// Budget for a complete delayed EKF epoch, normally every 10 ms. The 1-kHz
/// attitude publication period is independent of this computation budget.
pub const DEADLINE_US: u32 = 2_000;
pub const FRESH_US: u64 = 100_000;
pub const ATTITUDE_FRESH_US: u64 = 5_000;
const AID_FRESH_US: u64 = 1_000_000;
const DEADLINE_LIMIT: u8 = 3;
const RECOVERY_STEPS: u8 = 10;
const PENDING_MEASUREMENTS: usize = 16;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    Input,
    Sequence,
    Timestamp,
    Stale,
    Filter(FilterFault),
    Deadline,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FilterMode {
    Running,
    Degraded { reason: Fault, since: u64 },
    SoftResetting,
    Fatal,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Selection {
    Invalid,
    Filter,
    Mahony,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeasurementError {
    Invalid,
    Stale,
    Duplicate,
    Full,
    Inactive,
}

#[derive(Clone, Copy, Debug)]
pub struct Status {
    pub timestamp: u64,
    pub generation: u32,
    pub samples: u32,
    pub updates: u32,
    pub measurements: u32,
    pub rejected_measurements: u32,
    pub selection: Selection,
    pub mode: FilterMode,
    pub fault: Option<Fault>,
    pub last_fault: Option<Fault>,
    pub max_update_us: u32,
    pub deadline_misses: u32,
    pub recoveries: u32,
}

impl Status {
    const EMPTY: Self = Self {
        timestamp: 0,
        generation: 0,
        samples: 0,
        updates: 0,
        measurements: 0,
        rejected_measurements: 0,
        selection: Selection::Invalid,
        mode: FilterMode::Running,
        fault: None,
        last_fault: None,
        max_update_us: 0,
        deadline_misses: 0,
        recoveries: 0,
    };
}

/// Small immutable publication exchanged with the 1-kHz task. Publication time
/// may advance; sample time never advances without a real IMU integration.
#[derive(Clone, Copy, Debug)]
pub struct AttitudeEstimate {
    pub attitude: Attitude,
    pub angular_velocity: AngularVelocity,
    pub relative_yaw: bool,
}

impl AttitudeEstimate {
    pub fn at(mut self, now: u64) -> Option<Self> {
        if now < self.attitude.timestamp_sample
            || now - self.attitude.timestamp_sample > ATTITUDE_FRESH_US
        {
            return None;
        }
        self.attitude.timestamp = now;
        self.angular_velocity.timestamp = now;
        Some(self)
    }
}

#[derive(Default)]
pub struct ImuBridge {
    last_sequence: Option<u32>,
    last_timestamp: u64,
}

impl ImuBridge {
    pub const fn new() -> Self {
        Self {
            last_sequence: None,
            last_timestamp: 0,
        }
    }

    pub fn convert(&mut self, sample: icm42688::Sample, now: u64) -> Result<ImuDelta, Fault> {
        if now < sample.sample_timestamp_us || now - sample.sample_timestamp_us > FRESH_US {
            return Err(Fault::Stale);
        }
        if self
            .last_sequence
            .is_some_and(|last| sample.sequence != last.wrapping_add(1))
        {
            return Err(Fault::Sequence);
        }
        if self.last_sequence.is_some() && sample.sample_timestamp_us <= self.last_timestamp {
            return Err(Fault::Timestamp);
        }
        // TMST_CONFIG=0x1d: sensor delta timestamp, 16-us ticks. TIM2 sample
        // time and sensor integration duration remain separately observable.
        let duration_us = u32::from(sample.sensor_timestamp) * 16;
        if !(64..=256).contains(&duration_us) {
            return Err(Fault::Timestamp);
        }
        let gyro = sample.gyro_frd();
        let accel = sample.accel_frd();
        if gyro.iter().chain(&accel).any(|v| !v.is_finite())
            || gyro
                .iter()
                .any(|v| v.abs() >= 0.999 * 2_000_f32.to_radians())
        {
            return Err(Fault::Input);
        }
        let dt = duration_us as f32 * 1e-6;
        self.last_sequence = Some(sample.sequence);
        self.last_timestamp = sample.sample_timestamp_us;
        Ok(ImuDelta {
            timestamp: sample.sample_timestamp_us,
            delta_angle_rad: gyro.map(|v| v * dt),
            delta_velocity_m_s: accel.map(|v| v * dt),
            delta_angle_dt_s: dt,
            delta_velocity_dt_s: dt,
            delta_velocity_clipping: accel.map(|v| v.abs() >= 0.999 * 16.0 * 9.80665),
        })
    }
}

pub struct Runtime<F: NavFilter> {
    filter: F,
    initialize: fn(&mut F, &F::Params, u64) -> Result<(), FilterFault>,
    params: Option<F::Params>,
    fallback: Mahony,
    fallback_valid: bool,
    bridge: ImuBridge,
    enabled: bool,
    prefer_filter: bool,
    status: Status,
    pending: [Option<Measurement>; PENDING_MEASUREMENTS],
    last_measurement: [u64; 4],
    deadline_streak: u8,
    good_steps: u8,
    previous_attitude: Option<(u32, Selection, Attitude)>,
    reset_offset: u8,
}

impl<F: NavFilter> Runtime<F> {
    pub const fn dormant(filter: F) -> Self {
        Self::with_initializer(filter, initialize_value::<F>)
    }

    /// Large MCU filters supply an in-place storage initializer. This affects
    /// only construction/recovery; the seven-method NavFilter contract and
    /// statically dispatched sample/fusion paths stay the same.
    pub const fn with_initializer(
        filter: F,
        initialize: fn(&mut F, &F::Params, u64) -> Result<(), FilterFault>,
    ) -> Self {
        Self {
            filter,
            initialize,
            params: None,
            fallback: Mahony::new(),
            fallback_valid: false,
            bridge: ImuBridge::new(),
            enabled: false,
            prefer_filter: true,
            status: Status::EMPTY,
            pending: [None; PENDING_MEASUREMENTS],
            last_measurement: [0; 4],
            deadline_streak: 0,
            good_steps: 0,
            previous_attitude: None,
            reset_offset: 0,
        }
    }

    pub fn new(params: F::Params, t0: u64) -> Result<Self, FilterFault> {
        let filter = F::new(&params, t0);
        if let Some(fault) = filter.health().fault {
            return Err(fault);
        }
        let mut runtime = Self::dormant(filter);
        runtime.params = Some(params);
        runtime.enabled = true;
        runtime.status.generation = 1;
        Ok(runtime)
    }

    #[inline(never)]
    pub fn reconfigure(
        &mut self,
        params: F::Params,
        t0: u64,
        prefer_filter: bool,
    ) -> Result<(), FilterFault> {
        self.enabled = false;
        let configured = (self.initialize)(&mut self.filter, &params, t0);
        if configured.is_ok() {
            super::config_stage::enter(super::config_stage::HEALTH);
        }
        self.params = Some(params);
        self.prefer_filter = prefer_filter;
        self.fallback = Mahony::new();
        self.fallback_valid = false;
        self.bridge = ImuBridge::new();
        self.pending.fill(None);
        self.last_measurement = [0; 4];
        self.deadline_streak = 0;
        self.good_steps = 0;
        self.status = Status {
            generation: self.status.generation.wrapping_add(1),
            ..Status::EMPTY
        };
        if let Some(fault) = configured.err().or(self.filter.health().fault) {
            self.fail(Fault::Filter(fault), true);
            return Err(fault);
        }
        self.enabled = true;
        super::config_stage::enter(super::config_stage::IDLE);
        Ok(())
    }

    pub fn disable(&mut self) {
        self.enabled = false;
        self.status.selection = Selection::Invalid;
        self.pending.fill(None);
    }

    pub fn ingest_imu(&mut self, delta: ImuDelta, now: u64) -> Result<(), Fault> {
        if !self.enabled {
            return Ok(());
        }
        if now < delta.timestamp || now - delta.timestamp > FRESH_US {
            self.invalidate(Fault::Stale);
            return Err(Fault::Stale);
        }
        if delta.timestamp <= self.status.timestamp {
            self.invalidate(Fault::Timestamp);
            return Err(Fault::Timestamp);
        }
        if self.status.timestamp != 0 && delta.timestamp - self.status.timestamp > FRESH_US {
            self.invalidate(Fault::Stale);
        }
        self.status.timestamp = delta.timestamp;
        self.status.samples = self.status.samples.wrapping_add(1);
        // Keep the fallback warm on shared ~1-ms integrals, including probation.
        self.fallback_valid = self.fallback.update(&delta).is_ok();
        if self.prefer_filter
            && !matches!(
                self.status.mode,
                FilterMode::Fatal | FilterMode::SoftResetting
            )
        {
            self.filter.push_imu(&delta);
            self.check_filter_fault();
            self.flush_measurements();
        }
        self.select();
        Ok(())
    }

    pub fn apply_measurement(
        &mut self,
        measurement: Measurement,
        now: u64,
    ) -> Result<(), MeasurementError> {
        let result = self.enqueue_measurement(measurement, now);
        if result.is_err() {
            self.status.rejected_measurements = self.status.rejected_measurements.wrapping_add(1);
        }
        result
    }

    fn enqueue_measurement(
        &mut self,
        measurement: Measurement,
        now: u64,
    ) -> Result<(), MeasurementError> {
        if !self.enabled
            || !self.prefer_filter
            || matches!(
                self.status.mode,
                FilterMode::Fatal | FilterMode::SoftResetting
            )
        {
            return Err(MeasurementError::Inactive);
        }
        if !measurement.valid() {
            return Err(MeasurementError::Invalid);
        }
        let timestamp = measurement.timestamp();
        if now < timestamp || now - timestamp > AID_FRESH_US {
            return Err(MeasurementError::Stale);
        }
        let index = measurement.aid().index();
        if timestamp <= self.last_measurement[index] {
            return Err(MeasurementError::Duplicate);
        }
        let slot = self
            .pending
            .iter_mut()
            .find(|v| v.is_none())
            .ok_or(MeasurementError::Full)?;
        *slot = Some(measurement);
        self.last_measurement[index] = timestamp;
        self.status.measurements = self.status.measurements.wrapping_add(1);
        self.flush_measurements();
        Ok(())
    }

    fn flush_measurements(&mut self) {
        if matches!(
            self.status.mode,
            FilterMode::Fatal | FilterMode::SoftResetting
        ) {
            return;
        }
        for slot in &mut self.pending {
            if slot.is_some_and(|m| m.timestamp() <= self.status.timestamp) {
                self.filter.push_measurement(slot.take().unwrap());
            }
        }
        self.check_filter_fault();
    }

    /// Full covariance/fusion work belongs to this owner, after ingestion. Aid
    /// models consume only observations eligible at their delayed IMU horizon.
    pub fn maintain(&mut self, now: u64) -> StepReport {
        if !self.enabled || !self.prefer_filter {
            return StepReport::default();
        }
        if self.status.timestamp == 0 {
            return StepReport::default();
        }
        if now < self.status.timestamp || now - self.status.timestamp > FRESH_US {
            self.invalidate(Fault::Stale);
            return StepReport::default();
        }
        if self.status.mode == FilterMode::SoftResetting {
            self.recover();
            self.select();
            return StepReport::default();
        }
        if self.status.mode == FilterMode::Fatal {
            return StepReport::default();
        }
        self.step_filter(now)
    }

    // The fusion scratch frame must not sit beneath the recovery constructor.
    #[inline(never)]
    fn step_filter(&mut self, now: u64) -> StepReport {
        let report = self.filter.step(now);
        self.status.updates = self.status.updates.wrapping_add(u32::from(report.updated));
        self.check_filter_fault();
        if report.updated
            && self.filter.health().valid
            && matches!(self.status.mode, FilterMode::Degraded { reason, .. } if reason != Fault::Deadline)
        {
            self.good_step();
        }
        self.select();
        report
    }

    #[inline(never)]
    fn recover(&mut self) {
        let Some(params) = self.params.as_ref() else {
            return;
        };
        let configured = (self.initialize)(&mut self.filter, params, self.status.timestamp);
        if let Some(fault) = configured.err().or(self.filter.health().fault) {
            self.fail(Fault::Filter(fault), true);
            return;
        }
        self.pending.fill(None);
        self.last_measurement = [0; 4];
        self.status.generation = self.status.generation.wrapping_add(1);
        self.status.recoveries = self.status.recoveries.wrapping_add(1);
        self.status.mode = FilterMode::Degraded {
            reason: self.status.fault.unwrap_or(Fault::Stale),
            since: self.status.timestamp,
        };
        self.good_steps = 0;
        super::config_stage::enter(super::config_stage::IDLE);
    }

    pub fn invalidate(&mut self, fault: Fault) {
        self.bridge = ImuBridge::new();
        self.fallback = Mahony::new();
        self.fallback_valid = false;
        self.pending.fill(None);
        if self.status.mode != FilterMode::Fatal {
            self.fail(fault, false);
        }
        self.select();
    }

    fn fail(&mut self, fault: Fault, fatal: bool) {
        self.status.fault = Some(fault);
        self.status.last_fault = Some(fault);
        self.status.mode = if fatal {
            FilterMode::Fatal
        } else {
            FilterMode::SoftResetting
        };
        self.good_steps = 0;
    }

    fn check_filter_fault(&mut self) {
        if let Some(fault) = self.filter.health().fault {
            self.fail(Fault::Filter(fault), fault != FilterFault::Timeout);
        }
    }

    /// Call only for completed heavy epochs. Light ingress must not clear an
    /// overrun streak; one overrun remains diagnostic and does not stop F.
    pub fn record_duration(&mut self, elapsed_us: u32) {
        self.status.max_update_us = self.status.max_update_us.max(elapsed_us);
        if elapsed_us > DEADLINE_US {
            self.status.deadline_misses = self.status.deadline_misses.wrapping_add(1);
            self.deadline_streak = self.deadline_streak.saturating_add(1);
            self.good_steps = 0;
            self.status.last_fault = Some(Fault::Deadline);
            if self.deadline_streak >= DEADLINE_LIMIT
                && matches!(
                    self.status.mode,
                    FilterMode::Running | FilterMode::Degraded { .. }
                )
                && !matches!(
                    self.status.mode,
                    FilterMode::Degraded {
                        reason: Fault::Deadline,
                        ..
                    }
                )
            {
                self.status.fault = Some(Fault::Deadline);
                self.status.mode = FilterMode::Degraded {
                    reason: Fault::Deadline,
                    since: self.status.timestamp,
                };
            }
        } else {
            self.deadline_streak = 0;
            if self.filter.health().valid
                && matches!(
                    self.status.mode,
                    FilterMode::Degraded {
                        reason: Fault::Deadline,
                        ..
                    }
                )
            {
                self.good_step();
            }
        }
        self.select();
    }

    fn good_step(&mut self) {
        self.good_steps = self.good_steps.saturating_add(1);
        if self.good_steps >= RECOVERY_STEPS {
            self.status.mode = FilterMode::Running;
            self.status.fault = None;
        }
    }

    fn select(&mut self) {
        self.status.selection = if !self.enabled {
            Selection::Invalid
        } else if self.prefer_filter
            && self.status.mode == FilterMode::Running
            && self.filter.health().valid
        {
            Selection::Filter
        } else if self.fallback_valid {
            Selection::Mahony
        } else {
            Selection::Invalid
        };
    }

    pub fn attitude_at(&mut self, now: u64) -> Option<AttitudeEstimate> {
        let output = match self.status.selection {
            Selection::Filter => self.filter.output(),
            Selection::Mahony => NavFilter::output(&self.fallback),
            Selection::Invalid => return None,
        };
        if output.q_body_to_ned.iter().any(|v| !v.is_finite())
            || (math::norm(&output.q_body_to_ned) - 1.0).abs() > 1e-4
        {
            return None;
        }
        let mut attitude = Attitude {
            timestamp: now,
            timestamp_sample: output.timestamp,
            q: output.q_body_to_ned,
            delta_q_reset: output.delta_q_reset,
            quat_reset_counter: output.quat_reset_counter,
        };
        let mut reset_count = attitude.quat_reset_counter.wrapping_add(self.reset_offset);
        if let Some((generation, source, previous)) = self.previous_attitude {
            if generation != self.status.generation || source != self.status.selection {
                reset_count = previous.quat_reset_counter.wrapping_add(1);
                self.reset_offset = reset_count.wrapping_sub(attitude.quat_reset_counter);
                attitude.delta_q_reset =
                    math::normalize(math::multiply(attitude.q, math::inverse(previous.q)));
            } else if reset_count == previous.quat_reset_counter {
                attitude.delta_q_reset = previous.delta_q_reset;
            }
        }
        attitude.quat_reset_counter = reset_count;
        let estimate = AttitudeEstimate {
            attitude,
            angular_velocity: AngularVelocity {
                timestamp: now,
                timestamp_sample: output.timestamp,
                xyz: output.angular_velocity_body_rad_s,
                xyz_derivative: [f32::NAN; 3],
            },
            relative_yaw: !output.absolute_yaw,
        }
        .at(now)?;
        self.previous_attitude = Some((self.status.generation, self.status.selection, attitude));
        Some(estimate)
    }

    pub fn status(&self) -> Status {
        self.status
    }
    pub fn filter(&self) -> &F {
        &self.filter
    }
    pub(super) fn filter_mut(&mut self) -> &mut F {
        &mut self.filter
    }

    /// Legacy raw-sample adapter for replay callers. Product ingestion shares
    /// one bridge/downsampler before calling both runtimes' ingest_imu methods.
    pub fn update(
        &mut self,
        sample: icm42688::Sample,
        now: u64,
        baro: Option<super::imu_ekf::baro_mag::BaroAltitude>,
        gnss: Option<super::gnss::GnssObservation>,
        range: Option<super::range_sensor::RangeObservation>,
    ) {
        let delta = match self.bridge.convert(sample, now) {
            Ok(delta) => delta,
            Err(fault) => {
                self.invalidate(fault);
                return;
            }
        };
        if self.ingest_imu(delta, now).is_err() {
            return;
        }
        for measurement in [
            baro.map(Measurement::Baro),
            gnss.map(Measurement::Gnss),
            range.map(|v| {
                let [min_distance_m, max_distance_m] = v.limits.unwrap_or([0.1, 30.0]);
                Measurement::Range(super::RangeSample {
                    timestamp: v.timestamp,
                    distance_m: v.distance_m,
                    min_distance_m,
                    max_distance_m,
                    quality: v.quality_percent,
                })
            }),
        ]
        .into_iter()
        .flatten()
        {
            let _ = self.apply_measurement(measurement, now);
        }
        self.maintain(now);
    }
}

fn initialize_value<F: NavFilter>(
    filter: &mut F,
    params: &F::Params,
    t0: u64,
) -> Result<(), FilterFault> {
    *filter = F::new(params, t0);
    filter.health().fault.map_or(Ok(()), Err)
}

impl AidId {
    const fn index(self) -> usize {
        match self {
            Self::Gnss => 0,
            Self::Baro => 1,
            Self::Mag => 2,
            Self::Range => 3,
        }
    }
}

impl Measurement {
    pub fn valid(self) -> bool {
        match self {
            Self::Baro(v) => v.altitude_m.is_finite(),
            Self::Mag(v) => v.field_gauss.iter().all(|v| v.is_finite()),
            Self::Range(v) => {
                v.distance_m.is_finite()
                    && v.min_distance_m.is_finite()
                    && v.max_distance_m.is_finite()
                    && v.min_distance_m >= 0.0
                    && v.max_distance_m > v.min_distance_m
                    && (-1..=100).contains(&v.quality)
            }
            Self::Gnss(v) => {
                let s = v.sample;
                s.latitude_deg.is_finite()
                    && s.latitude_deg.abs() <= 90.0
                    && s.longitude_deg.is_finite()
                    && s.longitude_deg.abs() <= 180.0
                    && s.altitude_msl_m.is_finite()
                    && s.velocity_ned_m_s.iter().all(|v| v.is_finite())
                    && [
                        s.horizontal_accuracy_m,
                        s.vertical_accuracy_m,
                        s.speed_accuracy_m_s,
                        v.pdop,
                    ]
                    .iter()
                    .all(|v| v.is_finite() && *v >= 0.0)
            }
        }
    }
}
