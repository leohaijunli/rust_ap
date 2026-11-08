#![forbid(unsafe_code)]

//! Estimator boundary: sensor samples, replaceable filter traits and the
//! generated PX4 EKF2 parameter catalog. No HAL, transport or allocator
//! dependency, so firmware and host tests share one contract.

pub mod bias_estimator;
pub mod config_stage;
mod covariance_generated;
pub mod gnss;
pub mod gnss_checks;
mod gsf;
mod gsf_generated;
mod imu_downsampler;
pub mod imu_ekf;
mod imu_input;
mod mag_generated;
pub mod mahony;
mod math;
mod observation_buffer;
pub mod output_predictor;
pub mod params_generated;
pub mod product;
pub mod publication_reset;
pub mod range_math;
pub mod range_sensor;
pub mod report;
pub mod reset_status;
pub mod runtime;
pub mod runtime_params;
#[cfg(test)]
mod runtime_tests;
pub mod standard_output;
pub mod standard_ulog;
pub mod timing;
pub mod wmm;
mod wmm_generated;

pub use params_generated::EKF2_PARAMS;

/// Monotonic microseconds since boot, the base of every sensor sample.
pub type Timestamp = u64;

/// PX4 v1.17.0 error-state dimension, including terrain. Attitude error has
/// three components; the nominal state stores a four-component quaternion.
pub const STATE_COUNT: usize = 24;
pub const NOMINAL_STATE_COUNT: usize = 25;

pub type StateVector = [f32; NOMINAL_STATE_COUNT];
pub type ErrorStateVector = [f32; STATE_COUNT];
pub type Covariance = [[f32; STATE_COUNT]; STATE_COUNT];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParamKind {
    Int32,
    Float,
    Boolean,
    Enum,
    Bitmask,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParamValue {
    Int(i32),
    Float(f32),
}

/// Implementation state of a PX4 EKF2 parameter in this firmware.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Support {
    Active,
    Ignored,
    Planned,
}

/// One entry of the PX4 v1.17.0 `EKF2_*` catalog; `tools/gen_params.py` owns it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParamDef {
    pub name: &'static str,
    pub kind: ParamKind,
    pub default: ParamValue,
    pub min: Option<f32>,
    pub max: Option<f32>,
    pub unit: &'static str,
    pub support: Support,
}

/// Look up a parameter by its MAVLink `param_id`.
pub const fn ekf2_parameter(name: &str) -> Option<&'static ParamDef> {
    let wanted = name.as_bytes();
    let mut index = 0;
    while index < EKF2_PARAMS.len() {
        let candidate = EKF2_PARAMS[index].name.as_bytes();
        if candidate.len() == wanted.len() {
            let mut byte = 0;
            while byte < wanted.len() && candidate[byte] == wanted[byte] {
                byte += 1;
            }
            if byte == wanted.len() {
                return Some(&EKF2_PARAMS[index]);
            }
        }
        index += 1;
    }
    None
}

/// Body-frame IMU integration over one interval, coning/sculling corrected.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuDelta {
    pub timestamp: Timestamp,
    pub delta_angle_rad: [f32; 3],
    pub delta_velocity_m_s: [f32; 3],
    pub delta_angle_dt_s: f32,
    pub delta_velocity_dt_s: f32,
    pub delta_velocity_clipping: [bool; 3],
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GnssSample {
    pub timestamp: Timestamp,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_msl_m: f32,
    pub velocity_ned_m_s: [f32; 3],
    pub horizontal_accuracy_m: f32,
    pub vertical_accuracy_m: f32,
    pub speed_accuracy_m_s: f32,
    pub fix_type: u8,
    pub satellites: u8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagSample {
    pub timestamp: Timestamp,
    pub field_gauss: [f32; 3],
    /// Selected device/calibration changed: discard old magnetic history only.
    pub reset: bool,
}

/// Range finder sample as delivered by the sensor. Tilt and lever-arm
/// correction belong to the aid source, not to the driver.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RangeSample {
    pub timestamp: Timestamp,
    pub distance_m: f32,
    pub min_distance_m: f32,
    pub max_distance_m: f32,
    /// -1 unknown, 0 invalid, 1..100 measured quality, as in PX4.
    pub quality: i8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Measurement {
    Gnss(gnss::GnssObservation),
    Baro(imu_ekf::baro_mag::BaroAltitude),
    Mag(MagSample),
    Range(RangeSample),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AidId {
    Gnss,
    Baro,
    Mag,
    Range,
}

impl Measurement {
    pub const fn aid(self) -> AidId {
        match self {
            Self::Gnss(_) => AidId::Gnss,
            Self::Baro(_) => AidId::Baro,
            Self::Mag(_) => AidId::Mag,
            Self::Range(_) => AidId::Range,
        }
    }

    pub const fn timestamp(self) -> Timestamp {
        match self {
            Self::Gnss(sample) => sample.sample.timestamp,
            Self::Baro(sample) => sample.timestamp,
            Self::Mag(sample) => sample.timestamp,
            Self::Range(sample) => sample.timestamp,
        }
    }
}

/// Low-latency output of the prediction stage, in the local NED frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavOutput {
    pub timestamp: Timestamp,
    /// Body to NED quaternion ordered `w, x, y, z`.
    pub q_body_to_ned: [f32; 4],
    pub velocity_ned_m_s: [f32; 3],
    pub position_ned_m: [f32; 3],
    pub height_msl_m: f32,
    pub angular_velocity_body_rad_s: [f32; 3],
    pub delta_q_reset: [f32; 4],
    pub quat_reset_counter: u8,
    pub absolute_yaw: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StepReport {
    /// A delayed prediction/fusion epoch completed (not an IMU ingestion).
    pub updated: bool,
    pub fused: u16,
    pub rejected: u16,
}

impl StepReport {
    pub(super) fn from_aids<const N: usize>(
        updated: bool,
        before: [imu_ekf::baro_mag::AidStatus; N],
        after: [imu_ekf::baro_mag::AidStatus; N],
    ) -> Self {
        let mut result = Self {
            updated,
            ..Self::default()
        };
        for (before, after) in before.into_iter().zip(after) {
            if updated && after.timestamp != before.timestamp {
                result.fused += u16::from(after.fused);
                result.rejected += u16::from(after.rejected);
            }
        }
        result
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FilterFault {
    Input,
    Numerics,
    Timeout,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FilterHealth {
    pub valid: bool,
    pub fault: Option<FilterFault>,
}

impl FilterHealth {
    pub const OK: Self = Self {
        valid: true,
        fault: None,
    };
}

/// Algorithm-independent scheduling contract. Ingestion is bounded; `step`
/// services a pending delayed epoch. Mutating methods report failures through
/// `health`, including invalid construction. An unaligned filter is not faulty.
pub trait NavFilter {
    type Params;

    fn new(params: &Self::Params, t0: Timestamp) -> Self;
    fn push_imu(&mut self, sample: &ImuDelta);
    fn push_measurement(&mut self, measurement: Measurement);
    fn step(&mut self, now: Timestamp) -> StepReport;
    fn output(&self) -> NavOutput;
    fn health(&self) -> FilterHealth;
}

/// Supervisor state shared with aid sources; it also decides which sources may
/// run at all.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ControlStatus {
    pub tilt_aligned: bool,
    pub yaw_aligned: bool,
    pub gnss_available: bool,
    pub baro_available: bool,
    pub mag_available: bool,
    pub range_available: bool,
}

impl ControlStatus {
    pub const fn aid_available(self, aid: AidId) -> bool {
        match aid {
            AidId::Gnss => self.gnss_available,
            AidId::Baro => self.baro_available,
            AidId::Mag => self.mag_available,
            AidId::Range => self.range_available,
        }
    }
}

/// One scalar Kalman update prepared by an aid source: the innovation, its
/// variance and the matching row of the observation Jacobian.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScalarUpdate {
    pub innovation: f32,
    pub innovation_variance: f32,
    pub observation: ErrorStateVector,
}

impl ScalarUpdate {
    pub const EMPTY: Self = Self {
        innovation: 0.0,
        innovation_variance: 0.0,
        observation: [0.0; STATE_COUNT],
    };
}

/// A 3D measurement fuses at most three scalars sequentially.
pub const MAX_SCALAR_UPDATES: usize = 4;

/// Fixed-capacity batch filled by an aid source and consumed by the core.
#[derive(Clone, Copy, Debug)]
pub struct ScalarBatch {
    updates: [ScalarUpdate; MAX_SCALAR_UPDATES],
    len: usize,
}

impl ScalarBatch {
    pub const fn new() -> Self {
        Self {
            updates: [ScalarUpdate::EMPTY; MAX_SCALAR_UPDATES],
            len: 0,
        }
    }

    pub const fn len(&self) -> usize {
        self.len
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns `false` and drops the update when the batch is full.
    pub fn push(&mut self, update: ScalarUpdate) -> bool {
        if self.len == MAX_SCALAR_UPDATES {
            return false;
        }
        self.updates[self.len] = update;
        self.len += 1;
        true
    }

    pub fn as_slice(&self) -> &[ScalarUpdate] {
        &self.updates[..self.len]
    }
}

impl Default for ScalarBatch {
    fn default() -> Self {
        Self::new()
    }
}

/// Per-sensor measurement model. Aid sources own their gating counters and
/// emit scalar updates; they never write the covariance themselves.
pub trait AidSource<P> {
    fn id(&self) -> AidId;
    fn delay_us(&self, params: &P) -> u32;
    fn enabled(&self, params: &P, status: &ControlStatus) -> bool;
    fn scalar_updates(
        &mut self,
        state: &StateVector,
        covariance: &Covariance,
        out: &mut ScalarBatch,
    );
    fn on_reject(&mut self, status: &mut ControlStatus);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nominal_state_and_error_covariance_keep_distinct_px4_dimensions() {
        assert_eq!(StateVector::default().len(), 25);
        assert_eq!(ErrorStateVector::default().len(), 24);
        assert_eq!(core::mem::size_of::<Covariance>(), 24 * 24 * 4);
        assert_eq!(ScalarUpdate::EMPTY.observation.len(), 24);
    }

    #[test]
    fn ekf2_catalog_matches_the_px4_contract() {
        assert_eq!(EKF2_PARAMS.len(), 142);

        for (index, definition) in EKF2_PARAMS.iter().enumerate() {
            let name = definition.name;
            assert!(
                (1..=16).contains(&name.len()),
                "{name}: MAVLink param_id must be 1..=16 characters"
            );
            assert!(
                name.bytes()
                    .all(|byte| byte.is_ascii_uppercase() || byte.is_ascii_digit() || byte == b'_'),
                "{name}: not MAVLink parameter style"
            );
            assert!(
                EKF2_PARAMS[..index]
                    .iter()
                    .all(|earlier| earlier.name != name),
                "{name}: duplicate parameter name"
            );

            if let Some(min) = definition.min {
                assert!(min.is_finite(), "{name}: non-finite minimum");
            }
            if let Some(max) = definition.max {
                assert!(max.is_finite(), "{name}: non-finite maximum");
            }
            if let (Some(min), Some(max)) = (definition.min, definition.max) {
                assert!(min <= max, "{name}: minimum above maximum");
            }

            let value = match definition.default {
                ParamValue::Int(value) => value as f32,
                ParamValue::Float(value) => value,
            };
            assert!(value.is_finite(), "{name}: non-finite default");
            if let Some(min) = definition.min {
                assert!(value >= min, "{name}: default below minimum");
            }
            if let Some(max) = definition.max {
                assert!(value <= max, "{name}: default above maximum");
            }

            match (definition.kind, definition.default) {
                (ParamKind::Float, ParamValue::Float(_)) => {}
                (ParamKind::Float, ParamValue::Int(_)) => {
                    panic!("{name}: float parameter with an integer default")
                }
                (_, ParamValue::Int(_)) => {}
                (_, ParamValue::Float(_)) => {
                    panic!("{name}: integer parameter with a float default")
                }
            }
        }
    }

    #[test]
    fn ekf2_parameters_keep_px4_names_defaults_and_ranges() {
        let gps_ctrl = ekf2_parameter("EKF2_GPS_CTRL").expect("EKF2_GPS_CTRL");
        assert_eq!(gps_ctrl.kind, ParamKind::Bitmask);
        assert_eq!(gps_ctrl.default, ParamValue::Int(7));
        assert_eq!((gps_ctrl.min, gps_ctrl.max), (Some(0.0), Some(15.0)));

        let gps_delay = ekf2_parameter("EKF2_GPS_DELAY").expect("EKF2_GPS_DELAY");
        assert_eq!(gps_delay.default, ParamValue::Float(110.0));
        assert_eq!(gps_delay.unit, "ms");
        assert_eq!(gps_delay.support, Support::Planned);

        assert!(ekf2_parameter("EKF2_MISSING").is_none());
    }

    #[test]
    fn scalar_batch_keeps_order_and_drops_overflow() {
        let mut batch = ScalarBatch::new();
        assert!(batch.is_empty());

        for index in 0..MAX_SCALAR_UPDATES {
            let mut update = ScalarUpdate::EMPTY;
            update.innovation = index as f32;
            assert!(batch.push(update));
        }
        assert!(!batch.push(ScalarUpdate::EMPTY));
        assert_eq!(batch.len(), MAX_SCALAR_UPDATES);

        for (index, update) in batch.as_slice().iter().enumerate() {
            assert_eq!(update.innovation, index as f32);
        }
    }

    #[test]
    fn measurement_reports_its_aid_and_sample_time() {
        let gnss = Measurement::Gnss(gnss::GnssObservation {
            sample: GnssSample {
                timestamp: 10,
                latitude_deg: 0.0,
                longitude_deg: 0.0,
                altitude_msl_m: 0.0,
                velocity_ned_m_s: [0.0; 3],
                horizontal_accuracy_m: 1.0,
                vertical_accuracy_m: 1.0,
                speed_accuracy_m_s: 0.1,
                fix_type: 3,
                satellites: 12,
            },
            pdop: 1.0,
            spoofed: false,
        });
        let baro = Measurement::Baro(imu_ekf::baro_mag::BaroAltitude {
            timestamp: 20,
            altitude_m: 0.0,
        });
        let mag = Measurement::Mag(MagSample {
            timestamp: 30,
            field_gauss: [0.2, 0.0, 0.4],
            reset: false,
        });
        let range = Measurement::Range(RangeSample {
            timestamp: 40,
            distance_m: 1.5,
            min_distance_m: 0.1,
            max_distance_m: 12.0,
            quality: 80,
        });

        assert_eq!(gnss.aid(), AidId::Gnss);
        assert_eq!(baro.aid(), AidId::Baro);
        assert_eq!(mag.aid(), AidId::Mag);
        assert_eq!(range.aid(), AidId::Range);
        assert_eq!(
            [
                gnss.timestamp(),
                baro.timestamp(),
                mag.timestamp(),
                range.timestamp()
            ],
            [10, 20, 30, 40]
        );
    }

    #[test]
    fn control_status_gates_each_aid() {
        let status = ControlStatus {
            tilt_aligned: true,
            range_available: true,
            ..ControlStatus::default()
        };

        assert!(status.aid_available(AidId::Range));
        assert!(!status.aid_available(AidId::Gnss));
        assert!(!status.aid_available(AidId::Baro));
        assert!(!status.aid_available(AidId::Mag));
    }
}
