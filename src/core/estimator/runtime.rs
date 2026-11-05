//! HAL-free E7 input/parameter/supervision boundary. Firmware owns scheduling.
//! Missing aids remain None; software bring-up never qualifies navigation.

use crate::{
    config::Algorithm,
    icm42688,
    parameters::{Id, Snapshot as Parameters, COUNT},
};
use core::cell::RefCell;

use super::{
    gnss::{GnssEkf, GnssObservation, GnssParams},
    imu_ekf::{
        baro_mag::{BaroAltitude, BaroMagParams},
        range::RangeParams,
        ImuEkfParams,
    },
    mahony::Mahony,
    no_gnss::NoGnssEkf,
    output_predictor::OutputParams,
    range_sensor::RangeObservation,
    standard_output::{AngularVelocity, Attitude, Snapshot},
    FilterFault, ImuDelta,
};

pub const DEADLINE_US: u32 = 1_000;
pub const FRESH_US: u64 = 100_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    Input,
    Sequence,
    Timestamp,
    Stale,
    Ekf(FilterFault),
    Deadline,
}

#[derive(Clone, Copy)]
pub struct Params {
    pub enabled: bool,
    pub algorithm: Algorithm,
    pub imu: ImuEkfParams,
    pub aiding: BaroMagParams,
    pub range: RangeParams,
    pub gnss: GnssParams,
    pub output: OutputParams,
    mapped: [bool; COUNT],
}

impl Params {
    pub fn from_parameters(values: Parameters) -> Result<Self, FilterFault> {
        let mapped = RefCell::new([false; COUNT]);
        let get = |name: &str| {
            let id = Id::from_name(name).expect("fixed runtime parameter ID");
            mapped.borrow_mut()[id as usize] = true;
            values.get(id)
        };
        let vector = |names: [&str; 3]| names.map(&get);
        let mut result = Self {
            enabled: get("EST_EN") != 0.0,
            algorithm: if get("EST_ALGO") == 1.0 && get("EKF2_EN") != 0.0 {
                Algorithm::Ekf2
            } else {
                Algorithm::Mahony
            },
            imu: ImuEkfParams::default(),
            aiding: BaroMagParams::default(),
            range: RangeParams::default(),
            gnss: GnssParams::default(),
            output: OutputParams::default(),
            mapped: [false; COUNT],
        };
        result.imu.predict_us = get("EKF2_PREDICT_US") as i32;
        result.imu.delay_max_ms = get("EKF2_DELAY_MAX");
        result.imu.imu_ctrl = get("EKF2_IMU_CTRL") as i32;
        result.imu.velocity_limit = get("EKF2_VEL_LIM");
        result.imu.gyro_noise = get("EKF2_GYR_NOISE");
        result.imu.accel_noise = get("EKF2_ACC_NOISE");
        result.imu.gyro_bias_noise = get("EKF2_GYR_B_NOISE");
        result.imu.accel_bias_noise = get("EKF2_ACC_B_NOISE");
        result.imu.gyro_bias_init = get("EKF2_GBIAS_INIT");
        result.imu.accel_bias_init = get("EKF2_ABIAS_INIT");
        result.imu.angle_error_init = get("EKF2_ANGERR_INIT");
        result.imu.no_aid_noise = get("EKF2_NOAID_NOISE");
        result.imu.heading_noise = get("EKF2_HEAD_NOISE");
        result.imu.gyro_bias_limit = get("EKF2_GYR_B_LIM");
        result.imu.accel_bias_limit = get("EKF2_ABL_LIM");
        result.imu.accel_bias_accel_limit = get("EKF2_ABL_ACCLIM");
        result.imu.accel_bias_gyro_limit = get("EKF2_ABL_GYRLIM");
        result.imu.accel_bias_time_constant = get("EKF2_ABL_TAU");
        result.aiding.baro_noise = get("EKF2_BARO_NOISE");
        result.aiding.baro_gate = get("EKF2_BARO_GATE");
        result.aiding.baro_delay_ms = get("EKF2_BARO_DELAY");
        result.aiding.mag_noise = get("EKF2_MAG_NOISE");
        result.aiding.mag_gate = get("EKF2_MAG_GATE");
        result.aiding.mag_delay_ms = get("EKF2_MAG_DELAY");
        result.aiding.mag_earth_noise = get("EKF2_MAG_E_NOISE");
        result.aiding.mag_bias_noise = get("EKF2_MAG_B_NOISE");
        result.aiding.mag_declination_deg = get("EKF2_MAG_DECL");
        result.aiding.mag_check = get("EKF2_MAG_CHECK") as i32;
        result.aiding.mag_type = get("EKF2_MAG_TYPE") as i32;
        result.aiding.height_reference = get("EKF2_HGT_REF") as i32;
        result.gnss.control = get("EKF2_GPS_CTRL") as i32;
        result.gnss.mode = get("EKF2_GPS_MODE") as i32;
        result.gnss.delay_ms = get("EKF2_GPS_DELAY");
        result.gnss.position_noise_m = get("EKF2_GPS_P_NOISE");
        result.gnss.velocity_noise_m_s = get("EKF2_GPS_V_NOISE");
        result.gnss.position_gate = get("EKF2_GPS_P_GATE");
        result.gnss.velocity_gate = get("EKF2_GPS_V_GATE");
        result.gnss.offset_body_m = vector(["EKF2_GPS_POS_X", "EKF2_GPS_POS_Y", "EKF2_GPS_POS_Z"]);
        result.gnss.checks.mask = get("EKF2_GPS_CHECK") as u16;
        result.gnss.checks.minimum_fix = get("EKF2_REQ_FIX") as u8;
        result.gnss.checks.minimum_satellites = get("EKF2_REQ_NSATS") as u8;
        result.gnss.checks.maximum_pdop = get("EKF2_REQ_PDOP");
        result.gnss.checks.maximum_horizontal_accuracy_m = get("EKF2_REQ_EPH");
        result.gnss.checks.maximum_vertical_accuracy_m = get("EKF2_REQ_EPV");
        result.gnss.checks.maximum_speed_accuracy_m_s = get("EKF2_REQ_SACC");
        result.gnss.checks.healthy_time_us = (get("EKF2_REQ_GPS_H") * 1e6) as u64;
        result.gnss.checks.velocity_limit_m_s = result.imu.velocity_limit;
        result.range.control = get("EKF2_RNG_CTRL") as i32;
        result.range.delay_ms = get("EKF2_RNG_DELAY");
        result.range.noise_m = get("EKF2_RNG_NOISE");
        result.range.gate = get("EKF2_RNG_GATE");
        result.range.scale_error = get("EKF2_RNG_SFE");
        result.range.conditional_max_height_m = get("EKF2_RNG_A_HMAX");
        result.range.conditional_max_speed_m_s = get("EKF2_RNG_A_VMAX");
        result.range.kinematic_gate = get("EKF2_RNG_K_GATE");
        result.range.terrain_noise_m_s = get("EKF2_TERR_NOISE");
        result.range.terrain_gradient = get("EKF2_TERR_GRAD");
        result.range.minimum_ground_clearance_m = get("EKF2_MIN_RNG");
        result.range.offset_body_m = vector(["EKF2_RNG_POS_X", "EKF2_RNG_POS_Y", "EKF2_RNG_POS_Z"]);
        result.range.sensor.pitch_offset_rad = get("EKF2_RNG_PITCH");
        result.range.sensor.quality_hysteresis_us = (get("EKF2_RNG_QLTY_T") * 1e6) as u64;
        result.range.sensor.maximum_fog_distance_m = get("EKF2_RNG_FOG");
        result.output.position_time_constant_s = get("EKF2_TAU_POS");
        result.output.velocity_time_constant_s = get("EKF2_TAU_VEL");
        result.output.imu_offset_body_m =
            vector(["EKF2_IMU_POS_X", "EKF2_IMU_POS_Y", "EKF2_IMU_POS_Z"]);
        result.imu.buffer_length()?;
        result.output.validate()?;
        // The current GNSS core intentionally implements only Auto mode.
        if result.gnss.mode != 0
            || result.gnss.control & !7 != 0
            || result.gnss.checks.mask & !0x7ff != 0
            || result.aiding.mag_check & !7 != 0
            || ![0, 1, 5, 6].contains(&result.aiding.mag_type)
            || !(0..=3).contains(&result.aiding.height_reference)
            || !(0..=2).contains(&result.range.control)
            || result.range.kinematic_gate <= 0.0
        {
            return Err(FilterFault::Input);
        }
        result.mapped = mapped.into_inner();
        Ok(result)
    }

    pub fn maps(&self, id: Id) -> bool {
        self.mapped[id as usize]
    }
}

/// TMST_CONFIG=0x1d enables delta timestamps with 16-us resolution. The
/// reconstructed TIM2 stamp and the sensor integration duration are distinct.
#[derive(Default)]
pub struct ImuBridge {
    last_sequence: Option<u32>,
    last_timestamp: u64,
}

impl ImuBridge {
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Selection {
    Invalid,
    Ekf2,
    Mahony,
}

#[derive(Clone, Copy, Debug)]
pub struct Status {
    pub timestamp: u64,
    pub samples: u32,
    pub ekf_updates: u32,
    pub gnss_samples: u32,
    pub selection: Selection,
    pub fault: Option<Fault>,
    pub max_update_us: u32,
    pub deadline_misses: u32,
    pub independent_fault: Option<Fault>,
}

pub struct Runtime {
    enabled: bool,
    algorithm: Algorithm,
    /// Firmware-visible for split reconfigure diagnostics only; math state stays internal.
    pub main: GnssEkf,
    pub independent: NoGnssEkf,
    fallback: Mahony,
    bridge: ImuBridge,
    status: Status,
    fallback_valid: bool,
    last_angular_velocity: [f32; 3],
}

impl Runtime {
    /// Valid but inactive storage template, initialized directly in static SRAM.
    pub const fn dormant() -> Self {
        Self {
            enabled: false,
            algorithm: Algorithm::Mahony,
            main: GnssEkf::dormant(),
            independent: NoGnssEkf::dormant(),
            fallback: Mahony::new(),
            bridge: ImuBridge {
                last_sequence: None,
                last_timestamp: 0,
            },
            status: Status {
                timestamp: 0,
                samples: 0,
                ekf_updates: 0,
                gnss_samples: 0,
                selection: Selection::Invalid,
                fault: None,
                max_update_us: 0,
                deadline_misses: 0,
                independent_fault: None,
            },
            fallback_valid: false,
            last_angular_velocity: [0.0; 3],
        }
    }

    /// Reconstruct members sequentially instead of returning/copying one large
    /// Runtime through an async task's stack on every control transition.
    #[inline(never)]
    pub fn reconfigure(&mut self, params: Params) -> Result<(), FilterFault> {
        self.enabled = false;
        self.main
            .reconfigure(params.imu, params.aiding, params.range, params.gnss, 0)?;
        self.main.configure_output(params.output)?;
        self.independent
            .reconfigure(params.imu, params.aiding, params.range, 0)?;
        self.independent.configure_output(params.output)?;
        self.finish_reconfigure(params.enabled, params.algorithm);
        Ok(())
    }

    /// Activate the already-reconfigured storage; visible to the firmware task
    /// so a failing sub-step can be identified without re-running the others.
    pub fn finish_reconfigure(&mut self, enabled: bool, algorithm: Algorithm) {
        self.enabled = enabled;
        self.algorithm = algorithm;
        self.fallback = Mahony::default();
        self.bridge = ImuBridge::default();
        self.status = Status {
            timestamp: 0,
            samples: 0,
            ekf_updates: 0,
            gnss_samples: 0,
            selection: Selection::Invalid,
            fault: None,
            max_update_us: 0,
            deadline_misses: 0,
            independent_fault: None,
        };
        self.fallback_valid = false;
        self.last_angular_velocity = [0.0; 3];
    }

    pub fn new(params: Params, t0: u64) -> Result<Self, FilterFault> {
        let mut main = GnssEkf::new(params.imu, params.aiding, params.range, params.gnss, t0)?;
        let mut independent = NoGnssEkf::new(params.imu, params.aiding, params.range, t0)?;
        main.configure_output(params.output)?;
        independent.configure_output(params.output)?;
        Ok(Self {
            enabled: params.enabled,
            algorithm: params.algorithm,
            main,
            independent,
            fallback: Mahony::default(),
            bridge: ImuBridge::default(),
            status: Status {
                timestamp: t0,
                samples: 0,
                ekf_updates: 0,
                gnss_samples: 0,
                selection: Selection::Invalid,
                fault: None,
                max_update_us: 0,
                deadline_misses: 0,
                independent_fault: None,
            },
            fallback_valid: false,
            last_angular_velocity: [0.0; 3],
        })
    }

    #[inline(never)]
    pub fn update(
        &mut self,
        sample: icm42688::Sample,
        now: u64,
        baro: Option<BaroAltitude>,
        gnss: Option<GnssObservation>,
        range: Option<RangeObservation>,
    ) {
        self.fallback_valid = false;
        if !self.enabled {
            self.status.selection = Selection::Invalid;
            return;
        }
        let imu = match self.bridge.convert(sample, now) {
            Ok(imu) => imu,
            Err(fault) => {
                self.invalidate(fault);
                return;
            }
        };
        self.status.timestamp = imu.timestamp;
        self.status.samples = self.status.samples.wrapping_add(1);
        self.last_angular_velocity = imu.delta_angle_rad.map(|v| v / imu.delta_angle_dt_s);
        self.fallback_valid = self.fallback.update(&imu).is_ok();
        if self.algorithm == Algorithm::Ekf2 && self.status.fault.is_none() {
            if gnss.is_some() {
                self.status.gnss_samples = self.status.gnss_samples.wrapping_add(1);
            }
            match self.main.update_baro(&imu, baro, gnss, range) {
                Ok(updated) => {
                    self.status.ekf_updates =
                        self.status.ekf_updates.wrapping_add(u32::from(updated))
                }
                Err(fault) => self.status.fault = Some(Fault::Ekf(fault)),
            }
        }
        if self.algorithm == Algorithm::Ekf2 && self.status.independent_fault.is_none() {
            if let Err(fault) = self.independent.update_baro(&imu, baro, range) {
                self.status.independent_fault = Some(Fault::Ekf(fault));
            }
        }
        self.status.selection = if self.algorithm == Algorithm::Ekf2
            && self.status.fault.is_none()
            && self.main.core().health().valid
        {
            Selection::Ekf2
        } else if self.fallback_valid {
            Selection::Mahony
        } else {
            Selection::Invalid
        };
    }

    pub fn invalidate(&mut self, fault: Fault) {
        self.status.fault = Some(fault);
        self.status.selection = Selection::Invalid;
        self.fallback_valid = false;
        self.status.independent_fault = Some(fault);
    }

    pub fn record_duration(&mut self, elapsed_us: u32) {
        self.status.max_update_us = self.status.max_update_us.max(elapsed_us);
        if elapsed_us > DEADLINE_US {
            self.status.deadline_misses = self.status.deadline_misses.wrapping_add(1);
            self.status.fault = Some(Fault::Deadline);
            self.status.independent_fault = Some(Fault::Deadline);
            self.status.selection = if self.fallback_valid {
                Selection::Mahony
            } else {
                Selection::Invalid
            };
        }
    }

    pub fn status(&self) -> Status {
        self.status
    }

    #[inline(never)]
    pub fn snapshots(&mut self, now: u64) -> [Snapshot; 2] {
        if now < self.status.timestamp || now - self.status.timestamp > FRESH_US {
            self.invalidate(Fault::Stale);
        }
        let mut snapshots = [
            self.main.standard_output(now),
            self.independent.standard_output(now),
        ];
        let [main, independent] = &mut snapshots;
        if self.status.independent_fault.is_some() {
            independent.attitude = None;
            independent.angular_velocity = None;
            independent.odometry = None;
            independent.status.solution_status_flags &= !1;
        }
        if self.status.selection != Selection::Ekf2 {
            main.attitude = None;
            main.angular_velocity = None;
            main.odometry = None;
            main.status.solution_status_flags &= !1;
            if self.status.selection == Selection::Mahony && self.fallback_valid {
                if let Some(q) = self.fallback.output() {
                    main.attitude = Some(Attitude {
                        timestamp: now,
                        timestamp_sample: self.status.timestamp,
                        q,
                        delta_q_reset: [1.0, 0.0, 0.0, 0.0],
                        quat_reset_counter: 0,
                    });
                    main.angular_velocity = Some(AngularVelocity {
                        timestamp: now,
                        timestamp_sample: self.status.timestamp,
                        xyz: self.last_angular_velocity,
                        xyz_derivative: [f32::NAN; 3],
                    });
                    main.status.solution_status_flags |= 1;
                    main.local_position.heading = super::math::yaw(super::math::rotation(q));
                    main.local_position.heading_var = f32::INFINITY;
                    main.local_position.tilt_var = f32::NAN;
                }
            }
        }
        if self.algorithm != Algorithm::Ekf2 {
            independent.attitude = None;
            independent.angular_velocity = None;
            independent.odometry = None;
        }
        for snapshot in &mut snapshots {
            unqualified_navigation(snapshot);
        }
        snapshots
    }
}

/// Bring-up may report attitude, but cannot release flight-navigation validity
/// before the sensor/replay/HIL gates. Numerics remain available as diagnostics.
fn unqualified_navigation(snapshot: &mut Snapshot) {
    snapshot.local_position.xy_valid = false;
    snapshot.local_position.v_xy_valid = false;
    snapshot.local_position.z_valid = false;
    snapshot.local_position.v_z_valid = false;
    snapshot.local_position.heading_good_for_control = false;
    snapshot.global_position = None;
    snapshot.status.solution_status_flags &= !0x7e;
    if let Some(odometry) = &mut snapshot.odometry {
        odometry.position_variance = [f32::INFINITY; 3];
        odometry.velocity_variance = [f32::INFINITY; 3];
        // IMU/baro-only yaw is relative. Do not report a finite global heading
        // uncertainty while its reference is unobservable.
        if snapshot.status.control_mode_flags & (1 << 1) == 0 {
            odometry.orientation_variance[2] = f32::INFINITY;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(sequence: u32, timestamp: u64) -> icm42688::Sample {
        icm42688::Sample {
            sample_timestamp_us: timestamp,
            sequence,
            gyro_rad_s: [0.0; 3],
            accel_m_s2: [0.0, 0.0, 9.80665],
            temperature_c: 25.0,
            sensor_timestamp: 8,
        }
    }

    #[test]
    fn input_bridge_uses_frd_sensor_delta_and_rejects_gaps_stale_and_bad_time() {
        let mut bridge = ImuBridge::default();
        let imu = bridge.convert(sample(u32::MAX, 1_000), 1_100).unwrap();
        assert_eq!(imu.delta_velocity_dt_s, 0.000128);
        assert!(imu.delta_velocity_m_s[2] < 0.0);
        bridge.convert(sample(0, 1_125), 1_200).unwrap();
        assert_eq!(
            bridge.convert(sample(2, 1_250), 1_300),
            Err(Fault::Sequence)
        );
        assert_eq!(
            bridge.convert(sample(1, 1_000), 1_300),
            Err(Fault::Timestamp)
        );
        assert_eq!(bridge.convert(sample(1, 1_250), 101_251), Err(Fault::Stale));
    }

    #[test]
    fn runtime_defaults_disable_start_and_keep_all_142_parameters_in_one_registry() {
        let values = Parameters::default();
        let params = Params::from_parameters(values).unwrap();
        assert!(!params.enabled);
        assert_eq!(params.imu.predict_us, 10_000);
        assert_eq!(
            crate::parameters::REGISTRY
                .iter()
                .filter(|v| v.name.starts_with("EKF2_"))
                .count(),
            142
        );
    }

    #[test]
    fn real_imu_runtime_never_claims_navigation_and_deadline_reverts_to_mahony() {
        let params =
            Params::from_parameters(Parameters::default().with(Id::EstEnable, 1.0).unwrap())
                .unwrap();
        let mut runtime = Runtime::dormant();
        runtime.update(sample(0, 1_000_000), 1_000_010, None, None, None);
        assert_eq!(runtime.status().samples, 0);
        runtime.reconfigure(params).unwrap();
        for sequence in 0..12_000 {
            let time = 1_000_000 + (u64::from(sequence) + 1) * 125;
            runtime.update(sample(sequence, time), time + 10, None, None, None);
        }
        let snapshots = runtime.snapshots(2_500_100);
        assert!(runtime.status().ekf_updates > 0);
        assert!(snapshots[0].attitude.is_some());
        for snapshot in snapshots {
            assert!(snapshot.global_position.is_none());
            assert!(!snapshot.local_position.xy_valid && !snapshot.local_position.z_valid);
        }
        runtime.record_duration(DEADLINE_US + 1);
        assert_eq!(runtime.status().fault, Some(Fault::Deadline));
        assert_eq!(runtime.status().selection, Selection::Mahony);
        runtime.invalidate(Fault::Stale);
        assert!(runtime.snapshots(2_700_000)[0].attitude.is_none());
        runtime.reconfigure(params).unwrap();
        assert_eq!(runtime.status().samples, 0);
        assert_eq!(runtime.status().fault, None);
        assert_eq!(runtime.status().independent_fault, None);
        runtime.update(sample(20_000, 3_000_000), 3_000_010, None, None, None);
        assert_eq!(runtime.status().samples, 1);
        assert_eq!(runtime.status().fault, None);
    }

    #[test]
    fn independent_pressure_input_does_not_start_a_fake_magnetometer() {
        let params =
            Params::from_parameters(Parameters::default().with(Id::EstEnable, 1.0).unwrap())
                .unwrap();
        let mut runtime = Runtime::new(params, 0).unwrap();
        for sequence in 0_u32..32_000 {
            let time = 1_000_000 + (u64::from(sequence) + 1) * 125;
            let baro = sequence.is_multiple_of(400).then_some(BaroAltitude {
                timestamp: time,
                altitude_m: 25.0,
            });
            runtime.update(sample(sequence, time), time + 10, baro, None, None);
        }
        let snapshots = runtime.snapshots(5_000_100);
        for snapshot in snapshots {
            assert!(snapshot.aids[0].is_some());
            assert!(snapshot.aids[1].is_none());
            assert_eq!(
                snapshot.status.control_mode_flags & ((1 << 4) | (1 << 5) | (1 << 35)),
                0
            );
            assert!(snapshot.global_position.is_none());
        }
    }
}
