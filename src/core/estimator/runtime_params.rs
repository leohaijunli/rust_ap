//! Product parameter snapshot. The generic runtime does not interpret EKF fields.

use core::cell::RefCell;

use crate::{
    config::Algorithm,
    parameters::{Id, Snapshot, COUNT},
};

use super::{
    gnss::GnssParams,
    imu_ekf::{baro_mag::BaroMagParams, range::RangeParams, ImuEkfParams},
    output_predictor::OutputParams,
    FilterFault,
};

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
    pub fn from_parameters(values: Snapshot) -> Result<Self, FilterFault> {
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
        result.imu.no_aid_timeout_us = get("EKF2_NOAID_TOUT") as u64;
        result.imu.heading_noise = get("EKF2_HEAD_NOISE");
        result.imu.gyro_bias_limit = get("EKF2_GYR_B_LIM");
        result.imu.accel_bias_limit = get("EKF2_ABL_LIM");
        result.imu.accel_bias_accel_limit = get("EKF2_ABL_ACCLIM");
        result.imu.accel_bias_gyro_limit = get("EKF2_ABL_GYRLIM");
        result.imu.accel_bias_time_constant = get("EKF2_ABL_TAU");
        result.aiding.baro_control = get("EKF2_BARO_CTRL") as i32;
        result.aiding.baro_noise = get("EKF2_BARO_NOISE");
        result.aiding.baro_gate = get("EKF2_BARO_GATE");
        result.aiding.baro_delay_ms = get("EKF2_BARO_DELAY");
        result.aiding.ground_effect_deadzone_m = get("EKF2_GND_EFF_DZ");
        result.aiding.ground_effect_max_height_m = get("EKF2_GND_MAX_HGT");
        result.aiding.mag_noise = get("EKF2_MAG_NOISE");
        result.aiding.mag_gate = get("EKF2_MAG_GATE");
        result.aiding.mag_delay_ms = get("EKF2_MAG_DELAY");
        result.aiding.mag_earth_noise = get("EKF2_MAG_E_NOISE");
        result.aiding.mag_bias_noise = get("EKF2_MAG_B_NOISE");
        result.aiding.mag_declination_deg = get("EKF2_MAG_DECL");
        result.aiding.mag_check = get("EKF2_MAG_CHECK") as i32;
        result.aiding.mag_check_strength_gauss = get("EKF2_MAG_CHK_STR");
        result.aiding.mag_check_inclination_deg = get("EKF2_MAG_CHK_INC");
        result.aiding.mag_accel_limit_m_s2 = get("EKF2_MAG_ACCLIM");
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
        result.validate(true)?;
        result.mapped = mapped.into_inner();
        Ok(result)
    }

    pub fn maps(&self, id: Id) -> bool {
        self.mapped[id as usize]
    }

    /// Delayed-ring capacity implied by the IMU parameters, if they are valid.
    pub fn buffer_length(&self) -> Option<usize> {
        self.imu.buffer_length().ok()
    }

    pub fn validate(&self, gnss: bool) -> Result<(), FilterFault> {
        let length = self.imu.buffer_length()?;
        self.output.validate()?;
        if self.gnss.mode != 0
            || self.gnss.control & !7 != 0
            || self.gnss.checks.mask & !0x7ff != 0
            || self.aiding.mag_check & !7 != 0
            || ![0, 1, 5, 6].contains(&self.aiding.mag_type)
            || !(0..=3).contains(&self.aiding.height_reference)
            || !(0..=1).contains(&self.aiding.baro_control)
            || !(0..=2).contains(&self.range.control)
            || self.range.kinematic_gate <= 0.0
        {
            return Err(FilterFault::Input);
        }
        // Ring capacity is samples, whereas aid delays are time. Leave one
        // prediction period for timestamp-centering and scheduler quantization.
        let delay_ms = [
            if self.aiding.baro_control != 0 {
                self.aiding.baro_delay_ms
            } else {
                0.0
            },
            if self.aiding.mag_type != 5 {
                self.aiding.mag_delay_ms
            } else {
                0.0
            },
            if self.range.control != 0 {
                self.range.delay_ms
            } else {
                0.0
            },
            if gnss && self.gnss.control != 0 {
                self.gnss.delay_ms
            } else {
                0.0
            },
        ]
        .into_iter()
        .fold(0.0_f32, f32::max);
        let period = self.imu.predict_us as f32;
        if (length - 1) as f32 * period < delay_ms * 1000.0 + period {
            return Err(FilterFault::Input);
        }
        Ok(())
    }
}
