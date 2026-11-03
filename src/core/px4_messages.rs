// SPDX-License-Identifier: BSD-3-Clause
// Message fields adapted from PX4 v1.17.0; see tools/px4_msgs/LICENSE.
//! PX4 message contracts at the data boundary; vectors are FRD and SI units.

use core::f32::consts::PI;

use crate::icm42688::{Config, Sample, GRAVITY_M_S2};

pub const PX4_REVISION: &str = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4";
// PX4 DeviceId: ICM42688P (0x26), address 0, SPI bus 2, bus type SPI (2).
pub const ICM42688_DEVICE_ID: u32 = (0x26 << 16) | (2 << 3) | 2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorAccel {
    pub timestamp: u64,
    pub timestamp_sample: u64,
    pub device_id: u32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub temperature: f32,
    pub error_count: u32,
    pub clip_counter: [u8; 3],
    pub samples: u8,
}

impl SensorAccel {
    pub fn from_sample(timestamp: u64, sample: Sample, error_count: u32) -> Self {
        let [x, y, z] = sample.accel_frd();
        Self {
            timestamp,
            timestamp_sample: sample.sample_timestamp_us,
            device_id: ICM42688_DEVICE_ID,
            x,
            y,
            z,
            temperature: sample.temperature_c,
            error_count,
            clip_counter: clipping(
                [x, y, z],
                f32::from(Config::DEFAULT.accel_fsr_g) * GRAVITY_M_S2,
            ),
            samples: 1,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorGyro {
    pub timestamp: u64,
    pub timestamp_sample: u64,
    pub device_id: u32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub temperature: f32,
    pub error_count: u32,
    pub clip_counter: [u8; 3],
    pub samples: u8,
}

impl SensorGyro {
    pub fn from_sample(timestamp: u64, sample: Sample, error_count: u32) -> Self {
        let [x, y, z] = sample.gyro_frd();
        Self {
            timestamp,
            timestamp_sample: sample.sample_timestamp_us,
            device_id: ICM42688_DEVICE_ID,
            x,
            y,
            z,
            temperature: sample.temperature_c,
            error_count,
            clip_counter: clipping(
                [x, y, z],
                f32::from(Config::DEFAULT.gyro_fsr_dps) * PI / 180.0,
            ),
            samples: 1,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ActuatorArmed {
    pub timestamp: u64,
    pub armed: bool,
    pub prearmed: bool,
    pub ready_to_arm: bool,
    pub lockdown: bool,
    pub kill: bool,
    pub termination: bool,
    pub in_esc_calibration_mode: bool,
}

/// Fixed SensorGps.msg contract; built by the P4b-g process_fix2 flow.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorGps {
    pub timestamp: u64,
    pub timestamp_sample: u64,
    pub device_id: u32,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_msl_m: f64,
    pub altitude_ellipsoid_m: f64,
    pub s_variance_m_s: f32,
    pub c_variance_rad: f32,
    pub fix_type: u8,
    pub eph: f32,
    pub epv: f32,
    pub hdop: f32,
    pub vdop: f32,
    pub noise_per_ms: i32,
    pub automatic_gain_control: u16,
    pub jamming_state: u8,
    pub jamming_indicator: i32,
    pub spoofing_state: u8,
    pub authentication_state: u8,
    pub vel_m_s: f32,
    pub vel_n_m_s: f32,
    pub vel_e_m_s: f32,
    pub vel_d_m_s: f32,
    pub cog_rad: f32,
    pub vel_ned_valid: bool,
    pub timestamp_time_relative: i32,
    pub time_utc_usec: u64,
    pub satellites_used: u8,
    pub system_error: u32,
    pub heading: f32,
    pub heading_offset: f32,
    pub heading_accuracy: f32,
    pub rtcm_injection_rate: f32,
    pub selected_rtcm_instance: u8,
    pub rtcm_crc_failed: bool,
    pub rtcm_msg_used: u8,
}

fn clipping(vector: [f32; 3], range: f32) -> [u8; 3] {
    // PX4's single-sample publisher marks saturation at 99.9% of full scale.
    vector.map(|value| u8::from(value.abs() >= range * 0.999))
}
