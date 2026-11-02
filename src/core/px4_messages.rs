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

fn clipping(vector: [f32; 3], range: f32) -> [u8; 3] {
    // PX4's single-sample publisher marks saturation at 99.9% of full scale.
    vector.map(|value| u8::from(value.abs() >= range * 0.999))
}
