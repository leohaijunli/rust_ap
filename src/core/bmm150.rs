#![forbid(unsafe_code)]

//! BMM150 signed ADC/trim decoding and compensated FRD magnetic field.
//! Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.
//! Fixed source: PX4 d6f12ad1c4f70ad3230afd7d86e971421e02fef4,
//! src/drivers/magnetometer/bosch/bmm150/{BMM150.cpp,Bosch_BMM150_registers.hpp}.
//! Factory trim is not a vehicle hard/soft-iron calibration.

pub const ADDRESS: u8 = 0x10;
pub const CHIP_ID: u8 = 0x32;
// PX4 DeviceId: BMM150 (0x43), address 0x10, internal I2C bus 4 (type 1).
pub const DEVICE_ID: u32 = (0x43 << 16) | ((ADDRESS as u32) << 8) | (4 << 3) | 1;
pub const PERIOD_US: u64 = 50_000;
pub const FRESH_US: u64 = 300_000;
pub const DATA_LEN: usize = 9;
pub const POWER_ON: u8 = 0x01;
pub const SOFT_RESET: u8 = 0x83;
pub const SLEEP_MODE: u8 = 0x06;
pub const SELF_TEST: u8 = 0x01;

pub mod register {
    pub const CHIP_ID: u8 = 0x40;
    pub const DATA: u8 = 0x42;
    pub const POWER: u8 = 0x4b;
    pub const OP_MODE: u8 = 0x4c;
    pub const REP_XY: u8 = 0x51;
    pub const REP_Z: u8 = 0x52;
    pub const TRIM_XY1: u8 = 0x5d;
    pub const TRIM_Z4_XY2: u8 = 0x62;
    pub const TRIM_Z2_XYZ1: u8 = 0x68;
}

/// Fixed PX4 high-accuracy preset: normal mode, 20 Hz, nXY=47, nZ=83.
pub const SETTINGS: [(u8, u8, u8); 4] = [
    (register::POWER, POWER_ON, 0x83),
    (register::OP_MODE, 0x28, 0x3f),
    (register::REP_XY, 0x17, 0xff),
    (register::REP_Z, 0x52, 0xff),
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Identity,
    Trim,
    NotReady,
    Overflow,
    Hall,
    NonFinite,
}

/// A suspended BMM150 reads CHIP_ID as zero. Only the documented wake write
/// is allowed in that state; reset/configuration require a subsequent 0x32 ID.
pub fn needs_wake(chip_id: u8, power: u8) -> Result<bool, Error> {
    match chip_id {
        CHIP_ID => Ok(false),
        0 if power & POWER_ON == 0 => Ok(true),
        _ => Err(Error::Identity),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RawSample {
    pub x: i16,
    pub y: i16,
    pub z: i16,
    pub rhall: u16,
}

impl RawSample {
    pub fn decode(data: &[u8; DATA_LEN]) -> Result<Self, Error> {
        if data[6] & 1 == 0 {
            return Err(Error::NotReady);
        }
        let sample = Self {
            x: i16::from_le_bytes([data[0], data[1]]) >> 3,
            y: i16::from_le_bytes([data[2], data[3]]) >> 3,
            z: i16::from_le_bytes([data[4], data[5]]) >> 1,
            rhall: u16::from_le_bytes([data[6], data[7]]) >> 2,
        };
        if data[8] & 0x40 != 0 || sample.x == -4096 || sample.y == -4096 || sample.z == -16384 {
            return Err(Error::Overflow);
        }
        if sample.rhall == 0 {
            return Err(Error::Hall);
        }
        Ok(sample)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Trim {
    x1: i8,
    y1: i8,
    x2: i8,
    y2: i8,
    z1: u16,
    z2: i16,
    z3: i16,
    z4: i16,
    xy1: u8,
    xy2: i8,
    xyz1: u16,
}

impl Trim {
    pub fn decode(xy1: &[u8; 2], z4_xy2: &[u8; 4], z2_xyz1: &[u8; 10]) -> Result<Self, Error> {
        let trim = Self {
            x1: xy1[0] as i8,
            y1: xy1[1] as i8,
            x2: z4_xy2[2] as i8,
            y2: z4_xy2[3] as i8,
            z1: u16::from_le_bytes([z2_xyz1[2], z2_xyz1[3]]),
            z2: i16::from_le_bytes([z2_xyz1[0], z2_xyz1[1]]),
            z3: i16::from_le_bytes([z2_xyz1[6], z2_xyz1[7]]),
            z4: i16::from_le_bytes([z4_xy2[0], z4_xy2[1]]),
            xy1: z2_xyz1[9],
            xy2: z2_xyz1[8] as i8,
            xyz1: u16::from_le_bytes([z2_xyz1[4], z2_xyz1[5] & 0x7f]),
        };
        if trim.xyz1 == 0
            || trim.z1 == 0
            || trim.z2 == 0
            || xy1.iter().chain(z4_xy2).chain(z2_xyz1).all(|v| *v == 0xff)
        {
            return Err(Error::Trim);
        }
        Ok(trim)
    }

    /// Compensate the actual ADC burst; invalid data never becomes a zero field.
    /// PX4 uses ROTATION_NONE for this board's BMM150, then uT -> Gauss (0.01).
    pub fn compensate(&self, data: &[u8; DATA_LEN]) -> Result<[f32; 3], Error> {
        let raw = RawSample::decode(data)?;
        let hall = f32::from(raw.rhall);
        let deviation = f32::from(self.xyz1) * 16384.0 / hall - 16384.0;
        let quadratic = f32::from(self.xy2) * (deviation * deviation / 268435456.0);
        let correction = quadratic + deviation * f32::from(self.xy1) / 16384.0;
        let xy = |raw: i16, first: i8, second: i8| {
            let scale = f32::from(second) + 160.0;
            let adjusted = f32::from(raw) * ((correction + 256.0) * scale);
            ((adjusted / 8192.0) + f32::from(first) * 8.0) / 16.0
        };
        let z0 = f32::from(raw.z) - f32::from(self.z4);
        let z1 = hall - f32::from(self.xyz1);
        let z2 = f32::from(self.z3) * z1;
        let z3 = f32::from(self.z1) * hall / 32768.0;
        let denominator = f32::from(self.z2) + z3;
        if denominator == 0.0 {
            return Err(Error::Trim);
        }
        let z5 = z0 * 131072.0 - z2;
        let field = [
            xy(raw.x, self.x1, self.x2),
            xy(raw.y, self.y1, self.y2),
            (z5 / (denominator * 4.0)) / 16.0,
        ]
        .map(|v| v * 0.01);
        if field.iter().any(|v| !v.is_finite()) {
            return Err(Error::NonFinite);
        }
        Ok(field)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub timestamp_sample: u64,
    pub sequence: u32,
    pub field_gauss: [f32; 3],
    pub error_count: u32,
}

impl Sample {
    pub fn fresh(self, now: u64) -> bool {
        now >= self.timestamp_sample
            && now - self.timestamp_sample <= FRESH_US
            && self.field_gauss.iter().all(|v| v.is_finite())
            && self.field_gauss.iter().any(|v| *v != 0.0)
    }

    pub fn measurement(self, now: u64) -> Option<crate::estimator::Measurement> {
        self.fresh(now)
            .then_some(crate::estimator::Measurement::Mag(
                crate::estimator::MagSample {
                    timestamp: self.timestamp_sample,
                    field_gauss: self.field_gauss,
                },
            ))
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub fn data(x: i16, y: i16, z: i16, rhall: u16) -> [u8; DATA_LEN] {
        let mut data = [0; DATA_LEN];
        data[..2].copy_from_slice(&(x * 8).to_le_bytes());
        data[2..4].copy_from_slice(&(y * 8).to_le_bytes());
        data[4..6].copy_from_slice(&(z * 2).to_le_bytes());
        data[6..8].copy_from_slice(&((rhall << 2) | 1).to_le_bytes());
        data
    }

    pub fn trim() -> Trim {
        Trim::decode(&[1, 2], &[0; 4], &[0xe8, 3, 0, 0x80, 0, 0x20, 0, 0, 0, 0]).unwrap()
    }

    #[test]
    fn identity_wake_policy_does_not_configure_an_unidentified_device() {
        assert_eq!(needs_wake(0, 0), Ok(true));
        assert_eq!(needs_wake(CHIP_ID, POWER_ON), Ok(false));
        assert_eq!(needs_wake(0, POWER_ON), Err(Error::Identity));
        assert_eq!(needs_wake(0xff, 0), Err(Error::Identity));
        assert_eq!(DEVICE_ID, 0x431021);
    }

    #[test]
    fn signed_adc_and_trim_compensate_in_gauss_with_identity_board_rotation() {
        let bytes = data(70, -1, 200, 8192);
        assert_eq!(
            RawSample::decode(&bytes).unwrap(),
            RawSample {
                x: 70,
                y: -1,
                z: 200,
                rhall: 8192
            }
        );
        let field = trim().compensate(&bytes).unwrap();
        assert!((field[0] - 0.22375).abs() < 1e-6);
        assert!((field[1] - 0.006875).abs() < 1e-6);
        assert!((field[2] - (200.0 * 2048.0 / 9192.0) * 0.01).abs() < 1e-6);
        assert!(trim().compensate(&data(-70, 1, -200, 8192)).unwrap()[2] < 0.0);
    }

    #[test]
    fn unready_overflow_and_bad_trim_never_publish_a_zero_measurement() {
        assert_eq!(RawSample::decode(&[0; DATA_LEN]), Err(Error::NotReady));
        for bytes in [
            data(-4096, 0, 1, 8192),
            data(0, -4096, 1, 8192),
            data(0, 1, -16384, 8192),
        ] {
            assert_eq!(trim().compensate(&bytes), Err(Error::Overflow));
        }
        let mut bytes = data(1, 2, 3, 8192);
        bytes[8] = 0x40;
        assert_eq!(trim().compensate(&bytes), Err(Error::Overflow));
        assert_eq!(trim().compensate(&data(1, 2, 3, 0)), Err(Error::Hall));
        assert!(matches!(
            Trim::decode(&[0; 2], &[0; 4], &[0; 10]),
            Err(Error::Trim)
        ));
        assert!(matches!(
            Trim::decode(&[0xff; 2], &[0xff; 4], &[0xff; 10]),
            Err(Error::Trim)
        ));
        let zero_denominator =
            Trim::decode(&[1, 2], &[0; 4], &[0, 0xe0, 0, 0x80, 0, 0x20, 0, 0, 0, 0]).unwrap();
        assert_eq!(
            zero_denominator.compensate(&data(1, 2, 3, 8192)),
            Err(Error::Trim)
        );
    }

    #[test]
    fn freshness_and_finite_field_gate_the_existing_mag_measurement_entry() {
        let sample = Sample {
            timestamp_sample: 1_000_000,
            sequence: 1,
            field_gauss: [0.22, 0.0, 0.43],
            error_count: 0,
        };
        assert!(sample.measurement(1_000_000 + FRESH_US).is_some());
        assert!(sample.measurement(999_999).is_none());
        assert!(sample.measurement(1_000_001 + FRESH_US).is_none());
        for field_gauss in [[0.0; 3], [f32::NAN, 0.0, 1.0], [1.0, f32::INFINITY, 0.0]] {
            assert!((Sample {
                field_gauss,
                ..sample
            })
            .measurement(1_000_000)
            .is_none());
        }
    }
}
