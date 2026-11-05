#![forbid(unsafe_code)]

//! BMP388 trim/ADC decoding and fixed PX4 integer compensation.
//! Copyright (c) 2019 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.
//! Fixed source: PX4 d6f12ad1c4f70ad3230afd7d86e971421e02fef4,
//! src/drivers/barometer/bmp388/bmp388.{cpp,h}. Intermediate arithmetic is
//! widened to i128 so malformed calibration cannot cause signed overflow.

pub const ADDRESS: u8 = 0x77;
pub const CHIP_ID: u8 = 0x50;
pub const DEVICE_ID: u32 = (0x67 << 16) | ((ADDRESS as u32) << 8) | (4 << 3) | 1;
pub const CONVERSION_US: u64 = 43_300;
pub const FRESH_US: u64 = 200_000;
pub const TRIM_LEN: usize = 21;

pub mod register {
    pub const CHIP_ID: u8 = 0x00;
    pub const REVISION: u8 = 0x01;
    pub const ERROR: u8 = 0x02;
    pub const STATUS: u8 = 0x03;
    pub const POWER: u8 = 0x1b;
    pub const OVERSAMPLING: u8 = 0x1c;
    pub const ODR: u8 = 0x1d;
    pub const IIR: u8 = 0x1f;
    pub const TRIM_CRC: u8 = 0x30;
    pub const TRIM: u8 = 0x31;
    pub const COMMAND: u8 = 0x7e;
}

/// Pressure/temp enabled, sleep mode; pressure x16, temp x2, ODR 50 Hz is
/// unused in forced mode, IIR disabled. Readback masks exclude reserved bits.
pub const SETTINGS: [(u8, u8, u8); 4] = [
    (register::POWER, 0x03, 0x33),
    (register::OVERSAMPLING, 0x0c, 0x3f),
    (register::ODR, 0x02, 0x1f),
    (register::IIR, 0x00, 0x0e),
];
pub const FORCE: u8 = 0x13;
pub const RESET: u8 = 0xb6;
pub const COMMAND_READY: u8 = 0x10;
pub const DATA_READY: u8 = 0x60;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    CalibrationCrc,
    Calibration,
    NotReady,
    Sensor,
    Range,
}

#[derive(Clone, Copy)]
pub struct Calibration {
    t1: i128,
    t2: i128,
    t3: i128,
    p1: i128,
    p2: i128,
    p3: i128,
    p4: i128,
    p5: i128,
    p6: i128,
    p7: i128,
    p8: i128,
    p9: i128,
    p10: i128,
    p11: i128,
}

impl Calibration {
    pub fn decode(bytes: &[u8; TRIM_LEN], stored_crc: u8) -> Result<Self, Error> {
        if trim_crc(bytes) != stored_crc {
            return Err(Error::CalibrationCrc);
        }
        if bytes.iter().all(|v| *v == 0) || bytes.iter().all(|v| *v == 0xff) {
            return Err(Error::Calibration);
        }
        let unsigned = |i| i128::from(u16::from_le_bytes([bytes[i], bytes[i + 1]]));
        let signed = |i| i128::from(i16::from_le_bytes([bytes[i], bytes[i + 1]]));
        let short = |i| i128::from(bytes[i] as i8);
        let result = Self {
            t1: unsigned(0),
            t2: unsigned(2),
            t3: short(4),
            p1: signed(5),
            p2: signed(7),
            p3: short(9),
            p4: short(10),
            p5: unsigned(11),
            p6: unsigned(13),
            p7: short(15),
            p8: short(16),
            p9: signed(17),
            p10: short(19),
            p11: short(20),
        };
        if result.t1 == 0 || result.t2 == 0 {
            return Err(Error::Calibration);
        }
        Ok(result)
    }

    /// Raw status+P24+T24 as returned by one burst beginning at STATUS.
    pub fn compensate(&self, bytes: &[u8; 7], error_status: u8) -> Result<(f32, f32), Error> {
        if error_status & 7 != 0 {
            return Err(Error::Sensor);
        }
        if bytes[0] & DATA_READY != DATA_READY {
            return Err(Error::NotReady);
        }
        let adc = |i| {
            i128::from(
                u32::from(bytes[i])
                    | (u32::from(bytes[i + 1]) << 8)
                    | (u32::from(bytes[i + 2]) << 16),
            )
        };
        let raw_pressure = adc(1);
        let raw_temperature = adc(4);
        let difference = raw_temperature - 256 * self.t1;
        let t_lin =
            (self.t2 * difference * 262_144 + difference * difference * self.t3) / 4_294_967_296;
        let centi_temperature = t_lin * 25 / 16_384;
        if !(-4_000..=8_500).contains(&centi_temperature) {
            return Err(Error::Range);
        }
        // Keep the original integer divisions in order; algebraic regrouping
        // would change centi-Pascal rounding, particularly for signed terms.
        let square = t_lin * t_lin;
        let cube = ((square / 64) * t_lin) / 256;
        let offset = self.p5 * 140_737_488_355_328
            + (self.p8 * cube) / 32
            + self.p7 * square * 16
            + self.p6 * t_lin * 4_194_304;
        let sensitivity = (self.p1 - 16_384) * 70_368_744_177_664
            + (self.p4 * cube) / 32
            + self.p3 * square * 4
            + (self.p2 - 16_384) * t_lin * 2_097_152;
        let linear = (sensitivity / 16_777_216) * raw_pressure;
        let quadratic = ((self.p10 * t_lin + 65_536 * self.p9) * raw_pressure) / 8_192;
        let quadratic = ((raw_pressure * (quadratic / 10)) / 512) * 10;
        let cubic = (((self.p11 * raw_pressure * raw_pressure) / 65_536) * raw_pressure) / 128;
        let centi_pressure = ((offset / 4 + linear + quadratic + cubic) * 25) / 1_099_511_627_776;
        if !(3_000_000..=12_500_000).contains(&centi_pressure) {
            return Err(Error::Range);
        }
        Ok((
            centi_pressure as f32 / 100.0,
            centi_temperature as f32 / 100.0,
        ))
    }
}

pub fn trim_crc(bytes: &[u8; TRIM_LEN]) -> u8 {
    let mut crc = 0xff_u8;
    for byte in bytes {
        crc ^= *byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x1d
            } else {
                crc << 1
            };
        }
    }
    crc ^ 0xff
}

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub timestamp_sample: u64,
    pub sequence: u32,
    pub pressure_pa: f32,
    pub temperature_c: f32,
    pub error_count: u32,
}

impl Sample {
    pub fn fresh(self, now: u64) -> bool {
        now >= self.timestamp_sample && now - self.timestamp_sample <= FRESH_US
    }

    /// Standard pressure altitude, not absolute surveyed/GNSS height. The EKF
    /// baro bias/origin controller owns its reference; never substitute GNSS.
    pub fn pressure_altitude_m(self) -> f32 {
        44_330.77 * (1.0 - libm::powf(self.pressure_pa / 101_325.0, 0.190_263_1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    pub const TRIM: [u8; 21] = [
        108, 107, 0, 125, 1, 56, 74, 128, 62, 5, 254, 224, 46, 44, 1, 253, 1, 200, 0, 255, 1,
    ];

    #[test]
    fn crc_identity_and_little_endian_signed_coefficients_are_checked() {
        let crc = trim_crc(&TRIM);
        let calibration = Calibration::decode(&TRIM, crc).unwrap();
        assert_eq!((calibration.t1, calibration.t2), (27_500, 32_000));
        assert_eq!(
            (
                calibration.p1,
                calibration.p4,
                calibration.p7,
                calibration.p10
            ),
            (19_000, -2, -3, -1)
        );
        assert!(matches!(
            Calibration::decode(&TRIM, crc ^ 1),
            Err(Error::CalibrationCrc)
        ));
        assert!(matches!(
            Calibration::decode(&[0; 21], trim_crc(&[0; 21])),
            Err(Error::Calibration)
        ));
    }

    #[test]
    fn data_ready_errors_and_staleness_do_not_create_zero_measurements() {
        let calibration = Calibration::decode(&TRIM, trim_crc(&TRIM)).unwrap();
        assert_eq!(calibration.compensate(&[0; 7], 0), Err(Error::NotReady));
        assert_eq!(
            calibration.compensate(&[DATA_READY; 7], 1),
            Err(Error::Sensor)
        );
        let sample = Sample {
            timestamp_sample: 100,
            sequence: 0,
            pressure_pa: 101_325.0,
            temperature_c: 25.0,
            error_count: 0,
        };
        assert!(sample.fresh(100 + FRESH_US));
        assert!(!sample.fresh(99));
        assert!(!sample.fresh(101 + FRESH_US));
        assert_eq!(sample.pressure_altitude_m(), 0.0);
    }
}
