//! ICM-42688-P register contract and high-resolution FIFO decoding.

use core::f32::consts::PI;

pub const WHO_AM_I_VALUE: u8 = 0x47;
pub const READ: u8 = 0x80;
pub const FIFO_BYTES: usize = 2048;
pub const FIFO_PACKET_BYTES: usize = 20;
pub const MAX_FIFO_SAMPLES: usize = 8;
pub const SOURCE_ID: u8 = 0;
pub const GRAVITY_M_S2: f32 = 9.80665;
/// PX4 v1.17.0 / TDK DS-000347 section 12.7, TMST_RES=1 without CLKIN.
pub const FIFO_TIMESTAMP_SCALE_US: f32 = 16.0 * (32.0 / 30.0);

pub mod register {
    pub const DEVICE_CONFIG: u8 = 0x11;
    pub const INT_CONFIG: u8 = 0x14;
    pub const FIFO_CONFIG: u8 = 0x16;
    pub const INT_STATUS: u8 = 0x2d;
    pub const FIFO_COUNTH: u8 = 0x2e;
    pub const FIFO_DATA: u8 = 0x30;
    pub const SIGNAL_PATH_RESET: u8 = 0x4b;
    pub const INTF_CONFIG0: u8 = 0x4c;
    pub const INTF_CONFIG1: u8 = 0x4d;
    pub const PWR_MGMT0: u8 = 0x4e;
    pub const GYRO_CONFIG0: u8 = 0x4f;
    pub const ACCEL_CONFIG0: u8 = 0x50;
    pub const GYRO_CONFIG1: u8 = 0x51;
    pub const GYRO_ACCEL_CONFIG0: u8 = 0x52;
    pub const ACCEL_CONFIG1: u8 = 0x53;
    pub const TMST_CONFIG: u8 = 0x54;
    pub const FIFO_CONFIG1: u8 = 0x5f;
    pub const FIFO_CONFIG2: u8 = 0x60;
    pub const FIFO_CONFIG3: u8 = 0x61;
    pub const INT_CONFIG0: u8 = 0x63;
    pub const INT_CONFIG1: u8 = 0x64;
    pub const INT_SOURCE0: u8 = 0x65;
    pub const WHO_AM_I: u8 = 0x75;
    pub const REG_BANK_SEL: u8 = 0x76;

    pub const GYRO_CONFIG_STATIC2: u8 = 0x0b;
    pub const GYRO_CONFIG_STATIC3: u8 = 0x0c;
    pub const GYRO_CONFIG_STATIC4: u8 = 0x0d;
    pub const GYRO_CONFIG_STATIC5: u8 = 0x0e;
    pub const ACCEL_CONFIG_STATIC2: u8 = 0x03;
    pub const ACCEL_CONFIG_STATIC3: u8 = 0x04;
    pub const ACCEL_CONFIG_STATIC4: u8 = 0x05;
}

pub mod bits {
    pub const SOFT_RESET: u8 = 1 << 0;
    pub const RESET_DONE: u8 = 1 << 4;
    pub const FIFO_FULL: u8 = 1 << 1;
    pub const FIFO_FLUSH: u8 = 1 << 1;
    pub const LOW_NOISE: u8 = 0x0f;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Bank {
    Zero = 0,
    One = 1,
    Two = 2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RegisterSetting {
    pub bank: Bank,
    pub address: u8,
    pub set: u8,
    pub clear: u8,
}

impl RegisterSetting {
    pub const fn apply(self, current: u8) -> u8 {
        (current & !self.clear) | self.set
    }
}

// PX4 v1.17.0 ICM42688P register policy. ODR and FIFO watermark are
// overwritten from Config during initialization.
pub const SETTINGS: &[RegisterSetting] = &[
    setting(Bank::Zero, register::INT_CONFIG, 0x06, 0x01),
    setting(Bank::Zero, register::FIFO_CONFIG, 0xc0, 0x00),
    setting(Bank::Zero, register::INTF_CONFIG0, 0x33, 0x00),
    setting(Bank::Zero, register::INTF_CONFIG1, 0x40, 0x80),
    setting(Bank::Zero, register::PWR_MGMT0, bits::LOW_NOISE, 0x00),
    setting(Bank::Zero, register::GYRO_CONFIG0, 0x03, 0xfc),
    setting(Bank::Zero, register::ACCEL_CONFIG0, 0x03, 0xfc),
    setting(Bank::Zero, register::GYRO_CONFIG1, 0x00, 0x0c),
    setting(Bank::Zero, register::GYRO_ACCEL_CONFIG0, 0x00, 0xff),
    setting(Bank::Zero, register::ACCEL_CONFIG1, 0x00, 0x18),
    setting(Bank::Zero, register::TMST_CONFIG, 0x1d, 0x02),
    setting(Bank::Zero, register::FIFO_CONFIG1, 0x37, 0x08),
    setting(
        Bank::Zero,
        register::FIFO_CONFIG2,
        (8 * FIFO_PACKET_BYTES) as u8,
        0x00,
    ),
    setting(Bank::Zero, register::FIFO_CONFIG3, 0x00, 0x0f),
    setting(Bank::Zero, register::INT_CONFIG0, 0x08, 0x00),
    setting(Bank::Zero, register::INT_CONFIG1, 0x00, 0x10),
    setting(Bank::Zero, register::INT_SOURCE0, 0x04, 0x00),
    setting(Bank::One, register::GYRO_CONFIG_STATIC2, 0x00, 0x03),
    setting(Bank::One, register::GYRO_CONFIG_STATIC3, 0x0d, 0x32),
    setting(Bank::One, register::GYRO_CONFIG_STATIC4, 0xaa, 0x55),
    setting(Bank::One, register::GYRO_CONFIG_STATIC5, 0x80, 0x7f),
    setting(Bank::Two, register::ACCEL_CONFIG_STATIC2, 0x1a, 0x65),
    setting(Bank::Two, register::ACCEL_CONFIG_STATIC3, 0xaa, 0x55),
    setting(Bank::Two, register::ACCEL_CONFIG_STATIC4, 0x80, 0x7f),
];

const fn setting(bank: Bank, address: u8, set: u8, clear: u8) -> RegisterSetting {
    RegisterSetting {
        bank,
        address,
        set,
        clear,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rotation {
    Yaw270,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    pub odr_hz: u16,
    pub log_rate_hz: u16,
    pub accel_fsr_g: u8,
    pub gyro_fsr_dps: u16,
    pub rotation: Rotation,
    pub stale_after_us: u32,
}

impl Config {
    pub const DEFAULT: Self = Self {
        odr_hz: 8_000,
        log_rate_hz: 200,
        accel_fsr_g: 16,
        gyro_fsr_dps: 2_000,
        rotation: Rotation::Yaw270,
        stale_after_us: 2_000,
    };

    pub const fn validate(self) -> Result<Self, ConfigError> {
        if !matches!(self.odr_hz, 1_000 | 8_000) {
            return Err(ConfigError::Odr);
        }
        if self.log_rate_hz == 0
            || self.log_rate_hz > self.odr_hz
            || !self.odr_hz.is_multiple_of(self.log_rate_hz)
        {
            return Err(ConfigError::LogRate);
        }
        if self.accel_fsr_g != 16 {
            return Err(ConfigError::AccelRange);
        }
        if self.gyro_fsr_dps != 2_000 {
            return Err(ConfigError::GyroRange);
        }
        if self.stale_after_us < 1_000 {
            return Err(ConfigError::StaleLimit);
        }
        Ok(self)
    }

    pub const fn fifo_batch_samples(self) -> usize {
        (self.odr_hz / 1_000) as usize
    }

    pub const fn sample_period_us(self) -> u64 {
        1_000_000 / self.odr_hz as u64
    }

    pub const fn log_decimation(self) -> u32 {
        (self.odr_hz / self.log_rate_hz) as u32
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::DEFAULT
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfigError {
    Odr,
    LogRate,
    AccelRange,
    GyroRange,
    StaleLimit,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Sample {
    pub sample_timestamp_us: u64,
    pub sequence: u32,
    pub gyro_rad_s: [f32; 3],
    pub accel_m_s2: [f32; 3],
    pub temperature_c: f32,
    pub sensor_timestamp: u16,
}

/// One actual FIFO read, oldest sample first. Only `count` entries are valid.
#[derive(Clone, Copy, Debug, Default)]
pub struct FifoBatch {
    pub samples: [Sample; MAX_FIFO_SAMPLES],
    pub count: u8,
}

impl Sample {
    pub const fn age_us(self, now_us: u64) -> u32 {
        let age = now_us.saturating_sub(self.sample_timestamp_us);
        if age > u32::MAX as u64 {
            u32::MAX
        } else {
            age as u32
        }
    }

    pub const fn is_stale(self, now_us: u64, config: Config) -> bool {
        self.age_us(now_us) > config.stale_after_us
    }

    pub const fn accel_frd(self) -> [f32; 3] {
        [self.accel_m_s2[0], -self.accel_m_s2[1], -self.accel_m_s2[2]]
    }

    pub const fn gyro_frd(self) -> [f32; 3] {
        [self.gyro_rad_s[0], -self.gyro_rad_s[1], -self.gyro_rad_s[2]]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    Header,
    InvalidAccel,
    InvalidGyro,
    InvalidTemperature,
}

pub fn decode_fifo_packet(
    packet: &[u8; FIFO_PACKET_BYTES],
    sample_timestamp_us: u64,
    sequence: u32,
    config: Config,
) -> Result<Sample, DecodeError> {
    let header = packet[0];
    if header & 0x80 != 0 || header & 0x70 != 0x70 || header & 0x0c != 0x08 || header & 0x03 != 0 {
        return Err(DecodeError::Header);
    }

    let accel = [
        signed20(packet[1], packet[2], packet[17] >> 4),
        signed20(packet[3], packet[4], packet[18] >> 4),
        signed20(packet[5], packet[6], packet[19] >> 4),
    ];
    if accel.contains(&-524_288) {
        return Err(DecodeError::InvalidAccel);
    }
    let gyro = [
        signed20(packet[7], packet[8], packet[17] & 0x0f),
        signed20(packet[9], packet[10], packet[18] & 0x0f),
        signed20(packet[11], packet[12], packet[19] & 0x0f),
    ];
    if gyro.contains(&-524_288) {
        return Err(DecodeError::InvalidGyro);
    }
    let temperature_raw = i16::from_be_bytes([packet[13], packet[14]]);
    if temperature_raw == i16::MIN {
        return Err(DecodeError::InvalidTemperature);
    }

    let accel_scale = GRAVITY_M_S2 / 32_768.0;
    let gyro_scale = PI / (180.0 * 262.0);
    let accel = rotate(
        accel.map(|value| value as f32 * accel_scale),
        config.rotation,
    );
    let gyro = rotate(gyro.map(|value| value as f32 * gyro_scale), config.rotation);

    Ok(Sample {
        sample_timestamp_us,
        sequence,
        gyro_rad_s: gyro,
        accel_m_s2: accel,
        temperature_c: temperature_raw as f32 / 132.48 + 25.0,
        sensor_timestamp: u16::from_be_bytes([packet[15], packet[16]]),
    })
}

pub const fn reconstruct_u32_timestamp(now_us: u64, captured_low: u32) -> u64 {
    let mut timestamp = (now_us & !(u32::MAX as u64)) | captured_low as u64;
    if timestamp > now_us {
        timestamp = timestamp.saturating_sub(1u64 << 32);
    }
    timestamp
}

/// A fresh watermark IRQ already timestamps the last sample of the requested
/// batch. Only the read-time fallback is shifted for a sample left in the FIFO.
pub fn fifo_sample_timestamp(
    now_us: u64,
    drdy_us: Option<u64>,
    unread_duration_us: u64,
    config: Config,
) -> u64 {
    let batch_period_us = config.sample_period_us() * config.fifo_batch_samples() as u64;
    drdy_us
        .filter(|&timestamp| timestamp <= now_us && now_us - timestamp < batch_period_us)
        .unwrap_or_else(|| now_us.saturating_sub(unread_duration_us))
}

fn signed20(msb: u8, lsb: u8, low: u8) -> i32 {
    let value = (u32::from(msb) << 12) | (u32::from(lsb) << 4) | u32::from(low & 0x0f);
    if value & 0x80000 != 0 {
        (value | 0xfff0_0000) as i32
    } else {
        value as i32
    }
}

fn rotate(vector: [f32; 3], rotation: Rotation) -> [f32; 3] {
    match rotation {
        Rotation::Yaw270 => [vector[1], -vector[0], vector[2]],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn put20(
        packet: &mut [u8; FIFO_PACKET_BYTES],
        offset: usize,
        ext: usize,
        value: i32,
        accel: bool,
    ) {
        let raw = value as u32 & 0x000f_ffff;
        packet[offset] = (raw >> 12) as u8;
        packet[offset + 1] = (raw >> 4) as u8;
        if accel {
            packet[ext] |= ((raw & 0x0f) as u8) << 4;
        } else {
            packet[ext] |= (raw & 0x0f) as u8;
        }
    }

    fn packet(accel: [i32; 3], gyro: [i32; 3]) -> [u8; FIFO_PACKET_BYTES] {
        let mut bytes = [0u8; FIFO_PACKET_BYTES];
        bytes[0] = 0x78;
        for (axis, value) in accel.into_iter().enumerate() {
            put20(&mut bytes, 1 + axis * 2, 17 + axis, value, true);
        }
        for (axis, value) in gyro.into_iter().enumerate() {
            put20(&mut bytes, 7 + axis * 2, 17 + axis, value, false);
        }
        bytes[15..17].copy_from_slice(&59u16.to_be_bytes());
        bytes
    }

    #[test]
    fn default_config_matches_m1_contract() {
        assert_eq!(Config::DEFAULT.validate(), Ok(Config::DEFAULT));
        assert_eq!(Config::DEFAULT.fifo_batch_samples(), 8);
        assert_eq!(Config::DEFAULT.sample_period_us(), 125);
        assert_eq!(Config::DEFAULT.log_decimation(), 40);
        // At 1 kHz, 32 intervals contain 1875 ticks (58.59375 ticks/interval).
        assert_eq!(libm::roundf(1_875.0 * FIFO_TIMESTAMP_SCALE_US), 32_000.0);
        assert_eq!(SETTINGS[5].apply(0xff), 0x03);
        assert_eq!(SETTINGS[6].apply(0xff), 0x03);
        assert_eq!(SETTINGS[12].set, 160);
        let low_rate = Config {
            odr_hz: 1_000,
            log_rate_hz: 500,
            ..Config::DEFAULT
        };
        assert_eq!(low_rate.validate(), Ok(low_rate));
        assert_eq!(low_rate.fifo_batch_samples(), 1);
        assert_eq!(low_rate.sample_period_us(), 1_000);
        assert_eq!(low_rate.log_decimation(), 2);
    }

    #[test]
    fn high_resolution_packet_converts_si_and_yaw270() {
        let bytes = packet([32_768, 0, -32_768], [262, -524, 0]);
        let sample = decode_fifo_packet(&bytes, 12_000, 7, Config::DEFAULT).unwrap();
        assert!(sample.accel_m_s2[0].abs() < 1e-6);
        assert!((sample.accel_m_s2[1] + GRAVITY_M_S2).abs() < 1e-5);
        assert!((sample.accel_m_s2[2] + GRAVITY_M_S2).abs() < 1e-5);
        assert!((sample.gyro_rad_s[0] + 2.0 * PI / 180.0).abs() < 1e-6);
        assert!((sample.gyro_rad_s[1] + PI / 180.0).abs() < 1e-6);
        assert_eq!(sample.temperature_c, 25.0);
        assert_eq!(sample.sensor_timestamp, 59);
        assert_eq!(sample.accel_frd(), [0.0, GRAVITY_M_S2, GRAVITY_M_S2]);
    }

    #[test]
    fn malformed_and_invalid_packets_are_rejected() {
        let mut bytes = packet([0; 3], [0; 3]);
        bytes[0] = 0x80;
        assert_eq!(
            decode_fifo_packet(&bytes, 0, 0, Config::DEFAULT),
            Err(DecodeError::Header)
        );
        let bytes = packet([-524_288, 0, 0], [0; 3]);
        assert_eq!(
            decode_fifo_packet(&bytes, 0, 0, Config::DEFAULT),
            Err(DecodeError::InvalidAccel)
        );
        let mut bytes = packet([0; 3], [0; 3]);
        bytes[13..15].copy_from_slice(&i16::MIN.to_be_bytes());
        assert_eq!(
            decode_fifo_packet(&bytes, 0, 0, Config::DEFAULT),
            Err(DecodeError::InvalidTemperature)
        );
    }

    #[test]
    fn age_is_saturating_and_staleness_is_explicit() {
        let sample =
            decode_fifo_packet(&packet([0; 3], [0; 3]), 1_000, 0, Config::DEFAULT).unwrap();
        assert_eq!(sample.age_us(900), 0);
        assert!(!sample.is_stale(3_000, Config::DEFAULT));
        assert!(sample.is_stale(3_001, Config::DEFAULT));
        assert_eq!(sample.age_us(u64::MAX), u32::MAX);
    }

    #[test]
    fn watermark_timestamp_is_not_shifted_twice_when_one_sample_remains() {
        let config = Config::DEFAULT;
        assert_eq!(
            fifo_sample_timestamp(10_050, Some(10_000), 125, config),
            10_000
        );
        assert_eq!(fifo_sample_timestamp(10_050, None, 125, config), 9_925);
        assert_eq!(
            fifo_sample_timestamp(11_000, Some(10_000), 125, config),
            10_875
        );
        assert_eq!(
            fifo_sample_timestamp(10_050, Some(10_100), 0, config),
            10_050
        );
    }

    #[test]
    fn interrupt_timestamp_reconstruction_handles_u32_wrap() {
        assert_eq!(
            reconstruct_u32_timestamp((1u64 << 32) + 10, 5),
            (1u64 << 32) + 5
        );
        assert_eq!(
            reconstruct_u32_timestamp((1u64 << 32) + 10, u32::MAX - 5),
            u64::from(u32::MAX - 5)
        );
    }
}
