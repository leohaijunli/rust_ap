// SPDX-License-Identifier: MIT
// Register/scaling reference: LibDriver INA219 266a1ee32a5f70e4e5d47591ce28c4e1c387857c.
//! Fixed SEN0291 measurement contract: 10 mOhm shunt, +/-8 A, SI units.

// SEN0291 DIP switch A0=0, A1=1 selects 0x44; initialize auto-adopts a probed address.
pub const ADDRESS: u8 = 0x44;
pub const PROBE_ADDRESSES: [u8; 4] = [0x40, 0x41, 0x44, 0x45];
pub const SHUNT_RESISTANCE_OHM: f32 = 0.010;
pub const MAX_CURRENT_A: f32 = 8.0;
pub const MAX_BUS_VOLTAGE_V: f32 = 26.0;
pub const CURRENT_LSB_A: f32 = MAX_CURRENT_A / 32768.0;
// TI SBOS448G Figure 27: FS0 is read-only zero.
pub const CALIBRATION_VALUE: u16 = (0.04096 / (CURRENT_LSB_A * SHUNT_RESISTANCE_OHM)) as u16 & !1;
pub const RESET: u16 = 0x8000;
pub const RESET_CONFIGURATION: u16 = 0x399f;
// 32 V ADC range, +/-80 mV shunt, 12-bit ADCs, triggered shunt + bus.
pub const CONFIGURATION: u16 = 0x299b;
pub const CONVERSION_READY: u16 = 1 << 1;
pub const MATH_OVERFLOW: u16 = 1;
pub const SAMPLE_PERIOD_US: u64 = 100_000;
pub const STALE_AFTER_US: u64 = 300_000;

pub mod register {
    pub const CONFIGURATION: u8 = 0x00;
    pub const SHUNT_VOLTAGE: u8 = 0x01;
    pub const BUS_VOLTAGE: u8 = 0x02;
    pub const POWER: u8 = 0x03;
    pub const CURRENT: u8 = 0x04;
    pub const CALIBRATION: u8 = 0x05;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    NotReady,
    Overflow,
    VoltageRange,
    CurrentRange,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Measurement {
    pub timestamp_us: u64,
    pub bus_voltage_v: f32,
    pub shunt_voltage_v: f32,
    pub current_a: f32,
    pub power_w: f32,
}

impl Measurement {
    pub fn decode(
        bus: u16,
        shunt: i16,
        current: i16,
        timestamp_us: u64,
    ) -> Result<Self, DecodeError> {
        if bus & MATH_OVERFLOW != 0 {
            return Err(DecodeError::Overflow);
        }
        if bus & CONVERSION_READY == 0 {
            return Err(DecodeError::NotReady);
        }
        let bus_voltage_v = f32::from(bus >> 3) * 0.004;
        if bus_voltage_v > MAX_BUS_VOLTAGE_V {
            return Err(DecodeError::VoltageRange);
        }
        let shunt_voltage_v = f32::from(shunt) * 10e-6;
        if shunt_voltage_v.abs() >= MAX_CURRENT_A * SHUNT_RESISTANCE_OHM {
            return Err(DecodeError::CurrentRange);
        }
        let current_a = f32::from(current) * CURRENT_LSB_A;
        Ok(Self {
            timestamp_us,
            bus_voltage_v,
            shunt_voltage_v,
            current_a,
            // Signed load power from the same conversion, including reverse current.
            power_w: bus_voltage_v * current_a,
        })
    }

    pub const fn is_fresh(self, now_us: u64) -> bool {
        now_us >= self.timestamp_us && now_us - self.timestamp_us <= STALE_AFTER_US
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sen0291_calibration_matches_the_eight_amp_range() {
        assert_eq!(CALIBRATION_VALUE, 16776);
        assert_eq!((CONFIGURATION >> 11) & 3, 1);
        assert_eq!(CONFIGURATION & 7, 3);
        assert!((CURRENT_LSB_A * 4096.0 - 1.0).abs() < 1e-6);
        let register_at_one_amp = 1000 * u32::from(CALIBRATION_VALUE) / 4096;
        assert!((register_at_one_amp as f32 * CURRENT_LSB_A - 1.0).abs() <= CURRENT_LSB_A);
    }

    #[test]
    fn register_values_use_si_units() {
        let measurement = Measurement::decode(0x5dc2, 1000, 4096, 123).unwrap();
        assert!((measurement.bus_voltage_v - 12.0).abs() < 1e-5);
        assert!((measurement.shunt_voltage_v - 0.010).abs() < 1e-6);
        assert!((measurement.current_a - 1.0).abs() < 1e-6);
        assert!((measurement.power_w - 12.0).abs() < 1e-5);
        assert_eq!(measurement.timestamp_us, 123);
    }

    #[test]
    fn reverse_current_and_power_keep_their_sign() {
        let measurement = Measurement::decode(0x5dc2, -1000, -4096, 123).unwrap();
        assert!((measurement.current_a + 1.0).abs() < 1e-6);
        assert!((measurement.power_w + 12.0).abs() < 1e-5);
    }

    #[test]
    fn incomplete_or_overflowed_conversion_is_rejected() {
        assert_eq!(
            Measurement::decode(0x5dc0, 0, 0, 0),
            Err(DecodeError::NotReady)
        );
        assert_eq!(
            Measurement::decode(0x5dc3, 0, 0, 0),
            Err(DecodeError::Overflow)
        );
    }

    #[test]
    fn out_of_range_voltage_and_shunt_are_rejected() {
        assert_eq!(
            Measurement::decode((6501 << 3) | 2, 0, 0, 0),
            Err(DecodeError::VoltageRange)
        );
        for shunt in [-8001, -8000, 8000, 8001] {
            assert_eq!(
                Measurement::decode(2, shunt, 0, 0),
                Err(DecodeError::CurrentRange)
            );
        }
    }

    #[test]
    fn telemetry_expires_and_future_timestamps_are_rejected() {
        let measurement = Measurement::decode(2, 0, 0, 100).unwrap();
        assert!(measurement.is_fresh(100 + STALE_AFTER_US));
        assert!(!measurement.is_fresh(101 + STALE_AFTER_US));
        assert!(!measurement.is_fresh(99));
    }
}
