//! INA219 branch measurements mapped to BATTERY_STATUS without invented capacity.

use crate::ina219::Measurement;
use mavlink::common::*;

pub fn battery_status(value: Measurement, now: u64) -> Option<MavMessage> {
    if !value.is_fresh(now)
        || !value.bus_voltage_v.is_finite()
        || value.bus_voltage_v < 0.0
        || !value.current_a.is_finite()
    {
        return None;
    }
    let mut voltages = [u16::MAX; 10];
    voltages[0] = (value.bus_voltage_v * 1_000.0).clamp(0.0, (u16::MAX - 1) as f32) as u16;
    Some(MavMessage::BATTERY_STATUS(BATTERY_STATUS_DATA {
        // The monitor is on an independent branch, not necessarily the propulsion battery.
        battery_function: MavBatteryFunction::MAV_BATTERY_FUNCTION_UNKNOWN,
        mavtype: MavBatteryType::MAV_BATTERY_TYPE_UNKNOWN,
        temperature: i16::MAX,
        voltages,
        current_battery: (value.current_a * 100.0).clamp(i16::MIN as f32, i16::MAX as f32) as i16,
        current_consumed: -1,
        energy_consumed: -1,
        battery_remaining: -1,
        charge_state: MavBatteryChargeState::MAV_BATTERY_CHARGE_STATE_UNDEFINED,
        ..Default::default()
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn battery_units_unknown_fields_and_staleness_are_explicit() {
        let mut value = Measurement {
            timestamp_us: 100,
            bus_voltage_v: 12.5,
            shunt_voltage_v: 0.0,
            current_a: -1.25,
            power_w: -15.625,
        };
        let Some(MavMessage::BATTERY_STATUS(data)) = battery_status(value, 100) else {
            panic!()
        };
        assert_eq!(data.voltages[0], 12_500);
        assert!(data.voltages[1..]
            .iter()
            .all(|&voltage| voltage == u16::MAX));
        assert_eq!(data.current_battery, -125);
        assert_eq!(
            (
                data.current_consumed,
                data.energy_consumed,
                data.battery_remaining,
                data.temperature
            ),
            (-1, -1, -1, i16::MAX)
        );
        assert!(battery_status(value, 99).is_none());
        assert!(battery_status(value, 100 + crate::ina219::STALE_AFTER_US + 1).is_none());
        value.bus_voltage_v = f32::NAN;
        assert!(battery_status(value, 100).is_none());
    }
}
