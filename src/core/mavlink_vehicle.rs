//! Transport-free vehicle identity and observed sensor status; no arming policy.

use mavlink::common::*;

use crate::{bmm150, bmp388, gps, icm42688, px4_messages::SensorGps};

pub const CAPABILITIES: MavProtocolCapability =
    MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2
        .union(MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE);

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SensorState {
    pub present: bool,
    pub enabled: bool,
    pub healthy: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Sensors {
    pub gyro: SensorState,
    pub accel: SensorState,
    pub barometer: SensorState,
    pub magnetometer: SensorState,
    pub gps: SensorState,
    pub ahrs: SensorState,
}

pub fn heartbeat(armed: bool) -> MavMessage {
    let mut base_mode = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if armed {
        base_mode |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
    }
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        // PX4 MANUAL=1/sub_mode=0 is compatibility reporting, not a mode action.
        custom_mode: 1 << 16,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        // Retain QGC's PX4 `.ulg` download path without advertising general FTP.
        autopilot: MavAutopilot::MAV_AUTOPILOT_PX4,
        base_mode,
        system_status: MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    })
}

pub fn system_status(sensors: Sensors, load: Option<u16>) -> MavMessage {
    let mut status = SYS_STATUS_DATA {
        // Generated bitmask defaults select their first flag, not an empty mask.
        onboard_control_sensors_present: MavSysStatusSensor::empty(),
        onboard_control_sensors_enabled: MavSysStatusSensor::empty(),
        onboard_control_sensors_health: MavSysStatusSensor::empty(),
        onboard_control_sensors_present_extended: MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_enabled_extended: MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_health_extended: MavSysStatusSensorExtended::empty(),
        // Preserve the existing CPU fallback; sensor health never depends on it.
        load: load.unwrap_or(0),
        voltage_battery: u16::MAX,
        current_battery: -1,
        battery_remaining: -1,
        ..Default::default()
    };
    for (flag, sensor) in [
        (
            MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO,
            sensors.gyro,
        ),
        (
            MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL,
            sensors.accel,
        ),
        (
            MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE,
            sensors.barometer,
        ),
        (
            MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_MAG,
            sensors.magnetometer,
        ),
        (MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_GPS, sensors.gps),
        (MavSysStatusSensor::MAV_SYS_STATUS_AHRS, sensors.ahrs),
    ] {
        if !sensor.present {
            continue;
        }
        status.onboard_control_sensors_present |= flag;
        if sensor.enabled {
            status.onboard_control_sensors_enabled |= flag;
            if sensor.healthy {
                status.onboard_control_sensors_health |= flag;
            }
        }
    }
    MavMessage::SYS_STATUS(status)
}

/// Gyro/accel share identification and sample time, but retain separate validity.
pub fn imu_health(
    who_am_i: u8,
    initialized: bool,
    sample: Option<icm42688::Sample>,
    now: u64,
) -> (SensorState, SensorState) {
    let present = who_am_i == icm42688::WHO_AM_I_VALUE;
    let state = SensorState {
        present,
        enabled: present,
        healthy: false,
    };
    let Some(sample) = sample.filter(|sample| {
        present
            && initialized
            && now >= sample.sample_timestamp_us
            && !sample.is_stale(now, icm42688::Config::DEFAULT)
            && sample.temperature_c.is_finite()
    }) else {
        return (state, state);
    };
    (
        SensorState {
            healthy: sample.gyro_rad_s.iter().all(|v| v.is_finite()),
            ..state
        },
        SensorState {
            healthy: sample.accel_m_s2.iter().all(|v| v.is_finite()),
            ..state
        },
    )
}

/// A known device remains present during recovery; only accepted samples count.
pub fn barometer_health(chip_id: u8, sample: Option<bmp388::Sample>, now: u64) -> SensorState {
    let present = chip_id == bmp388::CHIP_ID;
    SensorState {
        present,
        enabled: present,
        healthy: present
            && sample.is_some_and(|sample| {
                sample.fresh(now)
                    && sample.pressure_pa.is_finite()
                    && sample.pressure_pa > 0.0
                    && sample.temperature_c.is_finite()
            }),
    }
}

/// Receiver/transport health, not a fix, aiding or navigation qualification gate.
pub fn gps_health(sample: Option<SensorGps>, now: u64) -> SensorState {
    SensorState {
        present: sample.is_some(),
        enabled: sample.is_some(),
        healthy: sample.is_some_and(|sample| {
            now >= sample.timestamp
                && now - sample.timestamp <= gps::GPS_FRESH_US
                && (gps::FixType::NoFix as u8..=gps::FixType::Ppp as u8).contains(&sample.fix_type)
                && sample.system_error == 0
        }),
    }
}

/// Known device presence is separate from a fresh measurement or EKF fusion.
pub fn magnetometer_health(chip_id: u8, sample: Option<bmm150::Sample>, now: u64) -> SensorState {
    let present = chip_id == bmm150::CHIP_ID;
    SensorState {
        present,
        enabled: present,
        healthy: present && sample.is_some_and(|sample| sample.fresh(now)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::{MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, Message};

    const NOW: u64 = 1_000_000;
    const HEALTHY: SensorState = SensorState {
        present: true,
        enabled: true,
        healthy: true,
    };

    fn status(sensors: Sensors) -> SYS_STATUS_DATA {
        let MavMessage::SYS_STATUS(status) = system_status(sensors, Some(291)) else {
            unreachable!();
        };
        status
    }

    #[test]
    fn quadrotor_identity_keeps_disarmed_px4_compatibility_and_limited_capabilities() {
        let MavMessage::HEARTBEAT(data) = heartbeat(false) else {
            unreachable!();
        };
        assert_eq!(data.mavtype, MavType::MAV_TYPE_QUADROTOR);
        assert_eq!(data.autopilot, MavAutopilot::MAV_AUTOPILOT_PX4);
        assert_eq!(
            data.base_mode,
            MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        );
        assert_eq!(data.custom_mode, 1 << 16);
        assert_eq!(data.system_status, MavState::MAV_STATE_STANDBY);
        assert_eq!(data.mavlink_version, 3);
        assert_eq!(
            CAPABILITIES,
            MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2
                | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE
        );
    }

    #[test]
    fn status_only_advertises_observed_sensors_and_keeps_battery_unknown() {
        let data = status(Sensors {
            gyro: HEALTHY,
            accel: HEALTHY,
            barometer: HEALTHY,
            magnetometer: HEALTHY,
            gps: HEALTHY,
            ahrs: HEALTHY,
        });
        assert_eq!(data.onboard_control_sensors_present.bits(), 0x0020_002f);
        assert_eq!(
            data.onboard_control_sensors_enabled,
            data.onboard_control_sensors_present
        );
        assert_eq!(
            data.onboard_control_sensors_health,
            data.onboard_control_sensors_enabled
        );
        assert_eq!(data.onboard_control_sensors_present_extended.bits(), 0);
        assert_eq!(data.onboard_control_sensors_enabled_extended.bits(), 0);
        assert_eq!(data.onboard_control_sensors_health_extended.bits(), 0);
        assert_eq!(data.load, 291);
        assert_eq!(
            (
                data.voltage_battery,
                data.current_battery,
                data.battery_remaining
            ),
            (u16::MAX, -1, -1)
        );
    }

    #[test]
    fn missing_disabled_or_failed_sources_cannot_claim_health() {
        let data = status(Sensors {
            gyro: SensorState {
                healthy: false,
                ..HEALTHY
            },
            accel: SensorState {
                present: false,
                ..HEALTHY
            },
            ahrs: SensorState {
                enabled: false,
                ..HEALTHY
            },
            ..Sensors::default()
        });
        assert_eq!(data.onboard_control_sensors_present.bits(), 0x0020_0001);
        assert_eq!(data.onboard_control_sensors_enabled.bits(), 1);
        assert_eq!(data.onboard_control_sensors_health.bits(), 0);
        assert_eq!(
            status(Sensors::default())
                .onboard_control_sensors_present
                .bits(),
            0
        );
    }

    #[test]
    fn imu_health_checks_identity_initialization_time_and_each_axis_group() {
        let mut sample = icm42688::Sample {
            sample_timestamp_us: NOW,
            ..icm42688::Sample::default()
        };
        let evaluate = |sample, now| imu_health(icm42688::WHO_AM_I_VALUE, true, sample, now);
        assert_eq!(evaluate(Some(sample), NOW), (HEALTHY, HEALTHY));
        let boundary = NOW + u64::from(icm42688::Config::DEFAULT.stale_after_us);
        assert_eq!(evaluate(Some(sample), boundary), (HEALTHY, HEALTHY));
        for (sample, now) in [
            (Some(sample), boundary + 1),
            (Some(sample), NOW - 1),
            (None, NOW),
        ] {
            let (gyro, accel) = evaluate(sample, now);
            assert!(gyro.present && accel.present);
            assert!(!gyro.healthy && !accel.healthy);
        }
        assert!(!imu_health(0, true, Some(sample), NOW).0.present);
        assert!(
            !imu_health(icm42688::WHO_AM_I_VALUE, false, Some(sample), NOW)
                .0
                .healthy
        );
        sample.gyro_rad_s[1] = f32::NAN;
        let (gyro, accel) = evaluate(Some(sample), NOW);
        assert!(!gyro.healthy && accel.healthy);
        sample.gyro_rad_s[1] = 0.0;
        sample.accel_m_s2[0] = f32::INFINITY;
        let (gyro, accel) = evaluate(Some(sample), NOW);
        assert!(gyro.healthy && !accel.healthy);
        sample.accel_m_s2[0] = 0.0;
        sample.temperature_c = f32::NAN;
        let (gyro, accel) = evaluate(Some(sample), NOW);
        assert!(!gyro.healthy && !accel.healthy);
    }

    #[test]
    fn barometer_health_rejects_missing_stale_future_and_nonfinite_samples() {
        let mut sample = bmp388::Sample {
            timestamp_sample: NOW,
            sequence: 1,
            pressure_pa: 101_325.0,
            temperature_c: 25.0,
            error_count: 0,
        };
        let evaluate = |sample, now| barometer_health(bmp388::CHIP_ID, sample, now);
        assert_eq!(evaluate(Some(sample), NOW + bmp388::FRESH_US), HEALTHY);
        for (sample, now) in [
            (Some(sample), NOW + bmp388::FRESH_US + 1),
            (Some(sample), NOW - 1),
            (None, NOW),
        ] {
            let state = evaluate(sample, now);
            assert!(state.present && state.enabled && !state.healthy);
        }
        assert!(!barometer_health(0, Some(sample), NOW).present);
        for pressure in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            sample.pressure_pa = pressure;
            assert!(!evaluate(Some(sample), NOW).healthy);
        }
        sample.pressure_pa = 101_325.0;
        sample.temperature_c = f32::NAN;
        assert!(!evaluate(Some(sample), NOW).healthy);
    }

    #[test]
    fn gps_transport_health_is_distinct_from_fix_and_expires_without_new_data() {
        // Use the bridge's explicit unknown fields, not a zero-default GPS DTO.
        let fix = crate::dronecan::gnss::GnssFix2 {
            timestamp_usec: NOW,
            gnss_timestamp_usec: 0,
            gnss_time_standard: 0,
            num_leap_seconds: 0,
            longitude_deg_1e8: 0,
            latitude_deg_1e8: 0,
            height_ellipsoid_mm: 0,
            height_msl_mm: 0,
            ned_velocity: [f32::NAN; 3],
            sats_used: 0,
            status: 0,
            mode: 0,
            sub_mode: 0,
            covariance: [f32::NAN; 6],
            covariance_len: 0,
            pdop: f32::NAN,
        };
        let mut sample = gps::process_fix2(&fix, 25, NOW, &gps::AuxiliaryCache::default());
        assert_eq!(sample.fix_type, gps::FixType::NoFix as u8);
        assert_eq!(gps_health(Some(sample), NOW + gps::GPS_FRESH_US), HEALTHY);
        assert_eq!(gps_health(None, NOW), SensorState::default());
        for now in [NOW - 1, NOW + gps::GPS_FRESH_US + 1] {
            let state = gps_health(Some(sample), now);
            assert!(state.present && state.enabled && !state.healthy);
        }
        sample.system_error = 1;
        assert!(!gps_health(Some(sample), NOW).healthy);
        sample.system_error = 0;
        sample.fix_type = gps::FixType::NoGps as u8;
        assert!(!gps_health(Some(sample), NOW).healthy);
    }

    #[test]
    fn magnetometer_health_needs_identity_and_fresh_finite_nonzero_data() {
        let sample = bmm150::Sample {
            timestamp_sample: NOW,
            sequence: 1,
            field_gauss: [0.22, 0.0, 0.43],
            error_count: 0,
        };
        assert_eq!(
            magnetometer_health(bmm150::CHIP_ID, Some(sample), NOW + bmm150::FRESH_US),
            HEALTHY
        );
        assert!(!magnetometer_health(0, Some(sample), NOW).present);
        for (sample, now) in [
            (None, NOW),
            (Some(sample), NOW - 1),
            (Some(sample), NOW + bmm150::FRESH_US + 1),
        ] {
            let state = magnetometer_health(bmm150::CHIP_ID, sample, now);
            assert!(state.present && state.enabled && !state.healthy);
        }
        let invalid = bmm150::Sample {
            field_gauss: [f32::NAN, 0.0, 0.43],
            ..sample
        };
        assert!(!magnetometer_health(bmm150::CHIP_ID, Some(invalid), NOW).healthy);
    }

    #[test]
    fn identity_and_sensor_masks_survive_mavlink2_encoding() {
        for message in [
            heartbeat(false),
            system_status(
                Sensors {
                    gyro: HEALTHY,
                    ..Sensors::default()
                },
                None,
            ),
        ] {
            let mut frame = MAVLinkV2MessageRaw::new();
            frame.serialize_message(
                MavHeader {
                    system_id: 1,
                    component_id: 1,
                    sequence: 0,
                },
                &message,
            );
            assert!(frame.has_valid_crc::<MavMessage>());
            let decoded =
                MavMessage::parse(MavlinkVersion::V2, frame.message_id(), frame.payload()).unwrap();
            assert_eq!(decoded, message);
        }
    }
}
