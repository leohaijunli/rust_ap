//! Read-only health telemetry and observed counter rates; no control permission.

use mavlink::common::DEBUG_FLOAT_ARRAY_DATA;

use crate::{
    config::Algorithm,
    estimator::{
        report::Report,
        runtime::{FilterMode, Selection, Status, FRESH_US},
    },
    mavlink_vehicle::SensorState,
};

pub const SENSOR_COUNT: usize = 6;
pub const COUNTER_COUNT: usize = SENSOR_COUNT + 1;
pub const WINDOW_US: u64 = 1_000_000;
pub const CACHE_FRESH_US: u64 = 2_000_000;

#[derive(Clone, Copy, Default)]
pub struct Sensor {
    pub count: u32,
    pub timestamp: Option<u64>,
    pub state: SensorState,
}

#[derive(Clone, Copy)]
pub struct Estimator {
    pub report: Report,
    pub epoch: u32,
    pub status: Option<Status>,
    pub attitude_fresh: bool,
    pub solution_flags: Option<u16>,
    pub fault_flags: Option<u32>,
}

impl Estimator {
    fn state(self, now: u64) -> u8 {
        if !self.report.requested {
            return 0;
        }
        let fresh = self
            .status
            .is_some_and(|status| now >= status.timestamp && now - status.timestamp <= FRESH_US);
        if !self.report.active || !self.attitude_fresh || !fresh {
            return 1;
        }
        match (self.report.configured, self.report.selection) {
            (_, Selection::Invalid) => 1,
            (Algorithm::Ekf2, Selection::Filter) => 2,
            (Algorithm::Mahony, _) => 3,
            (Algorithm::Ekf2, Selection::Mahony) => 4,
        }
    }
}

#[derive(Clone, Copy)]
struct Counters {
    timestamp: u64,
    values: [u32; COUNTER_COUNT],
    epoch: u32,
}

pub struct Rates {
    pub window_us: u64,
    pub hz: [f32; COUNTER_COUNT],
}

#[derive(Default)]
pub struct RateMonitor {
    previous: Option<Counters>,
}

impl RateMonitor {
    pub fn due(&self, now: u64) -> bool {
        self.previous.is_none_or(|previous| {
            now < previous.timestamp || now - previous.timestamp >= WINDOW_US
        })
    }

    /// One sampler serves both links; rereading the cache never creates a rate.
    pub fn observe(&mut self, now: u64, values: [u32; COUNTER_COUNT], epoch: u32) -> Option<Rates> {
        if !self.due(now) {
            return None;
        }
        let previous = self.previous.replace(Counters {
            timestamp: now,
            values,
            epoch,
        });
        let mut rates = Rates {
            window_us: 0,
            hz: [f32::NAN; COUNTER_COUNT],
        };
        if let Some(previous) = previous {
            if let Some(elapsed) = now
                .checked_sub(previous.timestamp)
                .filter(|elapsed| *elapsed > 0 && *elapsed < u64::from(u32::MAX))
            {
                rates.window_us = elapsed;
                for (index, rate) in rates.hz.iter_mut().enumerate() {
                    let delta = values[index].wrapping_sub(previous.values[index]);
                    // A normal wrap is small; counter reset is not an enormous Hz.
                    if delta <= i32::MAX as u32 && (index != 0 || epoch == previous.epoch) {
                        *rate = delta as f32 / (elapsed as f32 * 1e-6);
                    }
                }
            }
        }
        Some(rates)
    }
}

/// AP_HEALTH v1 field indices are documented in docs/mavlink-health.md.
pub fn message(
    now: u64,
    rates: Rates,
    estimator: Estimator,
    sensors: [Sensor; SENSOR_COUNT],
    gps_fix: Option<u8>,
    selected_mag_healthy: bool,
    cpu_permille: Option<u16>,
) -> DEBUG_FLOAT_ARRAY_DATA {
    let mut data = [f32::NAN; 58];
    let state = estimator.state(now);
    let healthy = state == 2
        && estimator.status.is_some_and(|status| {
            status.mode == FilterMode::Running
                && status.fault.is_none()
                && status.numerics.is_none()
        })
        && estimator.fault_flags == Some(0);
    data[0] = 1.0;
    data[1] = if rates.window_us == 0 {
        f32::NAN
    } else {
        rates.window_us as f32 * 1e-6
    };
    data[2] = f32::from(state);
    data[3] = u8::from(healthy) as f32;
    data[4] = rates.hz[0];
    let mut present = 0_u8;
    let mut enabled = 0_u8;
    let mut sensor_health = 0_u8;
    for (index, sensor) in sensors.into_iter().enumerate() {
        let bit = 1 << index;
        if sensor.state.present {
            present |= bit;
            if sensor.state.enabled {
                enabled |= bit;
                if sensor.state.healthy && sensor.timestamp.is_some_and(|time| time <= now) {
                    sensor_health |= bit;
                }
            }
        }
        data[9 + index] = rates.hz[1 + index];
        data[15 + index] = sensor
            .timestamp
            .and_then(|time| now.checked_sub(time))
            .map_or(f32::NAN, |age| age as f32 / 1_000.0);
    }
    data[5] = f32::from(present);
    data[6] = f32::from(sensor_health);
    data[7] = f32::from(enabled);
    data[8] = rates.hz.iter().enumerate().fold(0_u8, |mask, (index, hz)| {
        mask | (u8::from(hz.is_finite()) << index)
    }) as f32;
    data[21] = estimator
        .status
        .map_or(f32::NAN, |s| s.max_update_us as f32);
    data[22] = estimator
        .status
        .map_or(f32::NAN, |s| s.deadline_misses as f32);
    data[23] = estimator.solution_flags.map_or(f32::NAN, f32::from);
    data[24] = estimator.fault_flags.map_or(f32::NAN, |flags| flags as f32);
    data[25] = gps_fix.map_or(f32::NAN, f32::from);
    data[26] = u8::from(selected_mag_healthy) as f32;
    data[27] = 0.0; // Data health never grants flight qualification.
    data[28] = cpu_permille.map_or(f32::NAN, |load| f32::from(load) / 10.0);
    DEBUG_FLOAT_ARRAY_DATA {
        time_usec: now,
        array_id: 1,
        name: "AP_HEALTH".into(),
        data,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::{common::MavMessage, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, Message};

    #[test]
    fn rates_use_elapsed_counters_and_do_not_resample_for_each_link() {
        let mut monitor = RateMonitor::default();
        let first = monitor.observe(1_000_000, [0; COUNTER_COUNT], 1).unwrap();
        assert!(first.hz.iter().all(|value| value.is_nan()));
        assert!(monitor.observe(1_999_999, [9; COUNTER_COUNT], 1).is_none());
        let rates = monitor
            .observe(3_000_000, [200, 16000, 16000, 40, 38, 10, 0], 1)
            .unwrap();
        assert_eq!(rates.window_us, 2_000_000);
        assert_eq!(rates.hz, [100.0, 8000.0, 8000.0, 20.0, 19.0, 5.0, 0.0]);
    }

    #[test]
    fn counter_wrap_reset_epoch_and_bad_time_are_not_fake_rates() {
        let mut monitor = RateMonitor::default();
        monitor.observe(2_000_000, [u32::MAX - 4; COUNTER_COUNT], 1);
        let wrapped = monitor.observe(3_000_000, [2; COUNTER_COUNT], 1).unwrap();
        assert_eq!(wrapped.hz, [7.0; COUNTER_COUNT]);
        let epoch = monitor.observe(4_000_000, [9; COUNTER_COUNT], 2).unwrap();
        assert!(epoch.hz[0].is_nan());
        assert_eq!(&epoch.hz[1..], &[7.0; SENSOR_COUNT]);
        let reset = monitor.observe(5_000_000, [0; COUNTER_COUNT], 2).unwrap();
        assert!(reset.hz.iter().all(|value| value.is_nan()));
        let reversed = monitor.observe(1, [5; COUNTER_COUNT], 2).unwrap();
        assert_eq!(reversed.window_us, 0);
        assert!(reversed.hz.iter().all(|value| value.is_nan()));
    }

    fn estimator() -> Estimator {
        Estimator {
            report: Report {
                requested: true,
                active: true,
                configured: Algorithm::Ekf2,
                selection: Selection::Filter,
            },
            epoch: 1,
            status: Some(Status {
                timestamp: 1_000_000,
                generation: 1,
                samples: 1000,
                updates: 100,
                measurements: 20,
                rejected_measurements: 0,
                selection: Selection::Filter,
                mode: FilterMode::Running,
                fault: None,
                last_fault: None,
                numerics: None,
                numerics_stage: None,
                numerics_pair: [f32::NAN; 2],
                max_update_us: 1500,
                deadline_misses: 0,
                recoveries: 0,
            }),
            attitude_fresh: true,
            solution_flags: Some(1),
            fault_flags: Some(0),
        }
    }

    #[test]
    fn health_distinguishes_ekf_fallback_stale_disabled_and_unavailable_sensors() {
        let sensors = [Sensor {
            count: 10,
            timestamp: Some(999_000),
            state: SensorState {
                present: true,
                enabled: true,
                healthy: true,
            },
        }; SENSOR_COUNT];
        let make = |estimator, sensors| {
            message(
                1_000_000,
                Rates {
                    window_us: 1_000_000,
                    hz: [10.0; COUNTER_COUNT],
                },
                estimator,
                sensors,
                Some(4),
                true,
                Some(350),
            )
        };
        let good = make(estimator(), sensors);
        assert_eq!(&good.data[2..8], &[2.0, 1.0, 10.0, 63.0, 63.0, 63.0]);
        assert_eq!(good.data[27], 0.0);
        assert_eq!(good.data[28], 35.0);
        let mut degraded = estimator();
        degraded.report.selection = Selection::Mahony;
        assert_eq!(&make(degraded, sensors).data[2..4], &[4.0, 0.0]);
        degraded.report.configured = Algorithm::Mahony;
        assert_eq!(&make(degraded, sensors).data[2..4], &[3.0, 0.0]);
        degraded = estimator();
        degraded.fault_flags = Some(1);
        assert_eq!(make(degraded, sensors).data[3], 0.0);
        degraded = estimator();
        degraded.status.as_mut().unwrap().timestamp = 0;
        assert_eq!(&make(degraded, sensors).data[2..4], &[1.0, 0.0]);
        degraded = estimator();
        degraded.report.requested = false;
        assert_eq!(&make(degraded, sensors).data[2..4], &[0.0, 0.0]);
        let mut unavailable = sensors;
        unavailable[0].timestamp = Some(1_000_001);
        unavailable[1] = Sensor::default();
        unavailable[2].state.healthy = false;
        let failed = make(estimator(), unavailable);
        assert_eq!(failed.data[6], 56.0);
        assert!(failed.data[15].is_nan() && failed.data[16].is_nan());
        unavailable[2].state.enabled = false;
        let disabled = make(estimator(), unavailable);
        assert_eq!(disabled.data[7], 57.0);
        assert_eq!(disabled.data[6], 56.0);
    }

    #[test]
    fn debug_array_roundtrips_without_using_the_ina219_debug_vect_id() {
        let data = message(
            1_000_000,
            Rates {
                window_us: 0,
                hz: [f32::NAN; COUNTER_COUNT],
            },
            estimator(),
            [Sensor::default(); SENSOR_COUNT],
            None,
            false,
            None,
        );
        let message = MavMessage::DEBUG_FLOAT_ARRAY(data);
        assert_eq!(message.message_id(), 350);
        let header = MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        };
        let mut raw = MAVLinkV2MessageRaw::new();
        raw.serialize_message(header, &message);
        let decoded =
            MavMessage::parse(MavlinkVersion::V2, raw.message_id(), raw.payload()).unwrap();
        let MavMessage::DEBUG_FLOAT_ARRAY(decoded) = decoded else {
            panic!("wrong message");
        };
        assert_eq!(decoded.name.to_str().unwrap(), "AP_HEALTH");
        assert_eq!(decoded.array_id, 1);
        assert_eq!(decoded.time_usec, 1_000_000);
        assert!(decoded.data[4].is_nan());
        assert!(decoded.data[29..].iter().all(|value| value.is_nan()));
    }
}
