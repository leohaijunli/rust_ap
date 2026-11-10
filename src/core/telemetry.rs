//! Link-local telemetry timing and command validation, without HAL dependencies.

use mavlink::{common::*, MessageData};

use crate::parameters::{Id, Snapshot};

pub const MAIN_COMPONENT: u8 = 1;
pub const MIN_INTERVAL_US: i32 = 20_000;
pub const HEALTH_MIN_INTERVAL_US: i32 = 500_000;

pub struct Entry {
    pub kind: Kind,
    pub message_id: u32,
    pub component: u8,
    pub rate: Id,
}

macro_rules! stream_table {
    ($($kind:ident: $data:ident, $component:expr, $rate:ident;)+) => {
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        pub enum Kind { $($kind,)+ AutopilotVersion, ComponentMetadata }

        pub const STREAMS: &[Entry] = &[
            $(Entry { kind: Kind::$kind, message_id: $data::ID,
                component: $component, rate: Id::$rate },)+
        ];
    };
}

stream_table! {
    Heartbeat: HEARTBEAT_DATA, MAIN_COMPONENT, MavRateHeartbeat;
    SystemStatus: SYS_STATUS_DATA, MAIN_COMPONENT, MavRateHeartbeat;
    SystemTime: SYSTEM_TIME_DATA, MAIN_COMPONENT, MavRateHeartbeat;
    StatusText: STATUSTEXT_DATA, MAIN_COMPONENT, MavRateStatus;
    HighresImu: HIGHRES_IMU_DATA, MAIN_COMPONENT, MavRateSensors;
    ScaledPressure: SCALED_PRESSURE_DATA, MAIN_COMPONENT, MavRateSensors;
    PowerDebug: DEBUG_VECT_DATA, MAIN_COMPONENT, MavRatePower;
    BatteryStatus: BATTERY_STATUS_DATA, MAIN_COMPONENT, MavRatePower;
    Gps: GPS_RAW_INT_DATA, MAIN_COMPONENT, MavRateGps;
    Attitude: ATTITUDE_DATA, MAIN_COMPONENT, MavRateEstimator;
    AttitudeQuaternion: ATTITUDE_QUATERNION_DATA, MAIN_COMPONENT, MavRateEstimator;
    LocalPosition: LOCAL_POSITION_NED_DATA, MAIN_COMPONENT, MavRateEstimator;
    GlobalPosition: GLOBAL_POSITION_INT_DATA, MAIN_COMPONENT, MavRateEstimator;
    Altitude: ALTITUDE_DATA, MAIN_COMPONENT, MavRateEstimator;
    EstimatorStatus: ESTIMATOR_STATUS_DATA, MAIN_COMPONENT, MavRateEstimator;
    Health: DEBUG_FLOAT_ARRAY_DATA, MAIN_COMPONENT, MavRateHealth;
}

pub const COUNT: usize = STREAMS.len();

pub fn message_id(value: f32) -> Option<u32> {
    (value.is_finite() && (0.0..=16_777_215.0).contains(&value) && value == (value as u32) as f32)
        .then_some(value as u32)
}

pub fn requested_kind(id: u32, component: u8) -> Option<Kind> {
    STREAMS
        .iter()
        .find(|entry| entry.message_id == id && matches_component(component, entry.component))
        .map(|entry| entry.kind)
        .or_else(|| {
            if !matches_component(component, MAIN_COMPONENT) {
                return None;
            }
            match id {
                AUTOPILOT_VERSION_DATA::ID => Some(Kind::AutopilotVersion),
                COMPONENT_METADATA_DATA::ID => Some(Kind::ComponentMetadata),
                _ => None,
            }
        })
}

fn matches_component(target: u8, owner: u8) -> bool {
    target == 0 || target == owner
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IntervalError {
    Invalid,
    Unsupported,
}

pub struct Scheduler {
    overrides: [i32; COUNT],
    periods: [i32; COUNT],
    due: [u64; COUNT],
    cursor: usize,
}

impl Scheduler {
    pub fn new(parameters: &Snapshot, now: u64) -> Self {
        let mut scheduler = Self {
            overrides: [0; COUNT],
            periods: [i32::MIN; COUNT],
            due: [now; COUNT],
            cursor: 0,
        };
        scheduler.refresh(parameters, now);
        scheduler
    }

    pub fn refresh(&mut self, parameters: &Snapshot, now: u64) {
        for (index, entry) in STREAMS.iter().enumerate() {
            let period = if self.overrides[index] == 0 {
                let rate = parameters.get(entry.rate);
                if rate <= 0.0 {
                    -1
                } else {
                    (1_000_000.0_f64 / f64::from(rate))
                        .clamp(f64::from(MIN_INTERVAL_US), f64::from(i32::MAX))
                        as i32
                }
            } else {
                self.overrides[index]
            };
            if self.periods[index] != period {
                self.due[index] = if period < 0 {
                    u64::MAX
                } else if self.periods[index] == i32::MIN {
                    now
                } else {
                    now.saturating_add(period as u64)
                };
                self.periods[index] = period;
            }
        }
    }

    /// Skip missed beats instead of bursting stale samples after a slow write.
    pub fn take_due(&mut self, now: u64) -> Option<Kind> {
        for offset in 0..COUNT {
            let index = (self.cursor + offset) % COUNT;
            if self.periods[index] > 0 && self.due[index] <= now {
                self.due[index] = now.saturating_add(self.periods[index] as u64);
                self.cursor = (index + 1) % COUNT;
                return Some(STREAMS[index].kind);
            }
        }
        None
    }

    pub fn set_interval(
        &mut self,
        id: u32,
        component: u8,
        interval: f32,
        parameters: &Snapshot,
        now: u64,
    ) -> Result<(), IntervalError> {
        // Reject non-finite/overflowing values and rates that would flood 115200-baud TELEM1.
        if !(interval == -1.0
            || interval == 0.0
            || (interval.is_finite()
                && f64::from(interval) <= f64::from(i32::MAX)
                && interval >= MIN_INTERVAL_US as f32))
            || (id == DEBUG_FLOAT_ARRAY_DATA::ID
                && interval > 0.0
                && interval < HEALTH_MIN_INTERVAL_US as f32)
        {
            return Err(IntervalError::Invalid);
        }
        let value = interval as i32;
        let mut found = false;
        for (index, entry) in STREAMS.iter().enumerate() {
            if entry.message_id == id && matches_component(component, entry.component) {
                self.overrides[index] = value;
                found = true;
            }
        }
        if !found {
            return Err(IntervalError::Unsupported);
        }
        self.refresh(parameters, now);
        Ok(())
    }

    pub fn interval(&self, id: u32, component: u8) -> Option<i32> {
        STREAMS
            .iter()
            .position(|entry| {
                entry.message_id == id && matches_component(component, entry.component)
            })
            .map(|index| self.periods[index])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stream_table_and_request_ids_are_bounded_and_component_owned() {
        for (index, entry) in STREAMS.iter().enumerate() {
            assert_eq!(
                requested_kind(entry.message_id, entry.component),
                Some(entry.kind)
            );
            assert!(!STREAMS[..index]
                .iter()
                .any(|other| other.component == entry.component
                    && other.message_id == entry.message_id));
            assert_eq!(
                entry.rate.definition().group,
                crate::parameters::Group::Link
            );
        }
        assert_eq!(requested_kind(230, 197), None);
        assert_eq!(requested_kind(30, 197), None);
        assert_eq!(requested_kind(148, 1), Some(Kind::AutopilotVersion));
        for value in [f32::NAN, f32::INFINITY, -1.0, 30.5, 16_777_216.0] {
            assert_eq!(message_id(value), None);
        }
        assert_eq!(message_id(30.0), Some(30));
        assert_eq!(requested_kind(350, 1), Some(Kind::Health));
    }

    #[test]
    fn intervals_disable_restore_and_do_not_leak_between_links_or_components() {
        let defaults = Snapshot::default();
        let mut usb = Scheduler::new(&defaults, 0);
        let uart = Scheduler::new(&defaults, 0);
        usb.set_interval(230, 1, -1.0, &defaults, 0).unwrap();
        assert_eq!(usb.interval(230, 1), Some(-1));
        assert_eq!(usb.interval(230, 197), None);
        assert_eq!(uart.interval(230, 1), Some(100_000));
        usb.set_interval(230, 0, 250_000.0, &defaults, 1).unwrap();
        assert_eq!(usb.interval(230, 1), Some(250_000));
        usb.set_interval(230, 0, 0.0, &defaults, 2).unwrap();
        assert_eq!(usb.interval(230, 1), Some(100_000));
        let changed = defaults.with(Id::MavRateEstimator, 2.0).unwrap();
        usb.refresh(&changed, 3);
        assert_eq!(usb.interval(230, 1), Some(500_000));
        for interval in [f32::NAN, f32::INFINITY, -2.0, 1.0, 2_147_483_648.0] {
            assert_eq!(
                usb.set_interval(30, 1, interval, &defaults, 0),
                Err(IntervalError::Invalid)
            );
        }
        assert_eq!(
            usb.set_interval(9999, 1, 100_000.0, &defaults, 0),
            Err(IntervalError::Unsupported)
        );
    }

    #[test]
    fn due_work_is_fair_bounded_and_does_not_catch_up_with_stale_bursts() {
        let mut scheduler = Scheduler::new(&Snapshot::default(), 0);
        for entry in STREAMS {
            assert_eq!(scheduler.take_due(0), Some(entry.kind));
        }
        assert_eq!(scheduler.take_due(0), None);
        for _ in 0..COUNT {
            assert!(scheduler.take_due(10_000_000).is_some());
        }
        assert_eq!(scheduler.take_due(10_000_000), None);
        scheduler.refresh(
            &Snapshot::default().with(Id::MavRateGps, 0.0).unwrap(),
            10_000_001,
        );
        assert_eq!(scheduler.interval(24, 1), Some(-1));
        let reset = Scheduler::new(&Snapshot::default(), 0);
        assert_eq!(reset.interval(24, 1), Some(200_000));
        assert_eq!(reset.interval(350, 1), Some(1_000_000));
        assert_eq!(reset.interval(DEBUG_VECT_DATA::ID, 1), Some(100_000));
        assert_eq!(
            scheduler.set_interval(350, 1, 20_000.0, &Snapshot::default(), 10_000_002),
            Err(IntervalError::Invalid)
        );
        scheduler
            .set_interval(350, 1, 500_000.0, &Snapshot::default(), 10_000_002)
            .unwrap();
        assert_eq!(scheduler.interval(350, 1), Some(500_000));
        scheduler
            .set_interval(350, 1, -1.0, &Snapshot::default(), 10_000_002)
            .unwrap();
        assert_eq!(scheduler.interval(350, 1), Some(-1));
        assert_eq!(scheduler.interval(DEBUG_VECT_DATA::ID, 1), Some(100_000));
    }
}
