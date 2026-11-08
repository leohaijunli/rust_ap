//! Runtime per-source logging policy: enable bit and decimated rate (Hz).
//!
//! Values come from the P3a parameter registry (`LOG_EN_*` / `LOG_*_HZ`) and are
//! applied on the next sample. Sources share a `CriticalSectionRawMutex` rate
//! limiter so the interrupt CAN executor and the thread executor agree.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use crate::parameters::Id;

/// Logging groups that map the logged topics to the enable/rate parameters.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Group {
    Imu,
    Current,
    Can,
    Health,
    Event,
    Estimator,
    Barometer,
    Magnetometer,
}

pub const GROUP_COUNT: usize = 8;

impl Group {
    const fn enable(self) -> Id {
        match self {
            Group::Imu => Id::LogEnImu,
            Group::Current => Id::LogEnIna,
            Group::Can => Id::LogEnCan,
            Group::Health => Id::LogEnHealth,
            Group::Event => Id::LogEnEvent,
            Group::Estimator => Id::LogEnEst,
            Group::Barometer => Id::LogEnBaro,
            Group::Magnetometer => Id::LogEnMag,
        }
    }

    const fn rate(self) -> Option<Id> {
        match self {
            Group::Imu => Some(Id::LogImuHz),
            Group::Current => Some(Id::LogInaHz),
            Group::Can => Some(Id::LogCanHz),
            Group::Health => Some(Id::LogHealthHz),
            Group::Event => None,
            Group::Estimator => Some(Id::LogEstHz),
            Group::Barometer => Some(Id::LogBaroHz),
            Group::Magnetometer => Some(Id::LogMagHz),
        }
    }
}

struct State {
    enabled: [AtomicU8; GROUP_COUNT],
    period_us: [AtomicU32; GROUP_COUNT],
    // Rate-limit timestamps are read/written by both the thread executor and
    // the interrupt CAN executor, so they live behind a critical section.
    last_us: critical_section::Mutex<RefCell<[u64; GROUP_COUNT]>>,
    mag_last_us: critical_section::Mutex<RefCell<[u64; 5]>>,
}

impl State {
    const fn new() -> Self {
        Self {
            enabled: [const { AtomicU8::new(1) }; GROUP_COUNT],
            period_us: [const { AtomicU32::new(0) }; GROUP_COUNT],
            last_us: critical_section::Mutex::new(RefCell::new([0; GROUP_COUNT])),
            mag_last_us: critical_section::Mutex::new(RefCell::new([0; 5])),
        }
    }
}

static STATE: State = State::new();

/// Reset a group to its parameter default and clear its limiter.
fn refresh_default(group: Group) {
    let enabled = group.enable().definition().default != 0.0;
    STATE.enabled[group as usize].store(u8::from(enabled), Ordering::Relaxed);
    let period = group
        .rate()
        .map_or(0, |id| rate_period_us(id.definition().default));
    STATE.period_us[group as usize].store(period, Ordering::Relaxed);
    clear_last(group);
}

const fn rate_period_us(rate_hz: f32) -> u32 {
    (1_000_000.0 / rate_hz) as u32
}

fn clear_last(group: Group) {
    critical_section::with(|section| {
        let mut lasts = STATE.last_us.borrow(section).borrow_mut();
        lasts[group as usize] = 0;
        if group == Group::Magnetometer {
            *STATE.mag_last_us.borrow(section).borrow_mut() = [0; 5];
        }
    });
}

/// Apply a registry value (the parameter's current RAM value) to its group.
/// The caller passes the actual value so the setting takes effect immediately.
pub fn apply(id: Id, value: f32) {
    let Some(group) = group_for(id) else {
        return;
    };
    if group.enable() == id {
        STATE.enabled[group as usize].store(u8::from(value != 0.0), Ordering::Relaxed);
    } else if group.rate() == Some(id) {
        let period = rate_period_us(value).max(1);
        STATE.period_us[group as usize].store(period, Ordering::Relaxed);
    }
    clear_last(group);
}

/// Apply every group default. Called once at startup before parameters load.
pub fn refresh_all() {
    for group in [
        Group::Imu,
        Group::Current,
        Group::Can,
        Group::Health,
        Group::Event,
        Group::Estimator,
        Group::Barometer,
        Group::Magnetometer,
    ] {
        refresh_default(group);
    }
}

fn group_for(id: Id) -> Option<Group> {
    [
        Group::Imu,
        Group::Current,
        Group::Can,
        Group::Health,
        Group::Event,
        Group::Estimator,
        Group::Barometer,
        Group::Magnetometer,
    ]
    .into_iter()
    .find(|group| group.enable() == id || group.rate() == Some(id))
}

/// Whether a source is enabled AND its rate has elapsed since the last record.
pub fn rate(group: Group, timestamp_us: u64) -> bool {
    if STATE.enabled[group as usize].load(Ordering::Relaxed) == 0 {
        return false;
    }
    let period = STATE.period_us[group as usize].load(Ordering::Relaxed);
    critical_section::with(|section| {
        let mut lasts = STATE.last_us.borrow(section).borrow_mut();
        let last = lasts[group as usize];
        if period != 0 && timestamp_us.wrapping_sub(last) < u64::from(period) {
            return false;
        }
        lasts[group as usize] = timestamp_us;
        true
    })
}

/// Reject stale/future barometer samples before consuming a logging slot.
pub fn baro_rate(sample: crate::bmp388::Sample, now_us: u64) -> bool {
    sample.fresh(now_us) && rate(Group::Barometer, sample.timestamp_sample)
}

/// Whether an event source is enabled (no rate limiting).
pub fn enabled(group: Group) -> bool {
    STATE.enabled[group as usize].load(Ordering::Relaxed) != 0
}

/// Four raw magnetic instances and the selected stream each need a limiter:
/// a shared group timestamp would let one sensor starve all the other streams.
pub fn mag_rate(instance: usize, timestamp_us: u64) -> bool {
    if instance >= 5 || !enabled(Group::Magnetometer) {
        return false;
    }
    let period = STATE.period_us[Group::Magnetometer as usize].load(Ordering::Relaxed);
    critical_section::with(|section| {
        let mut last = STATE.mag_last_us.borrow(section).borrow_mut();
        if timestamp_us < last[instance] || timestamp_us - last[instance] < u64::from(period) {
            return false;
        }
        last[instance] = timestamp_us;
        true
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parameters::REGISTRY;

    #[test]
    fn registry_groups_cover_the_logged_sources() {
        assert_eq!(
            REGISTRY
                .iter()
                .filter(|entry| entry.name.starts_with("LOG_"))
                .count(),
            15
        );
        for group in [
            Group::Imu,
            Group::Current,
            Group::Can,
            Group::Health,
            Group::Event,
            Group::Estimator,
            Group::Barometer,
            Group::Magnetometer,
        ] {
            assert!(REGISTRY.iter().any(|entry| entry.id == group.enable()));
            if let Some(rate) = group.rate() {
                assert!(REGISTRY.iter().any(|entry| entry.id == rate));
            }
        }
        assert_eq!(rate_period_us(200.0), 5_000);
        assert_eq!(rate_period_us(10.0), 100_000);
    }

    #[test]
    fn apply_takes_effect_on_enable_and_rate() {
        refresh_all();
        // Defaults: IMU enabled at 200 Hz -> 5000 us period, last starts at 0.
        assert!(!rate(Group::Imu, 1_000));
        assert!(rate(Group::Imu, 5_000));
        assert!(!rate(Group::Imu, 5_100));
        assert!(rate(Group::Imu, 10_000));

        // Disabling rejects every sample.
        apply(Id::LogEnImu, 0.0);
        assert!(!rate(Group::Imu, 2_000_000));

        // Re-enabling and lowering the rate: the first sample after apply logs,
        // then the next is rejected until the longer 50 Hz period (20 ms) elapses.
        apply(Id::LogEnImu, 1.0);
        apply(Id::LogImuHz, 50.0);
        assert!(rate(Group::Imu, 3_000_000));
        assert!(!rate(Group::Imu, 3_000_000 + 10_000));
        assert!(rate(Group::Imu, 3_000_000 + 20_000));

        // Event groups only honor enable.
        apply(Id::LogEnEvent, 0.0);
        assert!(!enabled(Group::Event));
        apply(Id::LogEnEvent, 1.0);
        assert!(enabled(Group::Event));

        // At 1 Hz, a stale sample can steal the next fresh sample's slot.
        apply(Id::LogBaroHz, 1.0);
        let baro = crate::bmp388::Sample {
            timestamp_sample: 1_000_000,
            sequence: 0,
            pressure_pa: 101_325.0,
            temperature_c: 25.0,
            error_count: 0,
        };
        // Rejected stale/future samples must not consume a logging slot.
        assert!(!baro_rate(
            baro,
            baro.timestamp_sample + crate::bmp388::FRESH_US + 1
        ));
        let fresh = crate::bmp388::Sample {
            timestamp_sample: baro.timestamp_sample + crate::bmp388::FRESH_US + 2,
            ..baro
        };
        assert!(baro_rate(fresh, fresh.timestamp_sample));
        let early = crate::bmp388::Sample {
            timestamp_sample: fresh.timestamp_sample + 999_999,
            ..baro
        };
        assert!(!baro_rate(early, early.timestamp_sample));
        let next = crate::bmp388::Sample {
            timestamp_sample: fresh.timestamp_sample + 1_000_000,
            ..baro
        };
        assert!(baro_rate(next, next.timestamp_sample));
        let future = crate::bmp388::Sample {
            timestamp_sample: next.timestamp_sample + 1_000_000,
            ..baro
        };
        assert!(!baro_rate(future, future.timestamp_sample - 1));
        assert!(baro_rate(future, future.timestamp_sample));
        assert!(!baro_rate(future, future.timestamp_sample + 1));
        let boundary = crate::bmp388::Sample {
            timestamp_sample: future.timestamp_sample + 1_000_000,
            ..baro
        };
        assert!(baro_rate(
            boundary,
            boundary.timestamp_sample + crate::bmp388::FRESH_US
        ));
        apply(Id::LogBaroHz, Id::LogBaroHz.definition().default);

        // Raw magnetic instances and the selected stream cannot starve each other.
        apply(Id::LogEnMag, 1.0);
        apply(Id::LogMagHz, 20.0);
        for instance in 0..5 {
            assert!(mag_rate(instance, 5_000_000));
            assert!(!mag_rate(instance, 5_001_000));
        }
        assert!(!mag_rate(5, 5_000_000));
    }
}
