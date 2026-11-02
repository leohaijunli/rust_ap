//! Runtime per-source logging policy: enable bit and decimated rate (Hz).
//!
//! Values come from the P3a parameter registry (`LOG_EN_*` / `LOG_*_HZ`) and are
//! applied on the next sample. Sources share a `CriticalSectionRawMutex` rate
//! limiter so the interrupt CAN executor and the thread executor agree.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use crate::parameters::{Id, REGISTRY};

/// Logging groups that map the logged topics to the enable/rate parameters.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Group {
    Imu,
    Current,
    Can,
    Health,
    Event,
}

pub const GROUP_COUNT: usize = 5;

impl Group {
    const fn enable(self) -> Id {
        match self {
            Group::Imu => Id::LogEnImu,
            Group::Current => Id::LogEnIna,
            Group::Can => Id::LogEnCan,
            Group::Health => Id::LogEnHealth,
            Group::Event => Id::LogEnEvent,
        }
    }

    const fn rate(self) -> Option<Id> {
        match self {
            Group::Imu => Some(Id::LogImuHz),
            Group::Current => Some(Id::LogInaHz),
            Group::Can => Some(Id::LogCanHz),
            Group::Health => Some(Id::LogHealthHz),
            Group::Event => None,
        }
    }
}

struct State {
    enabled: [AtomicU8; GROUP_COUNT],
    period_us: [AtomicU32; GROUP_COUNT],
    // Rate-limit timestamps are read/written by both the thread executor and
    // the interrupt CAN executor, so they live behind a critical section.
    last_us: critical_section::Mutex<RefCell<[u64; GROUP_COUNT]>>,
}

impl State {
    const fn new() -> Self {
        Self {
            enabled: [const { AtomicU8::new(1) }; GROUP_COUNT],
            period_us: [const { AtomicU32::new(0) }; GROUP_COUNT],
            last_us: critical_section::Mutex::new(RefCell::new([0; GROUP_COUNT])),
        }
    }
}

static STATE: State = State::new();

/// Reload a group from the registry (defaults) and clear its limiter.
fn refresh(group: Group) {
    let enabled = definition_value(group.enable()) != 0.0;
    STATE.enabled[group as usize].store(u8::from(enabled), Ordering::Relaxed);
    let period = group
        .rate()
        .map_or(0, |id| rate_period_us(definition_value(id)));
    STATE.period_us[group as usize].store(period, Ordering::Relaxed);
    critical_section::with(|section| {
        *STATE.last_us.borrow(section).borrow_mut() = [0; GROUP_COUNT];
    });
}

fn definition_value(id: Id) -> f32 {
    REGISTRY
        .iter()
        .find(|entry| entry.id == id)
        .map_or(id.definition().default, |entry| entry.default)
}

const fn rate_period_us(rate_hz: f32) -> u32 {
    (1_000_000.0 / rate_hz) as u32
}

/// Apply a registry change; no-op for unrelated parameters.
pub fn apply(id: Id) {
    if let Some(group) = group_for(id) {
        refresh(group);
    }
}

/// Apply every group from the registry. Called once at startup and on load/save.
pub fn refresh_all() {
    for group in [
        Group::Imu,
        Group::Current,
        Group::Can,
        Group::Health,
        Group::Event,
    ] {
        refresh(group);
    }
}

fn group_for(id: Id) -> Option<Group> {
    [
        Group::Imu,
        Group::Current,
        Group::Can,
        Group::Health,
        Group::Event,
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

/// Whether an event source is enabled (no rate limiting).
pub fn enabled(group: Group) -> bool {
    STATE.enabled[group as usize].load(Ordering::Relaxed) != 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_groups_cover_the_logged_sources() {
        assert_eq!(
            REGISTRY
                .iter()
                .filter(|entry| entry.name.starts_with("LOG_"))
                .count(),
            9
        );
        for group in [
            Group::Imu,
            Group::Current,
            Group::Can,
            Group::Health,
            Group::Event,
        ] {
            assert!(REGISTRY.iter().any(|entry| entry.id == group.enable()));
            if let Some(rate) = group.rate() {
                assert!(REGISTRY.iter().any(|entry| entry.id == rate));
            }
        }
        assert_eq!(rate_period_us(200.0), 5_000);
        assert_eq!(rate_period_us(10.0), 100_000);
    }
}
