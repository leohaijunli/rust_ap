//! Last executed one-shot configuration step. A failed reconfigure keeps the
//! stage that faulted so board diagnostics can name it with one console read.

use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

/// Nothing running; the previous attempt either succeeded or never started.
pub const IDLE: u8 = 0;
/// Parameter range and delay-capacity validation.
pub const VALIDATE: u8 = 1;
/// IMU core reset, downsampler and delayed ring rebuild.
pub const IMU_INIT: u8 = 2;
/// Barometer and magnetometer aid rebuild.
pub const BARO_MAG: u8 = 3;
/// Range-aid rebuild and range sensor checks.
pub const RANGE: u8 = 4;
/// GNSS aid rebuild and health-check parameters.
pub const GNSS: u8 = 5;
/// Output predictor parameters.
pub const OUTPUT: u8 = 6;
/// Output configuration rejected because IMU input was already accepted.
pub const INPUT_SEEN: u8 = 7;
/// Filter health after a nominally successful configuration.
pub const HEALTH: u8 = 8;

static STAGE: AtomicU8 = AtomicU8::new(IDLE);
static RETRIES: AtomicU32 = AtomicU32::new(0);

/// Record the step that is about to run.
pub fn enter(stage: u8) {
    STAGE.store(stage, Ordering::Relaxed);
}

/// Step that ran last; equals the failing step after a configuration fault.
pub fn current() -> u8 {
    STAGE.load(Ordering::Relaxed)
}

/// Count one re-read of a configuration that reported a fault. The re-read is
/// allowed to overwrite the stage so the last attempted configure is named.
pub fn note_retry(stage: u8) {
    enter(stage);
    RETRIES.fetch_add(1, Ordering::Relaxed);
}

pub fn retry_count() -> u32 {
    RETRIES.load(Ordering::Relaxed)
}
