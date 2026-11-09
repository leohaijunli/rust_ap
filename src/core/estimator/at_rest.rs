//! Stationary detector synthesizing the PX4 `vehicle_at_rest` system flag for
//! zero-velocity/zero-gyro updates. It mirrors the fixed PX4 `LandDetector`
//! reverse timer: the vehicle counts as at rest until motion is detected, and
//! detected motion restarts the one-second quiet clock. The external safety
//! gate (disarmed and outputs disabled) plays the role of PX4's landed state.

use super::{math, Timestamp};

/// PX4 `LandDetector` `GYRO_NORM_MAX`: rates above 3 deg/s count as motion.
const GYRO_MOVE_LIMIT_RAD_S: f32 = 3.0 * (core::f64::consts::PI / 180.0) as f32;
/// Simplified stand-in for the PX4 vibration metric: a specific-force magnitude
/// deviating this much from gravity also counts as motion.
const ACCEL_MOVE_LIMIT_M_S2: f32 = 1.0;
/// PX4 requires 1 s without a move detection (`hrt_elapsed_time(...) > 1_s`).
const HOLD_TIME_US: u64 = 1_000_000;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) struct AtRestDetector {
    last_move: Option<Timestamp>,
}

impl AtRestDetector {
    pub(super) const fn new() -> Self {
        Self { last_move: None }
    }

    /// Feed one delayed IMU sample. `permitted` is the external safety gate.
    /// `None` means no motion was ever seen, which starts the clock already
    /// elapsed, exactly like PX4's zero-initialized move timestamp.
    pub(super) fn update(
        &mut self,
        timestamp: Timestamp,
        gyro_rad_s: [f32; 3],
        accel_m_s2: [f32; 3],
        permitted: bool,
    ) -> bool {
        let moving = math::norm(&gyro_rad_s) > GYRO_MOVE_LIMIT_RAD_S
            || (math::norm(&accel_m_s2) - math::GRAVITY).abs() > ACCEL_MOVE_LIMIT_M_S2;
        if moving {
            self.last_move = Some(timestamp);
        }
        let quiet_long_enough = match self.last_move {
            None => true,
            Some(last) => timestamp.saturating_sub(last) > HOLD_TIME_US,
        };
        quiet_long_enough && permitted
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const STILL_ACCEL: [f32; 3] = [0.0, 0.0, math::GRAVITY];

    #[test]
    fn rest_starts_immediately_and_survives_without_motion() {
        let mut detector = AtRestDetector::new();
        assert!(detector.update(0, [0.0; 3], STILL_ACCEL, true));
        assert!(detector.update(HOLD_TIME_US * 10, [0.0; 3], STILL_ACCEL, true));
    }

    #[test]
    fn motion_breaks_rest_and_restarts_the_clock() {
        let mut detector = AtRestDetector::new();
        assert!(detector.update(0, [0.0; 3], STILL_ACCEL, true));
        assert!(!detector.update(
            HOLD_TIME_US,
            [GYRO_MOVE_LIMIT_RAD_S * 1.1, 0.0, 0.0],
            STILL_ACCEL,
            true
        ));
        let restart = HOLD_TIME_US + 1;
        assert!(!detector.update(restart, [0.0; 3], STILL_ACCEL, true));
        // Exactly HOLD after the move detection the clock is not yet elapsed.
        assert!(!detector.update(2 * HOLD_TIME_US, [0.0; 3], STILL_ACCEL, true));
        assert!(detector.update(2 * HOLD_TIME_US + 1, [0.0; 3], STILL_ACCEL, true));
    }

    #[test]
    fn permission_gate_blocks_rest_and_releases_immediately() {
        let mut detector = AtRestDetector::new();
        assert!(!detector.update(0, [0.0; 3], STILL_ACCEL, false));
        assert!(!detector.update(HOLD_TIME_US * 10, [0.0; 3], STILL_ACCEL, false));
        assert!(detector.update(HOLD_TIME_US * 10 + 1, [0.0; 3], STILL_ACCEL, true));
    }

    #[test]
    fn accel_deviation_counts_as_motion() {
        let mut detector = AtRestDetector::new();
        let accel = [0.0, 0.0, math::GRAVITY + 2.0 * ACCEL_MOVE_LIMIT_M_S2];
        assert!(detector.update(0, [0.0; 3], STILL_ACCEL, true));
        assert!(!detector.update(HOLD_TIME_US, [0.0; 3], accel, true));
        assert!(!detector.update(2 * HOLD_TIME_US, [0.0; 3], STILL_ACCEL, true));
        assert!(detector.update(2 * HOLD_TIME_US + 1, [0.0; 3], STILL_ACCEL, true));
    }
}
