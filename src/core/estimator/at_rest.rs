//! Conservative stationary detector used to synthesize the PX4
//! `vehicle_at_rest` system flag for zero-velocity/zero-gyro updates.
//! It can only assert rest while motion is externally disabled and the IMU has
//! stayed quiet for a full hold window; any motion releases it immediately.

use super::{math, Timestamp};

/// About 2.9 deg/s: above a typical startup gyro bias, well below deliberate
/// rotation.
const GYRO_STILL_LIMIT_RAD_S: f32 = 0.05;
/// Specific-force magnitude may differ from gravity by this much at rest.
const ACCEL_STILL_LIMIT_M_S2: f32 = 1.0;
/// Require a full second of quiet samples before declaring rest.
const HOLD_TIME_US: u64 = 1_000_000;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) struct AtRestDetector {
    quiet_since: Option<Timestamp>,
    at_rest: bool,
}

impl AtRestDetector {
    pub(super) const fn new() -> Self {
        Self {
            quiet_since: None,
            at_rest: false,
        }
    }

    /// Feed one delayed IMU sample. `permitted` is the external safety gate:
    /// it must stay true for the whole hold window.
    pub(super) fn update(
        &mut self,
        timestamp: Timestamp,
        gyro_rad_s: [f32; 3],
        accel_m_s2: [f32; 3],
        permitted: bool,
    ) -> bool {
        let quiet = permitted
            && math::norm(&gyro_rad_s) <= GYRO_STILL_LIMIT_RAD_S
            && (math::norm(&accel_m_s2) - math::GRAVITY).abs() <= ACCEL_STILL_LIMIT_M_S2;
        if !quiet {
            self.quiet_since = None;
            self.at_rest = false;
            return false;
        }
        let since = *self.quiet_since.get_or_insert(timestamp);
        if timestamp.saturating_sub(since) >= HOLD_TIME_US {
            self.at_rest = true;
        }
        self.at_rest
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const STILL_ACCEL: [f32; 3] = [0.0, 0.0, math::GRAVITY];

    #[test]
    fn rest_requires_the_full_hold_window() {
        let mut detector = AtRestDetector::new();
        assert!(!detector.update(0, [0.0; 3], STILL_ACCEL, true));
        assert!(!detector.update(HOLD_TIME_US - 1, [0.0; 3], STILL_ACCEL, true));
        assert!(detector.update(HOLD_TIME_US, [0.0; 3], STILL_ACCEL, true));
    }

    #[test]
    fn motion_releases_immediately_and_restarts_the_hold() {
        let mut detector = AtRestDetector::new();
        assert!(!detector.update(0, [0.0; 3], STILL_ACCEL, true));
        assert!(detector.update(HOLD_TIME_US, [0.0; 3], STILL_ACCEL, true));
        assert!(!detector.update(
            HOLD_TIME_US + 1,
            [GYRO_STILL_LIMIT_RAD_S * 2.0, 0.0, 0.0],
            STILL_ACCEL,
            true
        ));
        let restart = HOLD_TIME_US + 2;
        assert!(!detector.update(restart, [0.0; 3], STILL_ACCEL, true));
        assert!(!detector.update(restart + HOLD_TIME_US - 1, [0.0; 3], STILL_ACCEL, true));
        assert!(detector.update(restart + HOLD_TIME_US, [0.0; 3], STILL_ACCEL, true));
    }

    #[test]
    fn permission_gate_blocks_rest_and_clears_it() {
        let mut detector = AtRestDetector::new();
        assert!(!detector.update(0, [0.0; 3], STILL_ACCEL, false));
        assert!(!detector.update(HOLD_TIME_US, [0.0; 3], STILL_ACCEL, false));
        assert!(!detector.update(HOLD_TIME_US + 1, [0.0; 3], STILL_ACCEL, true));
        assert!(detector.update(2 * HOLD_TIME_US + 1, [0.0; 3], STILL_ACCEL, true));
        assert!(!detector.update(2 * HOLD_TIME_US + 2, [0.0; 3], STILL_ACCEL, false));
    }

    #[test]
    fn accel_deviation_blocks_rest() {
        let mut detector = AtRestDetector::new();
        let accel = [0.0, 0.0, math::GRAVITY + 2.0 * ACCEL_STILL_LIMIT_M_S2];
        assert!(!detector.update(HOLD_TIME_US, [0.0; 3], accel, true));
    }
}
