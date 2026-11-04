//! PX4 reset counters/deltas, including accumulation within one EKF update.
//! Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.

use super::math;

/// Counter order: quaternion, horizontal velocity, vertical velocity,
/// horizontal position, height, HAGL. Counters intentionally wrap at 255.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ResetReport {
    pub counts: [u8; 6],
    pub quaternion_delta: [f32; 4],
    pub velocity_delta: [f32; 3],
    pub position_delta: [f32; 3],
    pub hagl_delta: f32,
}

impl Default for ResetReport {
    fn default() -> Self {
        Self {
            counts: [0; 6],
            quaternion_delta: math::IDENTITY,
            velocity_delta: [0.0; 3],
            position_delta: [0.0; 3],
            hagl_delta: 0.0,
        }
    }
}

#[derive(Default)]
pub(super) struct ResetStatus {
    pub report: ResetReport,
    previous: [u8; 6],
}

impl ResetStatus {
    pub fn begin_update(&mut self) {
        self.previous = self.report.counts;
    }
    pub fn quaternion_changed(&self) -> bool {
        self.report.counts[0] != self.previous[0]
    }
    pub fn quaternion(&mut self, delta: [f32; 4]) {
        self.report.quaternion_delta = if self.report.counts[0] == self.previous[0] {
            delta
        } else {
            math::normalize(math::multiply(delta, self.report.quaternion_delta))
        };
        self.report.counts[0] = self.report.counts[0].wrapping_add(1);
    }
    pub fn horizontal_velocity(&mut self, delta: [f32; 2]) {
        for (i, value) in delta.iter().enumerate() {
            if self.report.counts[1] == self.previous[1] {
                self.report.velocity_delta[i] = *value;
            } else {
                self.report.velocity_delta[i] += value;
            }
        }
        self.report.counts[1] = self.report.counts[1].wrapping_add(1);
    }
    pub fn vertical_velocity(&mut self, delta: f32) {
        if self.report.counts[2] == self.previous[2] {
            self.report.velocity_delta[2] = delta;
        } else {
            self.report.velocity_delta[2] += delta;
        }
        self.report.counts[2] = self.report.counts[2].wrapping_add(1);
    }
    pub fn horizontal_position(&mut self, delta: [f32; 2]) {
        for (i, value) in delta.iter().enumerate() {
            if self.report.counts[3] == self.previous[3] {
                self.report.position_delta[i] = *value;
            } else {
                self.report.position_delta[i] += value;
            }
        }
        self.report.counts[3] = self.report.counts[3].wrapping_add(1);
    }
    pub fn height(&mut self, delta: f32) {
        if self.report.counts[4] == self.previous[4] {
            self.report.position_delta[2] = delta;
        } else {
            self.report.position_delta[2] += delta;
        }
        self.report.counts[4] = self.report.counts[4].wrapping_add(1);
    }
    pub fn hagl(&mut self, delta: f32) {
        if self.report.counts[5] == self.previous[5] {
            self.report.hagl_delta = delta;
        } else {
            self.report.hagl_delta += delta;
        }
        self.report.counts[5] = self.report.counts[5].wrapping_add(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn resets_accumulate_only_within_the_update_and_counters_wrap() {
        let mut status = ResetStatus::default();
        status.begin_update();
        status.height(-10.0);
        status.height(-5.0);
        assert_eq!(status.report.position_delta[2], -15.0);
        status.begin_update();
        status.height(3.0);
        assert_eq!(status.report.position_delta[2], 3.0);
        status.report.counts[4] = 255;
        status.begin_update();
        status.height(1.0);
        assert_eq!(status.report.counts[4], 0);
        status.begin_update();
        status.quaternion(math::from_axis_angle([0.0, 0.0, 0.1]));
        status.quaternion(math::from_axis_angle([0.0, 0.0, 0.2]));
        assert!((math::to_axis_angle(status.report.quaternion_delta)[2] - 0.3).abs() < 1e-6);
    }
}
