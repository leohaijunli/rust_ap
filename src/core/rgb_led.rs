//! Fixed NCP5623-compatible RGB protocol observed on this USL at I2C1/0x39.
//! Copyright (c) 2012-2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.
//! Reference: PX4 d6f12ad1c4f70ad3230afd7d86e971421e02fef4,
//! src/drivers/lights/rgbled_ncp5623c. Address/protocol compatibility
//! is not an independent chip identity; actual USL colors must be checked.

use crate::{logger::Status, logger_switch::indicator_pattern};

pub const ADDRESS: u8 = 0x39;
pub const OFF: [u8; 3] = [0, 0, 0];
pub const TEST_COLORS: [[u8; 3]; 3] = [[96, 0, 0], [0, 96, 0], [0, 0, 96]];

/// PWM is low brightness; NCP also limits its common current to 8/31 of full
/// scale. Individual NCP commands use separate transactions, like PX4 write().
pub const fn writes([r, g, b]: [u8; 3]) -> [u8; 4] {
    let on = r != 0 || g != 0 || b != 0;
    [
        0x20 | if on { 8 } else { 0 },
        0x40 | (r >> 3),
        0x60 | (g >> 3),
        0x80 | (b >> 3),
    ]
}

pub fn logger_color(status: Status, elapsed_ms: u64) -> [u8; 3] {
    if !indicator_pattern(status).is_on(elapsed_ms) {
        return OFF;
    }
    match status {
        Status::Disabled => OFF,
        Status::Ready => [0, 96, 0],
        Status::Initializing | Status::Stopping | Status::Managing => [96, 96, 0],
        Status::Error => [96, 0, 0],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_controller_writes_keep_channel_order_pwm_bounds_and_off() {
        assert_eq!(writes([255, 128, 0]), [0x28, 0x5f, 0x70, 0x80]);
        assert_eq!(writes(OFF), [0x20, 0x40, 0x60, 0x80]);
    }

    #[test]
    fn logger_colors_follow_real_status_and_existing_blink_timing() {
        assert_eq!(logger_color(Status::Disabled, 0), OFF);
        assert_eq!(logger_color(Status::Ready, 0), [0, 96, 0]);
        assert_eq!(logger_color(Status::Initializing, 0), [96, 96, 0]);
        assert_eq!(logger_color(Status::Stopping, 250), OFF);
        assert_eq!(logger_color(Status::Error, 0), [96, 0, 0]);
        assert_eq!(logger_color(Status::Error, 100), OFF);
    }
}
