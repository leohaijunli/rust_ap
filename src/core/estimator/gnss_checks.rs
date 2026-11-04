//! Fixed PX4 initial/simplified GNSS quality checks for the in-air host profile.
//! Copyright (c) 2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.
//! Source: EKF/aid_sources/gnss/gnss_checks.{cpp,hpp}. Ground drift filters are
//! deliberately outside this initial profile; this does not qualify GNSS aiding.

use super::{ekf2_parameter, FilterFault, GnssSample, ParamValue};

fn default_value(name: &str) -> f32 {
    match ekf2_parameter(name)
        .expect("fixed GNSS catalog parameter")
        .default
    {
        ParamValue::Int(value) => value as f32,
        ParamValue::Float(value) => value,
    }
}

#[derive(Clone, Copy, Debug)]
pub struct GnssCheckParams {
    pub mask: u16,
    pub minimum_fix: u8,
    pub minimum_satellites: u8,
    pub maximum_pdop: f32,
    pub maximum_horizontal_accuracy_m: f32,
    pub maximum_vertical_accuracy_m: f32,
    pub maximum_speed_accuracy_m_s: f32,
    pub velocity_limit_m_s: f32,
    pub healthy_time_us: u64,
}

impl Default for GnssCheckParams {
    fn default() -> Self {
        Self {
            mask: default_value("EKF2_GPS_CHECK") as u16,
            minimum_fix: default_value("EKF2_REQ_FIX") as u8,
            minimum_satellites: default_value("EKF2_REQ_NSATS") as u8,
            maximum_pdop: default_value("EKF2_REQ_PDOP"),
            maximum_horizontal_accuracy_m: default_value("EKF2_REQ_EPH"),
            maximum_vertical_accuracy_m: default_value("EKF2_REQ_EPV"),
            maximum_speed_accuracy_m_s: default_value("EKF2_REQ_SACC"),
            velocity_limit_m_s: default_value("EKF2_VEL_LIM"),
            healthy_time_us: (default_value("EKF2_REQ_GPS_H") * 1e6) as u64,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct GnssChecks {
    params: GnssCheckParams,
    initial_passed: bool,
    passed: bool,
    last_fail_us: u64,
    last_pass_us: u64,
    previous_time_us: u64,
    failure_bits: u16,
}

impl GnssChecks {
    pub fn new(params: GnssCheckParams) -> Result<Self, FilterFault> {
        if params.mask & !0x7ff != 0
            || [
                params.maximum_pdop,
                params.maximum_horizontal_accuracy_m,
                params.maximum_vertical_accuracy_m,
                params.maximum_speed_accuracy_m_s,
                params.velocity_limit_m_s,
            ]
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0)
        {
            return Err(FilterFault::Input);
        }
        Ok(Self {
            params,
            initial_passed: false,
            passed: false,
            last_fail_us: 0,
            last_pass_us: 0,
            previous_time_us: 0,
            failure_bits: 0,
        })
    }

    pub fn initial_passed(&self) -> bool {
        self.initial_passed
    }
    pub fn passed(&self) -> bool {
        self.passed
    }
    pub fn last_pass_us(&self) -> u64 {
        self.last_pass_us
    }
    pub fn failure_bits(&self) -> u16 {
        self.failure_bits
    }

    /// A stopped aid restarts the short health probation, preserving the initial latch.
    pub fn reset(&mut self) {
        self.passed = false;
        self.last_fail_us = 0;
        self.last_pass_us = 0;
    }

    /// Separate PDOP/spoof indication belongs to the input boundary. Coordinates,
    /// timestamps and finite accuracies are validated before PX4's threshold logic.
    pub fn run(
        &mut self,
        sample: &GnssSample,
        pdop: f32,
        spoofed: bool,
        now_us: u64,
    ) -> Result<bool, FilterFault> {
        if now_us <= self.previous_time_us
            || sample.timestamp > now_us
            || !sample.latitude_deg.is_finite()
            || !sample.longitude_deg.is_finite()
            || sample.latitude_deg.abs() > 90.0
            || sample.longitude_deg.abs() > 180.0
            || [
                sample.altitude_msl_m,
                sample.horizontal_accuracy_m,
                sample.vertical_accuracy_m,
                sample.speed_accuracy_m_s,
                pdop,
            ]
            .iter()
            .any(|value| !value.is_finite())
            || sample.horizontal_accuracy_m < 0.0
            || sample.vertical_accuracy_m < 0.0
            || sample.speed_accuracy_m_s < 0.0
            || pdop < 0.0
            || sample
                .velocity_ned_m_s
                .iter()
                .any(|value| !value.is_finite())
        {
            return Err(FilterFault::Input);
        }
        self.previous_time_us = now_us;
        if self.last_fail_us == 0 {
            self.last_fail_us = now_us;
        }
        let simplified = self.initial_passed;
        let failures = [
            (
                10,
                0,
                sample.fix_type
                    < if simplified {
                        3
                    } else {
                        self.params.minimum_fix
                    },
            ),
            (
                0,
                1,
                !simplified && sample.satellites < self.params.minimum_satellites,
            ),
            (1, 2, !simplified && pdop > self.params.maximum_pdop),
            (
                2,
                3,
                sample.horizontal_accuracy_m
                    > if simplified {
                        50.0
                    } else {
                        self.params.maximum_horizontal_accuracy_m
                    },
            ),
            (
                3,
                4,
                sample.vertical_accuracy_m
                    > if simplified {
                        50.0
                    } else {
                        self.params.maximum_vertical_accuracy_m
                    },
            ),
            (
                4,
                5,
                sample.speed_accuracy_m_s
                    > if simplified {
                        10.0
                    } else {
                        self.params.maximum_speed_accuracy_m_s
                    },
            ),
            (
                7,
                8,
                !simplified
                    && super::math::norm(&sample.velocity_ned_m_s[..2])
                        > self.params.velocity_limit_m_s,
            ),
            (
                8,
                9,
                !simplified && sample.velocity_ned_m_s[2].abs() > self.params.velocity_limit_m_s,
            ),
            (9, 10, spoofed),
        ];
        self.failure_bits = failures
            .iter()
            .filter(|(_, _, failed)| *failed)
            .fold(0, |bits, (_, bit, _)| bits | (1 << bit));
        let healthy = !failures
            .iter()
            .any(|(mask_bit, _, failed)| *failed && self.params.mask & (1 << mask_bit) != 0);
        self.passed = false;
        if healthy {
            self.last_pass_us = now_us;
            let interval = if simplified {
                (self.params.healthy_time_us / 10).max(1_000_000)
            } else {
                self.params.healthy_time_us
            };
            if self.last_fail_us.saturating_add(interval) < now_us {
                self.initial_passed = true;
                self.passed = true;
            }
        } else {
            self.last_fail_us = now_us;
        }
        Ok(self.passed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fix(timestamp: u64) -> GnssSample {
        GnssSample {
            timestamp,
            latitude_deg: 47.0,
            longitude_deg: 8.0,
            altitude_msl_m: 100.0,
            velocity_ned_m_s: [0.0; 3],
            horizontal_accuracy_m: 0.3,
            vertical_accuracy_m: 0.5,
            speed_accuracy_m_s: 0.1,
            fix_type: 3,
            satellites: 12,
        }
    }

    #[test]
    fn health_delay_is_strict_and_initial_checks_latch_with_shorter_recovery() {
        let mut checks = GnssChecks::new(GnssCheckParams::default()).unwrap();
        assert!(!checks.run(&fix(1_000), 1.0, false, 1_000).unwrap());
        assert!(!checks
            .run(&fix(10_001_000), 1.0, false, 10_001_000)
            .unwrap());
        assert!(checks
            .run(&fix(10_001_001), 1.0, false, 10_001_001)
            .unwrap());
        let mut bad = fix(11_000_000);
        bad.fix_type = 2;
        assert!(!checks.run(&bad, 1.0, false, bad.timestamp).unwrap());
        assert!(checks.initial_passed());
        assert_eq!(checks.failure_bits() & 1, 1);
        assert!(!checks
            .run(&fix(12_000_000), 1.0, false, 12_000_000)
            .unwrap());
        assert!(checks
            .run(&fix(12_000_001), 1.0, false, 12_000_001)
            .unwrap());
    }
}
