//! Fixed PX4 range validity, stuck detection and five-sample fog/obstruction check.
//! Copyright (c) 2018-2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.
//! Source: EKF/aid_sources/range_finder/sensor_range_finder.{cpp,hpp}.

use super::{FilterFault, Timestamp};

#[derive(Clone, Copy, Debug)]
pub struct RangeSensorParams {
    pub minimum_m: f32,
    pub maximum_m: f32,
    pub pitch_offset_rad: f32,
    pub minimum_cos_tilt: f32,
    pub quality_hysteresis_us: u64,
    pub stuck_threshold_m: f32,
    pub maximum_fog_distance_m: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RangeObservation {
    pub timestamp: Timestamp,
    pub distance_m: f32,
    /// PX4 quality percent: -1 unknown, 0 invalid, 1..100 measured quality.
    pub quality_percent: i8,
    /// Actual sensor limits travel with the delayed observation. Host fixtures
    /// may use None to retain their explicitly configured synthetic limits.
    pub limits: Option<[f32; 2]>,
}

pub struct RangeSensorChecks {
    params: RangeSensorParams,
    sample: Option<RangeObservation>,
    ready: bool,
    healthy: bool,
    regular: bool,
    filtered_age_us: f32,
    bad_quality_time: u64,
    last_valid_time: u64,
    stuck: bool,
    stuck_minimum: f32,
    stuck_maximum: f32,
    cos_tilt: f32,
    median_samples: [f32; 5],
    median_head: usize,
    previous_median: f32,
    blocked: bool,
}

impl RangeSensorChecks {
    pub fn new(params: RangeSensorParams) -> Result<Self, FilterFault> {
        if [
            params.minimum_m,
            params.maximum_m,
            params.pitch_offset_rad,
            params.minimum_cos_tilt,
            params.stuck_threshold_m,
            params.maximum_fog_distance_m,
        ]
        .iter()
        .any(|value| !value.is_finite())
            || params.minimum_m < 0.0
            || params.maximum_m <= params.minimum_m
            || !(0.0..=1.0).contains(&params.minimum_cos_tilt)
            || params.stuck_threshold_m < 0.0
            || params.maximum_fog_distance_m < 0.0
        {
            return Err(FilterFault::Input);
        }
        Ok(Self {
            params,
            sample: None,
            ready: false,
            healthy: false,
            regular: false,
            filtered_age_us: 0.0,
            bad_quality_time: 0,
            last_valid_time: 0,
            stuck: false,
            stuck_minimum: 0.0,
            stuck_maximum: 0.0,
            cos_tilt: 1.0,
            median_samples: [0.0; 5],
            median_head: 0,
            previous_median: 0.0,
            blocked: false,
        })
    }

    pub fn set_sample(&mut self, sample: RangeObservation) -> Result<(), FilterFault> {
        if !sample.distance_m.is_finite() || !(-1..=100).contains(&sample.quality_percent) {
            return Err(FilterFault::Input);
        }
        if let Some([minimum, maximum]) = sample.limits {
            if !minimum.is_finite() || !maximum.is_finite() || minimum < 0.0 || maximum <= minimum {
                return Err(FilterFault::Input);
            }
            self.params.minimum_m = minimum;
            self.params.maximum_m = maximum;
        }
        self.sample = Some(sample);
        self.ready = true;
        Ok(())
    }

    pub fn healthy(&self) -> bool {
        self.healthy
    }
    pub fn data_healthy(&self) -> bool {
        self.ready && self.healthy
    }
    pub fn regularly_sending(&self) -> bool {
        self.regular
    }
    pub fn cos_tilt(&self) -> f32 {
        self.cos_tilt
    }
    pub fn distance_vertical_m(&self) -> Option<f32> {
        self.sample.map(|sample| sample.distance_m * self.cos_tilt)
    }

    pub fn distance_m(&self) -> Option<f32> {
        self.sample.map(|sample| sample.distance_m)
    }

    pub fn correct_mounting(&mut self, vertical_offset_m: f32) {
        if let Some(sample) = &mut self.sample {
            sample.distance_m += vertical_offset_m / self.cos_tilt;
        }
    }

    /// Called at the EKF update cadence; `new_data` distinguishes a freshly
    /// popped observation from repeated checks of the last stored sample.
    pub fn run(
        &mut self,
        now: Timestamp,
        rotation_body_to_ned: [[f32; 3]; 3],
        new_data: bool,
    ) -> Result<(), FilterFault> {
        self.ready = new_data;
        let Some(sample) = self.sample else {
            self.healthy = false;
            self.regular = false;
            return Ok(());
        };
        if sample.timestamp > now
            || rotation_body_to_ned
                .iter()
                .flatten()
                .any(|value| !value.is_finite())
        {
            return Err(FilterFault::Input);
        }
        self.cos_tilt = rotation_body_to_ned[2][0] * libm::sinf(self.params.pitch_offset_rad)
            + rotation_body_to_ned[2][2] * libm::cosf(self.params.pitch_offset_rad);
        let age = now - sample.timestamp;
        let alpha = 0.5 * 0.01;
        self.filtered_age_us = (self.filtered_age_us * (1.0 - alpha) + alpha * age as f32).min(4e6);
        if age > 400_000 || self.filtered_age_us >= 2e6 {
            self.healthy = false;
            self.regular = false;
            return Ok(());
        }
        self.regular = true;
        if !new_data {
            return Ok(());
        }
        self.healthy = false;
        if sample.quality_percent == 0 {
            self.bad_quality_time = now;
        }
        if now - self.bad_quality_time <= self.params.quality_hysteresis_us
            || self.cos_tilt <= self.params.minimum_cos_tilt
            || sample.distance_m < self.params.minimum_m
            || sample.distance_m > self.params.maximum_m
        {
            return Ok(());
        }
        if self.params.stuck_threshold_m == 0.0 {
            self.stuck = false;
        } else if sample.timestamp.saturating_sub(self.last_valid_time) > 10_000_000 {
            if self.stuck_maximum - self.stuck_minimum > self.params.stuck_threshold_m {
                self.stuck_minimum = 0.0;
                self.stuck_maximum = 0.0;
                self.stuck = false;
            } else {
                self.stuck_maximum = self.stuck_maximum.max(sample.distance_m);
                if self.stuck_minimum < 0.1 || sample.distance_m < self.stuck_minimum {
                    self.stuck_minimum = sample.distance_m;
                }
                self.stuck = true;
            }
        }
        if self.params.maximum_fog_distance_m > 0.0 {
            self.median_samples[self.median_head] = sample.distance_m * self.cos_tilt;
            self.median_head = (self.median_head + 1) % 5;
            let mut sorted = self.median_samples;
            sorted.sort_unstable_by(f32::total_cmp);
            let median = sorted[2];
            let fog = self.params.maximum_fog_distance_m;
            if !self.blocked && median < fog && self.previous_median - median > 2.0 * fog {
                self.blocked = true;
            } else if self.blocked && median > 2.0 * fog {
                self.blocked = false;
            }
            self.previous_median = median;
        }
        if !self.stuck && !self.blocked {
            self.healthy = true;
            self.last_valid_time = sample.timestamp;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const ROTATION: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    #[test]
    fn fog_median_blocks_a_sudden_near_return_and_requires_far_returns_to_recover() {
        let mut checks = RangeSensorChecks::new(RangeSensorParams {
            minimum_m: 0.1,
            maximum_m: 30.0,
            pitch_offset_rad: 0.0,
            minimum_cos_tilt: f32::from_bits(0x3f350481),
            quality_hysteresis_us: 0,
            stuck_threshold_m: 0.1,
            maximum_fog_distance_m: 3.0,
        })
        .unwrap();
        for i in 1..=5 {
            let time = i * 20_000;
            checks
                .set_sample(RangeObservation {
                    timestamp: time,
                    distance_m: 10.0,
                    quality_percent: 100,
                    limits: None,
                })
                .unwrap();
            checks.run(time, ROTATION, true).unwrap();
        }
        assert!(checks.data_healthy());
        for i in 6..=10 {
            let time = i * 20_000;
            checks
                .set_sample(RangeObservation {
                    timestamp: time,
                    distance_m: 1.0,
                    quality_percent: 100,
                    limits: None,
                })
                .unwrap();
            checks.run(time, ROTATION, true).unwrap();
        }
        assert!(!checks.data_healthy());
        for i in 11..=15 {
            let time = i * 20_000;
            checks
                .set_sample(RangeObservation {
                    timestamp: time,
                    distance_m: 10.0,
                    quality_percent: 100,
                    limits: None,
                })
                .unwrap();
            checks.run(time, ROTATION, true).unwrap();
        }
        assert!(checks.data_healthy());
    }

    #[test]
    fn range_quality_hysteresis_tilt_bounds_and_staleness_revoke_health() {
        let params = RangeSensorParams {
            minimum_m: 0.1,
            maximum_m: 30.0,
            pitch_offset_rad: 0.0,
            minimum_cos_tilt: f32::from_bits(0x3f350481), // Exact pinned 0.7071f, not 1/sqrt(2).
            quality_hysteresis_us: 1_000_000,
            stuck_threshold_m: 0.1,
            maximum_fog_distance_m: 0.0,
        };
        let mut checks = RangeSensorChecks::new(params).unwrap();
        checks
            .set_sample(RangeObservation {
                timestamp: 500_000,
                distance_m: 1.0,
                quality_percent: 100,
                limits: None,
            })
            .unwrap();
        checks.run(500_000, ROTATION, true).unwrap();
        assert!(!checks.healthy());
        checks
            .set_sample(RangeObservation {
                timestamp: 1_000_001,
                distance_m: 1.0,
                quality_percent: 100,
                limits: None,
            })
            .unwrap();
        checks.run(1_000_001, ROTATION, true).unwrap();
        assert!(checks.data_healthy());
        checks.run(1_400_002, ROTATION, false).unwrap();
        assert!(!checks.healthy());
        checks
            .set_sample(RangeObservation {
                timestamp: 1_500_000,
                distance_m: 31.0,
                quality_percent: 100,
                limits: None,
            })
            .unwrap();
        checks.run(1_500_000, ROTATION, true).unwrap();
        assert!(!checks.healthy());
        checks
            .set_sample(RangeObservation {
                timestamp: 1_600_000,
                distance_m: 1.0,
                quality_percent: 0,
                limits: None,
            })
            .unwrap();
        checks.run(1_600_000, ROTATION, true).unwrap();
        assert!(!checks.healthy());
        checks
            .set_sample(RangeObservation {
                timestamp: 2_700_001,
                distance_m: 1.0,
                quality_percent: 100,
                limits: None,
            })
            .unwrap();
        checks
            .run(
                2_700_001,
                [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.5]],
                true,
            )
            .unwrap();
        assert!(!checks.healthy());
    }
}
