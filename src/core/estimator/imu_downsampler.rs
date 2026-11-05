//! Translation of fixed PX4 ImuDownSampler / IMU RingBuffer push semantics.
//! Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause
//! Source mapping and license: docs/provenance.md, tools/px4_msgs/LICENSE.

use super::{math, ImuDelta};

const EMPTY: ImuDelta = ImuDelta {
    timestamp: 0,
    delta_angle_rad: [0.0; 3],
    delta_velocity_m_s: [0.0; 3],
    delta_angle_dt_s: 0.0,
    delta_velocity_dt_s: 0.0,
    delta_velocity_clipping: [false; 3],
};

pub(super) struct DownSampler {
    sample: ImuDelta,
    quaternion: math::Quaternion,
    count: u32,
    required_samples: u32,
    average_dt_s: f32,
    target_dt_s: f32,
    minimum_dt_s: f32,
    predict_us: u32,
}

impl DownSampler {
    pub const fn dormant() -> Self {
        Self {
            sample: EMPTY,
            quaternion: math::IDENTITY,
            count: 0,
            required_samples: 1,
            average_dt_s: 0.005,
            target_dt_s: 0.01,
            minimum_dt_s: 0.005,
            predict_us: 10_000,
        }
    }

    pub fn new(predict_us: u32) -> Self {
        let mut result = Self {
            sample: EMPTY,
            quaternion: math::IDENTITY,
            count: 0,
            required_samples: 1,
            average_dt_s: 0.005,
            target_dt_s: 0.01,
            minimum_dt_s: 0.005,
            predict_us,
        };
        result.reset();
        result
    }

    fn with_input_period(predict_us: u32, dt_s: f32) -> Self {
        let mut result = Self::new(predict_us);
        result.average_dt_s = dt_s;
        result.reset();
        result
    }

    fn reset(&mut self) {
        self.sample = EMPTY;
        self.quaternion = math::IDENTITY;
        self.count = 0;
        let target = self.predict_us.clamp(1000, 100_000) as f32 * 1e-6;
        self.required_samples = (libm::roundf(target / self.average_dt_s) as u32).max(1);
        self.target_dt_s = self.required_samples as f32 * self.average_dt_s;
        self.minimum_dt_s =
            (self.average_dt_s * (self.required_samples as f32 - 1.0)).max(self.average_dt_s * 0.5);
    }

    pub fn update(&mut self, input: &ImuDelta) -> Option<ImuDelta> {
        self.average_dt_s = 0.9 * self.average_dt_s + 0.1 * input.delta_angle_dt_s;
        self.sample.timestamp = input.timestamp;
        self.sample.delta_angle_dt_s += input.delta_angle_dt_s;
        self.sample.delta_velocity_dt_s += input.delta_velocity_dt_s;
        for axis in 0..3 {
            self.sample.delta_velocity_clipping[axis] |= input.delta_velocity_clipping[axis];
        }
        let delta_q = math::from_axis_angle(input.delta_angle_rad);
        self.quaternion = math::normalize(math::multiply(self.quaternion, delta_q));
        let delta_r = math::rotation(math::inverse(delta_q));
        self.sample.delta_velocity_m_s = math::rotate(delta_r, self.sample.delta_velocity_m_s);
        let rotated = math::rotate(delta_r, input.delta_velocity_m_s);
        for (axis, rotated_value) in rotated.into_iter().enumerate() {
            self.sample.delta_velocity_m_s[axis] +=
                (input.delta_velocity_m_s[axis] + rotated_value) * 0.5;
        }
        self.count += 1;
        if (self.count >= self.required_samples && self.sample.delta_angle_dt_s > self.minimum_dt_s)
            || self.sample.delta_angle_dt_s > self.target_dt_s
        {
            self.sample.delta_angle_rad = math::to_axis_angle(self.quaternion);
            let result = self.sample;
            self.reset();
            Some(result)
        } else {
            None
        }
    }

    pub fn numerics_valid(&self) -> bool {
        self.quaternion.iter().all(|value| value.is_finite())
            && self
                .sample
                .delta_velocity_m_s
                .iter()
                .all(|value| value.is_finite())
            && self.sample.delta_angle_dt_s.is_finite()
            && self.sample.delta_velocity_dt_s.is_finite()
    }
}

/// One board input bridge and one 8-kHz -> ~1-kHz integrator, shared by all
/// algorithm instances. Bias correction remains private to each filter.
pub struct SharedImu {
    bridge: super::runtime::ImuBridge,
    downsampler: DownSampler,
    clock_start: u64,
    integrated_us: u64,
    pub clock: ClockReport,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ClockReport {
    pub windows: u32,
    pub sensor_dt_us: u64,
    pub tim2_dt_us: u64,
    pub error_ppm: i32,
}

impl Default for SharedImu {
    fn default() -> Self {
        Self {
            bridge: super::runtime::ImuBridge::new(),
            downsampler: DownSampler::with_input_period(1_000, 0.000125),
            clock_start: 0,
            integrated_us: 0,
            clock: ClockReport::default(),
        }
    }
}

impl SharedImu {
    pub fn push(
        &mut self,
        sample: crate::icm42688::Sample,
        now: u64,
    ) -> Result<Option<ImuDelta>, super::runtime::Fault> {
        let delta = match self.bridge.convert(sample, now) {
            Ok(delta) => delta,
            Err(fault) => {
                *self = Self::default();
                return Err(fault);
            }
        };
        let duration = u64::from(sample.sensor_timestamp) * 16;
        if self.clock_start == 0 {
            self.clock_start = sample.sample_timestamp_us.saturating_sub(duration);
        }
        self.integrated_us += duration;
        let elapsed = sample.sample_timestamp_us.saturating_sub(self.clock_start);
        // A one-second window makes the 16-us sensor quantization visible
        // without pretending that each 10-ms window can establish 100 ppm.
        if elapsed >= 1_000_000 {
            self.clock = ClockReport {
                windows: self.clock.windows.wrapping_add(1),
                sensor_dt_us: self.integrated_us,
                tim2_dt_us: elapsed,
                error_ppm: (((self.integrated_us as i64 - elapsed as i64) * 1_000_000)
                    / elapsed as i64) as i32,
            };
            self.clock_start = sample.sample_timestamp_us;
            self.integrated_us = 0;
        }
        let result = self.downsampler.update(&delta);
        if !self.downsampler.numerics_valid() {
            *self = Self::default();
            return Err(super::runtime::Fault::Input);
        }
        Ok(result)
    }
}

pub(super) const MAX_IMU_BUFFER: usize = 64;

pub(super) struct ImuBuffer {
    samples: [ImuDelta; MAX_IMU_BUFFER],
    length: usize,
    head: usize,
    tail: usize,
    first_write: bool,
}

impl ImuBuffer {
    #[inline(never)]
    pub fn reset_to(&mut self, length: usize) {
        const INITIAL: ImuBuffer = ImuBuffer::new(2);
        *self = INITIAL;
        self.length = length;
    }

    pub const fn new(length: usize) -> Self {
        Self {
            samples: [EMPTY; MAX_IMU_BUFFER],
            length,
            head: 0,
            tail: 0,
            first_write: true,
        }
    }

    pub fn push(&mut self, sample: ImuDelta) -> ImuDelta {
        if !self.first_write {
            self.head = (self.head + 1) % self.length;
        }
        self.samples[self.head] = sample;
        if self.head == self.tail && !self.first_write {
            self.tail = (self.tail + 1) % self.length;
        } else {
            self.first_write = false;
        }
        // Deliberately do not pop: the pinned filter repeats the oldest sample
        // while this buffer fills. The committed oracle retains that plateau.
        self.samples[self.tail]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coning_sculling_and_clipping_follow_successive_body_frames() {
        let mut downsampler = DownSampler::new(10_000);
        let mut sample = EMPTY;
        sample.timestamp = 5000;
        sample.delta_angle_dt_s = 0.005;
        sample.delta_velocity_dt_s = 0.004;
        sample.delta_angle_rad = [0.0, 0.0, core::f32::consts::FRAC_PI_2];
        sample.delta_velocity_m_s = [1.0, 0.0, 0.0];
        sample.delta_velocity_clipping = [true, false, false];
        assert!(downsampler.update(&sample).is_none());
        sample.timestamp = 10_000;
        sample.delta_velocity_dt_s = 0.006;
        sample.delta_angle_rad = [core::f32::consts::FRAC_PI_2, 0.0, 0.0];
        sample.delta_velocity_m_s = [0.0, 1.0, 0.0];
        sample.delta_velocity_clipping = [false, false, true];
        let result = downsampler.update(&sample).unwrap();
        assert_eq!(result.timestamp, 10_000);
        assert_eq!(result.delta_angle_dt_s, 0.01);
        assert_eq!(result.delta_velocity_dt_s, 0.01);
        assert_eq!(result.delta_velocity_clipping, [true, false, true]);
        for (actual, expected) in result.delta_velocity_m_s.into_iter().zip([0.5, 0.5, 0.0]) {
            assert!((actual - expected).abs() < 1e-6);
        }
        let quaternion = math::from_axis_angle(result.delta_angle_rad);
        for value in quaternion {
            assert!((value - 0.5).abs() < 1e-6);
        }
    }
}
