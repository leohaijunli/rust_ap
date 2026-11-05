//! Fixed PX4 scalar BiasEstimator used by height aid sources.
//! Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause
//! License: tools/px4_msgs/LICENSE; source: EKF/bias_estimator/bias_estimator.*.

use super::FilterFault;

#[derive(Clone, Copy, Debug)]
pub struct BiasEstimator {
    state: f32,
    variance: f32,
    process_psd: f32,
    maximum_variance: f32,
    dt: f32,
    alpha: f32,
    signed_ratio: f32,
    since_positive: f32,
    since_negative: f32,
}

impl BiasEstimator {
    pub const fn new(state: f32, variance: f32) -> Result<Self, FilterFault> {
        if !state.is_finite() || !variance.is_finite() || variance < 0.0 {
            return Err(FilterFault::Input);
        }
        Ok(Self {
            state,
            variance,
            process_psd: 1.25e-6,
            maximum_variance: 2.0,
            dt: 0.01,
            alpha: 0.0,
            signed_ratio: 0.0,
            since_positive: 0.0,
            since_negative: 0.0,
        })
    }

    pub fn bias(&self) -> f32 {
        self.state
    }
    pub fn variance(&self) -> f32 {
        self.variance
    }

    pub fn set_bias(&mut self, bias: f32) -> Result<(), FilterFault> {
        if !bias.is_finite() {
            return Err(FilterFault::Input);
        }
        self.state = bias;
        Ok(())
    }

    pub fn set_noise(&mut self, nsd: f32, maximum_noise: f32) -> Result<(), FilterFault> {
        let process_psd = nsd * nsd;
        let maximum_variance = maximum_noise * maximum_noise;
        if nsd < 0.0
            || maximum_noise < 0.0
            || !process_psd.is_finite()
            || !maximum_variance.is_finite()
        {
            return Err(FilterFault::Input);
        }
        self.process_psd = process_psd;
        self.maximum_variance = maximum_variance.max(0.0001);
        Ok(())
    }

    pub fn reset(&mut self) {
        self.state = 0.0;
        self.variance = 0.0;
        self.signed_ratio = 0.0;
        self.since_positive = 0.0;
        self.since_negative = 0.0;
    }

    pub fn predict(&mut self, dt: f32) -> Result<(), FilterFault> {
        if !dt.is_finite() || dt < 0.0 {
            return Err(FilterFault::Input);
        }
        let offset = self.signed_ratio.abs() > 0.2
            || self.since_positive > 10.0
            || self.since_negative > 10.0;
        let mut growth = self.process_psd * dt;
        if offset {
            growth *= 1000.0;
        }
        if !growth.is_finite() {
            return Err(FilterFault::Numerics);
        }
        self.variance = (self.variance + growth).clamp(1e-8, self.maximum_variance);
        if dt > f32::EPSILON && (self.dt - dt).abs() > 0.001 {
            self.alpha = dt / (10.0 + dt);
            self.dt = dt;
        }
        Ok(())
    }

    /// Return the original NIS test ratio; rejection still updates the sequence
    /// monitor, as in PX4. HeightSensor reference/active gating is the caller's.
    pub fn fuse(
        &mut self,
        measurement: f32,
        measurement_variance: f32,
    ) -> Result<f32, FilterFault> {
        if !measurement.is_finite()
            || !measurement_variance.is_finite()
            || measurement_variance < 0.0
        {
            return Err(FilterFault::Input);
        }
        let innovation_variance = self.variance + measurement_variance.max(0.0001);
        let innovation = measurement - self.state;
        let gain = self.variance / innovation_variance;
        let ratio = innovation * innovation / (9.0 * innovation_variance);
        if !ratio.is_finite() {
            return Err(FilterFault::Numerics);
        }
        if ratio < 1.0 {
            self.state += gain * innovation;
            self.variance =
                (self.variance - gain * self.variance).clamp(1e-8, self.maximum_variance);
        }
        let sign = if innovation > 0.0 {
            1.0
        } else if innovation < 0.0 {
            -1.0
        } else {
            0.0
        };
        self.signed_ratio += self.alpha * ((sign * ratio).clamp(-1.0, 1.0) - self.signed_ratio);
        if innovation > 0.0 {
            self.since_positive = 0.0;
            self.since_negative += self.dt;
        } else {
            self.since_negative = 0.0;
            self.since_positive += self.dt;
        }
        Ok(ratio)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scalar_update_rejects_outlier_without_losing_bias_or_accepting_nan() {
        let mut estimator = BiasEstimator::new(0.0, 1.0).unwrap();
        assert!(estimator.fuse(1.0, 1.0).unwrap() < 1.0);
        assert_eq!(estimator.bias(), 0.5);
        assert_eq!(estimator.variance(), 0.5);
        assert!(estimator.fuse(100.0, 1.0).unwrap() > 1.0);
        assert_eq!(estimator.bias(), 0.5);
        assert_eq!(estimator.fuse(f32::NAN, 1.0), Err(FilterFault::Input));
        estimator.reset();
        estimator.predict(0.01).unwrap();
        assert!(estimator.variance() >= 1e-8);
    }
}
