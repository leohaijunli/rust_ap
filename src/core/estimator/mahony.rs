//! IMU-only Mahony fallback, scalar f32 adaptation of fixed Cybflight.
//! Source: cybird-robotics/cybflight 806af5648c336e7af81abefea31052db783387f5,
//! crates/cybflight_core/src/mahony.rs; Apache-2.0, docs/licenses/cybflight.txt.
//! Modified 2026-11-04: use FRD/NED gravity, fixed arrays/libm, reject non-finite
//! input and clipping. No magnetic heading or flight qualification is implied.

use super::{math, FilterFault, ImuDelta};

pub struct Mahony {
    q: [f32; 4],
    bias: [f32; 3],
    initialized: bool,
    timestamp: u64,
    angular_velocity: [f32; 3],
    fault: Option<FilterFault>,
    updated: bool,
}

impl Default for Mahony {
    fn default() -> Self {
        Self::new()
    }
}

impl Mahony {
    pub const fn dormant() -> Self {
        Self::new()
    }

    pub fn configure_from_snapshot(
        &mut self,
        _params: &super::runtime_params::Params,
        _t0: u64,
    ) -> Result<(), FilterFault> {
        *self = Self::new();
        Ok(())
    }

    pub const fn new() -> Self {
        Self {
            q: math::IDENTITY,
            bias: [0.0; 3],
            initialized: false,
            timestamp: 0,
            angular_velocity: [0.0; 3],
            fault: None,
            updated: false,
        }
    }
}

impl Mahony {
    pub fn update(&mut self, imu: &ImuDelta) -> Result<(), FilterFault> {
        let dt = imu.delta_velocity_dt_s;
        if !dt.is_finite()
            || !(1e-6..=0.01).contains(&dt)
            || !imu.delta_angle_dt_s.is_finite()
            || !(1e-6..=0.01).contains(&imu.delta_angle_dt_s)
            || imu.delta_velocity_clipping.iter().any(|v| *v)
            || imu
                .delta_velocity_m_s
                .iter()
                .chain(&imu.delta_angle_rad)
                .any(|v| !v.is_finite())
        {
            return Err(FilterFault::Input);
        }
        let accel = imu.delta_velocity_m_s.map(|v| v / dt);
        let norm = math::norm(&accel);
        if !norm.is_finite() || norm <= 0.981 {
            return Err(FilterFault::Input);
        }
        let orientation = if self.initialized {
            self.q
        } else {
            math::align_tilt(accel)
        };
        let measured = accel.map(|v| v / norm);
        let reference = math::rotate_inverse(math::rotation(orientation), [0.0, 0.0, -1.0]);
        let error = math::cross(measured, reference);
        let mut delta = [0.0; 3];
        let mut bias = self.bias;
        for i in 0..3 {
            bias[i] -= 0.3 * error[i] * dt;
            delta[i] = imu.delta_angle_rad[i] - bias[i] * imu.delta_angle_dt_s
                + error[i] * imu.delta_angle_dt_s;
        }
        let q = math::normalize(math::multiply(orientation, math::from_axis_angle(delta)));
        if q.iter().any(|v| !v.is_finite()) {
            return Err(FilterFault::Numerics);
        }
        self.q = q;
        self.bias = bias;
        self.initialized = true;
        self.timestamp = imu.timestamp;
        self.angular_velocity =
            core::array::from_fn(|i| imu.delta_angle_rad[i] / imu.delta_angle_dt_s - bias[i]);
        self.updated = true;
        Ok(())
    }

    pub fn output(&self) -> Option<[f32; 4]> {
        self.initialized.then_some(self.q)
    }
}

impl super::NavFilter for Mahony {
    type Params = super::runtime_params::Params;

    fn new(_params: &Self::Params, _t0: u64) -> Self {
        Self::new()
    }

    fn push_imu(&mut self, sample: &ImuDelta) {
        if self.fault.is_some() {
            return;
        }
        if sample.timestamp <= self.timestamp {
            self.fault = Some(FilterFault::Input);
        } else if self.timestamp != 0
            && sample.timestamp - self.timestamp > super::imu_ekf::IMU_FRESH_US
        {
            self.fault = Some(FilterFault::Timeout);
        } else if let Err(fault) = self.update(sample) {
            self.fault = Some(fault);
        }
    }

    fn push_measurement(&mut self, _measurement: super::Measurement) {
        // This IMU-only algorithm does not claim external aiding or absolute yaw.
    }

    fn step(&mut self, now: u64) -> super::StepReport {
        if self.timestamp != 0
            && (now < self.timestamp || now - self.timestamp > super::imu_ekf::IMU_FRESH_US)
        {
            self.fault = Some(FilterFault::Timeout);
        }
        super::StepReport {
            updated: core::mem::take(&mut self.updated),
            ..super::StepReport::default()
        }
    }

    fn output(&self) -> super::NavOutput {
        super::NavOutput {
            timestamp: self.timestamp,
            q_body_to_ned: self.q,
            velocity_ned_m_s: [f32::NAN; 3],
            position_ned_m: [f32::NAN; 3],
            height_msl_m: f32::NAN,
            angular_velocity_body_rad_s: self.angular_velocity,
            delta_q_reset: math::IDENTITY,
            quat_reset_counter: 0,
            absolute_yaw: false,
        }
    }

    fn health(&self) -> super::FilterHealth {
        super::FilterHealth {
            valid: self.initialized && self.fault.is_none(),
            fault: self.fault,
            numerics: None,
            stage: None,
            pair: [f32::NAN; 2],
        }
    }
}

impl Mahony {
    pub fn standard_output(&mut self, now: u64) -> super::standard_output::Snapshot {
        super::standard_output::Snapshot::attitude_only(
            super::NavFilter::output(self),
            super::NavFilter::health(self),
            now,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ned_stationary_and_yaw_inputs_keep_a_unit_body_quaternion() {
        let mut filter = Mahony::default();
        for index in 1..=4_000 {
            filter
                .update(&ImuDelta {
                    timestamp: index * 125,
                    delta_angle_rad: [0.0, 0.0, 0.000125],
                    delta_velocity_m_s: [0.0, 0.0, -math::GRAVITY * 0.000125],
                    delta_angle_dt_s: 0.000125,
                    delta_velocity_dt_s: 0.000125,
                    delta_velocity_clipping: [false; 3],
                })
                .unwrap();
        }
        let q = filter.output().unwrap();
        assert!((math::norm(&q) - 1.0).abs() < 2e-6);
        assert!((math::to_axis_angle(q)[2] - 0.5).abs() < 1e-4);
    }

    #[test]
    fn invalid_acceleration_or_clipping_cannot_claim_a_valid_fallback() {
        let mut filter = Mahony::default();
        let sample = ImuDelta {
            timestamp: 125,
            delta_angle_rad: [0.0; 3],
            delta_velocity_m_s: [0.0; 3],
            delta_angle_dt_s: 0.000125,
            delta_velocity_dt_s: 0.000125,
            delta_velocity_clipping: [false; 3],
        };
        assert!(filter.update(&sample).is_err());
        assert!(filter.output().is_none());
        assert!(filter
            .update(&ImuDelta {
                delta_velocity_m_s: [0.0, 0.0, -0.001],
                delta_velocity_clipping: [true; 3],
                ..sample
            })
            .is_err());
    }
}
