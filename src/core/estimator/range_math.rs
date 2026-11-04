//! Pinned PX4 HAGL observation mathematics, including the terrain correlation.
//! Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.
//! Source: generated/compute_hagl_{h,innov_var}.h. Sensor validity, mounting
//! correction and the range aid state machine remain separate responsibilities.

use super::{Covariance, FilterFault, ScalarUpdate, StateVector};

pub fn hagl_update(
    state: &StateVector,
    covariance: &Covariance,
    distance_vertical_m: f32,
    observation_variance_m2: f32,
) -> Result<ScalarUpdate, FilterFault> {
    if !distance_vertical_m.is_finite()
        || distance_vertical_m < 0.0
        || !observation_variance_m2.is_finite()
        || observation_variance_m2 <= 0.0
    {
        return Err(FilterFault::Input);
    }
    let innovation = state[24] - state[9] - distance_vertical_m;
    let variance = covariance[23][23] - covariance[23][8] - covariance[8][23]
        + covariance[8][8]
        + observation_variance_m2;
    if !innovation.is_finite() || !variance.is_finite() || variance < observation_variance_m2 {
        return Err(FilterFault::Numerics);
    }
    let mut observation = [0.0; 24];
    observation[8] = -1.0;
    observation[23] = 1.0;
    Ok(ScalarUpdate {
        innovation,
        innovation_variance: variance,
        observation,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hagl_innovation_variance_retains_height_terrain_cross_covariance() {
        let mut state = [0.0; 25];
        state[9] = -100.0;
        state[24] = -98.0;
        let mut covariance = [[0.0; 24]; 24];
        covariance[8][8] = 2.0;
        covariance[23][23] = 3.0;
        covariance[8][23] = 0.5;
        covariance[23][8] = 0.5;
        let update = hagl_update(&state, &covariance, 1.5, 0.25).unwrap();
        assert_eq!(update.innovation, 0.5);
        assert_eq!(update.innovation_variance, 4.25);
        assert_eq!(update.observation[8], -1.0);
        assert_eq!(update.observation[23], 1.0);
        covariance[8][23] = 10.0;
        assert_eq!(
            hagl_update(&state, &covariance, 1.5, 0.25),
            Err(FilterFault::Numerics)
        );
        assert_eq!(
            hagl_update(&state, &covariance, f32::NAN, 0.25),
            Err(FilterFault::Input)
        );
    }
}
