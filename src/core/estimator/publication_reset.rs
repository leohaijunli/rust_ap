//! Preserve visible reset counters/deltas when a task restarts or switches its
//! selected attitude source. This supplements, not replaces, core reset events.

use super::{math, standard_output::Snapshot};

#[derive(Clone, Copy)]
struct Previous {
    epoch: u32,
    source: u8,
    counts: [u8; 6],
    q: [f32; 4],
    velocity: [f32; 3],
    position: [f32; 3],
    bottom: f32,
}

#[derive(Default)]
pub struct PublicationReset {
    previous: Option<Previous>,
    offsets: [u8; 6],
}

impl PublicationReset {
    pub fn apply(&mut self, snapshot: &mut Snapshot, epoch: u32, source: u8) {
        let Some(attitude) = snapshot.attitude else {
            return;
        };
        let local = snapshot.local_position;
        let raw_counts = [
            attitude.quat_reset_counter,
            local.vxy_reset_counter,
            local.vz_reset_counter,
            local.xy_reset_counter,
            local.z_reset_counter,
            local.dist_bottom_reset_counter,
        ];
        let velocity = [local.vx, local.vy, local.vz];
        let position = [local.x, local.y, local.z];
        if let Some(previous) = self.previous {
            if previous.epoch != epoch || previous.source != source {
                for (index, raw) in raw_counts.into_iter().enumerate() {
                    self.offsets[index] = previous.counts[index].wrapping_add(1).wrapping_sub(raw);
                }
                let delta_q =
                    math::normalize(math::multiply(attitude.q, math::inverse(previous.q)));
                snapshot.attitude.as_mut().unwrap().delta_q_reset = delta_q;
                snapshot.local_position.delta_heading = math::yaw(math::rotation(delta_q));
                snapshot.local_position.delta_vxy = [
                    velocity[0] - previous.velocity[0],
                    velocity[1] - previous.velocity[1],
                ];
                snapshot.local_position.delta_vz = velocity[2] - previous.velocity[2];
                snapshot.local_position.delta_xy = [
                    position[0] - previous.position[0],
                    position[1] - previous.position[1],
                ];
                snapshot.local_position.delta_z = position[2] - previous.position[2];
                snapshot.local_position.delta_dist_bottom = local.dist_bottom - previous.bottom;
            }
        }
        let counts = core::array::from_fn(|i| raw_counts[i].wrapping_add(self.offsets[i]));
        snapshot.attitude.as_mut().unwrap().quat_reset_counter = counts[0];
        snapshot.local_position.heading_reset_counter = counts[0];
        snapshot.local_position.vxy_reset_counter = counts[1];
        snapshot.local_position.vz_reset_counter = counts[2];
        snapshot.local_position.xy_reset_counter = counts[3];
        snapshot.local_position.z_reset_counter = counts[4];
        snapshot.local_position.dist_bottom_reset_counter = counts[5];
        snapshot.status.reset_count_quat = counts[0];
        snapshot.status.reset_count_vel_ne = counts[1];
        snapshot.status.reset_count_vel_d = counts[2];
        snapshot.status.reset_count_pos_ne = counts[3];
        snapshot.status.reset_count_pod_d = counts[4];
        self.previous = Some(Previous {
            epoch,
            source,
            counts,
            q: attitude.q,
            velocity,
            position,
            bottom: local.dist_bottom,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        estimator::{product::ProductRuntime, runtime_params::Params, ImuDelta},
        parameters::{Id, Snapshot as Parameters},
    };

    #[test]
    fn task_restart_preserves_counters_and_reports_real_output_deltas() {
        let params =
            Params::from_parameters(Parameters::default().with(Id::EstEnable, 1.0).unwrap())
                .unwrap();
        let mut runtime = ProductRuntime::new(params, 0).unwrap();
        runtime
            .ingest_imu(
                ImuDelta {
                    timestamp: 1_000_000,
                    delta_angle_rad: [0.0; 3],
                    delta_velocity_m_s: [0.0, 0.0, -9.80665 * 0.001],
                    delta_angle_dt_s: 0.001,
                    delta_velocity_dt_s: 0.001,
                    delta_velocity_clipping: [false; 3],
                },
                1_000_010,
            )
            .unwrap();
        runtime.maintain(1_000_010);
        let mut snapshot = runtime.filter_mut().standard_output(1_000_020);
        snapshot.attitude = runtime.attitude_at(1_000_020).map(|v| v.attitude);
        let mut tracker = PublicationReset::default();
        tracker.apply(&mut snapshot, 1, 2);
        let mut next = snapshot;
        next.attitude.as_mut().unwrap().q = math::from_axis_angle([0.0, 0.0, 0.25]);
        next.attitude.as_mut().unwrap().quat_reset_counter = 0;
        next.local_position.x = 2.0;
        next.local_position.vx = 0.5;
        tracker.apply(&mut next, 2, 2);
        assert_eq!(
            next.attitude.unwrap().quat_reset_counter,
            snapshot
                .attitude
                .unwrap()
                .quat_reset_counter
                .wrapping_add(1)
        );
        assert!((math::to_axis_angle(next.attitude.unwrap().delta_q_reset)[2] - 0.25).abs() < 1e-6);
        assert_eq!(
            next.local_position.delta_xy[0],
            2.0 - snapshot.local_position.x
        );
        assert_eq!(
            next.local_position.delta_vxy[0],
            0.5 - snapshot.local_position.vx
        );
    }
}
