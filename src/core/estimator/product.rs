//! Compile-time algorithm choice and PX4 publication adapter. Scheduling lives
//! in Runtime<NavFilter>; the board driver imports only these product aliases.

#[cfg(feature = "estimator-mahony")]
use super::mahony::Mahony;
use super::{
    publication_reset::PublicationReset,
    runtime::{Runtime, Selection},
    standard_output::Snapshot,
};

pub use super::imu_input::{ClockReport, SharedImu};
pub use super::runtime_params::Params;

#[cfg(not(feature = "estimator-mahony"))]
pub type ProductFilter = super::gnss::GnssEkf;
#[cfg(feature = "estimator-mahony")]
pub type ProductFilter = Mahony;
pub type ProductRuntime = Runtime<ProductFilter>;

pub const fn main_storage() -> ProductRuntime {
    Runtime::with_initializer(
        ProductFilter::dormant(),
        ProductFilter::configure_from_snapshot,
    )
}

pub fn snapshot(runtime: &mut ProductRuntime, now: u64, resets: &mut PublicationReset) -> Snapshot {
    let mut snapshot = runtime.filter_mut().standard_output(now);
    let status = runtime.status();
    let source = match status.selection {
        Selection::Invalid => 0,
        Selection::Filter => 1,
        Selection::Mahony => 2,
    };
    resets.apply(&mut snapshot, status.generation, source);
    let attitude = runtime.attitude_at(now);
    snapshot.attitude = attitude.map(|v| v.attitude);
    snapshot.angular_velocity = attitude.map(|v| v.angular_velocity);
    snapshot.status.solution_status_flags &= !1;
    if let Some(estimate) = attitude {
        let q_reset = estimate.attitude.quat_reset_counter;
        snapshot.status.solution_status_flags |= 1;
        snapshot.status.reset_count_quat = q_reset;
        snapshot.local_position.heading_reset_counter = q_reset;
        snapshot.local_position.heading =
            super::math::yaw(super::math::rotation(estimate.attitude.q));
        if estimate.relative_yaw {
            snapshot.local_position.heading_var = f32::INFINITY;
        }
    }
    // Height telemetry follows the core's aiding validity, but never borrows
    // EKF height while the runtime is disabled, stale or using Mahony fallback.
    let vertical_available = status.selection == Selection::Filter && attitude.is_some();
    snapshot.local_position.z_valid &= vertical_available;
    snapshot.local_position.v_z_valid &= vertical_available;
    snapshot.local_position.dist_bottom_valid &= vertical_available;
    if !vertical_available {
        snapshot.status.solution_status_flags &= !((1 << 2) | (1 << 5) | (1 << 6));
    }
    // E7 navigation/control remain unqualified. Reporting measured height does
    // not authorize horizontal/global navigation or heading-based control.
    snapshot.local_position.xy_valid = false;
    snapshot.local_position.v_xy_valid = false;
    snapshot.local_position.heading_good_for_control = false;
    snapshot.global_position = None;
    snapshot.status.solution_status_flags &= !((1 << 1) | (1 << 3) | (1 << 4));
    snapshot
}
