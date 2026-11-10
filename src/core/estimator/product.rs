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

const FILTER_POSITION_FLAGS: u16 =
    (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 8) | (1 << 9);

/// A runtime freshness/selection gate can revoke core validity, never invent it.
pub fn constrain_data_validity(snapshot: &mut Snapshot, filter_data_available: bool) {
    let local = &mut snapshot.local_position;
    local.xy_valid &= filter_data_available;
    local.v_xy_valid &= filter_data_available;
    local.z_valid &= filter_data_available;
    local.v_z_valid &= filter_data_available;
    local.dist_bottom_valid &= filter_data_available;
    if !filter_data_available {
        snapshot.global_position = None;
        snapshot.status.solution_status_flags &= !FILTER_POSITION_FLAGS;
    }
}

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
    // Data-only publication follows the core; flight qualification is separate.
    constrain_data_validity(
        &mut snapshot,
        status.selection == Selection::Filter && attitude.is_some(),
    );
    // Publishing valid position/heading data never grants control permission.
    snapshot.local_position.heading_good_for_control = false;
    snapshot
}
