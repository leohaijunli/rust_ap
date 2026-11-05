//! Compile-time algorithm choice and PX4 publication adapter. Scheduling lives
//! in Runtime<NavFilter>; the board driver imports only these product aliases.

use super::{
    mahony::Mahony,
    no_gnss::NoGnssEkf,
    publication_reset::PublicationReset,
    runtime::{Runtime, Selection},
    standard_output::Snapshot,
};

pub use super::imu_downsampler::{ClockReport, SharedImu};
pub use super::runtime_params::Params;

#[cfg(not(feature = "estimator-mahony"))]
pub type ProductFilter = super::gnss::GnssEkf;
#[cfg(feature = "estimator-mahony")]
pub type ProductFilter = Mahony;
pub type ProductRuntime = Runtime<ProductFilter>;
pub type IndependentRuntime = Runtime<NoGnssEkf>;
pub type MahonyRuntime = Runtime<Mahony>;

pub const fn main_storage() -> ProductRuntime {
    Runtime::with_initializer(
        ProductFilter::dormant(),
        ProductFilter::configure_from_snapshot,
    )
}
pub const fn independent_storage() -> IndependentRuntime {
    Runtime::with_initializer(NoGnssEkf::dormant(), NoGnssEkf::configure_from_snapshot)
}

pub fn snapshots(
    main: &mut ProductRuntime,
    independent: &mut IndependentRuntime,
    now: u64,
    resets: &mut [PublicationReset; 2],
) -> [Snapshot; 2] {
    let mut snapshots = [
        main.filter_mut().standard_output(now),
        independent.filter_mut().standard_output(now),
    ];
    adapt(main, &mut snapshots[0], now, &mut resets[0]);
    adapt(independent, &mut snapshots[1], now, &mut resets[1]);
    snapshots
}

fn adapt<F: super::NavFilter>(
    runtime: &mut Runtime<F>,
    snapshot: &mut Snapshot,
    now: u64,
    resets: &mut PublicationReset,
) {
    let status = runtime.status();
    let source = match status.selection {
        Selection::Invalid => 0,
        Selection::Filter => 1,
        Selection::Mahony => 2,
    };
    resets.apply(snapshot, status.generation, source);
    let attitude = runtime.attitude_at(now);
    if status.selection != Selection::Filter {
        snapshot.odometry = None;
    }
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
    // E7 bring-up has not qualified navigation. Preserve numeric diagnostics
    // with explicit invalidity until the existing hardware gates are passed.
    snapshot.local_position.xy_valid = false;
    snapshot.local_position.v_xy_valid = false;
    snapshot.local_position.z_valid = false;
    snapshot.local_position.v_z_valid = false;
    snapshot.local_position.heading_good_for_control = false;
    snapshot.global_position = None;
    snapshot.status.solution_status_flags &= !0x7e;
    if let Some(odometry) = &mut snapshot.odometry {
        odometry.reset_counter = [
            snapshot.status.reset_count_quat,
            snapshot.status.reset_count_vel_ne,
            snapshot.status.reset_count_vel_d,
            snapshot.status.reset_count_pos_ne,
            snapshot.status.reset_count_pod_d,
        ]
        .into_iter()
        .fold(0_u8, u8::wrapping_add);
        odometry.position_variance = [f32::INFINITY; 3];
        odometry.velocity_variance = [f32::INFINITY; 3];
        if attitude.is_some_and(|v| v.relative_yaw) {
            odometry.orientation_variance[2] = f32::INFINITY;
        }
        if attitude.is_none() {
            snapshot.odometry = None;
        }
    }
}
