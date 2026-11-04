#![forbid(unsafe_code)]

//! Transport-free fixed PX4 MAVLink mappings; firmware streams stay disabled
//! until E7. Host examples use the same DTOs and MAVLink v2 serializer.
//! Copyright (c) 2020-2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.

use mavlink::{common::*, MAVLinkV2MessageRaw, MavHeader};

use crate::estimator::standard_output::{
    AngularVelocity, Attitude, EstimatorStatus, GlobalPosition, LocalPosition, Odometry, Snapshot,
    Source,
};

pub fn attitude(value: Attitude, angular: AngularVelocity) -> MavMessage {
    MavMessage::ATTITUDE_QUATERNION(ATTITUDE_QUATERNION_DATA {
        time_boot_ms: (value.timestamp / 1_000) as u32,
        q1: value.q[0],
        q2: value.q[1],
        q3: value.q[2],
        q4: value.q[3],
        rollspeed: angular.xyz[0],
        pitchspeed: angular.xyz[1],
        yawspeed: angular.xyz[2],
        repr_offset_q: [0.0; 4],
    })
}

/// LOCAL_POSITION_NED has no validity fields. Suppress it instead of presenting
/// an invalid axis (including no-GNSS xy) as a valid navigation measurement.
pub fn local_position(value: LocalPosition) -> Option<MavMessage> {
    (value.xy_valid
        && value.z_valid
        && value.v_xy_valid
        && value.v_z_valid
        && [value.x, value.y, value.z, value.vx, value.vy, value.vz]
            .iter()
            .all(|v| v.is_finite()))
    .then_some(MavMessage::LOCAL_POSITION_NED(LOCAL_POSITION_NED_DATA {
        time_boot_ms: (value.timestamp / 1_000) as u32,
        x: value.x,
        y: value.y,
        z: value.z,
        vx: value.vx,
        vy: value.vy,
        vz: value.vz,
    }))
}

/// Fixed PX4 casts truncate scaled units. Home altitude is separate from the
/// EKF origin; absent home uses AMSL altitude, exactly like the PX4 stream.
pub fn global_position(
    value: GlobalPosition,
    local: LocalPosition,
    home_altitude_m: Option<f32>,
) -> Option<MavMessage> {
    if !value.lat_lon_valid
        || !value.alt_valid
        || !local.v_xy_valid
        || !local.v_z_valid
        || !value.lat.is_finite()
        || !value.lon.is_finite()
        || !value.alt.is_finite()
        || [local.vx, local.vy, local.vz]
            .iter()
            .any(|v| !v.is_finite())
        || home_altitude_m.is_some_and(|altitude| !altitude.is_finite())
    {
        return None;
    }
    Some(MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
        time_boot_ms: (value.timestamp / 1_000) as u32,
        lat: (value.lat * 1e7) as i32,
        lon: (value.lon * 1e7) as i32,
        alt: (value.alt * 1_000.0) as i32,
        relative_alt: ((value.alt - home_altitude_m.unwrap_or(0.0)) * 1_000.0) as i32,
        vx: (local.vx * 100.0) as i16,
        vy: (local.vy * 100.0) as i16,
        vz: (local.vz * 100.0) as i16,
        hdg: if local.heading.is_finite() {
            let mut heading = libm::fmodf(local.heading, core::f32::consts::TAU);
            if heading < 0.0 {
                heading += core::f32::consts::TAU;
            }
            (heading.to_degrees() * 100.0) as u16
        } else {
            u16::MAX
        },
    }))
}

pub fn estimator_status(value: EstimatorStatus) -> MavMessage {
    MavMessage::ESTIMATOR_STATUS(ESTIMATOR_STATUS_DATA {
        time_usec: value.timestamp,
        flags: EstimatorStatusFlags::from_bits_truncate(value.solution_status_flags),
        vel_ratio: value.vel_test_ratio,
        pos_horiz_ratio: value.pos_test_ratio,
        pos_vert_ratio: value.hgt_test_ratio,
        mag_ratio: value.hdg_test_ratio,
        hagl_ratio: value.hagl_test_ratio,
        tas_ratio: value.tas_test_ratio,
        pos_horiz_accuracy: value.pos_horiz_accuracy,
        pos_vert_accuracy: value.pos_vert_accuracy,
    })
}

pub fn odometry(value: Odometry) -> MavMessage {
    let mut pose_covariance = [f32::NAN; 21];
    for (index, variance) in [0, 6, 11].into_iter().zip(value.position_variance) {
        pose_covariance[index] = variance;
    }
    for (index, variance) in [15, 18, 20].into_iter().zip(value.orientation_variance) {
        pose_covariance[index] = variance;
    }
    let mut velocity_covariance = [f32::NAN; 21];
    for (index, variance) in [0, 6, 11].into_iter().zip(value.velocity_variance) {
        velocity_covariance[index] = variance;
    }
    MavMessage::ODOMETRY(ODOMETRY_DATA {
        time_usec: value.timestamp_sample,
        frame_id: MavFrame::MAV_FRAME_LOCAL_NED,
        child_frame_id: MavFrame::MAV_FRAME_LOCAL_NED,
        x: value.position[0],
        y: value.position[1],
        z: value.position[2],
        q: value.q,
        vx: value.velocity[0],
        vy: value.velocity[1],
        vz: value.velocity[2],
        rollspeed: value.angular_velocity[0],
        pitchspeed: value.angular_velocity[1],
        yawspeed: value.angular_velocity[2],
        pose_covariance,
        velocity_covariance,
        reset_counter: value.reset_counter,
        estimator_type: MavEstimatorType::MAV_ESTIMATOR_TYPE_AUTOPILOT,
        quality: value.quality,
    })
}

pub fn messages(snapshot: &Snapshot, home_altitude_m: Option<f32>) -> [Option<MavMessage>; 4] {
    if snapshot.source == Source::NoGnss {
        return [
            snapshot.odometry.map(odometry),
            Some(estimator_status(snapshot.status)),
            None,
            None,
        ];
    }
    [
        snapshot
            .attitude
            .zip(snapshot.angular_velocity)
            .map(|(a, w)| attitude(a, w)),
        local_position(snapshot.local_position),
        snapshot
            .global_position
            .and_then(|global| global_position(global, snapshot.local_position, home_altitude_m)),
        Some(estimator_status(snapshot.status)),
    ]
}

pub fn encode(
    header: MavHeader,
    message: &MavMessage,
    out: &mut [u8; mavlink::MAX_FRAME_SIZE],
) -> usize {
    let mut frame = MAVLinkV2MessageRaw::new();
    frame.serialize_message(header, message);
    let bytes = frame.raw_bytes();
    out[..bytes.len()].copy_from_slice(bytes);
    bytes.len()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::{
        imu_ekf::{baro_mag::BaroMagParams, range::RangeParams, ImuEkfParams},
        no_gnss::NoGnssEkf,
    };

    #[test]
    fn global_units_home_altitude_and_invalid_local_suppression_match_px4() {
        let mut filter = NoGnssEkf::new(
            ImuEkfParams::default(),
            BaroMagParams::default(),
            RangeParams::default(),
            0,
        )
        .unwrap();
        let mut local = filter.standard_output(0).local_position;
        assert!(local_position(local).is_none());
        local.xy_valid = true;
        local.v_xy_valid = true;
        local.z_valid = true;
        local.v_z_valid = true;
        local.vx = 1.239;
        local.vy = -1.239;
        local.vz = 0.125;
        local.heading = -core::f32::consts::FRAC_PI_2;
        let global = GlobalPosition {
            timestamp: 1_234_567,
            timestamp_sample: 1_234_500,
            lat: 47.397742987,
            lon: -8.545594987,
            alt: 100.125,
            alt_ellipsoid: f32::NAN,
            lat_lon_valid: true,
            alt_valid: true,
            delta_alt: 0.0,
            delta_terrain: 0.0,
            lat_lon_reset_counter: 0,
            alt_reset_counter: 0,
            terrain_reset_counter: 0,
            eph: 0.5,
            epv: 0.8,
            terrain_alt: f32::NAN,
            terrain_alt_valid: false,
            dead_reckoning: false,
        };
        let Some(MavMessage::GLOBAL_POSITION_INT(message)) =
            global_position(global, local, Some(90.0))
        else {
            panic!("missing global message");
        };
        assert_eq!(message.time_boot_ms, 1_234);
        assert_eq!((message.lat, message.lon), (473_977_429, -85_455_949));
        assert_eq!((message.alt, message.relative_alt), (100_125, 10_125));
        assert_eq!((message.vx, message.vy, message.vz), (123, -123, 12));
        assert_eq!(message.hdg, 27_000);
        assert!(global_position(
            GlobalPosition {
                lat_lon_valid: false,
                ..global
            },
            local,
            None
        )
        .is_none());
        assert!(global_position(global, local, Some(f32::NAN)).is_none());
        assert!(local_position(LocalPosition {
            vx: f32::NAN,
            ..local
        })
        .is_none());
    }

    #[test]
    fn odometry_keeps_unknown_cross_terms_and_exact_diagonal_slots() {
        let MavMessage::ODOMETRY(message) = odometry(Odometry {
            timestamp_sample: 1_234_567,
            position: [1.0, -2.0, -3.0],
            q: [1.0, 0.0, 0.0, 0.0],
            velocity: [0.5, -0.5, 0.1],
            angular_velocity: [0.0, 0.0, 0.2],
            position_variance: [f32::INFINITY, f32::INFINITY, 9.0],
            velocity_variance: [f32::INFINITY, f32::INFINITY, 4.0],
            orientation_variance: [0.1, 0.2, 0.3],
            reset_counter: 255,
            quality: 0,
        }) else {
            panic!("wrong message");
        };
        assert_eq!(message.time_usec, 1_234_567);
        assert_eq!(message.frame_id, MavFrame::MAV_FRAME_LOCAL_NED);
        assert_eq!(message.child_frame_id, MavFrame::MAV_FRAME_LOCAL_NED);
        for index in [0, 6] {
            assert_eq!(message.pose_covariance[index], f32::INFINITY);
            assert_eq!(message.velocity_covariance[index], f32::INFINITY);
        }
        assert_eq!(message.pose_covariance[11], 9.0);
        assert_eq!(message.velocity_covariance[11], 4.0);
        for (index, value) in [15, 18, 20].into_iter().zip([0.1, 0.2, 0.3]) {
            assert_eq!(message.pose_covariance[index], value);
        }
        assert!(message.pose_covariance[1].is_nan());
        assert!(message.velocity_covariance[20].is_nan());
        assert_eq!(message.reset_counter, 255);
        assert_eq!(message.quality, 0);
    }
}
