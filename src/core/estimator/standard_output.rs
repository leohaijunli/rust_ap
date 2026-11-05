//! PX4 v1.17.0 estimator publication DTOs and validity/covariance mapping.
//! Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.
//! Field sources and deliberately unknown diagnostics: docs/message-contracts.md.

use super::{
    imu_ekf::{baro_mag::AidStatus, ImuEkf, IMU_FRESH_US},
    math,
    reset_status::ResetReport,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    Gnss,
    NoGnss,
}

impl Source {
    pub const fn instance(self) -> u8 {
        match self {
            Self::Gnss => 0,
            Self::NoGnss => 1,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Attitude {
    pub timestamp: u64,
    pub timestamp_sample: u64,
    pub q: [f32; 4],
    pub delta_q_reset: [f32; 4],
    pub quat_reset_counter: u8,
}

/// FRD body to local-world intrinsic ZYX angles, in radians. Derived once from
/// the selected quaternion; relative yaw does not establish a north reference.
#[derive(Clone, Copy, Debug)]
pub struct EulerAttitude {
    pub timestamp: u64,
    pub timestamp_sample: u64,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub quat_reset_counter: u8,
    pub relative_yaw: bool,
}

impl EulerAttitude {
    pub fn from_attitude(attitude: Attitude, relative_yaw: bool) -> Option<Self> {
        let q = attitude.q;
        if q.iter().any(|value| !value.is_finite())
            || (q.iter().map(|value| value * value).sum::<f32>() - 1.0).abs() > 1e-3
        {
            return None;
        }
        let rotation = math::rotation(q);
        Some(Self {
            timestamp: attitude.timestamp,
            timestamp_sample: attitude.timestamp_sample,
            roll: libm::atan2f(rotation[2][1], rotation[2][2]),
            pitch: libm::asinf((-rotation[2][0]).clamp(-1.0, 1.0)),
            yaw: libm::atan2f(rotation[1][0], rotation[0][0]),
            quat_reset_counter: attitude.quat_reset_counter,
            relative_yaw,
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AngularVelocity {
    pub timestamp: u64,
    pub timestamp_sample: u64,
    pub xyz: [f32; 3],
    pub xyz_derivative: [f32; 3],
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LocalPosition {
    pub timestamp: u64,
    pub timestamp_sample: u64,
    pub xy_valid: bool,
    pub z_valid: bool,
    pub v_xy_valid: bool,
    pub v_z_valid: bool,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub delta_xy: [f32; 2],
    pub xy_reset_counter: u8,
    pub delta_z: f32,
    pub z_reset_counter: u8,
    pub vx: f32,
    pub vy: f32,
    pub vz: f32,
    pub z_deriv: f32,
    pub delta_vxy: [f32; 2],
    pub vxy_reset_counter: u8,
    pub delta_vz: f32,
    pub vz_reset_counter: u8,
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub heading: f32,
    pub heading_var: f32,
    pub unaided_heading: f32,
    pub delta_heading: f32,
    pub heading_reset_counter: u8,
    pub heading_good_for_control: bool,
    pub tilt_var: f32,
    pub xy_global: bool,
    pub z_global: bool,
    pub ref_timestamp: u64,
    pub ref_lat: f64,
    pub ref_lon: f64,
    pub ref_alt: f32,
    pub dist_bottom_valid: bool,
    pub dist_bottom: f32,
    pub dist_bottom_var: f32,
    pub delta_dist_bottom: f32,
    pub dist_bottom_reset_counter: u8,
    pub dist_bottom_sensor_bitfield: u8,
    pub eph: f32,
    pub epv: f32,
    pub evh: f32,
    pub evv: f32,
    pub dead_reckoning: bool,
    pub vxy_max: f32,
    pub vz_max: f32,
    pub hagl_min: f32,
    pub hagl_max_z: f32,
    pub hagl_max_xy: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct GlobalPosition {
    pub timestamp: u64,
    pub timestamp_sample: u64,
    pub lat: f64,
    pub lon: f64,
    pub alt: f32,
    pub alt_ellipsoid: f32,
    pub lat_lon_valid: bool,
    pub alt_valid: bool,
    pub delta_alt: f32,
    pub delta_terrain: f32,
    pub lat_lon_reset_counter: u8,
    pub alt_reset_counter: u8,
    pub terrain_reset_counter: u8,
    pub eph: f32,
    pub epv: f32,
    pub terrain_alt: f32,
    pub terrain_alt_valid: bool,
    pub dead_reckoning: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EstimatorStatus {
    pub timestamp: u64,
    pub timestamp_sample: u64,
    pub output_tracking_error: [f32; 3],
    pub gps_check_fail_flags: u16,
    pub control_mode_flags: u64,
    pub filter_fault_flags: u32,
    pub pos_horiz_accuracy: f32,
    pub pos_vert_accuracy: f32,
    pub hdg_test_ratio: f32,
    pub vel_test_ratio: f32,
    pub pos_test_ratio: f32,
    pub hgt_test_ratio: f32,
    pub tas_test_ratio: f32,
    pub hagl_test_ratio: f32,
    pub beta_test_ratio: f32,
    pub solution_status_flags: u16,
    pub reset_count_vel_ne: u8,
    pub reset_count_vel_d: u8,
    pub reset_count_pos_ne: u8,
    /// The spelling is fixed by PX4 EstimatorStatus.msg.
    pub reset_count_pod_d: u8,
    pub reset_count_quat: u8,
    pub time_slip: f32,
    pub pre_flt_fail_innov_heading: bool,
    pub pre_flt_fail_innov_height: bool,
    pub pre_flt_fail_innov_pos_horiz: bool,
    pub pre_flt_fail_innov_vel_horiz: bool,
    pub pre_flt_fail_innov_vel_vert: bool,
    pub pre_flt_fail_mag_field_disturbed: bool,
    pub accel_device_id: u32,
    pub gyro_device_id: u32,
    pub baro_device_id: u32,
    pub mag_device_id: u32,
    pub health_flags: u8,
    pub timeout_flags: u8,
    pub mag_inclination_deg: f32,
    pub mag_inclination_ref_deg: f32,
    pub mag_strength_gs: f32,
    pub mag_strength_ref_gs: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct EstimatorStates {
    pub timestamp: u64,
    pub timestamp_sample: u64,
    pub states: [f32; 25],
    pub n_states: u8,
    pub covariances: [f32; 24],
}

/// Known raw diagnostics are kept separate from unavailable PX4 LPF/observation
/// metadata. The ULog adapter supplies NaN/zero for those unknown fields.
#[derive(Clone, Copy, Debug)]
pub struct AidSource {
    pub timestamp: u64,
    pub estimator_instance: u8,
    pub device_id: u32,
    pub status: AidStatus,
}

#[derive(Clone, Copy, Debug)]
pub struct Odometry {
    pub timestamp_sample: u64,
    pub position: [f32; 3],
    pub q: [f32; 4],
    pub velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
    pub position_variance: [f32; 3],
    pub orientation_variance: [f32; 3],
    pub velocity_variance: [f32; 3],
    pub reset_counter: u8,
    pub quality: i8,
}

#[derive(Clone, Copy, Debug)]
pub struct Snapshot {
    pub source: Source,
    pub attitude: Option<Attitude>,
    pub angular_velocity: Option<AngularVelocity>,
    pub local_position: LocalPosition,
    pub global_position: Option<GlobalPosition>,
    pub status: EstimatorStatus,
    pub states: EstimatorStates,
    /// Barometer, magnetometer, range, GNSS position, velocity, height.
    pub aids: [Option<AidSource>; 6],
    pub odometry: Option<Odometry>,
}

pub(super) struct GnssDiagnostics {
    pub aids: [AidStatus; 3],
    pub quality_failures: u16,
}

impl Snapshot {
    /// Generic attitude-only algorithms expose unknown navigation/uncertainty,
    /// rather than manufacturing an EKF state or finite position accuracy.
    pub fn attitude_only(output: super::NavOutput, health: super::FilterHealth, now: u64) -> Self {
        let live =
            health.valid && now >= output.timestamp && now - output.timestamp <= IMU_FRESH_US;
        Self {
            source: Source::Gnss,
            attitude: live.then_some(Attitude {
                timestamp: now,
                timestamp_sample: output.timestamp,
                q: output.q_body_to_ned,
                delta_q_reset: output.delta_q_reset,
                quat_reset_counter: output.quat_reset_counter,
            }),
            angular_velocity: live.then_some(AngularVelocity {
                timestamp: now,
                timestamp_sample: output.timestamp,
                xyz: output.angular_velocity_body_rad_s,
                xyz_derivative: [f32::NAN; 3],
            }),
            local_position: LocalPosition {
                timestamp: now,
                timestamp_sample: output.timestamp,
                x: f32::NAN,
                y: f32::NAN,
                z: f32::NAN,
                vx: f32::NAN,
                vy: f32::NAN,
                vz: f32::NAN,
                z_deriv: f32::NAN,
                ax: f32::NAN,
                ay: f32::NAN,
                az: f32::NAN,
                heading: math::yaw(math::rotation(output.q_body_to_ned)),
                heading_var: f32::INFINITY,
                tilt_var: f32::NAN,
                unaided_heading: f32::NAN,
                ref_lat: f64::NAN,
                ref_lon: f64::NAN,
                ref_alt: f32::NAN,
                dist_bottom: f32::NAN,
                dist_bottom_var: f32::INFINITY,
                eph: f32::INFINITY,
                epv: f32::INFINITY,
                evh: f32::INFINITY,
                evv: f32::INFINITY,
                vxy_max: f32::NAN,
                vz_max: f32::NAN,
                hagl_min: f32::NAN,
                hagl_max_z: f32::NAN,
                hagl_max_xy: f32::NAN,
                ..LocalPosition::default()
            },
            global_position: None,
            status: EstimatorStatus {
                timestamp: now,
                timestamp_sample: output.timestamp,
                solution_status_flags: u16::from(live),
                output_tracking_error: [f32::NAN; 3],
                pos_horiz_accuracy: f32::INFINITY,
                pos_vert_accuracy: f32::INFINITY,
                hdg_test_ratio: f32::NAN,
                vel_test_ratio: f32::NAN,
                pos_test_ratio: f32::NAN,
                hgt_test_ratio: f32::NAN,
                tas_test_ratio: f32::NAN,
                hagl_test_ratio: f32::NAN,
                beta_test_ratio: f32::NAN,
                time_slip: f32::NAN,
                mag_inclination_deg: f32::NAN,
                mag_inclination_ref_deg: f32::NAN,
                mag_strength_gs: f32::NAN,
                mag_strength_ref_gs: f32::NAN,
                ..EstimatorStatus::default()
            },
            states: EstimatorStates {
                timestamp: now,
                timestamp_sample: output.timestamp,
                states: [f32::NAN; 25],
                n_states: 0,
                covariances: [f32::NAN; 24],
            },
            aids: [None; 6],
            odometry: None,
        }
    }

    /// Capture once per publication cycle: reset the integral accumulators once,
    /// then share this immutable snapshot between ULog and both MAVLink links.
    pub(super) fn from_core(
        core: &mut ImuEkf,
        now: u64,
        source: Source,
        gnss: Option<GnssDiagnostics>,
    ) -> Self {
        let output = core.output();
        let resets = core.reset_report();
        let covariances = core::array::from_fn(|i| core.covariance()[i][i]);
        let live = core.health().valid
            && now >= output.timestamp
            && now - output.timestamp <= IMU_FRESH_US;
        let validity = core.local_position_validity();
        let horizontal_valid = live && source == Source::Gnss && validity[0];
        let vertical_valid = live && (validity[1] || validity[2]);
        let control = core.control_status();
        let global_reference = if source == Source::Gnss {
            core.origin.zip(core.origin_altitude)
        } else {
            None
        };
        let global_origin = global_reference.is_some();
        let heading_good = live
            && control & (1 << 1) != 0
            && (control & ((1 << 4) | (1 << 5)) == 0
                || (control & (1 << 23) != 0
                    && core.timestamp().saturating_sub(core.mag_alignment_start) > 1_000_000));
        let angular_velocity = core.angular_velocity_and_reset();
        let acceleration = core.velocity_derivative_and_reset();
        let yaw = |q| {
            let rotation = math::rotation(q);
            libm::atan2f(rotation[1][0], rotation[0][0])
        };
        let mut local = LocalPosition {
            timestamp: now,
            timestamp_sample: output.timestamp,
            xy_valid: horizontal_valid,
            z_valid: vertical_valid,
            v_xy_valid: horizontal_valid,
            v_z_valid: vertical_valid,
            x: output.position_ned_m[0],
            y: output.position_ned_m[1],
            z: output.position_ned_m[2],
            delta_xy: [resets.position_delta[0], resets.position_delta[1]],
            xy_reset_counter: resets.counts[3],
            delta_z: resets.position_delta[2],
            z_reset_counter: resets.counts[4],
            vx: output.velocity_ned_m_s[0],
            vy: output.velocity_ned_m_s[1],
            vz: output.velocity_ned_m_s[2],
            z_deriv: core.output_vertical_derivative(),
            delta_vxy: [resets.velocity_delta[0], resets.velocity_delta[1]],
            vxy_reset_counter: resets.counts[1],
            delta_vz: resets.velocity_delta[2],
            vz_reset_counter: resets.counts[2],
            ax: acceleration[0],
            ay: acceleration[1],
            az: acceleration[2],
            heading: yaw(output.q_body_to_ned),
            heading_var: covariances[2],
            unaided_heading: core.output_unaided_yaw(),
            delta_heading: yaw(resets.quaternion_delta),
            heading_reset_counter: resets.counts[0],
            heading_good_for_control: heading_good,
            tilt_var: covariances[0] + covariances[1],
            xy_global: global_origin,
            z_global: global_origin,
            ref_timestamp: if global_origin {
                core.origin_timestamp
            } else {
                0
            },
            ref_lat: f64::NAN,
            ref_lon: f64::NAN,
            ref_alt: f32::NAN,
            dist_bottom_valid: live && core.terrain_valid,
            dist_bottom: (core.state()[24] + core.global.altitude_m).max(0.0),
            dist_bottom_var: covariances[23],
            delta_dist_bottom: resets.hagl_delta,
            dist_bottom_reset_counter: resets.counts[5],
            dist_bottom_sensor_bitfield: u8::from(control & (1 << 39) != 0),
            eph: libm::sqrtf(covariances[6] + covariances[7]),
            epv: libm::sqrtf(covariances[8]),
            evh: libm::sqrtf(covariances[3] + covariances[4]),
            evv: libm::sqrtf(covariances[5]),
            dead_reckoning: control & ((1 << 29) | (1 << 30)) != 0,
            vxy_max: f32::INFINITY,
            vz_max: f32::INFINITY,
            hagl_min: f32::INFINITY,
            hagl_max_z: f32::INFINITY,
            hagl_max_xy: f32::INFINITY,
        };
        if let Some((origin, altitude)) = global_reference {
            local.ref_lat = origin.latitude_rad.to_degrees();
            local.ref_lon = origin.longitude_rad.to_degrees();
            local.ref_alt = altitude;
        }
        if let Some([minimum, maximum]) = core.range_height_limits() {
            local.hagl_min = minimum;
            local.hagl_max_z = maximum;
        }
        if !validity[0] {
            if let Some(diagnostics) = &gnss {
                if control & (1 << 2) != 0 {
                    local.eph = local
                        .eph
                        .max(math::norm(&diagnostics.aids[0].innovation[..2]));
                }
                if control & (1 << 44) != 0 {
                    local.evh = local.evh.max(math::norm(&diagnostics.aids[1].innovation));
                }
            }
        }
        let attitude = live.then_some(Attitude {
            timestamp: now,
            timestamp_sample: output.timestamp,
            q: output.q_body_to_ned,
            delta_q_reset: resets.quaternion_delta,
            quat_reset_counter: resets.counts[0],
        });
        let angular = live.then_some(AngularVelocity {
            timestamp: now,
            timestamp_sample: output.timestamp,
            xyz: angular_velocity,
            xyz_derivative: [f32::NAN; 3],
        });
        let global_position =
            (global_origin && horizontal_valid && live && validity[1] && control & (1 << 1) != 0)
                .then(|| {
                    let (lat, lon, alt) = core.output_global_position();
                    GlobalPosition {
                        timestamp: now,
                        timestamp_sample: output.timestamp,
                        lat,
                        lon,
                        alt,
                        alt_ellipsoid: f32::NAN,
                        lat_lon_valid: true,
                        alt_valid: true,
                        delta_alt: -resets.position_delta[2],
                        // Preserve EKF2.cpp's fixed delta_terrain=-delta_z mapping.
                        delta_terrain: -resets.position_delta[2],
                        lat_lon_reset_counter: resets.counts[3],
                        alt_reset_counter: resets.counts[4],
                        terrain_reset_counter: resets.counts[5],
                        eph: local.eph,
                        epv: local.epv,
                        terrain_alt: -core.state()[24],
                        terrain_alt_valid: live && core.terrain_valid,
                        dead_reckoning: local.dead_reckoning,
                    }
                });
        let mut aids = [None; 6];
        let aid = |status: AidStatus| {
            (status.timestamp != 0).then_some(AidSource {
                timestamp: now,
                estimator_instance: source.instance(),
                device_id: 0,
                status,
            })
        };
        if let Some((baro, mag)) = core.aid_status() {
            aids[0] = aid(baro);
            aids[1] = aid(mag);
        }
        aids[2] = core.range_status().and_then(aid);
        if let Some(diagnostics) = &gnss {
            for (destination, status) in aids[3..].iter_mut().zip(diagnostics.aids) {
                *destination = aid(status);
            }
        }
        let status = EstimatorStatus {
            timestamp: now,
            timestamp_sample: core.timestamp(),
            output_tracking_error: core.output_tracking_error(),
            gps_check_fail_flags: gnss.as_ref().map_or(0, |d| d.quality_failures),
            control_mode_flags: control,
            filter_fault_flags: core.fault_status(),
            pos_horiz_accuracy: if global_origin {
                local.eph
            } else {
                f32::INFINITY
            },
            pos_vert_accuracy: if global_origin {
                local.epv
            } else {
                f32::INFINITY
            },
            hdg_test_ratio: f32::NAN,
            vel_test_ratio: f32::NAN,
            pos_test_ratio: f32::NAN,
            hgt_test_ratio: f32::NAN,
            tas_test_ratio: f32::NAN,
            hagl_test_ratio: f32::NAN,
            beta_test_ratio: f32::NAN,
            solution_status_flags: solution_status(core, &local, gnss.as_ref(), live),
            reset_count_vel_ne: resets.counts[1],
            reset_count_vel_d: resets.counts[2],
            reset_count_pos_ne: resets.counts[3],
            reset_count_pod_d: resets.counts[4],
            reset_count_quat: resets.counts[0],
            time_slip: f32::NAN,
            pre_flt_fail_innov_heading: false,
            pre_flt_fail_innov_height: false,
            pre_flt_fail_innov_pos_horiz: false,
            pre_flt_fail_innov_vel_horiz: false,
            pre_flt_fail_innov_vel_vert: false,
            pre_flt_fail_mag_field_disturbed: control & (1 << 16) != 0,
            accel_device_id: 0,
            gyro_device_id: 0,
            baro_device_id: 0,
            mag_device_id: 0,
            health_flags: 0,
            timeout_flags: 0,
            mag_inclination_deg: f32::NAN,
            mag_inclination_ref_deg: f32::NAN,
            mag_strength_gs: f32::NAN,
            mag_strength_ref_gs: f32::NAN,
        };
        let rotation = math::rotation(core.quaternion());
        let orientation_variance = core::array::from_fn(|axis| {
            let mut variance = 0.0;
            for i in 0..3 {
                for j in 0..3 {
                    variance += rotation[i][axis] * core.covariance()[i][j] * rotation[j][axis];
                }
            }
            variance
        });
        let odometry = live.then_some(Odometry {
            timestamp_sample: output.timestamp,
            position: output.position_ned_m,
            q: output.q_body_to_ned,
            velocity: output.velocity_ned_m_s,
            angular_velocity,
            position_variance: [
                if horizontal_valid {
                    covariances[6]
                } else {
                    f32::INFINITY
                },
                if horizontal_valid {
                    covariances[7]
                } else {
                    f32::INFINITY
                },
                if local.z_valid {
                    covariances[8]
                } else {
                    f32::INFINITY
                },
            ],
            orientation_variance,
            velocity_variance: [
                if horizontal_valid {
                    covariances[3]
                } else {
                    f32::INFINITY
                },
                if horizontal_valid {
                    covariances[4]
                } else {
                    f32::INFINITY
                },
                if local.v_z_valid {
                    covariances[5]
                } else {
                    f32::INFINITY
                },
            ],
            reset_counter: odometry_reset_counter(resets),
            quality: 0,
        });
        Self {
            source,
            attitude,
            angular_velocity: angular,
            local_position: local,
            global_position,
            status,
            states: EstimatorStates {
                timestamp: now,
                timestamp_sample: core.timestamp(),
                states: *core.state(),
                n_states: 25,
                covariances,
            },
            aids,
            odometry,
        }
    }
}

fn solution_status(
    core: &ImuEkf,
    local: &LocalPosition,
    gnss: Option<&GnssDiagnostics>,
    live: bool,
) -> u16 {
    let control = core.control_status();
    let glitch = gnss.is_some_and(|d| {
        d.aids[0].ratio[..2]
            .iter()
            .chain(&d.aids[1].ratio)
            .any(|r| *r > 1.0)
    });
    let bits = [
        live && control & 1 != 0,
        local.xy_valid,
        local.v_z_valid,
        local.xy_valid,
        local.xy_global && local.xy_valid,
        live && control & ((1 << 9) | (1 << 10) | (1 << 11) | (1 << 44)) != 0,
        local.dist_bottom_valid,
        control & ((1 << 32) | (1 << 41) | (1 << 26)) != 0,
        live && core.horizontal_aiding(),
        live && control & (1 << 2) != 0,
        glitch,
        core.fault_status() & ((1 << 10) | (1 << 11)) != 0,
    ];
    bits.iter()
        .enumerate()
        .fold(0, |flags, (index, set)| flags | (u16::from(*set) << index))
}

fn odometry_reset_counter(resets: ResetReport) -> u8 {
    resets.counts[..5]
        .iter()
        .fold(0_u8, |sum, count| sum.wrapping_add(*count))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::{imu_ekf::ImuEkfParams, ImuDelta};

    #[test]
    fn euler_angles_keep_zyx_axes_radians_and_reject_invalid_quaternions() {
        let mut attitude = Attitude {
            timestamp: 1_234_567,
            timestamp_sample: 1_234_500,
            q: math::multiply(
                math::multiply(
                    math::from_axis_angle([0.0, 0.0, 1.1]),
                    math::from_axis_angle([0.0, -0.25, 0.0]),
                ),
                math::from_axis_angle([0.35, 0.0, 0.0]),
            ),
            delta_q_reset: math::IDENTITY,
            quat_reset_counter: 255,
        };
        let euler = EulerAttitude::from_attitude(attitude, true).unwrap();
        for (actual, expected) in [euler.roll, euler.pitch, euler.yaw]
            .into_iter()
            .zip([0.35, -0.25, 1.1])
        {
            assert!((actual - expected).abs() < 1e-6);
        }
        assert_eq!(
            (euler.timestamp, euler.timestamp_sample),
            (1_234_567, 1_234_500)
        );
        assert_eq!(euler.quat_reset_counter, 255);
        assert!(euler.relative_yaw);
        attitude.q = [0.0; 4];
        assert!(EulerAttitude::from_attitude(attitude, true).is_none());
        attitude.q[0] = f32::NAN;
        assert!(EulerAttitude::from_attitude(attitude, true).is_none());
    }

    fn aligned() -> ImuEkf {
        let mut core = ImuEkf::new(&ImuEkfParams::default(), 0).unwrap();
        for timestamp in (2_500..=1_000_000).step_by(2_500) {
            core.update(&ImuDelta {
                timestamp,
                delta_angle_rad: [0.0; 3],
                delta_velocity_m_s: [0.0, 0.0, -math::GRAVITY * 0.0025],
                delta_angle_dt_s: 0.0025,
                delta_velocity_dt_s: 0.0025,
                delta_velocity_clipping: [false; 3],
            })
            .unwrap();
        }
        // These are mapping tests, not an alignment qualification. Set the
        // already-initialized IMU fixture's alignment bit explicitly.
        core.control |= 1;
        assert!(core.health().valid);
        core
    }

    #[test]
    fn snapshot_keeps_nominal_and_error_indices_and_unknown_reference() {
        let mut core = aligned();
        for index in 0..24 {
            core.covariance[index][index] = (index + 1) as f32;
        }
        let snapshot = Snapshot::from_core(&mut core, 1_000_100, Source::NoGnss, None);
        let local = snapshot.local_position;
        assert_eq!(local.timestamp_sample, 1_000_000);
        assert_eq!(snapshot.states.timestamp_sample, core.timestamp());
        assert_eq!(snapshot.states.n_states, 25);
        assert_eq!(local.heading_var, 3.0);
        assert_eq!(local.tilt_var, 3.0);
        assert_eq!(local.eph, libm::sqrtf(15.0));
        assert_eq!(local.epv, 3.0);
        assert_eq!(local.evh, 3.0);
        assert_eq!(local.evv, libm::sqrtf(6.0));
        assert!(!local.xy_valid && !local.v_xy_valid && !local.xy_global && !local.z_global);
        assert!(local.ref_lat.is_nan() && local.ref_lon.is_nan() && local.ref_alt.is_nan());
        assert_eq!(local.ref_timestamp, 0);
        assert!(snapshot.status.pos_horiz_accuracy.is_infinite());
        assert!(snapshot.status.vel_test_ratio.is_nan());
        assert!(snapshot.global_position.is_none());
    }

    #[test]
    fn stale_or_faulted_output_revokes_publication_and_solution_validity() {
        let mut core = aligned();
        let stale = Snapshot::from_core(&mut core, 1_100_001, Source::Gnss, None);
        assert!(stale.attitude.is_none() && stale.odometry.is_none());
        assert!(!stale.local_position.z_valid);
        assert_eq!(stale.status.solution_status_flags & 0x7f, 0);
        core.check_fresh(1_100_001).unwrap_err();
        let faulted = Snapshot::from_core(&mut core, 1_000_000, Source::Gnss, None);
        assert!(faulted.attitude.is_none() && faulted.global_position.is_none());
        assert_eq!(faulted.status.solution_status_flags & 0x7f, 0);
    }

    #[test]
    fn reset_mapping_preserves_signs_counters_and_one_second_mag_guard() {
        let mut core = aligned();
        core.control |= (1 << 1) | (1 << 4) | (1 << 23);
        core.mag_alignment_start = core.timestamp();
        core.reset_yaw(0.2, 0.1);
        core.reset_vertical_velocity(2.0, 4.0);
        core.reset_altitude(3.0, 9.0);
        let resets = core.reset_report();
        let snapshot = Snapshot::from_core(&mut core, 1_000_000, Source::NoGnss, None);
        assert_eq!(snapshot.local_position.delta_z, resets.position_delta[2]);
        assert_eq!(snapshot.local_position.delta_vz, resets.velocity_delta[2]);
        assert_eq!(snapshot.status.reset_count_pod_d, resets.counts[4]);
        assert!(!snapshot.local_position.heading_good_for_control);
        assert!((snapshot.local_position.delta_heading - 0.2).abs() < 1e-6);
        assert_eq!(
            odometry_reset_counter(ResetReport {
                counts: [255, 1, 2, 3, 4, 200],
                ..resets
            }),
            9
        );
    }
}
