//! Separate estimator ULog registry shared by host and the E7 LOG_EST path.
//! Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.

use crate::ulog::{self, Error, Payload, Record};

use super::standard_output::{
    AidSource, AngularVelocity, Attitude, EstimatorStates, EstimatorStatus, GlobalPosition,
    LocalPosition, Snapshot,
};

pub const TOPIC_COUNT: usize = 12;
pub const FORMAT_CAPACITY: usize = 2_048;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum TopicId {
    Attitude = 11,
    AngularVelocity,
    LocalPosition,
    GlobalPosition,
    Status,
    States,
    BaroHeight,
    Mag,
    RangeHeight,
    GnssPosition,
    GnssVelocity,
    GnssHeight,
}

pub struct Topic {
    pub id: TopicId,
    pub name: &'static str,
    pub format: &'static str,
    pub payload_len: usize,
}

const AID_1D_FIELDS: &str = "uint64_t timestamp;uint64_t timestamp_sample;uint8_t estimator_instance;uint32_t device_id;uint64_t time_last_fuse;float observation;float observation_variance;float innovation;float innovation_filtered;float innovation_variance;float test_ratio;float test_ratio_filtered;bool innovation_rejected;bool fused;";
const AID_2D_FIELDS: &str = "uint64_t timestamp;uint64_t timestamp_sample;uint8_t estimator_instance;uint32_t device_id;uint64_t time_last_fuse;double[2] observation;float[2] observation_variance;float[2] innovation;float[2] innovation_filtered;float[2] innovation_variance;float[2] test_ratio;float[2] test_ratio_filtered;bool innovation_rejected;bool fused;";
const AID_3D_FIELDS: &str = "uint64_t timestamp;uint64_t timestamp_sample;uint8_t estimator_instance;uint32_t device_id;uint64_t time_last_fuse;float[3] observation;float[3] observation_variance;float[3] innovation;float[3] innovation_filtered;float[3] innovation_variance;float[3] test_ratio;float[3] test_ratio_filtered;bool innovation_rejected;bool fused;";

/// Aid topics share the same field definitions but have different topic names.
/// Aid `format` contains shared fields; `format_record` prefixes the topic name.
pub const TOPICS: [Topic; TOPIC_COUNT] = [
    Topic {
        id: TopicId::Attitude,
        name: "vehicle_attitude",
        format: "vehicle_attitude:uint64_t timestamp;uint64_t timestamp_sample;float[4] q;float[4] delta_q_reset;uint8_t quat_reset_counter;",
        payload_len: 49,
    },
    Topic {
        id: TopicId::AngularVelocity,
        name: "vehicle_angular_velocity",
        format: "vehicle_angular_velocity:uint64_t timestamp;uint64_t timestamp_sample;float[3] xyz;float[3] xyz_derivative;",
        payload_len: 40,
    },
    Topic {
        id: TopicId::LocalPosition,
        name: "vehicle_local_position",
        format: "vehicle_local_position:uint64_t timestamp;uint64_t timestamp_sample;bool xy_valid;bool z_valid;bool v_xy_valid;bool v_z_valid;float x;float y;float z;float[2] delta_xy;uint8_t xy_reset_counter;float delta_z;uint8_t z_reset_counter;float vx;float vy;float vz;float z_deriv;float[2] delta_vxy;uint8_t vxy_reset_counter;float delta_vz;uint8_t vz_reset_counter;float ax;float ay;float az;float heading;float heading_var;float unaided_heading;float delta_heading;uint8_t heading_reset_counter;bool heading_good_for_control;float tilt_var;bool xy_global;bool z_global;uint64_t ref_timestamp;double ref_lat;double ref_lon;float ref_alt;bool dist_bottom_valid;float dist_bottom;float dist_bottom_var;float delta_dist_bottom;uint8_t dist_bottom_reset_counter;uint8_t dist_bottom_sensor_bitfield;float eph;float epv;float evh;float evv;bool dead_reckoning;float vxy_max;float vz_max;float hagl_min;float hagl_max_z;float hagl_max_xy;",
        payload_len: 192,
    },
    Topic {
        id: TopicId::GlobalPosition,
        name: "vehicle_global_position",
        format: "vehicle_global_position:uint64_t timestamp;uint64_t timestamp_sample;double lat;double lon;float alt;float alt_ellipsoid;bool lat_lon_valid;bool alt_valid;float delta_alt;float delta_terrain;uint8_t lat_lon_reset_counter;uint8_t alt_reset_counter;uint8_t terrain_reset_counter;float eph;float epv;float terrain_alt;bool terrain_alt_valid;bool dead_reckoning;",
        payload_len: 67,
    },
    Topic {
        id: TopicId::Status,
        name: "estimator_status",
        format: "estimator_status:uint64_t timestamp;uint64_t timestamp_sample;float[3] output_tracking_error;uint16_t gps_check_fail_flags;uint64_t control_mode_flags;uint32_t filter_fault_flags;float pos_horiz_accuracy;float pos_vert_accuracy;float hdg_test_ratio;float vel_test_ratio;float pos_test_ratio;float hgt_test_ratio;float tas_test_ratio;float hagl_test_ratio;float beta_test_ratio;uint16_t solution_status_flags;uint8_t reset_count_vel_ne;uint8_t reset_count_vel_d;uint8_t reset_count_pos_ne;uint8_t reset_count_pod_d;uint8_t reset_count_quat;float time_slip;bool pre_flt_fail_innov_heading;bool pre_flt_fail_innov_height;bool pre_flt_fail_innov_pos_horiz;bool pre_flt_fail_innov_vel_horiz;bool pre_flt_fail_innov_vel_vert;bool pre_flt_fail_mag_field_disturbed;uint32_t accel_device_id;uint32_t gyro_device_id;uint32_t baro_device_id;uint32_t mag_device_id;uint8_t health_flags;uint8_t timeout_flags;float mag_inclination_deg;float mag_inclination_ref_deg;float mag_strength_gs;float mag_strength_ref_gs;",
        payload_len: 129,
    },
    Topic {
        id: TopicId::States,
        name: "estimator_states",
        format: "estimator_states:uint64_t timestamp;uint64_t timestamp_sample;float[25] states;uint8_t n_states;float[24] covariances;",
        payload_len: 213,
    },
    Topic { id: TopicId::BaroHeight, name: "estimator_aid_src_baro_hgt", format: AID_1D_FIELDS, payload_len: 59 },
    Topic { id: TopicId::Mag, name: "estimator_aid_src_mag", format: AID_3D_FIELDS, payload_len: 115 },
    Topic { id: TopicId::RangeHeight, name: "estimator_aid_src_rng_hgt", format: AID_1D_FIELDS, payload_len: 59 },
    Topic { id: TopicId::GnssPosition, name: "estimator_aid_src_gnss_pos", format: AID_2D_FIELDS, payload_len: 95 },
    Topic { id: TopicId::GnssVelocity, name: "estimator_aid_src_gnss_vel", format: AID_3D_FIELDS, payload_len: 115 },
    Topic { id: TopicId::GnssHeight, name: "estimator_aid_src_gnss_hgt", format: AID_1D_FIELDS, payload_len: 59 },
];

impl TopicId {
    const fn index(self) -> usize {
        self as usize - 11
    }

    pub const fn definition(self) -> &'static Topic {
        &TOPICS[self.index()]
    }

    pub const fn message_id(self) -> u16 {
        self as u16
    }
}

pub fn format_record(topic: TopicId, out: &mut [u8; FORMAT_CAPACITY]) -> Result<usize, Error> {
    let definition = topic.definition();
    if topic.index() < TopicId::BaroHeight.index() {
        return ulog::format_definition(definition.format, out);
    }
    let length = definition.name.len() + 1 + definition.format.len();
    if length + 3 > out.len() {
        return Err(Error::TooLong);
    }
    out[..2].copy_from_slice(&(length as u16).to_le_bytes());
    out[2] = b'F';
    let separator = 3 + definition.name.len();
    out[3..separator].copy_from_slice(definition.name.as_bytes());
    out[separator] = b':';
    out[separator + 1..length + 3].copy_from_slice(definition.format.as_bytes());
    Ok(length + 3)
}

pub fn subscription(topic: TopicId) -> Result<Record, Error> {
    ulog::raw_subscription(topic.message_id(), 0, topic.definition().name)
}

#[derive(Default)]
pub struct DataEncoder {
    last_timestamp: [u64; TOPIC_COUNT],
    last_sample: [u64; 6],
}

impl DataEncoder {
    pub const fn new() -> Self {
        Self {
            last_timestamp: [0; TOPIC_COUNT],
            last_sample: [0; 6],
        }
    }

    pub fn encode(
        &mut self,
        topic: TopicId,
        timestamp: u64,
        payload: &Payload,
    ) -> Result<Record, Error> {
        if payload.as_bytes().len() + 8 != topic.definition().payload_len {
            return Err(Error::InvalidLength);
        }
        if timestamp < self.last_timestamp[topic.index()] {
            return Err(Error::TimestampRegression);
        }
        let record = ulog::data_record(topic.message_id(), timestamp, payload.as_bytes())?;
        self.last_timestamp[topic.index()] = timestamp;
        Ok(record)
    }

    /// Bounded record callback, no allocation and no transport-specific state.
    /// The callback owns error/backpressure handling; this host gate does not
    /// change the caller's queue budget or logging policy.
    pub fn write_snapshot<E: From<Error>>(
        &mut self,
        snapshot: &Snapshot,
        mut write: impl FnMut(&Record) -> Result<(), E>,
    ) -> Result<(), E> {
        if let Some(attitude) = snapshot.attitude {
            write(&self.encode(
                TopicId::Attitude,
                attitude.timestamp,
                &attitude_payload(attitude),
            )?)?;
        }
        if let Some(angular) = snapshot.angular_velocity {
            write(&self.encode(
                TopicId::AngularVelocity,
                angular.timestamp,
                &angular_velocity_payload(angular),
            )?)?;
        }
        let local = snapshot.local_position;
        write(&self.encode(
            TopicId::LocalPosition,
            local.timestamp,
            &local_position_payload(local),
        )?)?;
        if let Some(global) = snapshot.global_position {
            write(&self.encode(
                TopicId::GlobalPosition,
                global.timestamp,
                &global_position_payload(global),
            )?)?;
        }
        write(&self.encode(
            TopicId::Status,
            snapshot.status.timestamp,
            &status_payload(snapshot.status),
        )?)?;
        write(&self.encode(
            TopicId::States,
            snapshot.states.timestamp,
            &states_payload(snapshot.states),
        )?)?;
        let topics = [
            TopicId::BaroHeight,
            TopicId::Mag,
            TopicId::RangeHeight,
            TopicId::GnssPosition,
            TopicId::GnssVelocity,
            TopicId::GnssHeight,
        ];
        for (index, (topic, aid)) in topics.into_iter().zip(snapshot.aids).enumerate() {
            if let Some(aid) = aid {
                if aid.status.timestamp <= self.last_sample[index] {
                    continue;
                }
                let dimensions = match topic {
                    TopicId::GnssPosition => 2,
                    TopicId::Mag | TopicId::GnssVelocity => 3,
                    _ => 1,
                };
                write(&self.encode(topic, aid.timestamp, &aid_payload(aid, dimensions))?)?;
                self.last_sample[index] = aid.status.timestamp;
            }
        }
        Ok(())
    }
}

fn floats(payload: &mut Payload, values: &[f32]) {
    for value in values {
        payload.push_f32(*value).unwrap();
    }
}
fn byte(payload: &mut Payload, value: u8) {
    payload.push_u8(value).unwrap();
}
fn time(timestamp_sample: u64) -> Payload {
    let mut payload = Payload::new();
    payload.push_u64(timestamp_sample).unwrap();
    payload
}

pub fn attitude_payload(value: Attitude) -> Payload {
    let mut payload = time(value.timestamp_sample);
    floats(&mut payload, &value.q);
    floats(&mut payload, &value.delta_q_reset);
    byte(&mut payload, value.quat_reset_counter);
    payload
}

pub fn angular_velocity_payload(value: AngularVelocity) -> Payload {
    let mut payload = time(value.timestamp_sample);
    floats(&mut payload, &value.xyz);
    floats(&mut payload, &value.xyz_derivative);
    payload
}

pub fn local_position_payload(v: LocalPosition) -> Payload {
    let mut p = time(v.timestamp_sample);
    for valid in [v.xy_valid, v.z_valid, v.v_xy_valid, v.v_z_valid] {
        byte(&mut p, u8::from(valid));
    }
    floats(&mut p, &[v.x, v.y, v.z]);
    floats(&mut p, &v.delta_xy);
    byte(&mut p, v.xy_reset_counter);
    floats(&mut p, &[v.delta_z]);
    byte(&mut p, v.z_reset_counter);
    floats(&mut p, &[v.vx, v.vy, v.vz, v.z_deriv]);
    floats(&mut p, &v.delta_vxy);
    byte(&mut p, v.vxy_reset_counter);
    floats(&mut p, &[v.delta_vz]);
    byte(&mut p, v.vz_reset_counter);
    floats(
        &mut p,
        &[
            v.ax,
            v.ay,
            v.az,
            v.heading,
            v.heading_var,
            v.unaided_heading,
            v.delta_heading,
        ],
    );
    byte(&mut p, v.heading_reset_counter);
    byte(&mut p, u8::from(v.heading_good_for_control));
    floats(&mut p, &[v.tilt_var]);
    byte(&mut p, u8::from(v.xy_global));
    byte(&mut p, u8::from(v.z_global));
    p.push_u64(v.ref_timestamp).unwrap();
    p.push_f64(v.ref_lat).unwrap();
    p.push_f64(v.ref_lon).unwrap();
    floats(&mut p, &[v.ref_alt]);
    byte(&mut p, u8::from(v.dist_bottom_valid));
    floats(
        &mut p,
        &[v.dist_bottom, v.dist_bottom_var, v.delta_dist_bottom],
    );
    byte(&mut p, v.dist_bottom_reset_counter);
    byte(&mut p, v.dist_bottom_sensor_bitfield);
    floats(&mut p, &[v.eph, v.epv, v.evh, v.evv]);
    byte(&mut p, u8::from(v.dead_reckoning));
    floats(
        &mut p,
        &[v.vxy_max, v.vz_max, v.hagl_min, v.hagl_max_z, v.hagl_max_xy],
    );
    p
}

pub fn global_position_payload(v: GlobalPosition) -> Payload {
    let mut p = time(v.timestamp_sample);
    p.push_f64(v.lat).unwrap();
    p.push_f64(v.lon).unwrap();
    floats(&mut p, &[v.alt, v.alt_ellipsoid]);
    byte(&mut p, u8::from(v.lat_lon_valid));
    byte(&mut p, u8::from(v.alt_valid));
    floats(&mut p, &[v.delta_alt, v.delta_terrain]);
    byte(&mut p, v.lat_lon_reset_counter);
    byte(&mut p, v.alt_reset_counter);
    byte(&mut p, v.terrain_reset_counter);
    floats(&mut p, &[v.eph, v.epv, v.terrain_alt]);
    byte(&mut p, u8::from(v.terrain_alt_valid));
    byte(&mut p, u8::from(v.dead_reckoning));
    p
}

pub fn status_payload(v: EstimatorStatus) -> Payload {
    let mut p = time(v.timestamp_sample);
    floats(&mut p, &v.output_tracking_error);
    p.push_u16(v.gps_check_fail_flags).unwrap();
    p.push_u64(v.control_mode_flags).unwrap();
    p.push_u32(v.filter_fault_flags).unwrap();
    floats(
        &mut p,
        &[
            v.pos_horiz_accuracy,
            v.pos_vert_accuracy,
            v.hdg_test_ratio,
            v.vel_test_ratio,
            v.pos_test_ratio,
            v.hgt_test_ratio,
            v.tas_test_ratio,
            v.hagl_test_ratio,
            v.beta_test_ratio,
        ],
    );
    p.push_u16(v.solution_status_flags).unwrap();
    for counter in [
        v.reset_count_vel_ne,
        v.reset_count_vel_d,
        v.reset_count_pos_ne,
        v.reset_count_pod_d,
        v.reset_count_quat,
    ] {
        byte(&mut p, counter);
    }
    floats(&mut p, &[v.time_slip]);
    for flag in [
        v.pre_flt_fail_innov_heading,
        v.pre_flt_fail_innov_height,
        v.pre_flt_fail_innov_pos_horiz,
        v.pre_flt_fail_innov_vel_horiz,
        v.pre_flt_fail_innov_vel_vert,
        v.pre_flt_fail_mag_field_disturbed,
    ] {
        byte(&mut p, u8::from(flag));
    }
    for id in [
        v.accel_device_id,
        v.gyro_device_id,
        v.baro_device_id,
        v.mag_device_id,
    ] {
        p.push_u32(id).unwrap();
    }
    byte(&mut p, v.health_flags);
    byte(&mut p, v.timeout_flags);
    floats(
        &mut p,
        &[
            v.mag_inclination_deg,
            v.mag_inclination_ref_deg,
            v.mag_strength_gs,
            v.mag_strength_ref_gs,
        ],
    );
    p
}

pub fn states_payload(v: EstimatorStates) -> Payload {
    let mut p = time(v.timestamp_sample);
    floats(&mut p, &v.states);
    byte(&mut p, v.n_states);
    floats(&mut p, &v.covariances);
    p
}

fn aid_payload(v: AidSource, dimensions: usize) -> Payload {
    assert!((1..=3).contains(&dimensions));
    let mut p = time(v.status.timestamp);
    byte(&mut p, 0); // Single estimator instance.
    p.push_u32(v.device_id).unwrap();
    p.push_u64(0).unwrap(); // Last fusion time is not available in AidStatus.
    if dimensions == 2 {
        for _ in 0..2 {
            p.push_f64(f64::NAN).unwrap();
        }
    } else {
        floats(&mut p, &[f32::NAN; 3][..dimensions]);
    }
    floats(&mut p, &[f32::NAN; 3][..dimensions]); // observation variance
    floats(&mut p, &v.status.innovation[..dimensions]);
    floats(&mut p, &[f32::NAN; 3][..dimensions]); // filtered innovation
    floats(&mut p, &v.status.variance[..dimensions]);
    floats(&mut p, &v.status.ratio[..dimensions]);
    floats(&mut p, &[f32::NAN; 3][..dimensions]); // signed filtered ratio
    byte(&mut p, u8::from(v.status.rejected));
    byte(&mut p, u8::from(v.status.fused));
    p
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn schemas_fit_metadata_storage_without_changing_the_runtime_registry() {
        let sizes = [
            ("uint64_t", 8),
            ("uint32_t", 4),
            ("uint16_t", 2),
            ("uint8_t", 1),
            ("bool", 1),
            ("float", 4),
            ("double", 8),
        ];
        for (index, topic) in TOPICS.iter().enumerate() {
            assert_eq!(topic.id.index(), index);
            assert!(ulog::TOPICS
                .iter()
                .all(|base| base.id as u16 != topic.id as u16));
            let mut bytes = [0; FORMAT_CAPACITY];
            let length = format_record(topic.id, &mut bytes).unwrap();
            let definition = core::str::from_utf8(&bytes[3..length]).unwrap();
            let fields = definition.split_once(':').unwrap().1;
            let actual: usize = fields
                .split(';')
                .filter(|s| !s.is_empty())
                .map(|field| {
                    let kind = field.split_once(' ').unwrap().0;
                    let (base, count) = kind.split_once('[').map_or((kind, 1), |(base, suffix)| {
                        (base, suffix.trim_end_matches(']').parse::<usize>().unwrap())
                    });
                    count * sizes.iter().find(|(name, _)| *name == base).unwrap().1
                })
                .sum();
            assert_eq!(topic.payload_len, actual, "{}", topic.name);
            assert!(TOPICS[..index].iter().all(|other| other.name != topic.name));
            assert_eq!(subscription(topic.id).unwrap().as_bytes()[3], 0);
        }
        assert_eq!(ulog::TOPIC_COUNT, 18);
        const { assert!(crate::log_queue::RUNTIME_RECORD_LEN >= 218) };
    }

    #[test]
    fn raw_registry_rejects_wrong_payload_and_timestamp_regression() {
        let mut encoder = DataEncoder::new();
        let value = Attitude {
            timestamp: 100,
            timestamp_sample: 90,
            q: [1.0, 0.0, 0.0, 0.0],
            delta_q_reset: [1.0, 0.0, 0.0, 0.0],
            quat_reset_counter: 2,
        };
        let payload = attitude_payload(value);
        let bytes = encoder
            .encode(TopicId::Attitude, value.timestamp, &payload)
            .unwrap();
        assert_eq!(&bytes.as_bytes()[3..5], &11_u16.to_le_bytes());
        assert_eq!(
            encoder.encode(TopicId::Attitude, 99, &payload),
            Err(Error::TimestampRegression)
        );
        assert_eq!(
            encoder.encode(TopicId::States, 100, &payload),
            Err(Error::InvalidLength)
        );
    }
}
