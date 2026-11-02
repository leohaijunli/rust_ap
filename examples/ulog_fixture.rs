use std::env;
use std::fs;
use std::mem::size_of;

use v6x_rust_led::icm42688::Sample;
use v6x_rust_led::px4_messages::{ActuatorArmed, SensorAccel, SensorGyro, PX4_REVISION};
use v6x_rust_led::ulog::{self, DataEncoder, TopicId};

fn main() {
    let path = env::args()
        .nth(1)
        .expect("usage: ulog_fixture <output.ulg>");
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&ulog::file_header(1_000_000));
    bytes.extend_from_slice(ulog::flag_bits().as_bytes());
    for topic in ulog::TOPICS {
        bytes.extend_from_slice(ulog::format(topic.id).unwrap().as_bytes());
    }
    for record in [
        ulog::information("char[7] sys_name", b"rust_ap").unwrap(),
        ulog::information("char[40] px4_msg_revision", PX4_REVISION.as_bytes()).unwrap(),
        ulog::information("char[3] body_frame", b"FRD").unwrap(),
        ulog::information("char[3] world_frame", b"NED").unwrap(),
        ulog::parameter_i32("MAV_SYS_ID", 1).unwrap(),
        ulog::parameter_f32("LINK_LED_HZ", 2.0).unwrap(),
    ] {
        bytes.extend_from_slice(record.as_bytes());
    }
    for topic in ulog::TOPICS {
        bytes.extend_from_slice(ulog::subscription(topic.id).unwrap().as_bytes());
    }
    bytes.extend_from_slice(ulog::padding(5).unwrap().as_bytes());
    let mut data = DataEncoder::new();
    for topic in ulog::TOPICS {
        if matches!(topic.id, TopicId::SensorAccel | TopicId::SensorGyro) {
            continue;
        }
        let timestamp = 1_000_100 + topic.id as u64;
        let fields = if topic.id == TopicId::ActuatorArmed {
            ulog::actuator_armed_payload(ActuatorArmed {
                timestamp,
                armed: false,
                prearmed: true,
                ready_to_arm: false,
                lockdown: true,
                kill: false,
                termination: true,
                in_esc_calibration_mode: false,
            })
            .as_bytes()
            .to_vec()
        } else {
            vec![0; topic.payload_len - size_of::<u64>()]
        };
        bytes.extend_from_slice(
            data.encode(topic.id, timestamp, &fields)
                .unwrap()
                .as_bytes(),
        );
    }
    for (index, (accel_m_s2, gyro_rad_s)) in [
        ([1.25, -2.5, 9.75], [0.125, -0.25, 0.5]),
        ([156.9, -156.9, 10.0], [34.9, -34.9, 0.0]),
    ]
    .into_iter()
    .enumerate()
    {
        let sample = Sample {
            sample_timestamp_us: 1_000_900 + index as u64 * 1_000,
            sequence: index as u32 * 8,
            gyro_rad_s,
            accel_m_s2,
            temperature_c: 36.5,
            sensor_timestamp: 0,
        };
        let timestamp = sample.sample_timestamp_us + 100;
        let accel = SensorAccel::from_sample(timestamp, sample, 7);
        let gyro = SensorGyro::from_sample(timestamp, sample, 7);
        for (topic, fields) in [
            (TopicId::SensorAccel, ulog::sensor_accel_payload(accel)),
            (TopicId::SensorGyro, ulog::sensor_gyro_payload(gyro)),
        ] {
            bytes.extend_from_slice(
                data.encode(topic, timestamp, fields.as_bytes())
                    .unwrap()
                    .as_bytes(),
            );
        }
    }
    bytes.extend_from_slice(ulog::dropout(25).as_bytes());
    fs::write(path, bytes).expect("write ULog fixture");
}
