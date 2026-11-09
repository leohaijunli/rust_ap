//! Synthetic two-source fixture, never hardware acceptance.
use std::{env, fs};
use v6x_rust_ap::{
    magnetometer::{Calibration, Config, Selector},
    px4_messages::SensorMag,
    ulog::{self, DataEncoder, TopicId},
};

fn raw(id: u32, time: u64) -> SensorMag {
    SensorMag {
        timestamp: time,
        timestamp_sample: time,
        device_id: id,
        x: 0.22,
        y: 0.07,
        z: 0.43,
        temperature: f32::NAN,
        error_count: 0,
    }
}

fn main() {
    let output = env::args()
        .nth(1)
        .expect("magnetometer_selection <output.ulg>");
    let internal = v6x_rust_ap::bmm150::DEVICE_ID;
    let external = 0x431009;
    let mut config = Config::default();
    config.slots[2] = Calibration {
        device_id: external,
        offset: [0.02, 0.01, 0.03],
        rotation: 2,
        ..Calibration::default()
    };
    let mut selector = Selector::new();
    selector.configure(config).unwrap();
    let mut bytes = Vec::from(ulog::file_header(1_000_000));
    bytes.extend_from_slice(ulog::flag_bits().as_bytes());
    let topics = [
        TopicId::SensorMag,
        TopicId::SensorMag1,
        TopicId::SensorMag2,
        TopicId::SensorMag3,
        TopicId::VehicleMagnetometer,
    ];
    for topic in topics {
        bytes.extend_from_slice(ulog::format(topic).unwrap().as_bytes());
    }
    for topic in topics {
        bytes.extend_from_slice(ulog::subscription(topic).unwrap().as_bytes());
    }
    let mut encoder = DataEncoder::new();
    for (id, time, topic) in [
        (internal, 1_000_000, TopicId::SensorMag),
        (external, 1_010_000, TopicId::SensorMag1),
    ] {
        let report = raw(id, time);
        selector.put(report, time).unwrap();
        bytes.extend_from_slice(
            encoder
                .encode(topic, time, ulog::sensor_mag_payload(report).as_bytes())
                .unwrap()
                .as_bytes(),
        );
        let selected = selector.select(time + 100).unwrap();
        bytes.extend_from_slice(
            encoder
                .encode(
                    TopicId::VehicleMagnetometer,
                    time + 100,
                    ulog::vehicle_magnetometer_payload(selected.data).as_bytes(),
                )
                .unwrap()
                .as_bytes(),
        );
        selector.delivered(selected);
    }
    config.slots[2].offset[0] = 0.03;
    selector.configure(config).unwrap();
    let selected = selector.select(1_020_100).unwrap();
    assert!(selected.reset);
    bytes.extend_from_slice(
        encoder
            .encode(
                TopicId::VehicleMagnetometer,
                1_020_100,
                ulog::vehicle_magnetometer_payload(selected.data).as_bytes(),
            )
            .unwrap()
            .as_bytes(),
    );
    selector.delivered(selected);
    selector.invalidate(external);
    let selected = selector.select(1_030_100).unwrap();
    assert_eq!(selected.data.device_id, internal);
    assert!(selected.reset);
    bytes.extend_from_slice(
        encoder
            .encode(
                TopicId::VehicleMagnetometer,
                1_030_100,
                ulog::vehicle_magnetometer_payload(selected.data).as_bytes(),
            )
            .unwrap()
            .as_bytes(),
    );
    assert!(selector.select(1_400_000).is_none());
    fs::write(output, bytes).unwrap();
}
