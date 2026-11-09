//! Host-only fixed-input BMM150 compensation and standard ULog fixture.

use std::{
    env, fs,
    io::{self, BufRead},
};
use v6x_rust_ap::{
    bmm150::{self, Sample, Trim},
    px4_messages::{SensorMag, VehicleMagnetometer, PX4_REVISION},
    ulog::{self, DataEncoder, TopicId},
};

fn block<const N: usize>(hex: &str) -> [u8; N] {
    assert_eq!(hex.len(), N * 2);
    let mut bytes = [0; N];
    for (index, byte) in bytes.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&hex[index * 2..index * 2 + 2], 16).unwrap();
    }
    bytes
}

fn fixture(path: &str) {
    let mut bytes = Vec::from(ulog::file_header(1_000_000));
    bytes.extend_from_slice(ulog::flag_bits().as_bytes());
    bytes.extend_from_slice(
        ulog::information("char[40] px4_msg_revision", PX4_REVISION.as_bytes())
            .unwrap()
            .as_bytes(),
    );
    let topics = [TopicId::SensorMag, TopicId::VehicleMagnetometer];
    for topic in topics {
        bytes.extend_from_slice(ulog::format(topic).unwrap().as_bytes());
    }
    for topic in topics {
        bytes.extend_from_slice(ulog::subscription(topic).unwrap().as_bytes());
    }
    let mut encoder = DataEncoder::new();
    for sequence in 0..2 {
        let sample = Sample {
            timestamp_sample: 1_001_000 + u64::from(sequence) * bmm150::PERIOD_US,
            sequence,
            field_gauss: [0.22, -0.07, 0.43],
            error_count: 7,
        };
        let sensor = SensorMag::from_sample(sample, sample.timestamp_sample + 100);
        for (topic, payload) in [
            (TopicId::SensorMag, ulog::sensor_mag_payload(sensor)),
            (
                TopicId::VehicleMagnetometer,
                ulog::vehicle_magnetometer_payload(VehicleMagnetometer::from_sensor(sensor)),
            ),
        ] {
            bytes.extend_from_slice(
                encoder
                    .encode(topic, sensor.timestamp, payload.as_bytes())
                    .unwrap()
                    .as_bytes(),
            );
        }
    }
    fs::write(path, bytes).unwrap();
}

fn main() {
    if let Some(path) = env::args().nth(1) {
        fixture(&path);
        return;
    }
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let values: Vec<_> = line.split_whitespace().collect();
        let trim = block::<16>(values[0]);
        let trim = Trim::decode(
            trim[..2].try_into().unwrap(),
            trim[2..6].try_into().unwrap(),
            trim[6..].try_into().unwrap(),
        )
        .unwrap();
        let (x, y, z, rhall): (i16, i16, i16, u16) = (
            values[1].parse().unwrap(),
            values[2].parse().unwrap(),
            values[3].parse().unwrap(),
            values[4].parse().unwrap(),
        );
        let mut data = [0; bmm150::DATA_LEN];
        data[..2].copy_from_slice(&(x * 8).to_le_bytes());
        data[2..4].copy_from_slice(&(y * 8).to_le_bytes());
        data[4..6].copy_from_slice(&(z * 2).to_le_bytes());
        data[6..8].copy_from_slice(&((rhall << 2) | 1).to_le_bytes());
        let field = trim.compensate(&data).unwrap();
        println!(
            "{:08x} {:08x} {:08x}",
            field[0].to_bits(),
            field[1].to_bits(),
            field[2].to_bits()
        );
    }
}
