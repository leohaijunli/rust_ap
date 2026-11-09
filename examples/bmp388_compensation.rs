//! Host-only compensation parity probe; input rows are trim hex, P24, T24.

use std::io::{self, BufRead};
use v6x_rust_ap::bmp388::{self, Calibration};

fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let fields: Vec<_> = line.split_whitespace().collect();
        assert_eq!(fields.len(), 3);
        assert_eq!(fields[0].len(), 42);
        let mut trim = [0; 21];
        for (i, byte) in trim.iter_mut().enumerate() {
            *byte = u8::from_str_radix(&fields[0][i * 2..i * 2 + 2], 16).unwrap();
        }
        let pressure: u32 = fields[1].parse().unwrap();
        let temperature: u32 = fields[2].parse().unwrap();
        let mut data = [0; 7];
        data[0] = bmp388::DATA_READY;
        data[1..4].copy_from_slice(&pressure.to_le_bytes()[..3]);
        data[4..7].copy_from_slice(&temperature.to_le_bytes()[..3]);
        let crc = bmp388::trim_crc(&trim);
        let calibration = Calibration::decode(&trim, crc).unwrap();
        let (pressure, temperature) = calibration.compensate(&data, 0).unwrap();
        println!(
            "{:02x} {:08x} {:08x}",
            crc,
            pressure.to_bits(),
            temperature.to_bits()
        );
    }
}
