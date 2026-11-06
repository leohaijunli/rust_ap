//! E6-H synthetic host publication fixture, never a firmware task or HIL run.

use std::{
    env,
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::Path,
};

use mavlink::MavHeader;
use v6x_rust_led::{
    estimator::{
        gnss::{GnssEkf, GnssObservation, GnssParams},
        imu_ekf::{
            baro_mag::{AidStatus, BaroMagParams, BaroMagSample},
            range::RangeParams,
            ImuEkf, ImuEkfParams,
        },
        range_sensor::RangeObservation,
        standard_ulog, GnssSample, ImuDelta, ParamValue, EKF2_PARAMS,
    },
    mavlink_estimator,
    px4_messages::PX4_REVISION,
    ulog,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const DT_S: f32 = 0.0025;
const DT_US: u64 = 2_500;
const WARMUP_US: u64 = 5_000_000;
const END_US: u64 = 65_000_000;
const ACCELERATION: f32 = 0.05;
const LATITUDE: f64 = 47.397742;
const LONGITUDE: f64 = 8.545594;
const ALTITUDE: f32 = 100.0;
const EARTH_RADIUS_M: f64 = 6_378_137.0;

fn update(filter: &mut GnssEkf, index: u64) -> Result<()> {
    let timestamp = index * DT_US;
    let (yaw, north, velocity) = truth(timestamp);
    let (sin, cos) = yaw.sin_cos();
    let acceleration = if timestamp > WARMUP_US {
        ACCELERATION
    } else {
        0.0
    };
    let imu = ImuDelta {
        timestamp,
        delta_angle_rad: [
            0.0,
            0.0,
            if timestamp > 1_000_000 {
                5_f32.to_radians() * DT_S
            } else {
                0.0
            },
        ],
        delta_velocity_m_s: [
            acceleration * cos * DT_S,
            -acceleration * sin * DT_S,
            -9.80665 * DT_S,
        ],
        delta_angle_dt_s: DT_S,
        delta_velocity_dt_s: DT_S,
        delta_velocity_clipping: [false; 3],
    };
    let aiding = index.is_multiple_of(8).then_some(BaroMagSample {
        timestamp,
        altitude_m: ALTITUDE,
        field_gauss: [0.22 * cos, -0.22 * sin, 0.43],
    });
    let range = index.is_multiple_of(20).then_some(RangeObservation {
        timestamp,
        distance_m: 1.0,
        quality_percent: 100,
        limits: None,
    });
    let gnss = index.is_multiple_of(80).then_some(GnssObservation {
        sample: GnssSample {
            timestamp,
            latitude_deg: LATITUDE + (north / EARTH_RADIUS_M).to_degrees(),
            longitude_deg: LONGITUDE,
            altitude_msl_m: ALTITUDE,
            velocity_ned_m_s: [velocity as f32, 0.0, 0.0],
            horizontal_accuracy_m: 0.5,
            vertical_accuracy_m: 0.8,
            speed_accuracy_m_s: 0.1,
            fix_type: 3,
            satellites: 12,
        },
        pdop: 1.0,
        spoofed: false,
    });
    filter
        .update(&imu, aiding, gnss, range)
        .map(|_| ())
        .map_err(|fault| format!("EKF failed at {timestamp}: {fault:?}").into())
}

fn aids(filter: &GnssEkf) -> [Option<AidStatus>; 6] {
    let mut result = [None; 6];
    if let Some((baro, mag)) = filter.core().aid_status() {
        result[0] = Some(baro);
        result[1] = Some(mag);
    }
    result[2] = filter.core().range_status();
    for (destination, status) in result[3..].iter_mut().zip(filter.status()) {
        *destination = Some(status);
    }
    result
}

/// Synthetic north acceleration reaches 90 m after the 60-second interval.
fn truth(timestamp: u64) -> (f32, f64, f64) {
    let yaw = (timestamp.saturating_sub(1_000_000) as f32 * 1e-6) * 5_f32.to_radians();
    let seconds = timestamp.saturating_sub(WARMUP_US) as f64 * 1e-6;
    (
        yaw,
        0.5 * f64::from(ACCELERATION) * seconds * seconds,
        f64::from(ACCELERATION) * seconds,
    )
}

fn information(file: &mut impl Write, name: &str, value: &str) -> Result<()> {
    let key = format!("char[{}] {name}", value.len());
    file.write_all(ulog::information(&key, value.as_bytes())?.as_bytes())?;
    Ok(())
}

fn write_header(file: &mut impl Write) -> Result<()> {
    file.write_all(&ulog::file_header(0))?;
    file.write_all(ulog::flag_bits().as_bytes())?;
    information(file, "sys_name", "rust_ap")?;
    information(file, "px4_msg_revision", PX4_REVISION)?;
    information(file, "body_frame", "FRD")?;
    information(file, "world_frame", "NED")?;
    information(
        file,
        "rust_e6_fixture",
        "synthetic host only; 5s warmup + 60s north acceleration",
    )?;
    let mut format_buffer = [0; standard_ulog::FORMAT_CAPACITY];
    for topic in &standard_ulog::TOPICS {
        let length = standard_ulog::format_record(topic.id, &mut format_buffer)?;
        file.write_all(&format_buffer[..length])?;
    }
    for parameter in EKF2_PARAMS {
        let record = match parameter.default {
            ParamValue::Int(value) => ulog::parameter_i32(parameter.name, value)?,
            ParamValue::Float(value) => ulog::parameter_f32(parameter.name, value)?,
        };
        file.write_all(record.as_bytes())?;
    }
    for topic in &standard_ulog::TOPICS {
        file.write_all(standard_ulog::subscription(topic.id)?.as_bytes())?;
    }
    Ok(())
}

fn trace_header(file: &mut impl Write) -> Result<()> {
    write!(file, "timestamp,latest,delayed,healthy,control,fault,global_lat,global_lon,global_alt,truth_yaw,truth_north,truth_velocity")?;
    for (prefix, count) in [
        ("q", 4),
        ("v", 3),
        ("p", 3),
        ("state", 25),
        ("cov", 24),
        ("reset", 6),
        ("delta_q", 4),
        ("delta_v", 3),
        ("delta_p", 3),
        ("tracking", 3),
    ] {
        for index in 0..count {
            write!(file, ",{prefix}{index}")?;
        }
    }
    writeln!(file, ",delta_hagl")?;
    Ok(())
}

fn write_trace(file: &mut impl Write, core: &ImuEkf, now: u64) -> Result<()> {
    let output = core.output();
    let (latitude, longitude, altitude) = core.output_global_position();
    let (yaw, north, velocity) = truth(output.timestamp);
    write!(file, "{now},{},{},{},{},{},{latitude:.12},{longitude:.12},{altitude:.9},{yaw:.9},{north:.9},{velocity:.9}",
        output.timestamp, core.timestamp(), u8::from(core.health().valid), core.control_status(), core.fault_status())?;
    let resets = core.reset_report();
    for vector in [
        &output.q_body_to_ned[..],
        &output.velocity_ned_m_s,
        &output.position_ned_m,
        core.state(),
    ] {
        for value in vector {
            write!(file, ",{value:.9e}")?;
        }
    }
    for index in 0..24 {
        write!(file, ",{:.9e}", core.covariance()[index][index])?;
    }
    for count in resets.counts {
        write!(file, ",{count}")?;
    }
    for vector in [
        &resets.quaternion_delta[..],
        &resets.velocity_delta,
        &resets.position_delta,
        &core.output_tracking_error(),
    ] {
        for value in vector {
            write!(file, ",{value:.9e}")?;
        }
    }
    writeln!(file, ",{:.9e}", resets.hagl_delta)?;
    Ok(())
}

fn run(directory: &Path) -> Result<()> {
    fs::create_dir_all(directory)?;
    let mut log = BufWriter::new(File::create(directory.join("estimator.ulg"))?);
    let mut frames = BufWriter::new(File::create(directory.join("mavlink.bin"))?);
    let mut trace = BufWriter::new(File::create(directory.join("core.csv"))?);
    let mut aid_trace = BufWriter::new(File::create(directory.join("aids.csv"))?);
    write_header(&mut log)?;
    trace_header(&mut trace)?;
    writeln!(aid_trace, "timestamp,slot,sample,innovation0,innovation1,innovation2,variance0,variance1,variance2,ratio0,ratio1,ratio2,fused,rejected")?;
    let mut filter = GnssEkf::new(
        ImuEkfParams::default(),
        BaroMagParams::default(),
        RangeParams::default(),
        GnssParams::default(),
        0,
    )
    .map_err(|fault| format!("EKF construction failed: {fault:?}"))?;
    let mut encoder = standard_ulog::DataEncoder::new();
    let mut frame_buffer = [0; mavlink::MAX_FRAME_SIZE];
    let mut sequence = 0_u8;
    let mut frame_count = 0;
    let mut publications = 0;
    let mut last_aid = [0; 6];
    for index in 1..=END_US / DT_US {
        update(&mut filter, index)?;
        if !index.is_multiple_of(20) {
            continue;
        }
        let now = index * DT_US + 100; // Distinct sample/publication clocks.
        write_trace(&mut trace, filter.core(), now)?;
        for (slot, status) in aids(&filter).into_iter().enumerate() {
            let Some(status) = status else {
                continue;
            };
            if status.timestamp <= last_aid[slot] {
                continue;
            }
            write!(aid_trace, "{now},{slot},{}", status.timestamp)?;
            for vector in [status.innovation, status.variance, status.ratio] {
                for value in vector {
                    write!(aid_trace, ",{value:.9e}")?;
                }
            }
            writeln!(
                aid_trace,
                ",{},{}",
                u8::from(status.fused),
                u8::from(status.rejected)
            )?;
            last_aid[slot] = status.timestamp;
        }
        let snapshot = filter.standard_output(now);
        encoder.write_snapshot::<Box<dyn Error>>(&snapshot, |record| {
            log.write_all(record.as_bytes())?;
            Ok(())
        })?;
        for message in mavlink_estimator::messages(&snapshot, None)
            .into_iter()
            .flatten()
        {
            let header = MavHeader {
                system_id: 1,
                component_id: 1,
                sequence,
            };
            let length = mavlink_estimator::encode(header, &message, &mut frame_buffer);
            sequence = sequence.wrapping_add(1);
            frames.write_all(&frame_buffer[..length])?;
            frame_count += 1;
        }
        publications += 1;
    }
    log.flush()?;
    frames.flush()?;
    trace.flush()?;
    aid_trace.flush()?;
    let summary = format!(
        "{{\"schema\":1,\"px4_ref\":\"{PX4_REVISION}\",\"mode\":\"gnss\",\"synthetic\":true,\"samples\":{},\"publications\":{publications},\"mavlink_frames\":{frame_count},\"warmup_seconds\":5,\"drift_seconds\":60,\"acceleration_m_s2\":0.05,\"free_integral_position_m\":90,\"hardware_validation\":\"not run\"}}",
        END_US / DT_US,
    );
    fs::write(directory.join("fixture.json"), format!("{summary}\n"))?;
    println!("{summary}");
    Ok(())
}

fn main() {
    let arguments: Vec<_> = env::args().skip(1).collect();
    let result = match arguments.as_slice() {
        [directory] => run(Path::new(directory)),
        _ => Err("usage: ekf2_standard <output-directory>".into()),
    };
    if let Err(error) = result {
        eprintln!("{error}");
        std::process::exit(1);
    }
}
