//! Host CSV intake / optional E1 IMU-core dump. Both modes hash the input DTO;
//! an output path enables Rust estimation, never firmware or hardware execution.

use std::{
    env,
    error::Error,
    fs::File,
    io::{BufRead, BufReader, BufWriter, Write},
    path::Path,
};

use v6x_rust_led::estimator::imu_ekf::baro_mag::{BaroMagParams, BaroMagSample};
use v6x_rust_led::estimator::{
    gnss::{GnssEkf, GnssObservation, GnssParams},
    imu_ekf::range::RangeParams,
    imu_ekf::{ImuEkf, ImuEkfParams},
    output_predictor::OutputParams,
    range_sensor::RangeObservation,
    GnssSample, ImuDelta,
};

const HEADER: &str = concat!(
    "timestamp_us,delta_angle_dt_us,delta_velocity_dt_us,delta_angle_x_rad,delta_angle_y_rad,",
    "delta_angle_z_rad,delta_velocity_x_m_s,delta_velocity_y_m_s,delta_velocity_z_m_s,",
    "delta_velocity_clipping"
);
const BARO_MAG_HEADER: &str = "timestamp_us,baro_altitude_m,mag_x_gauss,mag_y_gauss,mag_z_gauss";
const EXTRA_QUALITY_HEADER: &str = ",gps_fix_type,gps_hacc_m,gps_vacc_m,gps_sacc_m_s,gps_satellites,gps_pdop,gps_spoofed,range_valid";
const FIVE_SENSOR_HEADER: &str = concat!("timestamp_us,baro_altitude_m,mag_x_gauss,mag_y_gauss,mag_z_gauss,",
    "gps_valid,gps_latitude_deg,gps_longitude_deg,gps_altitude_m,gps_vn_m_s,gps_ve_m_s,gps_vd_m_s,range_distance_m,range_quality");

#[derive(Clone, Copy)]
struct AidInput {
    baro_mag: BaroMagSample,
    gnss: Option<GnssObservation>,
    range: Option<RangeObservation>,
}

// Heap storage is confined to this std host harness, not the estimator kernel.
enum HostFilter {
    Basic(Box<ImuEkf>),
    Main(Box<GnssEkf>),
}

impl HostFilter {
    fn core(&self) -> &ImuEkf {
        match self {
            Self::Basic(filter) => filter,
            Self::Main(filter) => filter.core(),
        }
    }
    fn update(
        &mut self,
        imu: &ImuDelta,
        aiding: Option<AidInput>,
    ) -> Result<bool, v6x_rust_led::estimator::FilterFault> {
        match self {
            Self::Basic(filter) => filter.update_aided(imu, aiding.map(|a| a.baro_mag)),
            Self::Main(filter) => filter.update(
                imu,
                aiding.map(|a| a.baro_mag),
                aiding.and_then(|a| a.gnss),
                aiding.and_then(|a| a.range),
            ),
        }
    }
}

fn unsigned<T: std::str::FromStr>(text: &str) -> Result<T, &'static str> {
    if text.is_empty() || !text.bytes().all(|byte| byte.is_ascii_digit()) {
        return Err("invalid unsigned IMU field");
    }
    text.parse().map_err(|_| "unsigned IMU field out of range")
}

fn finite(text: &str) -> Result<f32, &'static str> {
    let value: f32 = text.parse().map_err(|_| "invalid IMU float")?;
    if !value.is_finite() {
        return Err("non-finite IMU delta");
    }
    Ok(value)
}

fn sample(line: &str, previous: u64) -> Result<ImuDelta, &'static str> {
    let fields: Vec<_> = line.split(',').collect();
    if fields.len() != 10 {
        return Err("expected ten IMU fields");
    }
    let timestamp: u64 = unsigned(fields[0])?;
    let angle_dt: u32 = unsigned(fields[1])?;
    let velocity_dt: u32 = unsigned(fields[2])?;
    let clipping: u8 = unsigned(fields[9])?;
    if timestamp <= previous
        || !(1..=1_000_000).contains(&angle_dt)
        || !(1..=1_000_000).contains(&velocity_dt)
        || timestamp < u64::from(angle_dt)
        || timestamp < u64::from(velocity_dt)
        || clipping > 7
    {
        return Err("invalid IMU time or clipping mask");
    }
    Ok(ImuDelta {
        timestamp,
        delta_angle_rad: [finite(fields[3])?, finite(fields[4])?, finite(fields[5])?],
        delta_velocity_m_s: [finite(fields[6])?, finite(fields[7])?, finite(fields[8])?],
        delta_angle_dt_s: angle_dt as f32 * 1e-6,
        delta_velocity_dt_s: velocity_dt as f32 * 1e-6,
        delta_velocity_clipping: core::array::from_fn(|axis| clipping & (1 << axis) != 0),
    })
}

fn hash_bytes(hash: &mut u64, bytes: &[u8]) {
    for byte in bytes {
        *hash = (*hash ^ u64::from(*byte)).wrapping_mul(0x100_0000_01b3);
    }
}

fn hash_sample(hash: &mut u64, imu: &ImuDelta) {
    hash_bytes(hash, &imu.timestamp.to_le_bytes());
    for value in [imu.delta_angle_dt_s, imu.delta_velocity_dt_s]
        .into_iter()
        .chain(imu.delta_angle_rad)
        .chain(imu.delta_velocity_m_s)
    {
        hash_bytes(hash, &value.to_bits().to_le_bytes());
    }
    let clipping = imu
        .delta_velocity_clipping
        .iter()
        .enumerate()
        .fold(0, |mask, (axis, clipped)| {
            mask | (u8::from(*clipped) << axis)
        });
    hash_bytes(hash, &[clipping]);
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = env::args().collect();
    let flags = args.get(4..).unwrap_or_default();
    let nis = flags.iter().any(|flag| flag == "--nis");
    let output_mode = flags.iter().any(|flag| flag == "--output");
    let lever_arm = flags.iter().any(|flag| flag == "--lever-arm");
    if !(2..=6).contains(&args.len())
        || flags
            .iter()
            .any(|flag| !["--nis", "--lever-arm", "--output"].contains(&flag.as_str()))
        || (nis && output_mode)
    {
        return Err("Usage: ekf2_input INPUT.csv [OUTPUT.csv [BARO_MAG.csv [--nis]]]".into());
    }
    if args.len() >= 3
        && Path::new(&args[2]).exists()
        && Path::new(&args[1]).canonicalize()? == Path::new(&args[2]).canonicalize()?
    {
        return Err("input and output must be distinct files".into());
    }
    let mut lines = BufReader::new(File::open(&args[1])?).lines();
    if lines.next().ok_or("empty input")?? != HEADER {
        return Err("unexpected IMU CSV header".into());
    }
    let mut digest = 0xcbf2_9ce4_8422_2325;
    let mut count = 0u64;
    let mut first = 0;
    let mut previous = 0;
    let mut aid_samples = Vec::new();
    let mut five_sensor = false;
    if args.len() >= 4 {
        let mut lines = BufReader::new(File::open(&args[3])?).lines();
        let header = lines.next().ok_or("empty aid input")??;
        let extended = header == format!("{FIVE_SENSOR_HEADER}{EXTRA_QUALITY_HEADER}");
        five_sensor = header == FIVE_SENSOR_HEADER || extended;
        if header != BARO_MAG_HEADER && !five_sensor {
            return Err("unexpected baro/mag CSV header".into());
        }
        let mut previous = 0;
        for line in lines {
            let line = line?;
            let fields: Vec<_> = line.split(',').collect();
            if fields.len()
                != if extended {
                    22
                } else if five_sensor {
                    14
                } else {
                    5
                }
            {
                return Err("unexpected aid field count".into());
            }
            let timestamp: u64 = unsigned(fields[0])?;
            if timestamp <= previous {
                return Err("aid time is not strictly increasing".into());
            }
            previous = timestamp;
            let baro_mag = BaroMagSample {
                timestamp,
                altitude_m: finite(fields[1])?,
                field_gauss: [finite(fields[2])?, finite(fields[3])?, finite(fields[4])?],
            };
            let (gnss, range) = if five_sensor {
                let ready: u8 = unsigned(fields[5])?;
                if ready > 1 {
                    return Err("invalid GNSS ready flag".into());
                }
                let mut sample = GnssSample {
                    timestamp,
                    latitude_deg: fields[6].parse()?,
                    longitude_deg: fields[7].parse()?,
                    altitude_msl_m: finite(fields[8])?,
                    velocity_ned_m_s: [
                        finite(fields[9])?,
                        finite(fields[10])?,
                        finite(fields[11])?,
                    ],
                    horizontal_accuracy_m: 0.3,
                    vertical_accuracy_m: 0.5,
                    speed_accuracy_m_s: 0.1,
                    fix_type: 3,
                    satellites: 12,
                };
                let (pdop, spoofed, range_ready) = if extended {
                    sample.fix_type = unsigned(fields[14])?;
                    sample.horizontal_accuracy_m = finite(fields[15])?;
                    sample.vertical_accuracy_m = finite(fields[16])?;
                    sample.speed_accuracy_m_s = finite(fields[17])?;
                    sample.satellites = unsigned(fields[18])?;
                    let spoofed: u8 = unsigned(fields[20])?;
                    let range_ready: u8 = unsigned(fields[21])?;
                    if spoofed > 1 || range_ready > 1 {
                        return Err("invalid GNSS/range readiness".into());
                    }
                    (finite(fields[19])?, spoofed == 1, range_ready == 1)
                } else {
                    (1.0, false, true)
                };
                if !sample.latitude_deg.is_finite()
                    || !sample.longitude_deg.is_finite()
                    || sample.latitude_deg.abs() > 90.0
                    || sample.longitude_deg.abs() > 180.0
                {
                    return Err("invalid GNSS coordinate".into());
                }
                let range = RangeObservation {
                    timestamp,
                    distance_m: finite(fields[12])?,
                    quality_percent: fields[13].parse()?,
                };
                if !(-1..=100).contains(&range.quality_percent) {
                    return Err("invalid range quality".into());
                }
                (
                    if ready == 1 {
                        Some(GnssObservation {
                            sample,
                            pdop,
                            spoofed,
                        })
                    } else {
                        None
                    },
                    if range_ready { Some(range) } else { None },
                )
            } else {
                (None, None)
            };
            aid_samples.push(AidInput {
                baro_mag,
                gnss,
                range,
            });
        }
        if aid_samples.is_empty() {
            return Err("no aid samples".into());
        }
    }
    let mut aid_samples = aid_samples.into_iter().peekable();
    let mut consumed_aids = 0;
    let mut filter = if five_sensor {
        let mut gnss_params = GnssParams::default();
        let mut range_params = RangeParams::default();
        if lever_arm {
            let imu = [0.03_f32, -0.02, 0.01];
            gnss_params.offset_body_m = core::array::from_fn(|i| [0.2, 0.1, -0.05][i] - imu[i]);
            range_params.offset_body_m = core::array::from_fn(|i| [0.1, -0.05, -0.1][i] - imu[i]);
        }
        let mut main = GnssEkf::new(
            ImuEkfParams::default(),
            BaroMagParams::default(),
            range_params,
            gnss_params,
            0,
        )
        .map_err(|fault| format!("EKF parameters: {fault:?}"))?;
        if lever_arm {
            main.configure_output(OutputParams {
                imu_offset_body_m: [0.03, -0.02, 0.01],
                ..OutputParams::default()
            })
            .map_err(|fault| format!("output parameters: {fault:?}"))?;
        }
        HostFilter::Main(Box::new(main))
    } else {
        let basic = if args.len() >= 4 {
            ImuEkf::with_baro_mag(&ImuEkfParams::default(), BaroMagParams::default(), 0)
        } else {
            ImuEkf::new(&ImuEkfParams::default(), 0)
        };
        HostFilter::Basic(Box::new(
            basic.map_err(|fault| format!("EKF parameters: {fault:?}"))?,
        ))
    };
    let mut output = if args.len() >= 3 {
        Some(BufWriter::new(File::create_new(&args[2])?))
    } else {
        None
    };
    if let Some(output) = &mut output {
        if output_mode {
            writeln!(output, "timestamp_us,delayed_timestamp_us,control_status,fault_status,q_w,q_x,q_y,q_z,velocity_n,velocity_e,velocity_d,position_n,position_e,position_d,height_msl_m,latitude_deg,longitude_deg,vertical_derivative,unaided_yaw,track_angle,track_velocity,track_position")?;
        } else {
            write!(
                output,
                "timestamp_us,imu_timestamp_us,control_status,fault_status"
            )?;
            if nis {
                write!(output, ",baro_timestamp_us,baro_innovation,baro_variance,baro_ratio,baro_fused,baro_rejected,mag_timestamp_us")?;
                for kind in ["innovation", "variance", "ratio"] {
                    for i in 0..3 {
                        write!(output, ",mag_{kind}_{i}")?;
                    }
                }
                write!(output, ",mag_fused,mag_rejected")?;
                if five_sensor {
                    for (name, dimensions) in [
                        ("gnss_pos", 2),
                        ("gnss_vel", 3),
                        ("gnss_hgt", 1),
                        ("range", 1),
                    ] {
                        write!(output, ",{name}_timestamp_us")?;
                        for kind in ["innovation", "variance", "ratio"] {
                            for i in 0..dimensions {
                                write!(output, ",{name}_{kind}_{i}")?;
                            }
                        }
                        write!(output, ",{name}_fused,{name}_rejected")?;
                    }
                    write!(output, ",gnss_quality_failure_bits")?;
                }
            } else {
                for index in 0..25 {
                    write!(output, ",state_{index}")?;
                }
                for row in 0..24 {
                    for column in 0..24 {
                        write!(output, ",cov_{row}_{column}")?;
                    }
                }
            }
            writeln!(output)?;
        }
    }
    let mut updates = 0;
    for line in lines {
        let imu = sample(&line?, previous)?;
        if count == 0 {
            first = imu.timestamp;
        }
        previous = imu.timestamp;
        count += 1;
        hash_sample(&mut digest, &imu);
        if let Some(output) = &mut output {
            let aiding = if aid_samples
                .peek()
                .is_some_and(|sample| sample.baro_mag.timestamp <= imu.timestamp)
            {
                consumed_aids += 1;
                aid_samples.next()
            } else {
                None
            };
            if aid_samples
                .peek()
                .is_some_and(|sample| sample.baro_mag.timestamp <= imu.timestamp)
            {
                return Err("more than one aid event per IMU sample is not supported".into());
            }
            let updated = filter
                .update(&imu, aiding)
                .map_err(|fault| format!("IMU EKF: {fault:?}"))?;
            if updated {
                updates += 1;
            }
            if output_mode {
                let core = filter.core();
                let snapshot = core.output();
                write!(
                    output,
                    "{},{},{},{}",
                    snapshot.timestamp,
                    core.timestamp(),
                    core.control_status(),
                    core.fault_status()
                )?;
                for value in snapshot
                    .q_body_to_ned
                    .into_iter()
                    .chain(snapshot.velocity_ned_m_s)
                    .chain(snapshot.position_ned_m)
                {
                    write!(output, ",{value:.9e}")?;
                }
                let (lat, lon, _) = core.output_global_position();
                write!(
                    output,
                    ",{:.9e},{lat:.17e},{lon:.17e},{:.9e},{:.9e}",
                    snapshot.height_msl_m,
                    core.output_vertical_derivative(),
                    core.output_unaided_yaw()
                )?;
                for value in core.output_tracking_error() {
                    write!(output, ",{value:.9e}")?;
                }
                writeln!(output)?;
            } else if updated {
                let core = filter.core();
                write!(
                    output,
                    "{},{},{},{}",
                    core.timestamp(),
                    imu.timestamp,
                    core.control_status(),
                    core.fault_status()
                )?;
                if nis {
                    let (baro, mag) = core.aid_status().ok_or("missing aid status")?;
                    write!(
                        output,
                        ",{},{:.9e},{:.9e},{:.9e},{},{},{}",
                        baro.timestamp,
                        baro.innovation[0],
                        baro.variance[0],
                        baro.ratio[0],
                        u8::from(baro.fused),
                        u8::from(baro.rejected),
                        mag.timestamp
                    )?;
                    for values in [mag.innovation, mag.variance, mag.ratio] {
                        for value in values {
                            write!(output, ",{value:.9e}")?;
                        }
                    }
                    write!(
                        output,
                        ",{},{}",
                        u8::from(mag.fused),
                        u8::from(mag.rejected)
                    )?;
                    if let HostFilter::Main(main) = &filter {
                        let statuses = main.status();
                        for (status, dimensions) in [
                            (statuses[0], 2),
                            (statuses[1], 3),
                            (statuses[2], 1),
                            (core.range_status().ok_or("missing range status")?, 1),
                        ] {
                            write!(output, ",{}", status.timestamp)?;
                            for values in [status.innovation, status.variance, status.ratio] {
                                for value in values.iter().take(dimensions) {
                                    write!(output, ",{value:.9e}")?;
                                }
                            }
                            write!(
                                output,
                                ",{},{}",
                                u8::from(status.fused),
                                u8::from(status.rejected)
                            )?;
                        }
                        write!(output, ",{}", main.quality_failure_bits())?;
                    }
                } else {
                    for value in core
                        .state()
                        .iter()
                        .chain(core.covariance().iter().flatten())
                    {
                        write!(output, ",{value:.9e}")?;
                    }
                }
                writeln!(output)?;
            }
        }
    }
    if count == 0 {
        return Err("no IMU samples".into());
    }
    if aid_samples.peek().is_some() {
        return Err("aid sample lies after last IMU sample".into());
    }
    if let Some(output) = &mut output {
        output.flush()?;
    }
    println!(
        "{{\"samples\":{count},\"updates\":{updates},\"aid_samples\":{consumed_aids},\"first_timestamp_us\":{first},\"last_timestamp_us\":{previous},\"input_digest\":\"{digest:016x}\"}}"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const ROW: &str = "2000,1000,2000,0.1,0.2,0.3,1,2,3,5";

    #[test]
    fn csv_preserves_units_distinct_intervals_and_axis_clipping() {
        let imu = sample(ROW, 0).unwrap();
        assert_eq!(imu.timestamp, 2000);
        assert_eq!(imu.delta_angle_rad, [0.1, 0.2, 0.3]);
        assert_eq!(imu.delta_velocity_m_s, [1.0, 2.0, 3.0]);
        assert_eq!(imu.delta_angle_dt_s, 0.001);
        assert_eq!(imu.delta_velocity_dt_s, 0.002);
        assert_eq!(imu.delta_velocity_clipping, [true, false, true]);
    }

    #[test]
    fn csv_rejects_invalid_fields_and_non_monotonic_time() {
        for invalid in [
            "2000,1000,2000,0,0,0,1,2,3",
            "2000,0,2000,0,0,0,1,2,3,0",
            "2000,1000,2000,NaN,0,0,1,2,3,0",
            "2000,1000,2000,0,0,0,1,2,3,8",
            "2000,1000,2000,0,0,0,1,2,3,256",
            "-2000,1000,2000,0,0,0,1,2,3,0",
        ] {
            assert!(sample(invalid, 0).is_err(), "{invalid}");
        }
        assert!(sample(ROW, 2000).is_err());
    }
}
