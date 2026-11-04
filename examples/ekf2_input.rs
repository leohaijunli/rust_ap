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
    imu_ekf::{ImuEkf, ImuEkfParams},
    ImuDelta,
};

const HEADER: &str = concat!(
    "timestamp_us,delta_angle_dt_us,delta_velocity_dt_us,delta_angle_x_rad,delta_angle_y_rad,",
    "delta_angle_z_rad,delta_velocity_x_m_s,delta_velocity_y_m_s,delta_velocity_z_m_s,",
    "delta_velocity_clipping"
);

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
    if !(2..=5).contains(&args.len()) || (args.len() == 5 && args[4] != "--nis") {
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
    if args.len() >= 4 {
        let mut lines = BufReader::new(File::open(&args[3])?).lines();
        if lines.next().ok_or("empty aid input")??
            != "timestamp_us,baro_altitude_m,mag_x_gauss,mag_y_gauss,mag_z_gauss"
        {
            return Err("unexpected baro/mag CSV header".into());
        }
        let mut previous = 0;
        for line in lines {
            let line = line?;
            let fields: Vec<_> = line.split(',').collect();
            if fields.len() != 5 {
                return Err("expected five aid fields".into());
            }
            let timestamp: u64 = unsigned(fields[0])?;
            if timestamp <= previous {
                return Err("aid time is not strictly increasing".into());
            }
            previous = timestamp;
            aid_samples.push(BaroMagSample {
                timestamp,
                altitude_m: finite(fields[1])?,
                field_gauss: [finite(fields[2])?, finite(fields[3])?, finite(fields[4])?],
            });
        }
        if aid_samples.is_empty() {
            return Err("no aid samples".into());
        }
    }
    let mut aid_samples = aid_samples.into_iter().peekable();
    let mut consumed_aids = 0;
    let mut filter = if args.len() >= 4 {
        ImuEkf::with_baro_mag(&ImuEkfParams::default(), BaroMagParams::default(), 0)
    } else {
        ImuEkf::new(&ImuEkfParams::default(), 0)
    }
    .map_err(|fault| format!("EKF parameters: {fault:?}"))?;
    let mut output = if args.len() >= 3 {
        Some(BufWriter::new(File::create_new(&args[2])?))
    } else {
        None
    };
    if let Some(output) = &mut output {
        write!(
            output,
            "timestamp_us,imu_timestamp_us,control_status,fault_status"
        )?;
        if args.len() == 5 {
            write!(output, ",baro_timestamp_us,baro_innovation,baro_variance,baro_ratio,baro_fused,baro_rejected,mag_timestamp_us")?;
            for kind in ["innovation", "variance", "ratio"] {
                for i in 0..3 {
                    write!(output, ",mag_{kind}_{i}")?;
                }
            }
            write!(output, ",mag_fused,mag_rejected")?;
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
                .is_some_and(|sample| sample.timestamp <= imu.timestamp)
            {
                consumed_aids += 1;
                aid_samples.next()
            } else {
                None
            };
            if aid_samples
                .peek()
                .is_some_and(|sample| sample.timestamp <= imu.timestamp)
            {
                return Err("more than one aid event per IMU sample is not supported".into());
            }
            if filter
                .update_aided(&imu, aiding)
                .map_err(|fault| format!("IMU EKF: {fault:?}"))?
            {
                updates += 1;
                write!(
                    output,
                    "{},{},{},{}",
                    filter.timestamp(),
                    imu.timestamp,
                    filter.control_status(),
                    filter.fault_status()
                )?;
                if args.len() == 5 {
                    let (baro, mag) = filter.aid_status().ok_or("missing aid status")?;
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
                } else {
                    for value in filter
                        .state()
                        .iter()
                        .chain(filter.covariance().iter().flatten())
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
