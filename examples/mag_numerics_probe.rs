//! Offline replay of one captured parameter set over synthetic or logged samples.
//! Host diagnostics only: it never writes parameters or contacts the board.

use std::{env, fs};

use v6x_rust_led::{
    estimator::{
        gnss::GnssEkf, imu_ekf::baro_mag::BaroAltitude, imu_ekf::ImuEkf, product::Params,
        runtime::Runtime, ImuDelta, MagSample, Measurement,
    },
    parameters::{Snapshot, COUNT},
};

const STATE_COUNT: usize = 24;

fn main() {
    let args: Vec<_> = env::args().collect();
    let values: Vec<f32> = fs::read_to_string(&args[1])
        .unwrap()
        .split_whitespace()
        .map(|v| v.parse().unwrap())
        .collect();
    let values: [f32; COUNT] = values.try_into().unwrap();
    let params = Params::from_parameters(Snapshot::from_values(values).unwrap()).unwrap();
    let mut runtime = Runtime::<GnssEkf>::new(params, 0).unwrap();
    match args.len() {
        3 => run_synthetic(&mut runtime, &args[2]),
        4 => run_log(&mut runtime, &args[2], &args[3]),
        _ => panic!("usage: probe PARAMS INPUT | PARAMS IMU.csv EVENTS.csv"),
    }
}

fn run_synthetic(runtime: &mut Runtime<GnssEkf>, path: &str) {
    let input: Vec<f32> = fs::read_to_string(path)
        .unwrap()
        .split_whitespace()
        .map(|v| v.parse().unwrap())
        .collect();
    for index in 1..=30_000_u64 {
        let time = 1_000_000 + index * 1_000;
        let mut aids = Vec::new();
        if index % 45 == 0 {
            aids.push(Measurement::Baro(BaroAltitude {
                timestamp: time,
                altitude_m: 100.0,
            }));
        }
        if index % 50 == 0 {
            aids.push(Measurement::Mag(MagSample {
                timestamp: time,
                field_gauss: [input[6], input[7], input[8]],
                reset: index == 50,
            }));
        }
        let delta = ImuDelta {
            timestamp: time,
            delta_angle_rad: [input[3] * 0.001, input[4] * 0.001, input[5] * 0.001],
            delta_velocity_m_s: [input[0] * 0.001, input[1] * 0.001, input[2] * 0.001],
            delta_angle_dt_s: 0.001,
            delta_velocity_dt_s: 0.001,
            delta_velocity_clipping: [false; 3],
        };
        if advance(runtime, delta, &aids) {
            return;
        }
    }
}

fn run_log(runtime: &mut Runtime<GnssEkf>, imu_path: &str, events_path: &str) {
    let imu = fs::read_to_string(imu_path).unwrap();
    let events: Vec<(u64, Measurement)> = fs::read_to_string(events_path)
        .unwrap()
        .lines()
        .skip(1)
        .map(|line| {
            let cells: Vec<&str> = line.split(',').collect();
            let stamp: u64 = cells[0].parse().unwrap();
            let value = |index: usize| cells[index].parse::<f32>().unwrap();
            let measurement = if cells[1] == "B" {
                Measurement::Baro(BaroAltitude {
                    timestamp: stamp,
                    altitude_m: value(2),
                })
            } else {
                Measurement::Mag(MagSample {
                    timestamp: stamp,
                    field_gauss: [value(2), value(3), value(4)],
                    reset: false,
                })
            };
            (stamp, measurement)
        })
        .collect();
    let mut pending = 0;
    for line in imu.lines().skip(1) {
        let cells: Vec<&str> = line.split(',').collect();
        let number = |index: usize| cells[index].parse::<f32>().unwrap();
        let delta = ImuDelta {
            timestamp: cells[0].parse().unwrap(),
            delta_angle_rad: [number(3), number(4), number(5)],
            delta_velocity_m_s: [number(6), number(7), number(8)],
            delta_angle_dt_s: number(1) * 1e-6,
            delta_velocity_dt_s: number(2) * 1e-6,
            delta_velocity_clipping: [false; 3],
        };
        let mut aids = Vec::new();
        while pending < events.len() && events[pending].0 <= delta.timestamp {
            aids.push(events[pending].1);
            pending += 1;
        }
        if advance(runtime, delta, &aids) {
            return;
        }
    }
}

fn advance(runtime: &mut Runtime<GnssEkf>, imu: ImuDelta, aids: &[Measurement]) -> bool {
    let now = imu.timestamp;
    let _ = runtime.ingest_imu(imu, now);
    for aid in aids {
        let _ = runtime.apply_measurement(*aid, now);
    }
    runtime.maintain(now);
    let status = runtime.status();
    if now % 1_000_000 < 5_000 {
        let core = runtime.filter().core();
        println!(
            "t={:.3}s mode={:?} fault={:?} qerr={:.6} neg={:?} asym={:?} {} control={:x} measurements={} rejected={}",
            now as f32 * 1e-6,
            status.mode,
            status.fault,
            quaternion_error(core),
            negative_variances(core),
            asymmetry(core),
            if finite(core) { "finite" } else { "NON-FINITE" },
            core.control_status(),
            status.measurements,
            status.rejected_measurements,
        );
    }
    if status.fault.is_some() {
        let core = runtime.filter().core();
        println!(
            "fault at t={:.3}s mode={:?} fault={:?} state={:?}\nvariances={:?}\nnegative={:?}\nasymmetry={:?}",
            now as f32 * 1e-6,
            status.mode,
            status.fault,
            core.state(),
            (0..STATE_COUNT).map(|i| core.covariance()[i][i]).collect::<Vec<_>>(),
            negative_variances(core),
            asymmetry(core),
        );
        return true;
    }
    false
}

fn quaternion_error(core: &ImuEkf) -> f32 {
    let state = core.state();
    (state[..4].iter().map(|v| v * v).sum::<f32>().sqrt() - 1.0).abs()
}

fn negative_variances(core: &ImuEkf) -> Vec<usize> {
    let covariance = core.covariance();
    (0..STATE_COUNT)
        .filter(|i| covariance[*i][*i] < 0.0)
        .collect()
}

fn asymmetry(core: &ImuEkf) -> Vec<(usize, usize)> {
    let covariance = core.covariance();
    (0..STATE_COUNT)
        .flat_map(|i| {
            (0..i)
                .filter(move |j| covariance[i][*j] != covariance[*j][i])
                .map(move |j| (i, j))
        })
        .collect()
}

fn finite(core: &ImuEkf) -> bool {
    core.state().iter().all(|v| v.is_finite())
        && core.covariance().iter().flatten().all(|v| v.is_finite())
}
