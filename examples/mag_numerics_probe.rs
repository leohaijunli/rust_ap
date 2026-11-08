//! Offline synthetic replay using measured stationary values and captured parameters.
use std::{env, fs};
use v6x_rust_led::{
    estimator::{
        gnss::GnssEkf, imu_ekf::baro_mag::BaroAltitude, product::Params, runtime::Runtime,
        ImuDelta, MagSample, Measurement,
    },
    parameters::{Snapshot, COUNT},
};

fn main() {
    let args: Vec<_> = env::args().collect();
    let values: Vec<f32> = fs::read_to_string(&args[1])
        .unwrap()
        .split_whitespace()
        .map(|v| v.parse().unwrap())
        .collect();
    let values: [f32; COUNT] = values.try_into().unwrap();
    let input: Vec<f32> = fs::read_to_string(&args[2])
        .unwrap()
        .split_whitespace()
        .map(|v| v.parse().unwrap())
        .collect();
    let params = Params::from_parameters(Snapshot::from_values(values).unwrap()).unwrap();
    let mut runtime = Runtime::<GnssEkf>::new(params, 0).unwrap();
    for index in 1..=30_000_u64 {
        let time = 1_000_000 + index * 1000;
        let delta = ImuDelta {
            timestamp: time,
            delta_angle_rad: [input[3] * 0.001, input[4] * 0.001, input[5] * 0.001],
            delta_velocity_m_s: [input[0] * 0.001, input[1] * 0.001, input[2] * 0.001],
            delta_angle_dt_s: 0.001,
            delta_velocity_dt_s: 0.001,
            delta_velocity_clipping: [false; 3],
        };
        runtime.ingest_imu(delta, time).unwrap();
        if index % 45 == 0 {
            runtime
                .apply_measurement(
                    Measurement::Baro(BaroAltitude {
                        timestamp: time,
                        altitude_m: 100.0,
                    }),
                    time,
                )
                .unwrap();
        }
        if index % 50 == 0 {
            runtime
                .apply_measurement(
                    Measurement::Mag(MagSample {
                        timestamp: time,
                        field_gauss: [input[6], input[7], input[8]],
                        reset: index == 50,
                    }),
                    time,
                )
                .unwrap();
        }
        runtime.maintain(time);
        if index % 1000 == 0 || runtime.status().fault.is_some() {
            let core = runtime.filter().core();
            let state = core.state();
            let cov = core.covariance();
            let qnorm = state[..4].iter().map(|v| v * v).sum::<f32>().sqrt();
            let negative: Vec<_> = (0..24).filter(|i| cov[*i][*i] < 0.0).collect();
            let asymmetric: Vec<_> = (0..24)
                .flat_map(|i| {
                    (0..i)
                        .filter(move |j| cov[i][*j] != cov[*j][i])
                        .map(move |j| (i, j, cov[i][j], cov[j][i]))
                })
                .collect();
            println!("t={} status={:?} qnorm={} negative={:?} asymmetric={:?} state_finite={} covariance_finite={} control={:x}",index,runtime.status(),qnorm,negative,asymmetric,state.iter().all(|v|v.is_finite()),cov.iter().flatten().all(|v|v.is_finite()),core.control_status());
            if runtime.status().fault.is_some() {
                println!(
                    "state={:?}\nvariances={:?}",
                    state,
                    (0..24).map(|i| cov[i][i]).collect::<Vec<_>>()
                );
                break;
            }
        }
    }
}
