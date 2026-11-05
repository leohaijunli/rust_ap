//! Software gates for the generic, split runtime. Inputs are synthetic, not HIL.

use crate::{
    estimator::{
        gnss::{GnssEkf, GnssObservation},
        imu_ekf::baro_mag::BaroAltitude,
        mahony::Mahony,
        no_gnss::NoGnssEkf,
        product::{self, Params, SharedImu},
        runtime::{Fault, FilterMode, MeasurementError, Runtime, Selection, DEADLINE_US},
        GnssSample, ImuDelta, MagSample, Measurement, NavFilter, RangeSample,
    },
    icm42688::Sample,
    parameters::{Id, Snapshot},
};

fn params() -> Params {
    // These supervisor tests start from a known, near-level synthetic attitude.
    // Default-prior alignment remains covered by the fixed PX4 replays.
    Params::from_parameters(
        Snapshot::default()
            .with(Id::EstEnable, 1.0)
            .unwrap()
            .with(Id::Ekf2AngerrInit, 0.03)
            .unwrap(),
    )
    .unwrap()
}

#[test]
fn a_numeric_failure_requires_reconfiguration_while_the_warm_fallback_recovers() {
    let mut runtime = Runtime::<GnssEkf>::new(params(), 0).unwrap();
    let mut time = 1_000_000;
    advance(&mut runtime, &mut time, 500, 100);
    time += 1_000;
    let mut bad = imu(time);
    bad.delta_angle_rad[0] = f32::MAX;
    runtime.ingest_imu(bad, time).unwrap();
    assert_eq!(runtime.status().mode, FilterMode::Fatal);
    advance(&mut runtime, &mut time, 200, 100);
    assert_eq!(runtime.status().mode, FilterMode::Fatal);
    assert_eq!(runtime.status().selection, Selection::Mahony);
    runtime.reconfigure(params(), time, true).unwrap();
    advance(&mut runtime, &mut time, 500, 100);
    assert_eq!(runtime.status().mode, FilterMode::Running);
    assert_eq!(runtime.status().fault, None);
    assert_eq!(runtime.status().selection, Selection::Filter);
}

fn imu(timestamp: u64) -> ImuDelta {
    ImuDelta {
        timestamp,
        delta_angle_rad: [0.0; 3],
        delta_velocity_m_s: [0.0, 0.0, -9.80665 * 0.001],
        delta_angle_dt_s: 0.001,
        delta_velocity_dt_s: 0.001,
        delta_velocity_clipping: [false; 3],
    }
}

#[test]
fn shared_one_khz_integrals_track_the_raw_eight_khz_reference_during_rotation() {
    use super::{imu_ekf::baro_mag::BaroMagSample, math, runtime::ImuBridge};

    let mut p = params();
    p.gnss.checks.healthy_time_us = 1_000_000;
    let mut reference = GnssEkf::new(p.imu, p.aiding, p.range, p.gnss, 0).unwrap();
    let mut runtime = Runtime::<GnssEkf>::new(p, 0).unwrap();
    let mut bridge = ImuBridge::new();
    let mut shared = SharedImu::default();
    let mut truth = math::IDENTITY;
    let mut maximum = [0.0_f32; 3];
    let mut compared = 0;
    for index in 1..=80_000_u64 {
        let time = 1_000_000 + index * 125;
        let ticks = (index * 125 / 16 - (index - 1) * 125 / 16) as u16;
        let dt = f32::from(ticks) * 16e-6;
        let t = index as f32 * 0.000125;
        let rate = [0.2 * libm::sinf(t), 0.15 * libm::cosf(0.7 * t), 0.15];
        truth = math::normalize(math::multiply(
            truth,
            math::from_axis_angle(rate.map(|v| v * dt)),
        ));
        let rotation = math::rotation(truth);
        let gravity = math::rotate_inverse(rotation, [0.0, 0.0, -9.80665]);
        let field = math::rotate_inverse(rotation, [0.22, 0.0, 0.43]);
        let sample = Sample {
            sample_timestamp_us: time,
            sequence: index as u32,
            gyro_rad_s: [rate[0], -rate[1], -rate[2]],
            accel_m_s2: [gravity[0], -gravity[1], -gravity[2]],
            temperature_c: 25.0,
            sensor_timestamp: ticks,
        };
        let aiding = index.is_multiple_of(160).then_some(BaroMagSample {
            timestamp: time,
            altitude_m: 100.0,
            field_gauss: field,
        });
        let gnss = index.is_multiple_of(1_600).then_some(GnssObservation {
            sample: GnssSample {
                timestamp: time,
                latitude_deg: 47.397742,
                longitude_deg: 8.545594,
                altitude_msl_m: 100.0,
                velocity_ned_m_s: [0.0; 3],
                horizontal_accuracy_m: 0.5,
                vertical_accuracy_m: 0.8,
                speed_accuracy_m_s: 0.1,
                fix_type: 3,
                satellites: 14,
            },
            pdop: 1.0,
            spoofed: false,
        });
        reference
            .update(&bridge.convert(sample, time).unwrap(), aiding, gnss, None)
            .unwrap();
        let integrated = shared.push(sample, time).unwrap();
        if let Some(delta) = integrated {
            runtime.ingest_imu(delta, time).unwrap();
        }
        if let Some(aid) = aiding {
            runtime
                .apply_measurement(
                    Measurement::Baro(BaroAltitude {
                        timestamp: time,
                        altitude_m: aid.altitude_m,
                    }),
                    time,
                )
                .unwrap();
            runtime
                .apply_measurement(
                    Measurement::Mag(MagSample {
                        timestamp: time,
                        field_gauss: field,
                    }),
                    time,
                )
                .unwrap();
        }
        if let Some(gnss) = gnss {
            runtime
                .apply_measurement(Measurement::Gnss(gnss), time)
                .unwrap();
        }
        if integrated.is_some() {
            runtime.maintain(time);
            if index > 40_000 {
                let a = reference.core().output();
                let b = runtime.filter().output();
                let rotation_error =
                    math::multiply(a.q_body_to_ned, math::inverse(b.q_body_to_ned));
                maximum[0] = maximum[0].max(math::norm(&math::to_axis_angle(rotation_error)));
                maximum[1] = maximum[1].max(math::norm(&core::array::from_fn::<_, 3, _>(|i| {
                    a.velocity_ned_m_s[i] - b.velocity_ned_m_s[i]
                })));
                maximum[2] = maximum[2].max(math::norm(&core::array::from_fn::<_, 3, _>(|i| {
                    a.position_ned_m[i] - b.position_ned_m[i]
                })));
                compared += 1;
            }
        }
    }
    assert!(compared > 4_000);
    std::println!(
        "shared IMU A/B: compared={compared}, max angle_rad={} velocity_m_s={} position_m={}",
        maximum[0],
        maximum[1],
        maximum[2]
    );
    // Fixed before the first run: after 5 s warm-up, max attitude 0.5 deg,
    // velocity 0.1 m/s and position 0.1 m. This is synthetic A/B, not HIL.
    assert!(
        maximum[0] <= 0.5_f32.to_radians(),
        "attitude/velocity/position max={maximum:?}"
    );
    assert!(
        maximum[1] <= 0.1 && maximum[2] <= 0.1,
        "attitude/velocity/position max={maximum:?}"
    );
    assert_eq!(runtime.status().fault, None);
}

fn advance<F: NavFilter>(runtime: &mut Runtime<F>, time: &mut u64, count: usize, duration: u32) {
    for _ in 0..count {
        *time += 1_000;
        runtime.ingest_imu(imu(*time), *time).unwrap();
        if runtime.maintain(*time).updated {
            runtime.record_duration(duration);
        }
    }
}

#[test]
fn generic_algorithms_run_and_attitude_reads_do_not_advance_or_consume_state() {
    fn run<F: NavFilter<Params = Params>>() {
        let mut runtime = Runtime::<F>::new(params(), 0).unwrap();
        let mut time = 1_000_000;
        advance(&mut runtime, &mut time, 3_000, 100);
        let before = runtime.filter().output();
        let updates = runtime.status().updates;
        let first = runtime.attitude_at(time).unwrap();
        let second = runtime.attitude_at(time + 1_000).unwrap();
        assert_eq!(first.attitude.q, second.attitude.q);
        assert_eq!(second.attitude.timestamp_sample, time);
        assert_eq!(runtime.status().updates, updates);
        assert_eq!(
            runtime.filter().output().q_body_to_ned,
            before.q_body_to_ned
        );
        assert!(runtime.attitude_at(time + 5_001).is_none());
        assert!(
            runtime.filter().health().valid,
            "{}: {:?} {:?}",
            core::any::type_name::<F>(),
            runtime.filter().health(),
            runtime.status()
        );
    }
    run::<GnssEkf>();
    run::<NoGnssEkf>();
    run::<Mahony>();
    let mut main = product::main_storage();
    main.reconfigure(params(), 0, true).unwrap();
}

#[test]
fn a_single_overrun_continues_and_sustained_overruns_recover_without_reconfiguration() {
    let mut runtime = Runtime::<GnssEkf>::new(params(), 0).unwrap();
    let mut time = 1_000_000;
    advance(&mut runtime, &mut time, 3_000, 100);
    let before = runtime.status().updates;
    runtime.record_duration(DEADLINE_US + 1);
    assert_eq!(
        runtime.status().mode,
        FilterMode::Running,
        "{:?} control={:x}",
        runtime.status(),
        runtime.filter().core().control_status()
    );
    advance(&mut runtime, &mut time, 30, 100);
    assert!(runtime.status().updates > before);
    assert_eq!(runtime.status().fault, None);
    advance(&mut runtime, &mut time, 50, DEADLINE_US + 1);
    assert!(matches!(
        runtime.status().mode,
        FilterMode::Degraded {
            reason: Fault::Deadline,
            ..
        }
    ));
    assert_eq!(runtime.status().selection, Selection::Mahony);
    assert!(runtime.attitude_at(time).is_some());
    let degraded = runtime.status().mode;
    advance(&mut runtime, &mut time, 20, DEADLINE_US + 1);
    assert_eq!(runtime.status().mode, degraded);
    let misses = runtime.status().deadline_misses;
    advance(&mut runtime, &mut time, 150, 100);
    assert_eq!(
        runtime.status().mode,
        FilterMode::Running,
        "{:?} control={:x}",
        runtime.status(),
        runtime.filter().core().control_status()
    );
    assert_eq!(runtime.status().fault, None);
    assert_eq!(runtime.status().deadline_misses, misses);
    assert_eq!(runtime.status().last_fault, Some(Fault::Deadline));
    assert_eq!(runtime.status().recoveries, 0);
}

#[test]
fn product_publication_keeps_source_reset_metadata_and_revokes_stale_attitudes() {
    let mut main = product::main_storage();
    let mut independent = product::independent_storage();
    main.reconfigure(params(), 0, true).unwrap();
    independent.reconfigure(params(), 0, true).unwrap();
    let mut time = 1_000_000;
    for _ in 0..500 {
        time += 1_000;
        let mut delta = imu(time);
        delta.delta_angle_rad[2] = 0.0002;
        main.ingest_imu(delta, time).unwrap();
        independent.ingest_imu(delta, time).unwrap();
        main.maintain(time);
        independent.maintain(time);
    }
    let mut resets =
        core::array::from_fn(|_| crate::estimator::publication_reset::PublicationReset::default());
    let before = product::snapshots(&mut main, &mut independent, time, &mut resets);
    for _ in 0..3 {
        main.record_duration(DEADLINE_US + 1);
    }
    let fast = main.attitude_at(time).unwrap().attitude;
    let after = product::snapshots(&mut main, &mut independent, time, &mut resets);
    let attitude = after[0].attitude.unwrap();
    assert_eq!(
        attitude.quat_reset_counter,
        before[0]
            .attitude
            .unwrap()
            .quat_reset_counter
            .wrapping_add(1)
    );
    assert_eq!(attitude.quat_reset_counter, fast.quat_reset_counter);
    assert_eq!(attitude.delta_q_reset, fast.delta_q_reset);
    assert_eq!(after[0].status.reset_count_quat, fast.quat_reset_counter);
    assert!(after[0].odometry.is_none());
    for snapshot in product::snapshots(&mut main, &mut independent, time + 5_001, &mut resets) {
        assert!(snapshot.attitude.is_none());
        assert!(snapshot.odometry.is_none());
        assert!(snapshot.global_position.is_none());
        assert_eq!(snapshot.status.solution_status_flags & 1, 0);
    }
}

#[test]
fn sequence_gap_resynchronizes_shared_input_and_recovers_the_filter() {
    let mut shared = SharedImu::default();
    let mut runtime = Runtime::<GnssEkf>::new(params(), 0).unwrap();
    let mut sequence = 0_u32;
    for index in 0..64_000_u64 {
        if index == 32_000 {
            sequence += 1;
        }
        let time = 1_000_000 + (index + 1) * 125;
        let sample = Sample {
            sample_timestamp_us: time,
            sequence,
            gyro_rad_s: [0.0; 3],
            accel_m_s2: [0.0, 0.0, 9.80665],
            temperature_c: 25.0,
            sensor_timestamp: 8,
        };
        sequence += 1;
        match shared.push(sample, time) {
            Ok(Some(delta)) => {
                runtime.ingest_imu(delta, time).unwrap();
                runtime.maintain(time);
            }
            Ok(None) => {}
            Err(fault) => {
                assert_eq!(fault, Fault::Sequence);
                runtime.invalidate(fault);
                runtime.maintain(time);
            }
        }
    }
    assert_eq!(runtime.status().recoveries, 1);
    assert_eq!(
        runtime.status().mode,
        FilterMode::Running,
        "{:?} control={:x}",
        runtime.status(),
        runtime.filter().core().control_status()
    );
    assert_eq!(runtime.status().fault, None);
    assert_eq!(runtime.status().last_fault, Some(Fault::Sequence));
    assert!(shared.clock.windows > 0);
    // Synthetic 128-us sensor intervals at 125-us TIM2 spacing must expose,
    // rather than silently conceal, a ~2.4% clock disagreement.
    assert!((shared.clock.error_ppm - 24_000).abs() < 10);
}

#[test]
fn separate_aid_events_fuse_without_gnss_and_bad_gnss_cannot_change_independent_state() {
    let mut values = params();
    values.range.control = 2;
    let mut main = Runtime::<GnssEkf>::new(values, 0).unwrap();
    let mut independent = Runtime::<NoGnssEkf>::new(values, 0).unwrap();
    for index in 1..=6_000_u64 {
        let time = 1_000_000 + index * 1_000;
        main.ingest_imu(imu(time), time).unwrap();
        independent.ingest_imu(imu(time), time).unwrap();
        if index.is_multiple_of(20) {
            for event in [
                Measurement::Baro(BaroAltitude {
                    timestamp: time,
                    altitude_m: 25.0,
                }),
                Measurement::Mag(MagSample {
                    timestamp: time,
                    field_gauss: [0.22, 0.0, 0.43],
                }),
                Measurement::Range(RangeSample {
                    timestamp: time,
                    distance_m: 1.0,
                    min_distance_m: 0.1,
                    max_distance_m: 4.0,
                    quality: 100,
                }),
            ] {
                main.apply_measurement(event, time).unwrap();
                independent.apply_measurement(event, time).unwrap();
            }
        }
        main.maintain(time);
        independent.maintain(time);
    }
    for core in [main.filter().core(), independent.filter().core()] {
        let (baro, mag) = core.aid_status().unwrap();
        assert!(
            baro.fused && mag.fused,
            "baro={baro:?} mag={mag:?} health={:?} control={:x}",
            core.health(),
            core.control_status()
        );
        assert!(core.range_status().unwrap().timestamp > 0);
        assert_ne!(core.control_status() & (1 << 1), 0);
    }
    let state = *independent.filter().core().state();
    let covariance = *independent.filter().core().covariance();
    let bad = Measurement::Gnss(GnssObservation {
        sample: GnssSample {
            timestamp: 7_000_000,
            latitude_deg: f64::NAN,
            ..GnssSample::default()
        },
        pdop: 1.0,
        spoofed: false,
    });
    assert_eq!(
        main.apply_measurement(bad, 7_000_000),
        Err(MeasurementError::Invalid)
    );
    assert_eq!(independent.filter().core().state(), &state);
    assert_eq!(independent.filter().core().covariance(), &covariance);
    assert_eq!(main.status().fault, None);

    // Even the shared trait's GNSS variant is ignored by the no-GNSS algorithm.
    let mut no_gnss = <NoGnssEkf as NavFilter>::new(&values, 0);
    let before = *no_gnss.core().state();
    no_gnss.push_measurement(bad);
    assert_eq!(no_gnss.core().state(), &before);
    assert_eq!(no_gnss.health().fault, None);
}

#[test]
fn events_wait_for_imu_and_duplicate_or_invalid_measurements_do_not_latch_the_filter() {
    let mut runtime = Runtime::<GnssEkf>::new(params(), 0).unwrap();
    let event = Measurement::Baro(BaroAltitude {
        timestamp: 1_005_000,
        altitude_m: 25.0,
    });
    runtime.apply_measurement(event, 1_005_000).unwrap();
    runtime.maintain(1_005_000);
    assert_eq!(runtime.status().fault, None);
    for time in (1_001_000..=1_010_000).step_by(1_000) {
        runtime.ingest_imu(imu(time), time + 10_000).unwrap();
        runtime.maintain(time + 10_000);
    }
    assert_eq!(
        runtime.apply_measurement(event, 1_010_000),
        Err(MeasurementError::Duplicate)
    );
    assert_eq!(runtime.status().fault, None);
    assert_eq!(runtime.status().measurements, 1);
}

#[test]
fn snapshot_parameters_reconfigure_baro_and_reject_insufficient_delay_capacity() {
    let values = Snapshot::default()
        .with(Id::EstEnable, 1.0)
        .unwrap()
        .with(Id::Ekf2NoaidTout, 2_000_000.0)
        .unwrap()
        .with(Id::Ekf2GndEffDz, 3.0)
        .unwrap()
        .with(Id::Ekf2MagChkStr, 0.3)
        .unwrap();
    let p = Params::from_parameters(values).unwrap();
    for id in [Id::Ekf2NoaidTout, Id::Ekf2GndEffDz, Id::Ekf2MagChkStr] {
        assert!(p.maps(id));
    }
    assert_eq!(p.imu.no_aid_timeout_us, 2_000_000);
    assert_eq!(p.aiding.ground_effect_deadzone_m, 3.0);
    assert_eq!(p.aiding.mag_check_strength_gauss, 0.3);
    assert!(Params::from_parameters(values.with(Id::Ekf2DelayMax, 50.0).unwrap()).is_err());
    assert!(Params::from_parameters(values.with(Id::Ekf2PredictUs, 1_000.0).unwrap()).is_err());

    let mut runtime = Runtime::<GnssEkf>::new(p, 0).unwrap();
    let mut time = 1_000_000;
    for enabled in [true, false] {
        let p = Params::from_parameters(
            values
                .with(Id::Ekf2BaroCtrl, f32::from(u8::from(enabled)))
                .unwrap(),
        )
        .unwrap();
        runtime.reconfigure(p, time, true).unwrap();
        for index in 1..=3_000 {
            time += 1_000;
            runtime.ingest_imu(imu(time), time).unwrap();
            if index % 20 == 0 {
                runtime
                    .apply_measurement(
                        Measurement::Baro(BaroAltitude {
                            timestamp: time,
                            altitude_m: 25.0,
                        }),
                        time,
                    )
                    .unwrap();
            }
            runtime.maintain(time);
        }
        assert_eq!(
            runtime.filter().core().control_status() & (1 << 9) != 0,
            enabled
        );
        assert_eq!(runtime.filter().core().control_status() & (1 << 35), 0);
    }
}

#[test]
fn restart_from_board_defaults_reports_the_failing_configuration_stage() {
    use crate::estimator::config_stage;

    // Exactly the board defaults: `!ekf2 start` only enables EST_EN.
    let values = Snapshot::default().with(Id::EstEnable, 1.0).unwrap();
    let p = Params::from_parameters(values).unwrap();
    let mut main = product::main_storage();
    let mut independent = product::independent_storage();
    main.reconfigure(p, 0, true).unwrap();
    independent.reconfigure(p, 0, true).unwrap();
    assert_eq!(config_stage::current(), config_stage::IDLE);

    let mut time = 1_000_000;
    advance(&mut main, &mut time, 3_000, 100);
    advance(&mut independent, &mut time, 3_000, 100);

    main.reconfigure(p, 0, true)
        .unwrap_or_else(|fault| panic!("main {fault:?} at stage {}", config_stage::current()));
    independent.reconfigure(p, 0, true).unwrap_or_else(|fault| {
        panic!("independent {fault:?} at stage {}", config_stage::current())
    });
    assert_eq!(config_stage::current(), config_stage::IDLE);
}
