//! Single-owner estimator worker, shared IMU front end and 1-kHz attitude output.

use core::{
    fmt,
    sync::atomic::{AtomicU32, Ordering},
};

use cortex_m::peripheral::DWT;
use embassy_executor::{InterruptExecutor, SendSpawner};
use embassy_futures::select::{select3, Either3};
use embassy_stm32::{interrupt, peripherals, Peri};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, watch::Watch};
use embassy_time::{with_timeout, Duration, Instant, Timer};
use static_cell::ConstStaticCell;
use v6x_rust_led::{
    config::Algorithm,
    estimator::{
        config_stage,
        product::{self, ClockReport, Params, ProductRuntime, SharedImu},
        publication_reset::PublicationReset,
        runtime::{self, AttitudeEstimate, Fault, Runtime, Selection, Status},
        standard_output::{altitude, EulerAttitude, Snapshot},
        standard_ulog::DataEncoder,
        timing::{Timing, TimingReport},
        FilterFault, ImuDelta, Measurement, NavFilter,
    },
    log_config::{self, Group},
    log_queue::{Class, PushError},
    parameters, ulog,
};

#[derive(Clone, Copy)]
struct Control {
    epoch: u32,
    params: Params,
}

#[derive(Clone, Copy)]
pub struct Output {
    pub epoch: u32,
    pub status: Status,
    pub snapshot: Snapshot,
    pub euler: Option<EulerAttitude>,
}

#[derive(Clone, Copy)]
struct AttitudeFrame {
    epoch: u32,
    estimate: Option<AttitudeEstimate>,
}

#[derive(Clone, Copy, Debug)]
enum Input {
    Imu(ImuDelta),
    Fault(Fault),
}

#[derive(Clone, Copy)]
struct AidEvent {
    measurement: Measurement,
    received: u64,
    device_id: u32,
    mag_generation: [u32; 2],
}

#[derive(Clone, Copy)]
struct Performance {
    timings: [TimingReport; 5],
    clock: ClockReport,
}

struct WorkerTimings {
    main_ingest: Timing,
    main: Timing,
    attitude: Timing,
    snapshot: Timing,
}

#[derive(Clone, Copy)]
struct PublishStats {
    timing: TimingReport,
    releases: u32,
    missed: u32,
    repeated: u32,
    max_late_us: u32,
    max_age_us: u32,
}

static CONTROL: Watch<CriticalSectionRawMutex, Control, 0> = Watch::new();
static OUTPUT: Watch<CriticalSectionRawMutex, Output, 0> = Watch::new();
static ATTITUDE_INPUT: Watch<CriticalSectionRawMutex, AttitudeFrame, 0> = Watch::new();
static ATTITUDE_OUTPUT: Watch<CriticalSectionRawMutex, AttitudeFrame, 0> = Watch::new();
static INPUT: Channel<CriticalSectionRawMutex, Input, 16> = Channel::new();
static AIDS: Channel<CriticalSectionRawMutex, AidEvent, 16> = Channel::new();
static PERF: Watch<CriticalSectionRawMutex, Performance, 0> = Watch::new();
static FRONTEND_PERF: Watch<CriticalSectionRawMutex, (TimingReport, ClockReport), 0> = Watch::new();
static PUBLISH_PERF: Watch<CriticalSectionRawMutex, PublishStats, 0> = Watch::new();
static EPOCH: AtomicU32 = AtomicU32::new(0);
static ACTIVE_EPOCH: AtomicU32 = AtomicU32::new(0);
static APPLIED_ALGORITHM: AtomicU32 = AtomicU32::new(0);
static CONFIG_ERRORS: AtomicU32 = AtomicU32::new(0);
static CONFIGURE_MAX_US: AtomicU32 = AtomicU32::new(0);
static LAST_CONFIG_FAULT: AtomicU32 = AtomicU32::new(0);
static MAIN_CONFIG: AtomicU32 = AtomicU32::new(0);
static LOG_ERRORS: AtomicU32 = AtomicU32::new(0);
static INPUT_DROPS: AtomicU32 = AtomicU32::new(0);
static AID_DROPS: AtomicU32 = AtomicU32::new(0);
static MAIN: ConstStaticCell<ProductRuntime> = ConstStaticCell::new(product::main_storage());
static EXECUTOR: InterruptExecutor = InterruptExecutor::new();

pub fn start(timer: Peri<'static, peripherals::TIM5>, fast: SendSpawner) {
    EXECUTOR
        .start(interrupt::TIM5)
        .spawn(task(MAIN.take(), timer))
        .unwrap();
    fast.spawn(input_task()).unwrap();
    fast.spawn(attitude_task()).unwrap();
}

#[interrupt]
unsafe fn TIM5() {
    // The worker owns the runtime. Other tasks exchange DTOs.
    unsafe { EXECUTOR.on_interrupt() };
}

pub fn configure(values: parameters::Snapshot) -> Result<(), parameters::Error> {
    let params = Params::from_parameters(values).map_err(|_| parameters::Error::Configuration)?;
    if params.enabled && (crate::OUTPUT_STATE.armed || crate::OUTPUT_STATE.outputs_enabled) {
        return Err(parameters::Error::Configuration);
    }
    let epoch = EPOCH.fetch_add(1, Ordering::AcqRel).wrapping_add(1);
    CONTROL.sender().send(Control { epoch, params });
    Ok(())
}

/// Sensor publishers call this once per accepted sample with its real device ID.
pub fn submit_measurement(measurement: Measurement, device_id: u32) -> bool {
    if ACTIVE_EPOCH.load(Ordering::Acquire) == 0 {
        return false;
    }
    let event = AidEvent {
        measurement,
        received: Instant::now().as_micros(),
        device_id,
        mag_generation: [0; 2],
    };
    if AIDS.try_send(event).is_err() {
        AID_DROPS.fetch_add(1, Ordering::Relaxed);
        return false;
    }
    true
}

pub fn active_epoch() -> u32 {
    ACTIVE_EPOCH.load(Ordering::Acquire)
}

pub fn submit_magnetometer(selected: v6x_rust_led::magnetometer::Selected) -> bool {
    if active_epoch() == 0 {
        return false;
    }
    let event = AidEvent {
        measurement: Measurement::Mag(v6x_rust_led::estimator::MagSample {
            timestamp: selected.data.timestamp_sample,
            field_gauss: selected.data.magnetometer_ga,
            reset: selected.reset,
        }),
        received: Instant::now().as_micros(),
        device_id: selected.data.device_id,
        mag_generation: [selected.generation, selected.correction_generation],
    };
    if AIDS.try_send(event).is_err() {
        AID_DROPS.fetch_add(1, Ordering::Relaxed);
        return false;
    }
    true
}

pub use v6x_rust_led::estimator::report::Report;

pub fn report() -> Report {
    let epoch = ACTIVE_EPOCH.load(Ordering::Acquire);
    let now = Instant::now().as_micros();
    Report {
        requested: CONTROL.try_get().is_some_and(|value| value.params.enabled),
        active: epoch != 0,
        configured: if APPLIED_ALGORITHM.load(Ordering::Acquire) == 1 {
            Algorithm::Ekf2
        } else {
            Algorithm::Mahony
        },
        selection: OUTPUT
            .try_get()
            .filter(|output| {
                epoch != 0
                    && output.epoch == epoch
                    && now >= output.status.timestamp
                    && now - output.status.timestamp <= runtime::FRESH_US
            })
            .map_or(Selection::Invalid, |output| output.status.selection),
    }
}

pub fn latest() -> Option<Output> {
    let control = CONTROL.try_get()?;
    if !control.params.enabled {
        return None;
    }
    let mut output = OUTPUT.try_get()?;
    let now = Instant::now().as_micros();
    if output.epoch != control.epoch
        || now < output.status.timestamp
        || now - output.status.timestamp > runtime::FRESH_US
    {
        return None;
    }
    let estimate = ATTITUDE_OUTPUT
        .try_get()
        .filter(|v| v.epoch == control.epoch)
        .and_then(|v| v.estimate)
        .and_then(|v| v.at(now));
    output.snapshot.attitude = estimate.map(|v| v.attitude);
    output.snapshot.angular_velocity = estimate.map(|v| v.angular_velocity);
    output.snapshot.status.solution_status_flags &= !1;
    if let Some(v) = estimate {
        output.snapshot.status.solution_status_flags |= 1;
        output.snapshot.status.reset_count_quat = v.attitude.quat_reset_counter;
        output.snapshot.local_position.heading_reset_counter = v.attitude.quat_reset_counter;
    }
    output.euler = estimate.and_then(|v| EulerAttitude::from_attitude(v.attitude, v.relative_yaw));
    Some(output)
}

fn cycles_us(cycles: u32) -> u32 {
    cycles.div_ceil(crate::platform::CPU_HZ / 1_000_000)
}

fn send_input(input: Input) {
    if INPUT.try_send(input).is_ok() {
        return;
    }
    critical_section::with(|_| {
        INPUT_DROPS.fetch_add(INPUT.len() as u32, Ordering::Relaxed);
        INPUT.clear();
        INPUT.try_send(Input::Fault(Fault::Sequence)).unwrap();
        INPUT.try_send(input).unwrap();
    });
}

#[embassy_executor::task]
async fn input_task() {
    let mut epoch = 0;
    let mut shared = SharedImu::default();
    let mut timing = Timing::EMPTY;
    let mut next_report = Instant::now();
    let mut last_sample = 0;
    loop {
        let active = ACTIVE_EPOCH.load(Ordering::Acquire);
        if active == 0 {
            Timer::after_millis(1).await;
            continue;
        }
        if active != epoch {
            epoch = active;
            shared = SharedImu::default();
            timing = Timing::EMPTY;
            last_sample = 0;
        }
        match with_timeout(Duration::from_millis(20), crate::imu::next_for_ahrs()).await {
            Ok(batch) => {
                if ACTIVE_EPOCH.load(Ordering::Acquire) != epoch {
                    continue;
                }
                let started = DWT::cycle_count();
                let now = Instant::now().as_micros();
                match shared.push(&batch, now) {
                    Ok(Some(delta)) => {
                        last_sample = delta.timestamp;
                        send_input(Input::Imu(delta));
                    }
                    Ok(None) => {}
                    Err(fault) => send_input(Input::Fault(fault)),
                }
                timing.record(cycles_us(DWT::cycle_count().wrapping_sub(started)));
            }
            Err(_) => {
                if last_sample != 0
                    && Instant::now().as_micros().saturating_sub(last_sample) > runtime::FRESH_US
                {
                    send_input(Input::Fault(Fault::Stale));
                    shared = SharedImu::default();
                    last_sample = 0;
                }
            }
        }
        if Instant::now() >= next_report {
            FRONTEND_PERF.sender().send((timing.report(), shared.clock));
            next_report = Instant::now() + Duration::from_secs(1);
        }
    }
}

#[embassy_executor::task]
async fn attitude_task() {
    let mut release = Instant::now().as_micros() + 1_000;
    let mut last_sample = 0;
    let mut timing = Timing::EMPTY;
    let mut stats = PublishStats {
        timing: TimingReport::default(),
        releases: 0,
        missed: 0,
        repeated: 0,
        max_late_us: 0,
        max_age_us: 0,
    };
    loop {
        Timer::at(Instant::from_micros(release)).await;
        let now = Instant::now().as_micros();
        let late = now.saturating_sub(release);
        release += (late / 1_000 + 1) * 1_000;
        let epoch = ACTIVE_EPOCH.load(Ordering::Acquire);
        if epoch == 0 {
            continue;
        }
        let started = DWT::cycle_count();
        stats.releases = stats.releases.wrapping_add(1);
        stats.missed = stats.missed.saturating_add((late / 1_000) as u32);
        stats.max_late_us = stats.max_late_us.max(late.min(u64::from(u32::MAX)) as u32);
        let estimate = ATTITUDE_INPUT
            .try_get()
            .filter(|v| v.epoch == epoch)
            .and_then(|v| v.estimate)
            .and_then(|v| v.at(now));
        if let Some(main) = estimate {
            let sample = main.attitude.timestamp_sample;
            stats.repeated = stats
                .repeated
                .wrapping_add(u32::from(sample == last_sample));
            stats.max_age_us = stats.max_age_us.max((now - sample) as u32);
            last_sample = sample;
        }
        ATTITUDE_OUTPUT
            .sender()
            .send(AttitudeFrame { epoch, estimate });
        timing.record(cycles_us(DWT::cycle_count().wrapping_sub(started)));
        if stats.releases.is_multiple_of(1_000) {
            stats.timing = timing.report();
            PUBLISH_PERF.sender().send(stats);
        }
    }
}

#[inline(never)]
fn service<F: NavFilter>(runtime: &mut Runtime<F>, now: u64, timing: &mut Timing) {
    let started = DWT::cycle_count();
    let report = runtime.maintain(now);
    if report.updated {
        let us = cycles_us(DWT::cycle_count().wrapping_sub(started));
        timing.record(us);
        runtime.record_duration(us);
    }
}

#[embassy_executor::task]
async fn task(main: &'static mut ProductRuntime, _timer: Peri<'static, peripherals::TIM5>) {
    let mut epoch = 0;
    let mut enabled = false;
    let mut resets = PublicationReset::default();
    let mut devices = [0_u32; 4];
    let mut mag_identity = None;
    let mut next_publish = Instant::now();
    let mut timing = WorkerTimings {
        main_ingest: Timing::EMPTY,
        main: Timing::EMPTY,
        attitude: Timing::EMPTY,
        snapshot: Timing::EMPTY,
    };
    loop {
        let Some(control) = CONTROL.try_get() else {
            Timer::after_millis(20).await;
            continue;
        };
        if control.epoch != epoch {
            epoch = control.epoch;
            ACTIVE_EPOCH.store(0, Ordering::Release);
            crate::imu::set_ahrs_active(false);
            INPUT.clear();
            AIDS.clear();
            main.disable();
            enabled = false;
            devices = [0; 4];
            mag_identity = None;
            if control.params.enabled {
                let started = Instant::now();
                let prefer_filter = control.params.algorithm == Algorithm::Ekf2;
                // Configuration is one-shot and not on a hot loop: after a
                // health fault the whole configure is re-read once, a delay
                // apart, so a transient memory value is not blamed twice.
                let mut attempt = 0;
                let configured = loop {
                    let result = main.reconfigure(control.params, 0, prefer_filter);
                    let stage = config_stage::current();
                    MAIN_CONFIG.store(pack_config(stage, result.err()), Ordering::Relaxed);
                    if result.is_ok() || attempt >= 2 {
                        break result;
                    }
                    attempt += 1;
                    config_stage::note_retry(stage);
                    Timer::after_millis(1).await;
                };
                CONFIGURE_MAX_US.fetch_max(
                    started.elapsed().as_micros().min(u64::from(u32::MAX)) as u32,
                    Ordering::Relaxed,
                );
                match configured {
                    Ok(()) => {
                        LAST_CONFIG_FAULT.store(0, Ordering::Relaxed);
                        enabled = true;
                        APPLIED_ALGORITHM.store(
                            u32::from(prefer_filter && !cfg!(feature = "estimator-mahony")),
                            Ordering::Release,
                        );
                        ACTIVE_EPOCH.store(epoch, Ordering::Release);
                        crate::imu::set_ahrs_active(true);
                    }
                    Err(fault) => {
                        LAST_CONFIG_FAULT.store(filter_fault_code(fault), Ordering::Relaxed);
                        CONFIG_ERRORS.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        }
        if !enabled {
            Timer::after_millis(20).await;
            continue;
        }
        let event = select3(INPUT.receive(), AIDS.receive(), Timer::after_millis(20)).await;
        if EPOCH.load(Ordering::Acquire) != epoch {
            continue;
        }
        let now = Instant::now().as_micros();
        match event {
            Either3::First(Input::Imu(delta)) => {
                let started = DWT::cycle_count();
                let _ = main.ingest_imu(delta, now); // Runtime retains input failures.
                timing
                    .main_ingest
                    .record(cycles_us(DWT::cycle_count().wrapping_sub(started)));
            }
            Either3::First(Input::Fault(fault)) => {
                main.invalidate(fault);
            }
            Either3::Second(mut event) => {
                let index = match event.measurement {
                    Measurement::Baro(_) => 0,
                    Measurement::Mag(_) => 1,
                    Measurement::Range(_) => 2,
                    Measurement::Gnss(_) => 3,
                };
                let identity = (event.device_id, event.mag_generation);
                if let Measurement::Mag(ref mut sample) = event.measurement {
                    // A dropped/rejected reset must not let the next sample from
                    // a different device/calibration blend into old mag history.
                    sample.reset |= mag_identity != Some(identity);
                }
                let accepted =
                    event.received <= now && main.apply_measurement(event.measurement, now).is_ok();
                if accepted {
                    devices[index] = event.device_id;
                    if index == 1 {
                        mag_identity = Some(identity);
                    }
                }
            }
            Either3::Third(()) => {}
        }
        // Publish the newest full-rate output before entering heavy work.
        let started = DWT::cycle_count();
        ATTITUDE_INPUT.sender().send(AttitudeFrame {
            epoch,
            estimate: main.attitude_at(now),
        });
        timing
            .attitude
            .record(cycles_us(DWT::cycle_count().wrapping_sub(started)));
        main.set_stationary_permitted(
            !crate::OUTPUT_STATE.armed && !crate::OUTPUT_STATE.outputs_enabled,
        );
        service(main, now, &mut timing.main);
        let now = Instant::now();
        let started = DWT::cycle_count();
        ATTITUDE_INPUT.sender().send(AttitudeFrame {
            epoch,
            estimate: main.attitude_at(now.as_micros()),
        });
        timing
            .attitude
            .record(cycles_us(DWT::cycle_count().wrapping_sub(started)));
        if now >= next_publish {
            let started = DWT::cycle_count();
            publish(main, epoch, now.as_micros(), devices, &mut resets);
            timing
                .snapshot
                .record(cycles_us(DWT::cycle_count().wrapping_sub(started)));
            let (shared, clock) = FRONTEND_PERF.try_get().unwrap_or_default();
            PERF.sender().send(Performance {
                timings: [
                    shared,
                    timing.main_ingest.report(),
                    timing.main.report(),
                    timing.attitude.report(),
                    timing.snapshot.report(),
                ],
                clock,
            });
            next_publish = now + Duration::from_millis(50);
        }
    }
}

#[inline(never)]
fn publish(
    main: &mut ProductRuntime,
    epoch: u32,
    now: u64,
    devices: [u32; 4],
    resets: &mut PublicationReset,
) {
    let mut snapshot = product::snapshot(main, now, resets);
    snapshot.status.accel_device_id = v6x_rust_led::px4_messages::ICM42688_DEVICE_ID;
    snapshot.status.gyro_device_id = v6x_rust_led::px4_messages::ICM42688_DEVICE_ID;
    snapshot.status.baro_device_id = devices[0];
    snapshot.status.mag_device_id = devices[1];
    for (aid_index, aid) in snapshot.aids.iter_mut().enumerate() {
        if let Some(aid) = aid {
            aid.device_id = devices[aid_index.min(3)];
        }
    }
    OUTPUT.sender().send(Output {
        epoch,
        status: main.status(),
        snapshot,
        // latest() derives Euler angles from the matching fast attitude frame.
        euler: None,
    });
}

#[derive(Debug)]
enum LogError {
    Codec(ulog::Error),
    Queue(PushError),
}
impl From<ulog::Error> for LogError {
    fn from(value: ulog::Error) -> Self {
        Self::Codec(value)
    }
}

#[embassy_executor::task]
pub async fn log_task() {
    let mut encoder = DataEncoder::new();
    let mut logger_was_ready = false;
    let mut file_index = 0;
    let mut last_output = 0;
    loop {
        Timer::after_millis(20).await;
        let ready = crate::sd_logger::ready();
        let index = crate::sd_logger::snapshot().file_index;
        if ready && (!logger_was_ready || index != file_index) {
            encoder = DataEncoder::new();
            last_output = 0;
        }
        logger_was_ready = ready;
        file_index = index;
        if !ready {
            continue;
        }
        let Some(output) = latest() else {
            continue;
        };
        let timestamp = output.snapshot.status.timestamp;
        if timestamp <= last_output {
            continue;
        }
        last_output = timestamp;
        if !log_config::rate(Group::Estimator, timestamp) {
            continue;
        }
        let record = runtime_record(output.epoch, output.status, timestamp);
        if crate::sd_logger::LOGGER
            .try_record(Class::Normal, timestamp, record)
            .is_err()
        {
            LOG_ERRORS.fetch_add(1, Ordering::Relaxed);
        }
        let result = encoder.write_snapshot::<LogError>(&output.snapshot, |record| {
            crate::sd_logger::LOGGER
                .try_record(Class::Normal, timestamp, *record)
                .map_err(LogError::Queue)
        });
        if let Err(error) = result {
            match error {
                LogError::Codec(error) => {
                    let _ = error;
                }
                LogError::Queue(error) => {
                    let _ = error;
                }
            }
            LOG_ERRORS.fetch_add(1, Ordering::Relaxed);
        }
        if let Some(euler) = output.euler {
            let record = ulog::attitude_euler_record(euler).expect("fixed Euler schema");
            if crate::sd_logger::LOGGER
                .try_record(Class::Normal, timestamp, record)
                .is_err()
            {
                LOG_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
        }
        let baro = crate::barometer::latest();
        if let Some(value) = altitude(
            output.snapshot.local_position,
            baro.map(|sample| sample.pressure_altitude_m()),
            timestamp,
        ) {
            let record = ulog::altitude_record(value).expect("fixed altitude schema");
            if crate::sd_logger::LOGGER
                .try_record(Class::Normal, timestamp, record)
                .is_err()
            {
                LOG_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

fn runtime_record(epoch: u32, status: Status, now: u64) -> ulog::Record {
    let mut payload = ulog::Payload::new();
    payload.push_u64(status.timestamp).unwrap();
    for value in [
        epoch,
        status.samples,
        status.updates,
        status.measurements,
        status.deadline_misses,
        status.max_update_us,
        status.recoveries,
        status.rejected_measurements,
    ] {
        payload.push_u32(value).unwrap();
    }
    payload
        .push_u8(match status.selection {
            Selection::Invalid => 0,
            Selection::Filter => 1,
            Selection::Mahony => 2,
        })
        .unwrap();
    payload.push_u8(fault_code(status.fault)).unwrap();
    payload.push_u8(0).unwrap();
    ulog::data_record(
        ulog::TopicId::EstimatorRuntime as u16,
        now,
        payload.as_bytes(),
    )
    .expect("fixed runtime schema")
}

fn filter_fault_code(fault: FilterFault) -> u32 {
    match fault {
        FilterFault::Input => 1,
        FilterFault::Timeout => 2,
        FilterFault::Numerics => 3,
    }
}

/// Configuration stage in the low byte, filter fault in the next byte.
fn pack_config(stage: u8, fault: Option<FilterFault>) -> u32 {
    u32::from(stage) | (fault.map_or(0, filter_fault_code) << 8)
}

fn fault_code(fault: Option<Fault>) -> u8 {
    match fault {
        None => 0,
        Some(Fault::Input) => 1,
        Some(Fault::Sequence) => 2,
        Some(Fault::Timestamp) => 3,
        Some(Fault::Stale) => 4,
        Some(Fault::Filter(FilterFault::Input)) => 5,
        Some(Fault::Filter(FilterFault::Timeout)) => 6,
        Some(Fault::Filter(FilterFault::Numerics)) => 7,
        Some(Fault::Deadline) => 8,
    }
}

pub fn write_diagnostics(output: &mut impl fmt::Write) -> fmt::Result {
    let control = CONTROL.try_get();
    writeln!(output, "EKF2_RUNTIME schema=3 requested={} epoch={} active={} data_only=true nav_qualified=false config_errors={} last_config_fault={} config_stage={} configure_max_us={} input_drops={} aid_drops={} log_errors={} worker=TIM5/P4 frontend_publish=TIM7/P3 update_budget_us={}",
        control.is_some_and(|v| v.params.enabled), control.map_or(0, |v| v.epoch), ACTIVE_EPOCH.load(Ordering::Acquire) != 0,
        CONFIG_ERRORS.load(Ordering::Relaxed), LAST_CONFIG_FAULT.load(Ordering::Relaxed), config_stage::current(), CONFIGURE_MAX_US.load(Ordering::Relaxed),
        INPUT_DROPS.load(Ordering::Relaxed), AID_DROPS.load(Ordering::Relaxed), LOG_ERRORS.load(Ordering::Relaxed), runtime::DEADLINE_US)?;
    let main_config = MAIN_CONFIG.load(Ordering::Relaxed);
    writeln!(
        output,
        "EKF2_CONFIG main_stage={} main_fault={} config_retries={}",
        main_config & 0xff,
        (main_config >> 8) & 0xff,
        config_stage::retry_count()
    )?;
    if let Some(control) = control {
        let params = control.params;
        writeln!(output, "EKF2_CONFIG_PARAMS buffer_length={:?} predict_us={} delay_max_ms={} imu_ctrl={} baro_ctrl={} baro_delay_ms={} mag_type={} mag_delay_ms={} range_ctrl={} range_delay_ms={} gps_ctrl={} gps_delay_ms={} gps_mode={}",
            params.buffer_length(), params.imu.predict_us, params.imu.delay_max_ms, params.imu.imu_ctrl,
            params.aiding.baro_control, params.aiding.baro_delay_ms, params.aiding.mag_type, params.aiding.mag_delay_ms,
            params.range.control, params.range.delay_ms, params.gnss.control, params.gnss.delay_ms, params.gnss.mode)?;
    }
    if let Some(state) = OUTPUT.try_get() {
        let status = state.status;
        writeln!(output, "EKF2_STATE instance=main samples={} updates={} measurements={} selected={:?} mode={:?} fault={:?} last_fault={:?} numerics={:?} numerics_stage={:?} numerics_pair=[{:e},{:e}] recoveries={} deadline_misses={} max_update_us={}",
            status.samples, status.updates, status.measurements, status.selection, status.mode, status.fault,
            status.last_fault, status.numerics, status.numerics_stage, status.numerics_pair[0], status.numerics_pair[1],
            status.recoveries, status.deadline_misses, status.max_update_us)?;
        writeln!(
            output,
            "EKF2_AIDS baro_available={} mag_available={} range_available={}",
            state.snapshot.aids[0].is_some(),
            state.snapshot.aids[1].is_some(),
            state.snapshot.aids[2].is_some()
        )?;
        if let Some(mag) = state.snapshot.aids[1] {
            writeln!(
                output,
                "EKF2_MAG device_id={} timestamp_sample={} fused={} rejected={}",
                mag.device_id, mag.status.timestamp, mag.status.fused, mag.status.rejected
            )?;
        }
    }
    if let Some(perf) = PERF.try_get() {
        for (name, timing) in [
            "shared_input",
            "main_ingest",
            "main_step",
            "prepare_attitude",
            "snapshot",
        ]
        .into_iter()
        .zip(perf.timings)
        {
            writeln!(output, "EKF2_TIMING path={name} n={} max_us={} p99_upper_us={} dwt_includes_preemption=true",
                timing.samples, timing.max_us, timing.p99_upper_us)?;
        }
        writeln!(
            output,
            "EKF2_CLOCK windows={} sensor_dt_us={} tim8_dt_us={} error_ppm={} sensor_samples={} integrated_dt_us={} integration_error_ppm={} raw_sensor_dt_us={} integration_clock=TIM8",
            perf.clock.windows,
            perf.clock.sensor_dt_us,
            perf.clock.tim8_dt_us,
            perf.clock.error_ppm,
            perf.clock.sensor_samples,
            perf.clock.integrated_dt_us,
            perf.clock.integration_error_ppm,
            perf.clock.raw_sensor_dt_us
        )?;
    }
    if let Some(stats) = PUBLISH_PERF.try_get() {
        writeln!(output, "EKF2_PUBLISH target_hz=1000 releases={} missed={} repeated={} max_late_us={} max_age_us={} max_us={} p99_upper_us={}",
            stats.releases, stats.missed, stats.repeated, stats.max_late_us, stats.max_age_us,
            stats.timing.max_us, stats.timing.p99_upper_us)?;
    }
    Ok(())
}
