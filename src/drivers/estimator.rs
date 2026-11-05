//! E7 data-only estimator task below the IMU executor. No control/output
//! permission is granted here; missing aids stay absent and nav stays unqualified.

use core::fmt;
use core::sync::atomic::{AtomicU32, Ordering};

use core::cell::RefCell;
use embassy_executor::InterruptExecutor;
use embassy_stm32::{interrupt, peripherals, Peri};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{with_timeout, Duration, Instant, Timer};
use static_cell::ConstStaticCell;
use v6x_rust_led::{
    estimator::{
        gnss::GnssObservation,
        imu_ekf::baro_mag::BaroAltitude,
        publication_reset::PublicationReset,
        runtime::{self, Fault, Params, Runtime, Selection, Status},
        standard_output::{EulerAttitude, Snapshot},
        standard_ulog::DataEncoder,
        FilterFault, GnssSample,
    },
    log_config::{self, Group},
    log_queue::{Class, PushError},
    parameters::{self, Id},
    ulog,
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
    pub snapshots: [Snapshot; 2],
    pub euler: [Option<EulerAttitude>; 2],
}

static CONTROL: Watch<CriticalSectionRawMutex, Control, 0> = Watch::new();
static OUTPUT: Watch<CriticalSectionRawMutex, Output, 0> = Watch::new();
static EPOCH: AtomicU32 = AtomicU32::new(0);
static CONFIG_ERRORS: AtomicU32 = AtomicU32::new(0);
static CONFIGURE_MAX_US: AtomicU32 = AtomicU32::new(0);
static LAST_CONFIG_FAULT: AtomicU32 = AtomicU32::new(u32::MAX);
/// Snapshot of the live clamped IMU params taken right after a failed reconfigure.
static FAILED_IMU_PARAMS: cortex_m::interrupt::Mutex<RefCell<[f32; 20]>> =
    cortex_m::interrupt::Mutex::new(RefCell::new([f32::NAN; 20]));
static LOG_ERRORS: AtomicU32 = AtomicU32::new(0);
static RUNTIME: ConstStaticCell<Runtime> = ConstStaticCell::new(Runtime::dormant());
static EXECUTOR: InterruptExecutor = InterruptExecutor::new();

pub fn start(timer: Peri<'static, peripherals::TIM5>) {
    // TIM5 is reserved as a software-pend vector; its timer is not run.
    EXECUTOR
        .start(interrupt::TIM5)
        .spawn(task(RUNTIME.take(), timer))
        .unwrap();
}

#[interrupt]
unsafe fn TIM5() {
    // This executor is polled only by its own non-reentrant interrupt.
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

pub fn maps(id: Id) -> bool {
    CONTROL
        .try_get()
        .is_some_and(|control| control.params.maps(id))
}

pub fn latest() -> Option<Output> {
    let control = CONTROL.try_get()?;
    if !control.params.enabled {
        return None;
    }
    let output = OUTPUT.try_get()?;
    let now = Instant::now().as_micros();
    if output.epoch != control.epoch
        || now < output.status.timestamp
        || now - output.status.timestamp > runtime::FRESH_US
    {
        return None;
    }
    Some(output)
}

fn stage(number: u32) -> impl Fn(FilterFault) -> u32 {
    move |fault| {
        number * 10
            + match fault {
                FilterFault::Input => 1,
                FilterFault::Timeout => 2,
                FilterFault::Numerics => 3,
            }
    }
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
pub async fn task(runtime: &'static mut Runtime, _timer: Peri<'static, peripherals::TIM5>) {
    let mut epoch = 0;
    let mut enabled = false;
    let mut resets = [PublicationReset::default(), PublicationReset::default()];
    let mut last_baro = 0;
    let mut last_gps = 0;
    let mut gps_device_id = 0;
    let mut next_publish = Instant::now();
    loop {
        let Some(control) = CONTROL.try_get() else {
            Timer::after_millis(20).await;
            continue;
        };
        if control.epoch != epoch {
            epoch = control.epoch;
            crate::imu::set_ahrs_active(false);
            enabled = false;
            last_baro = 0;
            last_gps = 0;
            if control.params.enabled {
                let started = Instant::now();
                // Split reconfigure so the failing sub-step is visible on board.
                // Fault code: 0 ok; stage*10 + variant, stage 1..4 in call order,
                // variant 1=Input 2=Timeout 3=Numerics 4=other. Stage 1 additionally
                // encodes the inner GnssEkf reconfigure chain in tens: 11 imu buffer
                // params, 12 baro/mag aids, 13 range aids, 14 gnss aids.
                let configure_once = |runtime: &mut Runtime| {
                    let params = &control.params;
                    // Constructor-level diagnostics ahead of the full reconfigure.
                    // 0 = ok, 1 = imu buffer params, 2 = baro/mag, 3 = range, 4 = gnss.
                    let probe = v6x_rust_led::estimator::constructor_probe(
                        params.aiding,
                        params.range,
                        params.gnss,
                    );
                    if probe != 0 {
                        return Err(probe);
                    }
                    let staged = runtime.main.reconfigure_staged(
                        params.imu,
                        params.aiding,
                        params.range,
                        params.gnss,
                        0,
                    );
                    if staged != 0 {
                        return Err(10 + staged);
                    }
                    runtime
                        .main
                        .configure_output(params.output)
                        .map_err(stage(2))?;
                    let staged = runtime.independent.reconfigure_staged(
                        params.imu,
                        params.aiding,
                        params.range,
                        0,
                    );
                    if staged != 0 {
                        return Err(30 + staged);
                    }
                    runtime
                        .independent
                        .configure_output(params.output)
                        .map_err(stage(4))?;
                    runtime.finish_reconfigure(params.enabled, params.algorithm);
                    Ok(())
                };
                // One retry: the constructor inputs are catalog-validated; a single
                // transient failure with otherwise-identical inputs must not wedge
                // bring-up. The failure code stays visible for the root-cause hunt.
                let mut configured = configure_once(runtime);
                if configured.is_err() {
                    configured = configure_once(runtime);
                }
                let fault = configured.as_ref().err().copied().unwrap_or(0);
                LAST_CONFIG_FAULT.store(fault, Ordering::Relaxed);
                if fault != 0 {
                    let core = runtime.main.core();
                    let imu = core.debug_params();
                    let live = [
                        imu.predict_us as f32,
                        imu.delay_max_ms,
                        imu.imu_ctrl as f32,
                        imu.velocity_limit,
                        imu.gyro_noise,
                        imu.accel_noise,
                        imu.gyro_bias_noise,
                        imu.accel_bias_noise,
                        imu.gyro_bias_init,
                        imu.accel_bias_init,
                        imu.angle_error_init,
                        imu.no_aid_noise,
                        imu.heading_noise,
                        imu.gyro_bias_limit,
                        imu.accel_bias_limit,
                        imu.accel_bias_accel_limit,
                        imu.accel_bias_gyro_limit,
                        imu.accel_bias_time_constant,
                        u8::from(core.debug_input_seen()) as f32,
                        0.0,
                    ];
                    cortex_m::interrupt::free(|cs| {
                        *FAILED_IMU_PARAMS.borrow(cs).borrow_mut() = live;
                    });
                }
                CONFIGURE_MAX_US.fetch_max(
                    started.elapsed().as_micros().min(u64::from(u32::MAX)) as u32,
                    Ordering::Relaxed,
                );
                match configured {
                    Ok(()) => {
                        enabled = true;
                        crate::imu::set_ahrs_active(true);
                    }
                    Err(_) => {
                        CONFIG_ERRORS.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        }
        if !enabled {
            Timer::after_millis(20).await;
            continue;
        }
        let filter = &mut *runtime;
        match with_timeout(Duration::from_millis(20), crate::imu::next_for_ahrs()).await {
            Ok(sample) => {
                if CONTROL
                    .try_get()
                    .is_some_and(|latest| latest.epoch != epoch)
                {
                    continue;
                }
                let now = Instant::now().as_micros();
                let baro = crate::barometer::latest()
                    .filter(|baro| {
                        baro.timestamp_sample > last_baro
                            && baro.timestamp_sample <= sample.sample_timestamp_us
                    })
                    .map(|baro| {
                        last_baro = baro.timestamp_sample;
                        BaroAltitude {
                            timestamp: baro.timestamp_sample,
                            altitude_m: baro.pressure_altitude_m(),
                        }
                    });
                let gps = gnss(
                    sample.sample_timestamp_us,
                    now,
                    &mut last_gps,
                    &mut gps_device_id,
                );
                let started = Instant::now();
                filter.update(sample, now, baro, gps, None);
                filter
                    .record_duration(started.elapsed().as_micros().min(u64::from(u32::MAX)) as u32);
            }
            Err(_) => {
                let status = filter.status();
                if Instant::now().as_micros().saturating_sub(status.timestamp) > runtime::FRESH_US {
                    filter.invalidate(Fault::Stale);
                }
            }
        }
        let now = Instant::now();
        if now < next_publish {
            continue;
        }
        next_publish = now + Duration::from_millis(50);
        let baro_device = if last_baro != 0 {
            v6x_rust_led::bmp388::DEVICE_ID
        } else {
            0
        };
        publish(
            filter,
            epoch,
            now,
            [baro_device, gps_device_id],
            &mut resets,
        );
    }
}

// Keep publication temporaries out of the 8-kHz async poll/update call chain.
#[inline(never)]
fn publish(
    filter: &mut Runtime,
    epoch: u32,
    now: Instant,
    devices: [u32; 2],
    resets: &mut [PublicationReset; 2],
) {
    let mut snapshots = filter.snapshots(now.as_micros());
    let status = filter.status();
    for (index, snapshot) in snapshots.iter_mut().enumerate() {
        snapshot.status.accel_device_id = v6x_rust_led::px4_messages::ICM42688_DEVICE_ID;
        snapshot.status.gyro_device_id = v6x_rust_led::px4_messages::ICM42688_DEVICE_ID;
        if devices[0] != 0 {
            snapshot.status.baro_device_id = devices[0];
            if let Some(aid) = &mut snapshot.aids[0] {
                aid.device_id = devices[0];
            }
        }
        if index == 0 {
            for aid in snapshot.aids[3..].iter_mut().flatten() {
                aid.device_id = devices[1];
            }
        }
        let source = if index == 0 {
            match status.selection {
                Selection::Invalid => 0,
                Selection::Ekf2 => 1,
                Selection::Mahony => 2,
            }
        } else {
            1
        };
        resets[index].apply(snapshot, epoch, source);
    }
    let euler = core::array::from_fn(|index| {
        snapshots[index].attitude.and_then(|attitude| {
            EulerAttitude::from_attitude(
                attitude,
                (index == 0 && status.selection == Selection::Mahony)
                    || snapshots[index].status.control_mode_flags & (1 << 1) == 0,
            )
        })
    });
    OUTPUT.sender().send(Output {
        epoch,
        status,
        snapshots,
        euler,
    });
}

/// Encoding runs in its own thread-executor task, so its scratch space is never
/// retained underneath another thread task or added to the 8-kHz EKF poll.
#[embassy_executor::task]
pub async fn log_task() {
    let mut encoders = [DataEncoder::new(0), DataEncoder::new(1)];
    let mut logger_was_ready = false;
    let mut file_index = 0;
    let mut last_output = 0;
    loop {
        Timer::after_millis(20).await;
        let ready = crate::sd_logger::ready();
        let index = crate::sd_logger::snapshot().file_index;
        if ready && (!logger_was_ready || index != file_index) {
            encoders = [DataEncoder::new(0), DataEncoder::new(1)];
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
        let timestamp = output.snapshots[0].status.timestamp;
        if timestamp <= last_output {
            continue;
        }
        last_output = timestamp;
        if log_config::rate(Group::Estimator, timestamp) {
            let runtime_record = runtime_record(output.epoch, output.status, timestamp);
            if crate::sd_logger::LOGGER
                .try_record(Class::Normal, timestamp, runtime_record)
                .is_err()
            {
                LOG_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
            for (index, (encoder, snapshot)) in
                encoders.iter_mut().zip(&output.snapshots).enumerate()
            {
                let result = encoder.write_snapshot::<LogError>(snapshot, |record| {
                    crate::sd_logger::LOGGER
                        .try_record(Class::Normal, timestamp, *record)
                        .map_err(LogError::Queue)
                });
                if let Err(error) = result {
                    // Queue drops already generate dropout evidence; encoder
                    // faults remain separately visible instead of being ignored.
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
                if let Some(euler) = output.euler[index] {
                    let record = ulog::attitude_euler_record(index as u8, euler)
                        .expect("fixed Euler schema and instance");
                    if crate::sd_logger::LOGGER
                        .try_record(Class::Normal, timestamp, record)
                        .is_err()
                    {
                        LOG_ERRORS.fetch_add(1, Ordering::Relaxed);
                    }
                }
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
        status.ekf_updates,
        status.gnss_samples,
        status.deadline_misses,
        status.max_update_us,
    ] {
        payload.push_u32(value).unwrap();
    }
    payload
        .push_u8(match status.selection {
            Selection::Invalid => 0,
            Selection::Ekf2 => 1,
            Selection::Mahony => 2,
        })
        .unwrap();
    payload.push_u8(fault_code(status.fault)).unwrap();
    payload
        .push_u8(fault_code(status.independent_fault))
        .unwrap();
    payload.push_u8(0).unwrap();
    ulog::data_record(
        ulog::TopicId::EstimatorRuntime as u16,
        now,
        payload.as_bytes(),
    )
    .expect("fixed runtime diagnostic fits")
}

fn fault_code(fault: Option<Fault>) -> u8 {
    match fault {
        None => 0,
        Some(Fault::Input) => 1,
        Some(Fault::Sequence) => 2,
        Some(Fault::Timestamp) => 3,
        Some(Fault::Stale) => 4,
        Some(Fault::Ekf(v6x_rust_led::estimator::FilterFault::Input)) => 5,
        Some(Fault::Ekf(v6x_rust_led::estimator::FilterFault::Timeout)) => 6,
        Some(Fault::Ekf(v6x_rust_led::estimator::FilterFault::Numerics)) => 7,
        Some(Fault::Deadline) => 8,
    }
}

fn gnss(imu_time: u64, now: u64, last: &mut u64, device: &mut u32) -> Option<GnssObservation> {
    let gps = crate::dronecan_can::latest_gps()?;
    if gps.timestamp <= *last
        || gps.timestamp > imu_time
        || now.saturating_sub(gps.timestamp) > v6x_rust_led::gps::GPS_FRESH_US
        || gps.fix_type < 3
        || !gps.vel_ned_valid
        || !gps.latitude_deg.is_finite()
        || !gps.longitude_deg.is_finite()
        || !gps.altitude_msl_m.is_finite()
        || [
            gps.eph,
            gps.epv,
            gps.s_variance_m_s,
            gps.hdop,
            gps.vdop,
            gps.vel_n_m_s,
            gps.vel_e_m_s,
            gps.vel_d_m_s,
        ]
        .iter()
        .any(|v| !v.is_finite())
        || gps.s_variance_m_s < 0.0
        || gps.eph < 0.0
        || gps.epv < 0.0
    {
        return None;
    }
    *last = gps.timestamp;
    *device = gps.device_id;
    Some(GnssObservation {
        sample: GnssSample {
            timestamp: gps.timestamp,
            latitude_deg: gps.latitude_deg,
            longitude_deg: gps.longitude_deg,
            altitude_msl_m: gps.altitude_msl_m as f32,
            velocity_ned_m_s: [gps.vel_n_m_s, gps.vel_e_m_s, gps.vel_d_m_s],
            horizontal_accuracy_m: gps.eph,
            vertical_accuracy_m: gps.epv,
            // Fixed EKF2.cpp forwards this field directly despite its legacy
            // name; taking another square root would change PX4 semantics.
            speed_accuracy_m_s: gps.s_variance_m_s,
            fix_type: gps.fix_type,
            satellites: gps.satellites_used,
        },
        pdop: libm::sqrtf(gps.hdop * gps.hdop + gps.vdop * gps.vdop),
        spoofed: gps.spoofing_state > 1,
    })
}

pub fn write_diagnostics(output: &mut impl fmt::Write) -> fmt::Result {
    let control = CONTROL.try_get();
    let latest = latest();
    writeln!(output, "EKF2_RUNTIME schema=1 requested={} epoch={} active={} data_only=true nav_qualified=false mag_available=false range_available=false baro_available={} config_errors={} log_errors={} configure_max_us={} executor=TIM5 priority=4 imu_priority=3",
        control.is_some_and(|c| c.params.enabled), control.map_or(0, |c| c.epoch), latest.is_some(), crate::barometer::latest().is_some(),
        CONFIG_ERRORS.load(Ordering::Relaxed), LOG_ERRORS.load(Ordering::Relaxed), CONFIGURE_MAX_US.load(Ordering::Relaxed))?;
    let fault = LAST_CONFIG_FAULT.load(Ordering::Relaxed);
    writeln!(
        output,
        "EKF2_CONFIG last_fault={} (0=ok; stage*10+variant: 11-15 main_cfg sub-steps, 2x main_out, 3x ind_cfg sub-steps, 4x ind_out; variant 1=input 2=timeout 3=numerics)",
        fault
    )?;
    writeln!(
        output,
        "EKF2_OUTPUT_FAULT code={} (0=none 1=params 2=input_seen 3=buffer_length; 40+i catalog miss, 50+i value range, 70 predict, 71 ctrl, 72 tau, 73 length)",
        v6x_rust_led::estimator::last_output_fault()
    )?;
    let (value, min, max) = v6x_rust_led::estimator::failed_range();
    writeln!(
        output,
        "EKF2_FAILED_RANGE value={value} min={min} max={max} (bits {:08x} {:08x} {:08x})",
        value.to_bits(),
        min.to_bits(),
        max.to_bits()
    )?;
    let values = cortex_m::interrupt::free(|cs| *FAILED_IMU_PARAMS.borrow(cs).borrow());
    writeln!(
        output,
        "EKF2_LIVE_IMU_PARAMS predict_us={} delay_max_ms={} imu_ctrl={} vel_lim={} gyr_noise={} acc_noise={} gyr_b_noise={} acc_b_noise={} gbias_init={} abias_init={} angerr_init={} noaid_noise={} head_noise={} gyr_b_lim={} abl_lim={} abl_acclim={} abl_gyrlim={} abl_tau={} input_seen={} pad={}",
        values[0], values[1], values[2], values[3], values[4], values[5], values[6],
        values[7], values[8], values[9], values[10], values[11], values[12], values[13],
        values[14], values[15], values[16], values[17], values[18], values[19]
    )?;
    if let Some(output_state) = latest {
        let status = output_state.status;
        writeln!(output, "EKF2_STATE timestamp_sample={} samples={} updates={} gnss_samples={} selected={:?} fault={:?} independent_fault={:?} max_update_us={} deadline_misses={} relative_yaw=true",
            status.timestamp, status.samples, status.ekf_updates, status.gnss_samples, status.selection, status.fault,
            status.independent_fault, status.max_update_us, status.deadline_misses)?;
    }
    Ok(())
}
