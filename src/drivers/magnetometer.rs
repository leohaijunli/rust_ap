//! Single-owner calibrated magnetic selection; drivers publish raw sensor_mag.

use core::{
    fmt,
    sync::atomic::{AtomicU32, Ordering},
};
use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, watch::Watch};
use embassy_time::{Instant, Timer};
use v6x_rust_led::{
    magnetometer::{Config, Selector, SourceStatus},
    mavlink_vehicle::SensorState,
    parameters::{self, Snapshot},
    px4_messages::{SensorMag, VehicleMagnetometer},
};

#[derive(Clone, Copy)]
enum Event {
    Sample(SensorMag),
    Fault(u32),
}
#[derive(Clone, Copy)]
struct State {
    selected: Option<VehicleMagnetometer>,
    sources: [SourceStatus; 4],
    switches: u32,
    failovers: u32,
}
static EVENTS: Channel<CriticalSectionRawMutex, Event, 16> = Channel::new();
static CONFIG: Watch<CriticalSectionRawMutex, Config, 0> = Watch::new();
static LATEST: Watch<CriticalSectionRawMutex, State, 0> = Watch::new();
static INPUT_DROPS: AtomicU32 = AtomicU32::new(0);
static REJECTED: AtomicU32 = AtomicU32::new(0);

pub fn configure(values: Snapshot) -> Result<(), parameters::Error> {
    if crate::OUTPUT_STATE.armed || crate::OUTPUT_STATE.outputs_enabled {
        return Err(parameters::Error::Configuration);
    }
    let config = Config::from_parameters(values).map_err(|_| parameters::Error::Configuration)?;
    CONFIG.sender().send(config);
    Ok(())
}

/// The common entry for real local/DroneCAN drivers, never a fictitious source.
pub fn publish(report: SensorMag) {
    if EVENTS.try_send(Event::Sample(report)).is_err() {
        INPUT_DROPS.fetch_add(1, Ordering::Relaxed);
    }
}
pub fn invalidate(device_id: u32) {
    if EVENTS.try_send(Event::Fault(device_id)).is_err() {
        INPUT_DROPS.fetch_add(1, Ordering::Relaxed);
    }
}
pub fn latest() -> Option<VehicleMagnetometer> {
    LATEST
        .try_get()?
        .selected
        .filter(|s| s.fresh(Instant::now().as_micros()))
}
pub fn health() -> SensorState {
    let state = LATEST.try_get();
    let raw = crate::bmm150_driver::health();
    let has_bmm = state.is_some_and(|s| {
        s.sources
            .iter()
            .any(|s| s.device_id == v6x_rust_led::bmm150::DEVICE_ID)
    });
    let bmm_priority = CONFIG
        .try_get()
        .unwrap_or_default()
        .priority(v6x_rust_led::bmm150::DEVICE_ID)
        .unwrap_or(0);
    SensorState {
        present: raw.present || state.is_some_and(|s| s.sources.iter().any(|s| s.device_id != 0)),
        enabled: state.is_some_and(|s| {
            s.sources
                .iter()
                .any(|s| s.device_id != 0 && s.priority != 0)
        }) || raw.present && !has_bmm && bmm_priority != 0,
        healthy: latest().is_some(),
    }
}

#[embassy_executor::task]
pub async fn run() -> ! {
    let mut selector = Selector::new();
    let mut config = Config::default();
    let mut epoch = 0;
    let mut published = None;
    let mut queued = None;
    loop {
        if let Some(next) = CONFIG.try_get().filter(|next| *next != config) {
            // Only configure() can populate this Watch, after full validation.
            if selector.configure(next).is_err() {
                REJECTED.fetch_add(1, Ordering::Relaxed);
            } else {
                config = next;
            }
        }
        match select(EVENTS.receive(), Timer::after_millis(10)).await {
            Either::First(Event::Sample(raw)) => {
                match selector.put(raw, Instant::now().as_micros()) {
                    Ok(index) => crate::sd_logger::record_raw_mag(raw, index),
                    Err(_) => {
                        REJECTED.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
            Either::First(Event::Fault(id)) => selector.invalidate(id),
            Either::Second(()) => {}
        }
        let now = Instant::now().as_micros();
        let current_epoch = crate::estimator::active_epoch();
        if epoch != current_epoch {
            epoch = current_epoch;
            selector.reset_delivery();
            queued = None;
        }
        let selected = selector.select(now);
        if let Some(selected) = selected {
            let key = (
                selected.data.device_id,
                selected.data.timestamp_sample,
                selected.data.calibration_count,
                selected.generation,
                selected.correction_generation,
            );
            if published != Some(key) {
                crate::sd_logger::record_vehicle_mag(selected.data);
                published = Some(key);
            }
            if (queued != Some(key) || selected.reset)
                && crate::estimator::submit_magnetometer(selected)
            {
                selector.delivered(selected);
                queued = Some(key);
            }
        } else {
            published = None;
            queued = None;
        }
        LATEST.sender().send(State {
            selected: selected.map(|s| s.data),
            sources: selector.status(now),
            switches: selector.switches,
            failovers: selector.failovers,
        });
    }
}

pub fn write_diagnostics(out: &mut impl fmt::Write) -> fmt::Result {
    let selected = latest();
    let state = LATEST.try_get();
    writeln!(out, "MAG_SELECT schema=1 valid={} selected_id={} calibration_count={} switches={} failovers={} input_drops={} rejected={} mode=single_source board_rotation=0 nav_qualified=false",
        selected.is_some(), selected.map_or(0,|s| s.device_id), selected.map_or(0,|s| s.calibration_count),
        state.map_or(0,|s| s.switches), state.map_or(0,|s| s.failovers), INPUT_DROPS.load(Ordering::Relaxed), REJECTED.load(Ordering::Relaxed))?;
    if let Some(state) = state {
        for (index, source) in state
            .sources
            .into_iter()
            .enumerate()
            .filter(|(_, s)| s.device_id != 0)
        {
            writeln!(out, "MAG_SOURCE index={} device_id={} external={} calibration_bound={} priority={} effective_priority={} confidence={} flags={:x} sample_us={}", index,source.device_id,source.external,source.calibrated,source.priority,source.effective_priority,source.confidence,source.flags,source.timestamp_sample)?;
        }
    }
    if let Some(s) = selected {
        writeln!(
            out,
            "MAG_BODY sample_us={} x_gauss={} y_gauss={} z_gauss={}",
            s.timestamp_sample, s.magnetometer_ga[0], s.magnetometer_ga[1], s.magnetometer_ga[2]
        )?;
    }
    crate::bmm150_driver::write_diagnostics(out)
}
