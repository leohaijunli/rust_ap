//! Message builders and a bounded, non-blocking publication shortcut.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, watch::Watch};
use mavlink::common::{MavMessage, DEBUG_FLOAT_ARRAY_DATA};
use v6x_rust_ap::{
    mavlink_health::{self, Estimator, RateMonitor, Sensor},
    telemetry::Kind,
};

use crate::mavlink_core::{self, EstimatorMessage};

pub struct Outgoing {
    pub component: u8,
    pub message: MavMessage,
}

pub static PUBLISHED: Channel<CriticalSectionRawMutex, Outgoing, 8> = Channel::new();
static HEALTH: Watch<CriticalSectionRawMutex, DEBUG_FLOAT_ARRAY_DATA, 0> = Watch::new();

/// Observe successful data counters once per second, not once per link/message.
pub fn update_health(monitor: &mut RateMonitor) {
    let now = embassy_time::Instant::now().as_micros();
    if !monitor.due(now) {
        return;
    }
    let imu = crate::imu::latest();
    let (gyro, accel) = crate::imu::health();
    let imu_count = crate::imu::sample_count();
    let baro = crate::barometer::latest();
    let mag = crate::bmm150_driver::latest();
    let gps = crate::dronecan_can::latest_gps();
    let ina = crate::ina219::latest();
    let gps_health =
        v6x_rust_ap::mavlink_vehicle::gps_health(gps, embassy_time::Instant::now().as_micros());
    let sensors = [
        Sensor {
            count: imu_count,
            timestamp: imu.map(|s| s.sample_timestamp_us),
            state: gyro,
        },
        Sensor {
            count: imu_count,
            timestamp: imu.map(|s| s.sample_timestamp_us),
            state: accel,
        },
        Sensor {
            count: crate::barometer::sample_count(),
            timestamp: baro.map(|s| s.timestamp_sample),
            state: crate::barometer::health(),
        },
        Sensor {
            count: crate::bmm150_driver::sample_count(),
            timestamp: mag.map(|s| s.timestamp_sample),
            state: crate::bmm150_driver::health(),
        },
        Sensor {
            count: crate::dronecan_can::gps_sample_count(),
            timestamp: gps.map(|s| s.timestamp),
            state: gps_health,
        },
        Sensor {
            count: crate::ina219::sample_count(),
            timestamp: ina.map(|s| s.timestamp_us),
            state: crate::ina219::health(),
        },
    ];
    let output = crate::estimator::latest();
    let output = output.as_ref();
    let estimator = Estimator {
        report: crate::estimator::report(),
        epoch: crate::estimator::active_epoch(),
        status: output.map(|v| v.status),
        attitude_fresh: output.is_some_and(|v| v.snapshot.attitude.is_some()),
        solution_flags: output.map(|v| v.snapshot.status.solution_status_flags),
        fault_flags: output.map(|v| v.snapshot.status.filter_fault_flags),
    };
    let mut counters = [0; mavlink_health::COUNTER_COUNT];
    counters[0] = estimator.status.map_or(0, |status| status.updates);
    for (counter, sensor) in counters[1..].iter_mut().zip(sensors) {
        *counter = sensor.count;
    }
    // Sampling/health time follows all reads, since higher-priority producers can preempt them.
    let now = embassy_time::Instant::now().as_micros();
    if let Some(mut rates) = monitor.observe(now, counters, estimator.epoch) {
        if estimator.status.is_none()
            || !estimator.report.active
            || estimator.report.configured != v6x_rust_ap::config::Algorithm::Ekf2
        {
            rates.hz[0] = f32::NAN;
        }
        let data = mavlink_health::message(
            now,
            rates,
            estimator,
            sensors,
            gps.filter(|_| gps_health.healthy).map(|s| s.fix_type),
            crate::magnetometer::health().healthy,
            crate::diagnostics::cpu_load(),
        );
        HEALTH.sender().send(data);
    }
}

#[derive(Debug)]
pub enum PublishError {
    Full,
}

/// A producer never waits for either link; a full input queue is explicit.
/// Live delivery is best effort; slow-link overwrites are reported by `!metrics`.
pub fn publish(message: MavMessage) -> Result<(), PublishError> {
    PUBLISHED
        .try_send(Outgoing {
            component: mavlink_core::COMPONENT_ID,
            message,
        })
        .map_err(|_| PublishError::Full)
}

pub fn build(kind: Kind, now: u64, gps_no_fix_due: &mut u64, on_demand: bool) -> Option<Outgoing> {
    let message = match kind {
        Kind::Heartbeat => Some(mavlink_core::heartbeat()),
        Kind::SystemStatus => Some(mavlink_core::system_status()),
        Kind::SystemTime => Some(mavlink_core::system_time()),
        Kind::StatusText => Some(mavlink_core::status_text()),
        Kind::Health => HEALTH
            .try_get()
            .filter(|data| {
                now >= data.time_usec && now - data.time_usec <= mavlink_health::CACHE_FRESH_US
            })
            .map(MavMessage::DEBUG_FLOAT_ARRAY),
        Kind::HighresImu => mavlink_core::highres_imu(),
        Kind::ScaledPressure => mavlink_core::scaled_pressure(),
        Kind::PowerDebug => mavlink_core::ina219_status(),
        Kind::BatteryStatus => mavlink_core::battery_status(),
        Kind::Gps => {
            if let Some(message) = mavlink_core::gps_raw_int() {
                *gps_no_fix_due = now.saturating_add(1_000_000);
                Some(message)
            } else if on_demand || now >= *gps_no_fix_due {
                *gps_no_fix_due = now.saturating_add(1_000_000);
                Some(mavlink_core::gps_raw_int_no_fix())
            } else {
                None
            }
        }
        Kind::AutopilotVersion => Some(mavlink_core::autopilot_version()),
        Kind::ComponentMetadata => Some(mavlink_core::component_metadata()),
        estimator => {
            let kind = match estimator {
                Kind::Attitude => EstimatorMessage::AttitudeEuler,
                Kind::AttitudeQuaternion => EstimatorMessage::AttitudeQuaternion,
                Kind::LocalPosition => EstimatorMessage::LocalPosition,
                Kind::GlobalPosition => EstimatorMessage::GlobalPosition,
                Kind::Altitude => EstimatorMessage::Altitude,
                Kind::EstimatorStatus => EstimatorMessage::Status,
                _ => unreachable!(),
            };
            mavlink_core::estimator_message(kind)
        }
    }?;
    Some(Outgoing {
        component: mavlink_core::COMPONENT_ID,
        message,
    })
}
