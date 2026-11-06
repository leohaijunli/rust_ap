//! Message builders and a bounded, non-blocking publication shortcut.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use mavlink::common::MavMessage;
use v6x_rust_led::telemetry::Kind;

use crate::mavlink_core::{self, EstimatorMessage};

pub struct Outgoing {
    pub component: u8,
    pub message: MavMessage,
}

pub static PUBLISHED: Channel<CriticalSectionRawMutex, Outgoing, 8> = Channel::new();

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
