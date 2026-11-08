#![no_std]

//! Hardware-independent accounting, diagnostics and parameter state.

#[cfg(test)]
extern crate std;

#[path = "core/bmm150.rs"]
pub mod bmm150;
#[path = "core/bmp388.rs"]
pub mod bmp388;

#[path = "core/component_metadata.rs"]
pub mod component_metadata;
#[path = "core/config.rs"]
pub mod config;
#[path = "core/dronecan/mod.rs"]
pub mod dronecan;
#[path = "core/estimator/mod.rs"]
pub mod estimator;
#[path = "core/fault_record.rs"]
pub mod fault_record;
#[path = "core/gps.rs"]
pub mod gps;
#[path = "core/icm42688.rs"]
pub mod icm42688;
#[path = "core/ina219.rs"]
pub mod ina219;
#[path = "core/log_config.rs"]
pub mod log_config;
#[cfg(test)]
#[path = "storage/log_files.rs"]
mod log_files;
#[path = "core/log_queue.rs"]
pub mod log_queue;
#[path = "core/logger.rs"]
pub mod logger;
#[path = "core/logger_switch.rs"]
pub mod logger_switch;
#[path = "core/magnetometer.rs"]
pub mod magnetometer;
#[path = "core/mavcan.rs"]
pub mod mavcan;
#[path = "core/mavftp.rs"]
pub mod mavftp;
#[path = "core/mavlink_estimator.rs"]
pub mod mavlink_estimator;
#[path = "core/mavlink_magnetometer.rs"]
pub mod mavlink_magnetometer;
#[path = "core/mavlink_power.rs"]
pub mod mavlink_power;
#[path = "core/mavlink_protocol.rs"]
pub mod mavlink_protocol;
#[path = "core/mavlink_vehicle.rs"]
pub mod mavlink_vehicle;
#[path = "core/metrics.rs"]
pub mod metrics;
#[path = "core/parameter_store.rs"]
pub mod parameter_store;
#[path = "core/parameters.rs"]
pub mod parameters;
#[path = "core/px4_messages.rs"]
pub mod px4_messages;
#[path = "core/px4io.rs"]
pub mod px4io;
#[path = "core/slcan.rs"]
pub mod slcan;
#[path = "core/telemetry.rs"]
pub mod telemetry;
#[path = "core/ulog.rs"]
pub mod ulog;
#[path = "core/wall_clock.rs"]
pub mod wall_clock;

pub mod build_info {
    include!(concat!(env!("OUT_DIR"), "/build_info.rs"));
}
