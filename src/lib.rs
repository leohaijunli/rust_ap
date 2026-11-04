#![no_std]

//! Hardware-independent accounting, diagnostics and parameter state.

#[cfg(test)]
extern crate std;

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
#[path = "core/log_queue.rs"]
pub mod log_queue;
#[path = "core/logger.rs"]
pub mod logger;
#[path = "core/logger_switch.rs"]
pub mod logger_switch;
#[path = "core/mavcan.rs"]
pub mod mavcan;
#[path = "core/mavftp.rs"]
pub mod mavftp;
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
#[path = "core/ulog.rs"]
pub mod ulog;
#[path = "core/wall_clock.rs"]
pub mod wall_clock;

pub mod build_info {
    include!(concat!(env!("OUT_DIR"), "/build_info.rs"));
}
