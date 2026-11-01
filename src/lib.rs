#![no_std]

//! Hardware-independent accounting, diagnostics and parameter state.

pub mod fault_record;
pub mod metrics;
pub mod parameter_store;
pub mod parameters;

pub mod build_info {
    include!(concat!(env!("OUT_DIR"), "/build_info.rs"));
}
