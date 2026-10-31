#![no_std]

//! Hardware-independent P1 accounting and retained diagnostic format.

pub mod fault_record;
pub mod metrics;

pub mod build_info {
    include!(concat!(env!("OUT_DIR"), "/build_info.rs"));
}
