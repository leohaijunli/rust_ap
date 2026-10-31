//! P0 compile-only ABI measurement. Sizes exclude temporary stack and task state.
#![no_std]

use core::mem::{align_of, size_of};
use cybflight_core::{
    eskf::Eskf,
    indi::controller::IndiController,
    mpc::sqp_solver::{FullSqpSolver, SimpleSqpSolver},
    trajectory_planning::{bfgs_trust::BfgsWorkspace, minco_snap::MincoSnap},
};

// Each pair is (size, alignment), in the order documented above.
// Dump this section from the thumbv7em object; host pointer sizes differ.
#[used]
#[no_mangle]
pub static P0_CORE_ABI: [u32; 12] = [
    size_of::<IndiController>() as u32,
    align_of::<IndiController>() as u32,
    size_of::<Eskf>() as u32,
    align_of::<Eskf>() as u32,
    size_of::<SimpleSqpSolver>() as u32,
    align_of::<SimpleSqpSolver>() as u32,
    size_of::<FullSqpSolver>() as u32,
    align_of::<FullSqpSolver>() as u32,
    size_of::<BfgsWorkspace>() as u32,
    align_of::<BfgsWorkspace>() as u32,
    size_of::<MincoSnap>() as u32,
    align_of::<MincoSnap>() as u32,
];
