//! Host-only P0 diagnostic of an unchanged upstream, with no CAN device access.

use dronecan::{Id, Transfer};
use std::panic::{catch_unwind, AssertUnwindSafe};

fn main() {
    // A start-of-multiframe tail with no CRC/payload must be rejected safely.
    let short_start_panics = catch_unwind(AssertUnwindSafe(|| {
        let mut storage = [0; 32];
        let mut transfer = Transfer::new(storage.as_mut_slice());
        let _ = transfer.add_frame(&[0x80]);
    }))
    .is_err();

    // Two transfers differ only in their CRC. A verifier cannot accept both.
    let accepts = |crc: [u8; 2]| {
        let mut storage = [0; 32];
        let mut transfer = Transfer::new(storage.as_mut_slice());
        let first = [crc[0], crc[1], 1, 2, 3, 4, 5, 0x80];
        transfer.add_frame(&first).unwrap();
        matches!(transfer.add_frame(&[6, 7, 8, 0x60]), Ok(Some(_)))
    };
    let distinct_crcs_both_accepted = accepts([0, 0]) && accepts([1, 0]);

    let mut storage = [0; 8];
    let mut transfer = Transfer::new(storage.as_mut_slice());
    let invalid_initial_toggle_accepted = matches!(transfer.add_frame(&[1, 0xe0]), Ok(Some(_)));

    println!("short_start_panics={short_start_panics}");
    println!("distinct_crcs_both_accepted={distinct_crcs_both_accepted}");
    println!("invalid_initial_toggle_accepted={invalid_initial_toggle_accepted}");
    println!(
        "service_node_zero_accepted={}",
        Id::service(0, 0, 1, true, 0).is_some()
    );
}
