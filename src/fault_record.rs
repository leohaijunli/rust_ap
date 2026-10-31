//! RTC BKP16..31 wire format. BKP0 remains the PX4 bootloader request.

pub const WORDS: usize = 16;
pub const MAGIC: u32 = 0x5031_4631; // P1F1, format version 1.
pub const PANIC: u32 = 1;
pub const HARD_FAULT: u32 = 2;
pub const UNEXPECTED_EXCEPTION: u32 = 3;
pub const STACK_GUARD: u32 = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaultRecord(pub [u32; WORDS]);

impl FaultRecord {
    pub fn new(kind: u32, source: u64) -> Self {
        let mut words = [0; WORDS];
        words[0] = MAGIC;
        words[1] = kind;
        words[2] = source as u32;
        words[3] = (source >> 32) as u32;
        Self(words)
    }

    pub fn seal(&mut self) {
        self.0[15] = checksum(&self.0[..15]);
    }

    pub fn valid(&self) -> bool {
        self.0[0] == MAGIC
            && matches!(
                self.0[1],
                PANIC | HARD_FAULT | UNEXPECTED_EXCEPTION | STACK_GUARD
            )
            && self.0[15] == checksum(&self.0[..15])
    }
}

// IEEE CRC-32, little-endian words, same convention as Python zlib.crc32.
fn checksum(words: &[u32]) -> u32 {
    let mut crc = u32::MAX;
    for word in words {
        for byte in word.to_le_bytes() {
            crc ^= u32::from(byte);
            for _ in 0..8 {
                crc = (crc >> 1) ^ (0xedb8_8320 & 0u32.wrapping_sub(crc & 1));
            }
        }
    }
    !crc
}

pub fn file_hash(file: &str) -> u32 {
    file.as_bytes().iter().fold(0x811c_9dc5, |hash, &byte| {
        (hash ^ u32::from(byte)).wrapping_mul(0x0100_0193)
    })
}

/// Validate the basic exception frame before any fault-time dereference.
/// On ARMv7-M the core frame is at SP for both basic and extended FP frames;
/// FP registers follow it (NuttX arm_exception.S:118-137). Validate the full
/// allocation within the shared application stack, away from the emergency
/// stack. Stacking/unstacking/lazy-FP errors invalidate it.
pub fn frame_is_readable(sp: u32, cfsr: u32, exc_return: u32) -> bool {
    const STACK_ERRORS: u32 = (1 << 3) | (1 << 4) | (1 << 5) | (1 << 11) | (1 << 12) | (1 << 13);
    matches!(
        exc_return,
        0xffff_fff1 | 0xffff_fff9 | 0xffff_fffd | 0xffff_ffe1 | 0xffff_ffe9 | 0xffff_ffed
    ) && cfsr & STACK_ERRORS == 0
        && sp.is_multiple_of(4)
        && (0x2407_0000..=0x2408_0000 - if exc_return & 0x10 == 0 { 104 } else { 32 }).contains(&sp)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_matches_independent_ieee_vector() {
        // zlib.crc32(bytes(range(8))) == 0x88aa689f.
        assert_eq!(checksum(&[0x0302_0100, 0x0706_0504]), 0x88aa_689f);
    }

    #[test]
    fn file_hash_matches_the_host_decoder_fixture() {
        // tools/decode_fault_dump.py pins the same value; keep both in sync.
        assert_eq!(file_hash("src/platform.rs"), 0x6065_2e12);
        assert_ne!(file_hash("src/platform.rs"), file_hash("src/main.rs"));
    }

    #[test]
    fn every_single_bit_corruption_is_rejected() {
        let mut record = FaultRecord::new(HARD_FAULT, 0x1234_5678_9abc_def0);
        record.0[6] = 0x0802_4320;
        record.seal();
        assert!(record.valid());
        for index in 0..WORDS {
            for bit in 0..32 {
                let mut corrupt = record;
                corrupt.0[index] ^= 1 << bit;
                assert!(!corrupt.valid());
            }
        }
    }

    #[test]
    fn interrupted_commit_never_accepts_partial_record() {
        let mut record = FaultRecord::new(PANIC, 123);
        record.seal();
        let mut retained = FaultRecord([0; WORDS]);
        for index in 1..WORDS {
            retained.0[index] = record.0[index];
            assert!(!retained.valid());
        }
        retained.0[0] = record.0[0]; // commit word written last
        assert!(retained.valid());
        retained.0[0] = 0; // invalidate before any overwrite
        assert!(!retained.valid());
    }

    #[test]
    fn damaged_stacks_and_invalid_exception_returns_are_not_dereferenced() {
        assert!(frame_is_readable(0x2407_ff80, 1 << 16, 0xffff_fff9));
        assert!(frame_is_readable(0x2407_ff00, 0, 0xffff_ffe9));
        assert!(frame_is_readable(0x2407_ffe0, 0, 0xffff_fff9));
        assert!(!frame_is_readable(0x2407_ffe0, 0, 0xffff_ffe9));
        assert!(!frame_is_readable(0x2406_ff00, 0, 0xffff_fff9));
        for sp in [0, 0x2400_0001, 0x2407_fff0, 0x2408_0000, u32::MAX] {
            assert!(!frame_is_readable(sp, 0, 0xffff_fff9));
        }
        assert!(!frame_is_readable(0x2407_ff80, 1 << 12, 0xffff_fff9));
        assert!(!frame_is_readable(0x2407_ff80, 0, 0xffff_fff5));
    }
}
