//! FRAM tail layout and byte-atomic commit protocol. No HAL or transport dependency.

use crate::parameters::{self, Snapshot, COUNT, REGISTRY};

pub const FORMAT_VERSION: u16 = 1;
pub const START: u16 = 0x7800;
pub const SLOT_SIZE: usize = 1024;
pub const RECORD_SIZE: usize = 24 + COUNT * 4;
const HEADER_SIZE: usize = 20;
const COMMITTED: u8 = 0xa5;
const MAGIC: &[u8; 4] = b"RPAR";
const _: () = assert!(RECORD_SIZE <= SLOT_SIZE);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Slot {
    A,
    B,
}

impl Slot {
    pub const ALL: [Self; 2] = [Self::A, Self::B];

    pub const fn address(self) -> u16 {
        START + self as u16 * SLOT_SIZE as u16
    }

    fn other(self) -> Self {
        match self {
            Self::A => Self::B,
            Self::B => Self::A,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Active {
    pub slot: Slot,
    pub generation: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Record {
    pub generation: u32,
    pub snapshot: Snapshot,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    Uncommitted,
    Magic,
    Length,
    Crc,
    Version,
    Schema,
    Values,
}

pub fn decode(bytes: &[u8]) -> Result<Record, DecodeError> {
    if bytes.len() < HEADER_SIZE + 4 {
        return Err(DecodeError::Length);
    }
    if bytes[0] != COMMITTED {
        return Err(DecodeError::Uncommitted);
    }
    if &bytes[1..5] != MAGIC || bytes[5] != 0 {
        return Err(DecodeError::Magic);
    }
    let payload_len = usize::from(u16::from_le_bytes(bytes[18..20].try_into().unwrap()));
    let crc_offset = HEADER_SIZE + payload_len;
    if crc_offset + 4 > bytes.len() || crc_offset + 4 > SLOT_SIZE {
        return Err(DecodeError::Length);
    }
    if checksum(&bytes[1..crc_offset]) != word(bytes, crc_offset) {
        return Err(DecodeError::Crc);
    }
    if u16::from_le_bytes(bytes[6..8].try_into().unwrap()) != FORMAT_VERSION {
        return Err(DecodeError::Version);
    }
    if word(bytes, 8) != parameters::schema_tag()
        || usize::from(u16::from_le_bytes(bytes[16..18].try_into().unwrap())) != COUNT
        || payload_len != COUNT * 4
    {
        return Err(DecodeError::Schema);
    }
    let mut values = [0.0; COUNT];
    for (index, value) in values.iter_mut().enumerate() {
        *value = f32::from_bits(word(bytes, HEADER_SIZE + index * 4));
    }
    Ok(Record {
        generation: word(bytes, 12),
        snapshot: Snapshot::from_values(values).map_err(|_| DecodeError::Values)?,
    })
}

/// Select the newest compatible, valid record; serial arithmetic also handles wrap.
pub fn newest(records: &[Result<Record, DecodeError>; 2]) -> Option<(Active, Snapshot)> {
    let mut selected: Option<(Active, Snapshot)> = None;
    for (slot, record) in Slot::ALL.into_iter().zip(records) {
        let Ok(record) = record else { continue };
        if selected.is_none_or(|(active, _)| {
            let delta = record.generation.wrapping_sub(active.generation);
            delta != 0 && delta < 0x8000_0000
        }) {
            selected = Some((
                Active {
                    slot,
                    generation: record.generation,
                },
                record.snapshot,
            ));
        }
    }
    selected
}

#[derive(Clone, Copy)]
pub struct OutputState {
    pub armed: bool,
    pub outputs_enabled: bool,
}

#[derive(Debug, PartialEq, Eq)]
pub struct OutputsActive;

pub struct Write<'a> {
    pub address: u16,
    pub bytes: &'a [u8],
}

pub struct Commit {
    pub active: Active,
    bytes: [u8; RECORD_SIZE],
}

impl Commit {
    pub fn new(
        previous: Option<Active>,
        snapshot: Snapshot,
        outputs: OutputState,
    ) -> Result<Self, OutputsActive> {
        if outputs.armed || outputs.outputs_enabled {
            return Err(OutputsActive);
        }
        let active = previous.map_or(
            Active {
                slot: Slot::A,
                generation: 1,
            },
            |previous| Active {
                slot: previous.slot.other(),
                generation: previous.generation.wrapping_add(1),
            },
        );
        let mut bytes = [0; RECORD_SIZE];
        bytes[0] = COMMITTED;
        bytes[1..5].copy_from_slice(MAGIC);
        bytes[6..8].copy_from_slice(&FORMAT_VERSION.to_le_bytes());
        bytes[8..12].copy_from_slice(&parameters::schema_tag().to_le_bytes());
        bytes[12..16].copy_from_slice(&active.generation.to_le_bytes());
        bytes[16..18].copy_from_slice(&(COUNT as u16).to_le_bytes());
        bytes[18..20].copy_from_slice(&((COUNT * 4) as u16).to_le_bytes());
        for (index, entry) in REGISTRY.iter().enumerate() {
            let offset = HEADER_SIZE + index * 4;
            bytes[offset..offset + 4]
                .copy_from_slice(&snapshot.get(entry.id).to_bits().to_le_bytes());
        }
        let crc_offset = RECORD_SIZE - 4;
        let crc = checksum(&bytes[1..crc_offset]);
        bytes[crc_offset..].copy_from_slice(&crc.to_le_bytes());
        Ok(Self { active, bytes })
    }

    pub fn bytes(&self) -> &[u8; RECORD_SIZE] {
        &self.bytes
    }

    /// Invalidate the inactive slot, write/verify its body, then commit one byte.
    /// Callers must read back each write before advancing to the next phase.
    pub fn writes(&self) -> [Write<'_>; 3] {
        let address = self.active.slot.address();
        [
            Write {
                address,
                bytes: &[0],
            },
            Write {
                address: address + 1,
                bytes: &self.bytes[1..],
            },
            Write {
                address,
                bytes: &self.bytes[..1],
            },
        ]
    }
}

fn word(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

// IEEE CRC-32 over the explicit wire bytes, compatible with zlib.crc32.
fn checksum(bytes: &[u8]) -> u32 {
    let mut crc = u32::MAX;
    for &byte in bytes {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb8_8320 & 0u32.wrapping_sub(crc & 1));
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parameters::{Id, Parameters};

    const OFF: OutputState = OutputState {
        armed: false,
        outputs_enabled: false,
    };

    fn snapshot(hz: f32) -> Snapshot {
        let mut parameters = Parameters::default();
        parameters.set(Id::LinkLedHz, hz).unwrap();
        parameters.snapshot()
    }

    fn reseal(bytes: &mut [u8; RECORD_SIZE]) {
        let crc = checksum(&bytes[1..RECORD_SIZE - 4]);
        bytes[RECORD_SIZE - 4..].copy_from_slice(&crc.to_le_bytes());
    }

    #[test]
    fn corruption_and_incompatible_records_fall_back_to_the_valid_slot() {
        assert_eq!(checksum(b"123456789"), 0xcbf4_3926);
        let old = Commit::new(None, snapshot(1.0), OFF).unwrap();
        let next = Commit::new(Some(old.active), snapshot(2.0), OFF).unwrap();
        let old_record = decode(old.bytes()).unwrap();
        assert_eq!(
            newest(&[Ok(old_record), decode(next.bytes())]),
            Some((next.active, snapshot(2.0)))
        );
        for index in 0..RECORD_SIZE {
            for bit in 0..8 {
                let mut corrupt = *next.bytes();
                corrupt[index] ^= 1 << bit;
                let decoded = decode(&corrupt);
                assert!(decoded.is_err());
                assert_eq!(
                    newest(&[Ok(old_record), decoded]),
                    Some((old.active, snapshot(1.0)))
                );
            }
        }
        for (offset, value, error) in [
            (6, 2, DecodeError::Version),
            (8, next.bytes()[8] ^ 1, DecodeError::Schema),
        ] {
            let mut bytes = *next.bytes();
            bytes[offset] = value;
            reseal(&mut bytes);
            assert_eq!(decode(&bytes), Err(error));
            assert_eq!(
                newest(&[decode(&bytes), Err(DecodeError::Uncommitted)]),
                None
            );
        }
        let mut bytes = *next.bytes();
        let offset = HEADER_SIZE + Id::LinkLedHz as usize * 4;
        bytes[offset..offset + 4].copy_from_slice(&f32::NAN.to_bits().to_le_bytes());
        reseal(&mut bytes);
        assert_eq!(decode(&bytes), Err(DecodeError::Values));
    }

    #[test]
    fn every_interrupted_byte_preserves_the_previous_commit() {
        let mut slots = [[0; SLOT_SIZE]; 2];
        let mut previous = None;
        for hz in [1.5, 2.0, 3.0] {
            let commit =
                Commit::new(previous.map(|(active, _)| active), snapshot(hz), OFF).unwrap();
            for (phase, write) in commit.writes().iter().enumerate() {
                for (index, &byte) in write.bytes.iter().enumerate() {
                    let offset = usize::from(write.address - START) + index;
                    slots[offset / SLOT_SIZE][offset % SLOT_SIZE] = byte;
                    let restored = newest(&[decode(&slots[0]), decode(&slots[1])]);
                    let expected = if phase == 2 {
                        Some((commit.active, snapshot(hz)))
                    } else {
                        previous
                    };
                    assert_eq!(restored, expected, "phase={phase} byte={index}");
                }
            }
            previous = Some((commit.active, snapshot(hz)));
        }
        // Save permission is checked before any write plan can be obtained.
        for outputs in [
            OutputState {
                armed: true,
                outputs_enabled: false,
            },
            OutputState {
                armed: false,
                outputs_enabled: true,
            },
        ] {
            assert!(matches!(
                Commit::new(None, snapshot(1.0), outputs),
                Err(OutputsActive)
            ));
        }
        assert!(slots
            .iter()
            .all(|slot| slot[RECORD_SIZE..].iter().all(|&byte| byte == 0)));
    }

    #[test]
    fn newest_selection_handles_generation_wrap() {
        let old = Commit::new(
            Some(Active {
                slot: Slot::A,
                generation: u32::MAX - 1,
            }),
            snapshot(1.0),
            OFF,
        )
        .unwrap();
        let next = Commit::new(Some(old.active), snapshot(2.0), OFF).unwrap();
        assert_eq!(next.active.generation, 0);
        assert_eq!(
            newest(&[decode(next.bytes()), decode(old.bytes())]),
            Some((next.active, snapshot(2.0)))
        );
    }
}
