//! FRAM tail layout and byte-atomic commit protocol. No HAL or transport dependency.
//!
//! Records are keyed by a hash of the parameter name, not by registry position,
//! so adding or removing parameters never discards the engineer's settings:
//! unknown keys are ignored and new parameters keep their default. Only values
//! that differ from the default are stored, which keeps a record far below the
//! slot size even though every parameter is writable.

use crate::parameters::{self, Access, Snapshot, REGISTRY};

pub const FORMAT_VERSION: u16 = 2;
pub const START: u16 = 0x7800;
pub const SLOT_SIZE: usize = 1024;
const HEADER_SIZE: usize = 20;
const COMMITTED: u8 = 0xa5;
const MAGIC: &[u8; 4] = b"RPAR";
/// Name hash plus value. Room is reserved for the whole slot, not the current
/// registry, so a larger registry still fits as long as the number of changed
/// values does.
const ENTRY_SIZE: usize = 6;
pub const MAX_ENTRIES: usize = (SLOT_SIZE - HEADER_SIZE - 4) / ENTRY_SIZE;
pub const RECORD_SIZE: usize = HEADER_SIZE + MAX_ENTRIES * ENTRY_SIZE + 4;
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
    /// True for a positional record written before name-keyed storage; callers
    /// keep it dirty so the next save rewrites it in the current format.
    pub migrated: bool,
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
    let count = usize::from(u16::from_le_bytes(bytes[16..18].try_into().unwrap()));
    let version = u16::from_le_bytes(bytes[6..8].try_into().unwrap());
    let (snapshot, migrated) = match version {
        FORMAT_VERSION => (decode_entries(bytes, count, payload_len)?, false),
        1 => (decode_positional(bytes, count, payload_len)?, true),
        _ => return Err(DecodeError::Version),
    };
    Ok(Record {
        generation: word(bytes, 12),
        snapshot,
        migrated,
    })
}

/// Keys that are not in this build are ignored, not treated as corruption: the
/// same record must stay valid across registry additions and removals.
fn decode_entries(bytes: &[u8], count: usize, payload_len: usize) -> Result<Snapshot, DecodeError> {
    if payload_len != count * ENTRY_SIZE || count > MAX_ENTRIES {
        return Err(DecodeError::Length);
    }
    let mut values = *Snapshot::default().values();
    for index in 0..count {
        let offset = HEADER_SIZE + index * ENTRY_SIZE;
        let hash = u16::from_le_bytes(bytes[offset..offset + 2].try_into().unwrap());
        let value = f32::from_bits(word(bytes, offset + 2));
        // A range that changed between builds leaves the stored value out of
        // range; the new default is then the honest answer.
        let Some(id) = parameters::Id::from_hash(hash) else {
            continue;
        };
        let definition = id.definition();
        if definition.access == Access::ReadOnly || definition.validate(value).is_err() {
            continue;
        }
        values[id as usize] = value;
    }
    Snapshot::from_values(values).map_err(|_| DecodeError::Values)
}

/// Positional records from the schema-tagged format predate name keys.
fn decode_positional(
    bytes: &[u8],
    count: usize,
    payload_len: usize,
) -> Result<Snapshot, DecodeError> {
    let schema = word(bytes, 8);
    if count == 0
        || parameters::prefix_schema_tag(count) != Some(schema)
        || payload_len != count * 4
    {
        return Err(DecodeError::Schema);
    }
    let mut values = *Snapshot::default().values();
    for (index, value) in values[..count].iter_mut().enumerate() {
        *value = f32::from_bits(word(bytes, HEADER_SIZE + index * 4));
    }
    Snapshot::from_values(values).map_err(|_| DecodeError::Values)
}

/// Select the newest compatible, valid record; serial arithmetic also handles wrap.
pub fn newest(records: &[Result<Record, DecodeError>; 2]) -> Option<(Active, Record)> {
    let mut selected: Option<(Active, Record)> = None;
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
                *record,
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
pub enum CommitError {
    /// Persisting is only allowed with the vehicle disarmed and outputs closed.
    OutputsActive,
    /// More values differ from their default than the slot can key.
    Capacity,
}

pub struct Write<'a> {
    pub address: u16,
    pub bytes: &'a [u8],
}

pub struct Commit {
    pub active: Active,
    bytes: [u8; RECORD_SIZE],
    len: usize,
}

impl Commit {
    pub fn new(
        previous: Option<Active>,
        snapshot: Snapshot,
        outputs: OutputState,
    ) -> Result<Self, CommitError> {
        if outputs.armed || outputs.outputs_enabled {
            return Err(CommitError::OutputsActive);
        }
        let changed = REGISTRY
            .iter()
            .filter(|entry| snapshot.get(entry.id) != entry.default)
            .count();
        if changed > MAX_ENTRIES {
            return Err(CommitError::Capacity);
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
        let payload_len = changed * ENTRY_SIZE;
        let len = HEADER_SIZE + payload_len + 4;
        let mut bytes = [0; RECORD_SIZE];
        bytes[0] = COMMITTED;
        bytes[1..5].copy_from_slice(MAGIC);
        bytes[6..8].copy_from_slice(&FORMAT_VERSION.to_le_bytes());
        bytes[8..12].copy_from_slice(&parameters::schema_tag().to_le_bytes());
        bytes[12..16].copy_from_slice(&active.generation.to_le_bytes());
        bytes[16..18].copy_from_slice(&(changed as u16).to_le_bytes());
        bytes[18..20].copy_from_slice(&(payload_len as u16).to_le_bytes());
        let mut offset = HEADER_SIZE;
        for entry in REGISTRY {
            let value = snapshot.get(entry.id);
            if value == entry.default {
                continue;
            }
            bytes[offset..offset + 2].copy_from_slice(&entry.hash.to_le_bytes());
            bytes[offset + 2..offset + 6].copy_from_slice(&value.to_bits().to_le_bytes());
            offset += ENTRY_SIZE;
        }
        let crc = checksum(&bytes[1..HEADER_SIZE + payload_len]);
        bytes[HEADER_SIZE + payload_len..len].copy_from_slice(&crc.to_le_bytes());
        Ok(Self { active, bytes, len })
    }

    /// Exactly the bytes that belong to this record, without the slot tail.
    pub fn data(&self) -> &[u8] {
        &self.bytes[..self.len]
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
                bytes: &self.bytes[1..self.len],
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

    fn reseal(bytes: &mut [u8], len: usize) {
        let crc = checksum(&bytes[1..len - 4]);
        bytes[len - 4..len].copy_from_slice(&crc.to_le_bytes());
    }

    #[test]
    fn registry_keys_are_unique_so_no_value_can_shadow_another() {
        for (index, entry) in REGISTRY.iter().enumerate() {
            assert_eq!(
                Id::from_hash(parameters::name_hash(entry.name)),
                Some(entry.id),
                "{} collides or is unreachable",
                entry.name
            );
            assert_eq!(
                REGISTRY[..index]
                    .iter()
                    .filter(|other| parameters::name_hash(other.name)
                        == parameters::name_hash(entry.name))
                    .count(),
                0
            );
        }
    }

    #[test]
    fn only_changed_values_are_stored_and_unknown_keys_are_ignored() {
        let commit = Commit::new(None, snapshot(2.0), OFF).unwrap();
        assert_eq!(
            usize::from(u16::from_le_bytes(
                commit.data()[16..18].try_into().unwrap()
            )),
            1
        );
        let record = decode(commit.data()).unwrap();
        assert_eq!(record.snapshot, snapshot(2.0));
        assert_eq!(record.snapshot.get(Id::LogImuHz), 200.0);
        assert!(!record.migrated);

        // A key that this build does not know must not fail the whole record.
        let mut bytes = commit.data().to_vec();
        let hash = parameters::name_hash("REMOVED_PARAM") | 0x8000;
        bytes[HEADER_SIZE..HEADER_SIZE + 2].copy_from_slice(&hash.to_le_bytes());
        let len = bytes.len();
        let mut fixed = [0_u8; RECORD_SIZE];
        fixed[..len].copy_from_slice(&bytes);
        reseal(&mut fixed, len);
        assert_eq!(decode(&fixed[..len]).unwrap().snapshot, Snapshot::default());

        // A known, out-of-range stored value keeps its default.
        let mut bytes = fixed;
        bytes[HEADER_SIZE..HEADER_SIZE + 2]
            .copy_from_slice(&parameters::name_hash("LINK_LED_HZ").to_le_bytes());
        bytes[HEADER_SIZE + 2..HEADER_SIZE + 6].copy_from_slice(&42.0_f32.to_le_bytes());
        reseal(&mut bytes, len);
        assert_eq!(decode(&bytes[..len]).unwrap().snapshot, Snapshot::default());
    }

    #[test]
    fn positional_records_migrate_without_losing_values() {
        let mut bytes = [0_u8; 68];
        bytes[0] = COMMITTED;
        bytes[1..5].copy_from_slice(MAGIC);
        bytes[6..8].copy_from_slice(&1_u16.to_le_bytes());
        bytes[8..12].copy_from_slice(&parameters::prefix_schema_tag(11).unwrap().to_le_bytes());
        bytes[12..16].copy_from_slice(&7_u32.to_le_bytes());
        bytes[16..18].copy_from_slice(&11_u16.to_le_bytes());
        bytes[18..20].copy_from_slice(&44_u16.to_le_bytes());
        let mut values = *Snapshot::default().values();
        values[parameters::Id::LinkLedHz as usize] = 2.0;
        for (index, value) in values[..11].iter().enumerate() {
            bytes[20 + index * 4..24 + index * 4].copy_from_slice(&value.to_le_bytes());
        }
        let crc = checksum(&bytes[1..64]);
        bytes[64..68].copy_from_slice(&crc.to_le_bytes());
        let record = decode(&bytes).unwrap();
        assert_eq!(record.generation, 7);
        assert_eq!(record.snapshot.get(parameters::Id::LinkLedHz), 2.0);
        assert_eq!(record.snapshot.get(parameters::Id::LogImuHz), 200.0);
        assert_eq!(record.snapshot.get(parameters::Id::EstEnable), 0.0);
        assert_eq!(record.snapshot.get(parameters::Id::Ekf2PredictUs), 10_000.0);
        assert!(record.migrated);
    }

    #[test]
    fn shipped_159_parameter_record_migrates_and_new_rates_keep_defaults() {
        assert_eq!(Id::Ekf2En as usize, 17);
        assert_eq!(Id::MavRateHeartbeat as usize, 159);
        let mut parameters = Parameters::default();
        parameters.set(Id::LinkLedHz, 2.0).unwrap();
        parameters.set(Id::Ekf2PredictUs, 20_000.0).unwrap();
        let snapshot = parameters.snapshot();
        let count = 159;
        let len = HEADER_SIZE + count * 4 + 4;
        let mut bytes = [0; SLOT_SIZE];
        bytes[0] = COMMITTED;
        bytes[1..5].copy_from_slice(MAGIC);
        bytes[6..8].copy_from_slice(&1_u16.to_le_bytes());
        bytes[8..12].copy_from_slice(&parameters::prefix_schema_tag(count).unwrap().to_le_bytes());
        bytes[16..18].copy_from_slice(&(count as u16).to_le_bytes());
        bytes[18..20].copy_from_slice(&((count * 4) as u16).to_le_bytes());
        for (index, value) in snapshot.values()[..count].iter().enumerate() {
            bytes[HEADER_SIZE + index * 4..HEADER_SIZE + (index + 1) * 4]
                .copy_from_slice(&value.to_le_bytes());
        }
        reseal(&mut bytes, len);
        let record = decode(&bytes).unwrap();
        assert!(record.migrated);
        assert_eq!(record.snapshot, snapshot);
        parameters.adopt(record.snapshot);
        assert!(parameters.is_dirty());
        let commit = Commit::new(None, parameters.snapshot(), OFF).unwrap();
        assert_eq!(decode(commit.data()).unwrap().snapshot, snapshot);
        assert!(!decode(commit.data()).unwrap().migrated);
    }

    #[test]
    fn corruption_and_incompatible_records_fall_back_to_the_valid_slot() {
        assert_eq!(checksum(b"123456789"), 0xcbf4_3926);
        let old = Commit::new(None, snapshot(1.0), OFF).unwrap();
        let next = Commit::new(Some(old.active), snapshot(2.0), OFF).unwrap();
        let mut values = *Snapshot::default().values();
        values[Id::LinkLedHz as usize] = 2.0;
        let expected = Snapshot::from_values(values).unwrap();
        assert_eq!(
            newest(&[decode(old.data()), decode(next.data())]),
            Some((next.active, decode(next.data()).unwrap()))
        );
        for index in 0..next.data().len() {
            for bit in 0..8 {
                let mut corrupt = [0_u8; RECORD_SIZE];
                corrupt[..next.data().len()].copy_from_slice(next.data());
                corrupt[index] ^= 1 << bit;
                let decoded = decode(&corrupt[..next.data().len()]);
                assert!(decoded.is_err());
                assert_eq!(
                    newest(&[decode(old.data()), decoded])
                        .map(|(active, record)| (active, record.snapshot)),
                    Some((old.active, snapshot(1.0)))
                );
            }
        }
        for (offset, value, error) in [(6, 3, DecodeError::Version), (5, 1, DecodeError::Magic)] {
            let mut bytes = [0_u8; RECORD_SIZE];
            let len = next.data().len();
            bytes[..len].copy_from_slice(next.data());
            bytes[offset] = value;
            reseal(&mut bytes, len);
            assert_eq!(decode(&bytes[..len]), Err(error));
            assert_eq!(
                newest(&[decode(&bytes[..len]), Err(DecodeError::Uncommitted)]),
                None
            );
        }
        let mut invalid = *Snapshot::default().values();
        invalid[Id::MavSysId as usize] = 2.0;
        let len = next.data().len();
        let mut bytes = [0_u8; RECORD_SIZE];
        bytes[..len].copy_from_slice(next.data());
        bytes[HEADER_SIZE + 2..HEADER_SIZE + 6].copy_from_slice(&2.0_f32.to_le_bytes());
        reseal(&mut bytes, len);
        assert_eq!(decode(&bytes[..len]).unwrap().snapshot, expected);
        assert_eq!(
            Snapshot::from_values(invalid),
            Err(parameters::Error::ReadOnly)
        );
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
                    assert_eq!(
                        restored.map(|(active, record)| (active, record.snapshot)),
                        expected,
                        "phase={phase} byte={index}"
                    );
                }
            }
            previous = Some((commit.active, snapshot(hz)));
        }
        assert!(slots
            .iter()
            .all(|slot| slot[RECORD_SIZE..].iter().all(|&byte| byte == 0)));
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
            assert_eq!(
                Commit::new(None, snapshot(1.0), outputs).err(),
                Some(CommitError::OutputsActive)
            );
        }
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
            newest(&[decode(next.data()), decode(old.data())])
                .map(|(active, record)| (active, record.snapshot)),
            Some((next.active, snapshot(2.0)))
        );
    }
}
