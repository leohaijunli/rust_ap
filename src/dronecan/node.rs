//! Fixed-capacity node directory and trusted four-ESC status mapping.

use super::dsdl::{EscStatus, NodeInfo, NodeStatus};

pub const ESC_COUNT: usize = 4;
pub const NODE_CAPACITY: usize = 16;
pub const ESC_STALE_US: u64 = 100_000;
pub const NODE_OFFLINE_US: u64 = 3_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EscMapping {
    source_by_index: [Option<u8>; ESC_COUNT],
}

impl EscMapping {
    pub fn new(source_by_index: [Option<u8>; ESC_COUNT]) -> Result<Self, MappingError> {
        for (index, source) in source_by_index.iter().enumerate() {
            let Some(source) = source else { continue };
            if !(1..=127).contains(source) {
                return Err(MappingError::NodeId);
            }
            if source_by_index[..index].contains(&Some(*source)) {
                return Err(MappingError::DuplicateNode);
            }
        }
        Ok(Self { source_by_index })
    }

    pub const fn source(&self, esc_index: usize) -> Option<u8> {
        self.source_by_index[esc_index]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MappingError {
    NodeId,
    DuplicateNode,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EscSlot {
    pub source: u8,
    pub status: EscStatus,
    pub received_at_us: u64,
    pub sequence: u32,
}

impl EscSlot {
    pub fn fresh(&self, now_us: u64) -> bool {
        now_us.saturating_sub(self.received_at_us) <= ESC_STALE_US
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EscStats {
    pub trusted: u32,
    pub candidates: u32,
    pub source_mismatch: u32,
    pub index_out_of_range: u32,
    pub invalid_value: u32,
}

pub struct EscMonitor {
    mapping: EscMapping,
    slots: [Option<EscSlot>; ESC_COUNT],
    candidates: [Option<u8>; ESC_COUNT],
    stats: EscStats,
}

impl EscMonitor {
    pub const fn new(mapping: EscMapping) -> Self {
        Self {
            mapping,
            slots: [None; ESC_COUNT],
            candidates: [None; ESC_COUNT],
            stats: EscStats {
                trusted: 0,
                candidates: 0,
                source_mismatch: 0,
                index_out_of_range: 0,
                invalid_value: 0,
            },
        }
    }

    pub fn observe(
        &mut self,
        source: u8,
        status: EscStatus,
        received_at_us: u64,
    ) -> EscObservation {
        if status.voltage.is_infinite()
            || status.current.is_infinite()
            || status.temperature.is_infinite()
        {
            self.stats.invalid_value = self.stats.invalid_value.saturating_add(1);
            return EscObservation::InvalidValue;
        }
        let index = usize::from(status.esc_index);
        if index >= ESC_COUNT {
            self.stats.index_out_of_range = self.stats.index_out_of_range.saturating_add(1);
            return EscObservation::IndexOutOfRange;
        }
        let Some(expected_source) = self.mapping.source(index) else {
            self.candidates[index] = Some(source);
            self.stats.candidates = self.stats.candidates.saturating_add(1);
            return EscObservation::Candidate;
        };
        if source != expected_source {
            self.stats.source_mismatch = self.stats.source_mismatch.saturating_add(1);
            return EscObservation::SourceMismatch;
        }

        let sequence = self.slots[index].map_or(1, |slot| slot.sequence.saturating_add(1));
        self.slots[index] = Some(EscSlot {
            source,
            status,
            received_at_us,
            sequence,
        });
        self.stats.trusted = self.stats.trusted.saturating_add(1);
        EscObservation::Trusted
    }

    pub const fn slot(&self, esc_index: usize) -> Option<EscSlot> {
        self.slots[esc_index]
    }

    pub const fn candidate(&self, esc_index: usize) -> Option<u8> {
        self.candidates[esc_index]
    }

    pub const fn stats(&self) -> EscStats {
        self.stats
    }

    pub const fn mapping(&self) -> EscMapping {
        self.mapping
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EscObservation {
    Trusted,
    Candidate,
    SourceMismatch,
    IndexOutOfRange,
    InvalidValue,
}

#[derive(Clone, Copy)]
pub struct NodeEntry {
    pub node_id: u8,
    pub status: NodeStatus,
    pub status_received_at_us: u64,
    pub info: Option<NodeInfo>,
}

impl NodeEntry {
    pub fn online(&self, now_us: u64) -> bool {
        self.status.mode != 7
            && now_us.saturating_sub(self.status_received_at_us) <= NODE_OFFLINE_US
    }
}

pub struct NodeDirectory {
    entries: [Option<NodeEntry>; NODE_CAPACITY],
}

impl NodeDirectory {
    pub const fn new() -> Self {
        Self {
            entries: [None; NODE_CAPACITY],
        }
    }

    /// Returns true only when a previously unseen node was added.
    pub fn update_status(&mut self, node_id: u8, status: NodeStatus, now_us: u64) -> bool {
        if let Some(entry) = self.entry_mut(node_id) {
            if status.uptime_sec < entry.status.uptime_sec {
                entry.info = None;
            }
            entry.status = status;
            entry.status_received_at_us = now_us;
            return false;
        }
        let slot = self.entries.iter().position(Option::is_none).or_else(|| {
            self.entries
                .iter()
                .position(|entry| entry.is_some_and(|entry| !entry.online(now_us)))
        });
        let Some(slot) = slot else {
            return false;
        };
        self.entries[slot] = Some(NodeEntry {
            node_id,
            status,
            status_received_at_us: now_us,
            info: None,
        });
        true
    }

    pub fn update_info(&mut self, node_id: u8, info: NodeInfo) -> bool {
        let Some(entry) = self.entry_mut(node_id) else {
            return false;
        };
        entry.info = Some(info);
        true
    }

    pub fn entry(&self, node_id: u8) -> Option<&NodeEntry> {
        self.entries
            .iter()
            .flatten()
            .find(|entry| entry.node_id == node_id)
    }

    pub fn entries(&self) -> impl Iterator<Item = &NodeEntry> {
        self.entries.iter().flatten()
    }

    fn entry_mut(&mut self, node_id: u8) -> Option<&mut NodeEntry> {
        self.entries
            .iter_mut()
            .flatten()
            .find(|entry| entry.node_id == node_id)
    }
}

impl Default for NodeDirectory {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn status(index: u8) -> EscStatus {
        EscStatus {
            error_count: 0,
            voltage: f32::NAN,
            current: f32::NAN,
            temperature: f32::NAN,
            rpm: 0,
            power_rating_pct: 0,
            esc_index: index,
        }
    }

    #[test]
    fn mapping_rejects_duplicates_and_never_trusts_unconfigured_candidates() {
        assert_eq!(
            EscMapping::new([Some(10), Some(10), None, None]),
            Err(MappingError::DuplicateNode)
        );
        let mapping = EscMapping::new([Some(10), None, Some(12), Some(13)]).unwrap();
        let mut monitor = EscMonitor::new(mapping);
        assert_eq!(
            monitor.observe(11, status(0), 1),
            EscObservation::SourceMismatch
        );
        assert_eq!(monitor.observe(11, status(1), 2), EscObservation::Candidate);
        assert_eq!(monitor.candidate(1), Some(11));
        assert_eq!(monitor.slot(1), None);
        assert_eq!(monitor.observe(10, status(0), 3), EscObservation::Trusted);
        assert!(monitor.slot(0).unwrap().fresh(100_003));
        assert!(!monitor.slot(0).unwrap().fresh(100_004));
        assert_eq!(
            monitor.observe(10, status(4), 4),
            EscObservation::IndexOutOfRange
        );
    }

    #[test]
    fn node_status_becomes_offline_and_info_requires_a_known_node() {
        let mut directory = NodeDirectory::new();
        let status = NodeStatus::operational(1);
        assert!(directory.update_status(10, status, 5));
        assert!(!directory.update_status(10, NodeStatus::operational(2), 10));
        assert!(directory.entry(10).unwrap().online(3_000_010));
        assert!(!directory.entry(10).unwrap().online(3_000_011));

        let info = NodeInfo {
            status,
            software: super::super::dsdl::SoftwareVersion::basic(1, 0),
            hardware_major: 0,
            hardware_minor: 0,
            unique_id: [1; 16],
            name: [0; 80],
            name_len: 0,
        };
        assert!(!directory.update_info(11, info));
        assert!(directory.update_info(10, info));
    }

    #[test]
    fn bounded_can_state_stays_within_the_initial_ram_budget() {
        let bytes = core::mem::size_of::<super::super::transport::RxTransport>()
            + core::mem::size_of::<[super::super::transport::Payload; 8]>()
            + core::mem::size_of::<EscMonitor>()
            + core::mem::size_of::<NodeDirectory>();
        assert!(bytes <= 32 * 1024, "bounded CAN state uses {bytes} bytes");
    }

    #[test]
    fn explicit_offline_restart_and_full_directory_are_handled() {
        let mut directory = NodeDirectory::new();
        for node_id in 1..=NODE_CAPACITY as u8 {
            assert!(directory.update_status(
                node_id,
                NodeStatus::operational(u32::from(node_id) + 10),
                0,
            ));
        }
        let mut offline = NodeStatus::operational(20);
        offline.mode = 7;
        assert!(!directory.update_status(1, offline, 1));
        assert!(!directory.entry(1).unwrap().online(1));
        assert!(directory.update_status(17, NodeStatus::operational(1), 2));
        assert!(directory.entry(1).is_none());

        let info = NodeInfo {
            status: NodeStatus::operational(1),
            software: super::super::dsdl::SoftwareVersion::basic(1, 0),
            hardware_major: 0,
            hardware_minor: 0,
            unique_id: [1; 16],
            name: [0; 80],
            name_len: 0,
        };
        assert!(directory.update_info(2, info));
        assert!(directory.entry(2).unwrap().info.is_some());
        assert!(!directory.update_status(2, NodeStatus::operational(1), 4));
        assert!(directory.entry(2).unwrap().info.is_none());
    }
}
