//! Classic-CAN DroneCAN v0 transfer framing and bounded reassembly.

pub const MAX_TRANSFER_PAYLOAD: usize = 512;
pub const RX_SLOT_COUNT: usize = 12;
pub const RX_SESSION_COUNT: usize = 64;

const TRANSFER_ID_TIMEOUT_US: u64 = 2_000_000;
const FRAME_TIMEOUT_US: u64 = 100_000;
const MESSAGE_TRANSFER_TIMEOUT_US: u64 = 100_000;
const SERVICE_TRANSFER_TIMEOUT_US: u64 = 1_000_000;
const RESERVED_MESSAGE_SLOTS: usize = 4;
const RESERVED_MESSAGE_SESSIONS: usize = 16;
const NO_SLOT: u8 = u8::MAX;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    id: u32,
    data: [u8; 8],
    len: u8,
}

impl Frame {
    pub fn new(id: u32, data: &[u8]) -> Result<Self, FrameError> {
        if id > 0x1fff_ffff {
            return Err(FrameError::Id);
        }
        if data.is_empty() || data.len() > 8 {
            return Err(FrameError::Length);
        }
        let mut bytes = [0; 8];
        bytes[..data.len()].copy_from_slice(data);
        Ok(Self {
            id,
            data: bytes,
            len: data.len() as u8,
        })
    }

    pub const fn id(&self) -> u32 {
        self.id
    }

    pub fn data(&self) -> &[u8] {
        &self.data[..usize::from(self.len)]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    Id,
    Length,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransferKind {
    Message {
        data_type_id: u16,
    },
    Service {
        service_id: u8,
        request: bool,
        destination: u8,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Metadata {
    pub priority: u8,
    pub source: u8,
    pub transfer_id: u8,
    pub kind: TransferKind,
}

impl Metadata {
    pub fn message(
        priority: u8,
        data_type_id: u16,
        source: u8,
        transfer_id: u8,
    ) -> Result<Self, MetadataError> {
        let metadata = Self {
            priority,
            source,
            transfer_id,
            kind: TransferKind::Message { data_type_id },
        };
        metadata.validate()?;
        Ok(metadata)
    }

    pub fn service(
        priority: u8,
        service_id: u8,
        request: bool,
        source: u8,
        destination: u8,
        transfer_id: u8,
    ) -> Result<Self, MetadataError> {
        let metadata = Self {
            priority,
            source,
            transfer_id,
            kind: TransferKind::Service {
                service_id,
                request,
                destination,
            },
        };
        metadata.validate()?;
        Ok(metadata)
    }

    pub fn from_id(id: u32, transfer_id: u8) -> Result<Self, MetadataError> {
        if id > 0x1fff_ffff || transfer_id > 31 {
            return Err(MetadataError::Range);
        }
        let priority = ((id >> 24) & 0x1f) as u8;
        let source = (id & 0x7f) as u8;
        if source == 0 {
            return Err(MetadataError::AnonymousUnsupported);
        }
        if id & (1 << 7) == 0 {
            return Self::message(priority, ((id >> 8) & 0xffff) as u16, source, transfer_id);
        }

        let destination = ((id >> 8) & 0x7f) as u8;
        Self::service(
            priority,
            ((id >> 16) & 0xff) as u8,
            id & (1 << 15) != 0,
            source,
            destination,
            transfer_id,
        )
    }

    pub fn id(&self) -> Result<u32, MetadataError> {
        self.validate()?;
        let common = u32::from(self.priority) << 24 | u32::from(self.source);
        Ok(match self.kind {
            TransferKind::Message { data_type_id } => common | u32::from(data_type_id) << 8,
            TransferKind::Service {
                service_id,
                request,
                destination,
            } => {
                common
                    | u32::from(service_id) << 16
                    | u32::from(request) << 15
                    | u32::from(destination) << 8
                    | 1 << 7
            }
        })
    }

    fn validate(&self) -> Result<(), MetadataError> {
        if self.priority > 31 || self.transfer_id > 31 || !(1..=127).contains(&self.source) {
            return Err(MetadataError::Range);
        }
        if let TransferKind::Service { destination, .. } = self.kind {
            if !(1..=127).contains(&destination) {
                return Err(MetadataError::Range);
            }
        }
        Ok(())
    }

    fn descriptor(&self) -> Descriptor {
        Descriptor {
            source: self.source,
            kind: self.kind,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MetadataError {
    Range,
    AnonymousUnsupported,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TypeInfo {
    pub signature: u64,
    pub max_payload: u16,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Payload {
    bytes: [u8; MAX_TRANSFER_PAYLOAD],
    len: u16,
}

impl Payload {
    pub const fn new() -> Self {
        Self {
            bytes: [0; MAX_TRANSFER_PAYLOAD],
            len: 0,
        }
    }

    pub fn from_slice(bytes: &[u8]) -> Result<Self, PayloadError> {
        if bytes.len() > MAX_TRANSFER_PAYLOAD {
            return Err(PayloadError::TooLong);
        }
        let mut payload = Self::new();
        payload.bytes[..bytes.len()].copy_from_slice(bytes);
        payload.len = bytes.len() as u16;
        Ok(payload)
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.bytes[..usize::from(self.len)]
    }

    pub fn capacity_mut(&mut self) -> &mut [u8] {
        &mut self.bytes
    }

    pub fn set_len(&mut self, len: usize) -> Result<(), PayloadError> {
        if len > self.bytes.len() {
            return Err(PayloadError::TooLong);
        }
        self.len = len as u16;
        Ok(())
    }
}

impl Default for Payload {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PayloadError {
    TooLong,
}

pub struct TxFrames<'a> {
    metadata: Metadata,
    payload: &'a [u8],
    crc: u16,
    stream_offset: usize,
    frame_index: usize,
    done: bool,
    id: u32,
}

impl<'a> TxFrames<'a> {
    pub fn new(
        metadata: Metadata,
        type_info: TypeInfo,
        payload: &'a [u8],
    ) -> Result<Self, TxError> {
        if payload.len() > usize::from(type_info.max_payload)
            || payload.len() > MAX_TRANSFER_PAYLOAD
        {
            return Err(TxError::PayloadTooLong);
        }
        let id = metadata.id().map_err(TxError::Metadata)?;
        Ok(Self {
            metadata,
            payload,
            crc: transfer_crc(type_info.signature, payload),
            stream_offset: 0,
            frame_index: 0,
            done: false,
            id,
        })
    }

    pub const fn frame_count(&self) -> usize {
        if self.payload.len() <= 7 {
            1
        } else {
            (self.payload.len() + 2).div_ceil(7)
        }
    }

    fn stream_len(&self) -> usize {
        if self.payload.len() <= 7 {
            self.payload.len()
        } else {
            self.payload.len() + 2
        }
    }

    fn stream_byte(&self, index: usize) -> u8 {
        if self.payload.len() <= 7 {
            self.payload[index]
        } else {
            match index {
                0 => self.crc as u8,
                1 => (self.crc >> 8) as u8,
                _ => self.payload[index - 2],
            }
        }
    }
}

impl Iterator for TxFrames<'_> {
    type Item = Frame;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }
        let stream_len = self.stream_len();
        let count = (stream_len - self.stream_offset).min(7);
        let mut data = [0; 8];
        for (offset, byte) in data[..count].iter_mut().enumerate() {
            *byte = self.stream_byte(self.stream_offset + offset);
        }
        let first = self.frame_index == 0;
        let last = self.stream_offset + count == stream_len;
        data[count] = u8::from(first) << 7
            | u8::from(last) << 6
            | ((self.frame_index as u8 & 1) << 5)
            | self.metadata.transfer_id;
        self.stream_offset += count;
        self.frame_index += 1;
        self.done = last;
        Frame::new(self.id, &data[..=count]).ok()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxError {
    Metadata(MetadataError),
    PayloadTooLong,
}

pub fn transfer_crc(signature: u64, payload: &[u8]) -> u16 {
    let crc = crc16(0xffff, &signature.to_le_bytes());
    crc16(crc, payload)
}

pub fn crc16(mut crc: u16, bytes: &[u8]) -> u16 {
    for &byte in bytes {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                crc << 1 ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Descriptor {
    source: u8,
    kind: TransferKind,
}

const EMPTY_DESCRIPTOR: Descriptor = Descriptor {
    source: 1,
    kind: TransferKind::Message { data_type_id: 0 },
};

#[derive(Clone, Copy)]
struct Session {
    used: bool,
    descriptor: Descriptor,
    expected_transfer_id: u8,
    last_transfer_us: u64,
    slot: u8,
}

impl Session {
    const EMPTY: Self = Self {
        used: false,
        descriptor: EMPTY_DESCRIPTOR,
        expected_transfer_id: 0,
        last_transfer_us: 0,
        slot: NO_SLOT,
    };
}

#[derive(Clone, Copy)]
struct RxSlot {
    used: bool,
    descriptor: Descriptor,
    raw_id: u32,
    transfer_id: u8,
    expected_toggle: bool,
    started_us: u64,
    last_frame_us: u64,
    type_info: TypeInfo,
    len: u16,
    bytes: [u8; MAX_TRANSFER_PAYLOAD + 2],
}

impl RxSlot {
    const EMPTY: Self = Self {
        used: false,
        descriptor: EMPTY_DESCRIPTOR,
        raw_id: 0,
        transfer_id: 0,
        expected_toggle: false,
        started_us: 0,
        last_frame_us: 0,
        type_info: TypeInfo {
            signature: 0,
            max_payload: 0,
        },
        len: 0,
        bytes: [0; MAX_TRANSFER_PAYLOAD + 2],
    };
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RxStats {
    pub transfers: u32,
    pub duplicates: u32,
    pub transfer_id_gaps: u32,
    pub crc_errors: u32,
    pub timeouts: u32,
    pub rejected: u32,
    pub capacity_errors: u32,
}

#[derive(Debug, PartialEq, Eq)]
pub struct CompletedTransfer {
    pub metadata: Metadata,
    pub payload: Payload,
}

pub struct RxTransport {
    sessions: [Session; RX_SESSION_COUNT],
    slots: [RxSlot; RX_SLOT_COUNT],
    stats: RxStats,
}

impl RxTransport {
    pub const fn new() -> Self {
        Self {
            sessions: [Session::EMPTY; RX_SESSION_COUNT],
            slots: [RxSlot::EMPTY; RX_SLOT_COUNT],
            stats: RxStats {
                transfers: 0,
                duplicates: 0,
                transfer_id_gaps: 0,
                crc_errors: 0,
                timeouts: 0,
                rejected: 0,
                capacity_errors: 0,
            },
        }
    }

    pub const fn stats(&self) -> RxStats {
        self.stats
    }

    pub fn accept(
        &mut self,
        frame: &Frame,
        now_us: u64,
        local_node_id: u8,
        type_info: impl Fn(&Metadata) -> Option<TypeInfo>,
    ) -> Result<Option<CompletedTransfer>, RxError> {
        self.expire(now_us);
        let data = frame.data();
        let tail = Tail::parse(*data.last().ok_or(RxError::Frame)?);
        let metadata = Metadata::from_id(frame.id(), tail.transfer_id).map_err(|_| RxError::Id)?;
        if let TransferKind::Service { destination, .. } = metadata.kind {
            if destination != local_node_id {
                return self.reject(RxError::WrongAddress);
            }
        }
        let Some(info) = type_info(&metadata) else {
            return self.reject(RxError::UnknownType);
        };
        if usize::from(info.max_payload) > MAX_TRANSFER_PAYLOAD {
            return self.reject(RxError::PayloadTooLong);
        }

        if tail.start && tail.end {
            if tail.toggle {
                return self.reject(RxError::Tail);
            }
            let payload = &data[..data.len() - 1];
            if payload.len() > usize::from(info.max_payload) {
                return self.reject(RxError::PayloadTooLong);
            }
            let session = self.prepare_start(metadata.descriptor(), tail.transfer_id, now_us)?;
            let payload = Payload::from_slice(payload).map_err(|_| RxError::PayloadTooLong)?;
            self.advance(session, tail.transfer_id, now_us);
            self.stats.transfers = self.stats.transfers.saturating_add(1);
            return Ok(Some(CompletedTransfer { metadata, payload }));
        }

        if tail.start {
            if tail.toggle || data.len() != 8 {
                return self.reject(RxError::Tail);
            }
            if info.max_payload <= 7 {
                return self.reject(RxError::NonCanonical);
            }
            let slot = self.allocate_slot(metadata.kind).ok_or_else(|| {
                self.stats.capacity_errors = self.stats.capacity_errors.saturating_add(1);
                RxError::Capacity
            })?;
            let session = self.prepare_start(metadata.descriptor(), tail.transfer_id, now_us)?;
            let state = &mut self.slots[slot];
            *state = RxSlot {
                used: true,
                descriptor: metadata.descriptor(),
                raw_id: frame.id(),
                transfer_id: tail.transfer_id,
                expected_toggle: true,
                started_us: now_us,
                last_frame_us: now_us,
                type_info: info,
                len: 7,
                bytes: [0; MAX_TRANSFER_PAYLOAD + 2],
            };
            state.bytes[..7].copy_from_slice(&data[..7]);
            self.sessions[session].slot = slot as u8;
            return Ok(None);
        }

        self.accept_continuation(frame, metadata, tail, now_us)
    }

    fn accept_continuation(
        &mut self,
        frame: &Frame,
        metadata: Metadata,
        tail: Tail,
        now_us: u64,
    ) -> Result<Option<CompletedTransfer>, RxError> {
        let descriptor = metadata.descriptor();
        let Some(session_index) = self.session_index(descriptor) else {
            return self.reject(RxError::MissedStart);
        };
        let slot_index = usize::from(self.sessions[session_index].slot);
        if slot_index >= self.slots.len() || !self.slots[slot_index].used {
            return self.reject(RxError::MissedStart);
        }
        let data = frame.data();
        let slot = &self.slots[slot_index];
        if frame.id() != slot.raw_id || descriptor != slot.descriptor {
            return self.reject(RxError::Id);
        }
        if tail.transfer_id != slot.transfer_id {
            return self.reject(RxError::TransferId);
        }
        if tail.toggle != slot.expected_toggle {
            return self.reject(RxError::Toggle);
        }
        if (!tail.end && data.len() != 8) || (tail.end && data.len() < 2) {
            return self.reject(RxError::Tail);
        }

        let bytes = &data[..data.len() - 1];
        let end = usize::from(slot.len) + bytes.len();
        let limit = usize::from(slot.type_info.max_payload) + 2;
        if end > limit || end > MAX_TRANSFER_PAYLOAD + 2 {
            self.abort_slot(session_index, slot_index);
            self.advance(session_index, tail.transfer_id, now_us);
            return self.reject(RxError::PayloadTooLong);
        }
        let slot = &mut self.slots[slot_index];
        slot.bytes[usize::from(slot.len)..end].copy_from_slice(bytes);
        slot.len = end as u16;
        slot.last_frame_us = now_us;
        slot.expected_toggle = !slot.expected_toggle;
        if !tail.end {
            return Ok(None);
        }

        let stream_len = usize::from(slot.len);
        let expected_crc = u16::from_le_bytes([slot.bytes[0], slot.bytes[1]]);
        let payload_len = stream_len.saturating_sub(2);
        if payload_len <= 7 {
            self.abort_slot(session_index, slot_index);
            self.advance(session_index, tail.transfer_id, now_us);
            return self.reject(RxError::NonCanonical);
        }
        let actual_crc = transfer_crc(slot.type_info.signature, &slot.bytes[2..stream_len]);
        if actual_crc != expected_crc {
            self.abort_slot(session_index, slot_index);
            self.advance(session_index, tail.transfer_id, now_us);
            self.stats.crc_errors = self.stats.crc_errors.saturating_add(1);
            return Err(RxError::Crc);
        }

        let payload =
            Payload::from_slice(&slot.bytes[2..stream_len]).map_err(|_| RxError::PayloadTooLong)?;
        self.abort_slot(session_index, slot_index);
        self.advance(session_index, tail.transfer_id, now_us);
        self.stats.transfers = self.stats.transfers.saturating_add(1);
        Ok(Some(CompletedTransfer { metadata, payload }))
    }

    fn prepare_start(
        &mut self,
        descriptor: Descriptor,
        transfer_id: u8,
        now_us: u64,
    ) -> Result<usize, RxError> {
        if let Some(index) = self.session_index(descriptor) {
            let timed_out = now_us.saturating_sub(self.sessions[index].last_transfer_us)
                > TRANSFER_ID_TIMEOUT_US;
            let incomplete = self.sessions[index].slot != NO_SLOT;
            if timed_out {
                if incomplete {
                    let slot = usize::from(self.sessions[index].slot);
                    self.abort_slot(index, slot);
                }
                self.sessions[index].expected_transfer_id = transfer_id;
            } else if incomplete {
                if transfer_id == self.sessions[index].expected_transfer_id
                    || transfer_id
                        == previous_transfer_id(self.sessions[index].expected_transfer_id)
                {
                    self.stats.duplicates = self.stats.duplicates.saturating_add(1);
                    return Err(RxError::Duplicate);
                }
                let slot = usize::from(self.sessions[index].slot);
                self.abort_slot(index, slot);
                if transfer_id != self.sessions[index].expected_transfer_id {
                    self.stats.transfer_id_gaps = self.stats.transfer_id_gaps.saturating_add(1);
                    self.sessions[index].expected_transfer_id = transfer_id;
                }
            } else if transfer_id == self.sessions[index].expected_transfer_id {
                // Expected transfer.
            } else if transfer_id == previous_transfer_id(self.sessions[index].expected_transfer_id)
            {
                self.stats.duplicates = self.stats.duplicates.saturating_add(1);
                return Err(RxError::Duplicate);
            } else {
                self.sessions[index].expected_transfer_id = transfer_id;
                self.stats.transfer_id_gaps = self.stats.transfer_id_gaps.saturating_add(1);
            }
            self.sessions[index].last_transfer_us = now_us;
            return Ok(index);
        }

        let start = if matches!(descriptor.kind, TransferKind::Service { .. }) {
            RESERVED_MESSAGE_SESSIONS
        } else {
            0
        };
        let Some(index) = self.sessions[start..]
            .iter()
            .position(|session| !session.used)
            .map(|index| start + index)
        else {
            self.stats.capacity_errors = self.stats.capacity_errors.saturating_add(1);
            return Err(RxError::Capacity);
        };
        self.sessions[index] = Session {
            used: true,
            descriptor,
            expected_transfer_id: transfer_id,
            last_transfer_us: now_us,
            slot: NO_SLOT,
        };
        Ok(index)
    }

    fn advance(&mut self, session: usize, transfer_id: u8, now_us: u64) {
        self.sessions[session].expected_transfer_id = next_transfer_id(transfer_id);
        self.sessions[session].last_transfer_us = now_us;
        self.sessions[session].slot = NO_SLOT;
    }

    fn session_index(&self, descriptor: Descriptor) -> Option<usize> {
        self.sessions
            .iter()
            .position(|session| session.used && session.descriptor == descriptor)
    }

    fn allocate_slot(&self, kind: TransferKind) -> Option<usize> {
        let start = if matches!(kind, TransferKind::Service { .. }) {
            RESERVED_MESSAGE_SLOTS
        } else {
            0
        };
        self.slots[start..]
            .iter()
            .position(|slot| !slot.used)
            .map(|index| start + index)
    }

    fn abort_slot(&mut self, session: usize, slot: usize) {
        if slot < self.slots.len() {
            self.slots[slot].used = false;
            self.slots[slot].len = 0;
        }
        self.sessions[session].slot = NO_SLOT;
    }

    fn expire(&mut self, now_us: u64) {
        for slot_index in 0..self.slots.len() {
            if !self.slots[slot_index].used {
                continue;
            }
            let total_timeout = if matches!(
                self.slots[slot_index].descriptor.kind,
                TransferKind::Service { .. }
            ) {
                SERVICE_TRANSFER_TIMEOUT_US
            } else {
                MESSAGE_TRANSFER_TIMEOUT_US
            };
            let frame_expired =
                now_us.saturating_sub(self.slots[slot_index].last_frame_us) > FRAME_TIMEOUT_US;
            let transfer_expired =
                now_us.saturating_sub(self.slots[slot_index].started_us) > total_timeout;
            if frame_expired || transfer_expired {
                if let Some(session) = self
                    .sessions
                    .iter()
                    .position(|session| session.used && usize::from(session.slot) == slot_index)
                {
                    self.abort_slot(session, slot_index);
                } else {
                    self.slots[slot_index].used = false;
                }
                self.stats.timeouts = self.stats.timeouts.saturating_add(1);
            }
        }
        for session in &mut self.sessions {
            if session.used
                && session.slot == NO_SLOT
                && now_us.saturating_sub(session.last_transfer_us) > TRANSFER_ID_TIMEOUT_US
            {
                *session = Session::EMPTY;
            }
        }
    }

    fn reject<T>(&mut self, error: RxError) -> Result<T, RxError> {
        self.stats.rejected = self.stats.rejected.saturating_add(1);
        Err(error)
    }
}

impl Default for RxTransport {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RxError {
    Frame,
    Id,
    WrongAddress,
    UnknownType,
    Tail,
    Duplicate,
    MissedStart,
    TransferId,
    Toggle,
    Capacity,
    PayloadTooLong,
    NonCanonical,
    Crc,
}

#[derive(Clone, Copy)]
struct Tail {
    start: bool,
    end: bool,
    toggle: bool,
    transfer_id: u8,
}

impl Tail {
    const fn parse(byte: u8) -> Self {
        Self {
            start: byte & 0x80 != 0,
            end: byte & 0x40 != 0,
            toggle: byte & 0x20 != 0,
            transfer_id: byte & 0x1f,
        }
    }
}

const fn next_transfer_id(transfer_id: u8) -> u8 {
    transfer_id.wrapping_add(1) & 0x1f
}

const fn previous_transfer_id(transfer_id: u8) -> u8 {
    transfer_id.wrapping_sub(1) & 0x1f
}

#[cfg(test)]
mod tests {
    use super::*;

    const RAW: TypeInfo = TypeInfo {
        signature: 0x217f_5c87_d7ec_951d,
        max_payload: 35,
    };

    fn raw_info(metadata: &Metadata) -> Option<TypeInfo> {
        matches!(metadata.kind, TransferKind::Message { data_type_id: 1030 }).then_some(RAW)
    }

    #[test]
    fn metadata_matches_reference_ids_and_rejects_node_zero() {
        let message = Metadata::message(8, 1030, 42, 0).unwrap();
        assert_eq!(message.id(), Ok(0x0804_062a));
        assert_eq!(Metadata::from_id(0x0804_062a, 31).unwrap().transfer_id, 31);
        let request = Metadata::service(16, 1, true, 42, 73, 0).unwrap();
        assert_eq!(request.id(), Ok(0x1001_c9aa));
        let response = Metadata::service(16, 1, false, 73, 42, 0).unwrap();
        assert_eq!(response.id(), Ok(0x1001_2ac9));
        assert_eq!(
            Metadata::from_id(0x0804_0600, 0),
            Err(MetadataError::AnonymousUnsupported)
        );
        assert_eq!(
            Metadata::from_id(0x1001_c900, 0),
            Err(MetadataError::AnonymousUnsupported)
        );
    }

    #[test]
    fn crc_and_fragmentation_match_reference_vectors() {
        assert_eq!(crc16(0xffff, b"123456789"), 0x29b1);
        let metadata = Metadata::message(8, 1030, 42, 0).unwrap();
        let payload = [0; 14];
        assert_eq!(transfer_crc(RAW.signature, &payload), 0x998d);
        let frames: [Frame; 3] = TxFrames::new(metadata, RAW, &payload)
            .unwrap()
            .collect::<std::vec::Vec<_>>()
            .try_into()
            .unwrap();
        assert_eq!(frames[0].data(), &[0x8d, 0x99, 0, 0, 0, 0, 0, 0x80]);
        assert_eq!(frames[1].data(), &[0, 0, 0, 0, 0, 0, 0, 0x20]);
        assert_eq!(frames[2].data(), &[0, 0, 0x40]);

        let empty: std::vec::Vec<_> = TxFrames::new(metadata, RAW, &[]).unwrap().collect();
        assert_eq!(empty.len(), 1);
        assert_eq!(empty[0].data(), &[0xc0]);
    }

    #[test]
    fn reassembly_checks_crc_toggle_duplicates_and_timeout() {
        let metadata = Metadata::message(8, 1030, 42, 30).unwrap();
        let payload = [0; 14];
        let frames: std::vec::Vec<_> = TxFrames::new(metadata, RAW, &payload).unwrap().collect();
        let mut receiver = RxTransport::new();
        assert!(receiver
            .accept(&frames[0], 0, 73, raw_info)
            .unwrap()
            .is_none());
        assert!(receiver
            .accept(&frames[1], 1, 73, raw_info)
            .unwrap()
            .is_none());
        let transfer = receiver
            .accept(&frames[2], 2, 73, raw_info)
            .unwrap()
            .unwrap();
        assert_eq!(transfer.payload.as_slice(), payload);

        assert_eq!(
            receiver.accept(&frames[0], 3, 73, raw_info),
            Err(RxError::Duplicate)
        );

        let next = Metadata::message(8, 1030, 42, 31).unwrap();
        let mut wrong_toggle = TxFrames::new(next, RAW, &payload).unwrap().next().unwrap();
        wrong_toggle.data[7] ^= 0x20;
        assert_eq!(
            receiver.accept(&wrong_toggle, 4, 73, raw_info),
            Err(RxError::Tail)
        );

        let next_frames: std::vec::Vec<_> = TxFrames::new(next, RAW, &payload).unwrap().collect();
        assert!(receiver
            .accept(&next_frames[0], 10, 73, raw_info)
            .unwrap()
            .is_none());
        assert_eq!(
            receiver.accept(&next_frames[1], 100_011, 73, raw_info),
            Err(RxError::MissedStart)
        );
        assert_eq!(receiver.stats().timeouts, 1);
    }

    #[test]
    fn bad_crc_and_missing_start_never_publish() {
        let metadata = Metadata::message(8, 1030, 42, 0).unwrap();
        let payload = [0; 14];
        let mut frames: std::vec::Vec<_> =
            TxFrames::new(metadata, RAW, &payload).unwrap().collect();
        frames[0].data[0] ^= 1;
        let mut receiver = RxTransport::new();
        for frame in &frames[..2] {
            assert!(receiver.accept(frame, 1, 73, raw_info).unwrap().is_none());
        }
        assert_eq!(
            receiver.accept(&frames[2], 2, 73, raw_info),
            Err(RxError::Crc)
        );

        let valid: std::vec::Vec<_> =
            TxFrames::new(Metadata::message(8, 1030, 43, 0).unwrap(), RAW, &payload)
                .unwrap()
                .collect();
        assert_eq!(
            receiver.accept(&valid[1], 3, 73, raw_info),
            Err(RxError::MissedStart)
        );
    }

    #[test]
    fn sessions_accept_transfer_id_wrap_and_interleaved_sources() {
        let mut receiver = RxTransport::new();
        for (time, transfer_id) in [30, 31, 0, 1].into_iter().enumerate() {
            let metadata = Metadata::message(8, 1030, 42, transfer_id).unwrap();
            let frame = TxFrames::new(metadata, RAW, &[0; 7])
                .unwrap()
                .next()
                .unwrap();
            let transfer = receiver
                .accept(&frame, time as u64, 73, raw_info)
                .unwrap()
                .unwrap();
            assert_eq!(transfer.metadata.transfer_id, transfer_id);
        }
        let duplicate = TxFrames::new(Metadata::message(8, 1030, 42, 1).unwrap(), RAW, &[0; 7])
            .unwrap()
            .next()
            .unwrap();
        assert_eq!(
            receiver.accept(&duplicate, 5, 73, raw_info),
            Err(RxError::Duplicate)
        );

        let payload = [0; 14];
        let first: std::vec::Vec<_> =
            TxFrames::new(Metadata::message(8, 1030, 50, 0).unwrap(), RAW, &payload)
                .unwrap()
                .collect();
        let second: std::vec::Vec<_> =
            TxFrames::new(Metadata::message(8, 1030, 51, 0).unwrap(), RAW, &payload)
                .unwrap()
                .collect();
        for index in 0..2 {
            assert!(receiver
                .accept(&first[index], 10 + index as u64, 73, raw_info)
                .unwrap()
                .is_none());
            assert!(receiver
                .accept(&second[index], 10 + index as u64, 73, raw_info)
                .unwrap()
                .is_none());
        }
        assert!(receiver
            .accept(&first[2], 20, 73, raw_info)
            .unwrap()
            .is_some());
        assert!(receiver
            .accept(&second[2], 20, 73, raw_info)
            .unwrap()
            .is_some());
    }

    #[test]
    fn service_addressing_and_reserved_message_slots_are_enforced() {
        fn all_info(_: &Metadata) -> Option<TypeInfo> {
            Some(RAW)
        }

        let request = Metadata::service(16, 1, true, 42, 73, 0).unwrap();
        let frame = TxFrames::new(request, RAW, &[]).unwrap().next().unwrap();
        let mut receiver = RxTransport::new();
        assert_eq!(
            receiver.accept(&frame, 0, 74, all_info),
            Err(RxError::WrongAddress)
        );
        assert!(receiver.accept(&frame, 1, 73, all_info).unwrap().is_some());

        let payload = [0; 14];
        for source in 1..=8 {
            let metadata = Metadata::service(16, 1, true, source, 73, 0).unwrap();
            let first = TxFrames::new(metadata, RAW, &payload)
                .unwrap()
                .next()
                .unwrap();
            assert!(receiver.accept(&first, 10, 73, all_info).unwrap().is_none());
        }
        let ninth = TxFrames::new(
            Metadata::service(16, 1, true, 9, 73, 0).unwrap(),
            RAW,
            &payload,
        )
        .unwrap()
        .next()
        .unwrap();
        assert_eq!(
            receiver.accept(&ninth, 10, 73, all_info),
            Err(RxError::Capacity)
        );

        let message = TxFrames::new(Metadata::message(8, 1030, 100, 0).unwrap(), RAW, &payload)
            .unwrap()
            .next()
            .unwrap();
        assert!(receiver
            .accept(&message, 10, 73, all_info)
            .unwrap()
            .is_none());
    }

    #[test]
    fn malformed_first_frames_do_not_consume_state_and_old_sot_keeps_current_transfer() {
        fn empty_service(metadata: &Metadata) -> Option<TypeInfo> {
            matches!(
                metadata.kind,
                TransferKind::Service {
                    service_id: 1,
                    request: true,
                    ..
                }
            )
            .then_some(TypeInfo {
                signature: 0xee46_8a81_21c4_6a9e,
                max_payload: 0,
            })
        }

        let mut receiver = RxTransport::new();
        for source in 1..=70 {
            let metadata = Metadata::service(16, 1, true, source, 73, 0).unwrap();
            let malformed =
                Frame::new(metadata.id().unwrap(), &[0, 0, 0, 0, 0, 0, 0, 0x80]).unwrap();
            assert_eq!(
                receiver.accept(&malformed, u64::from(source), 73, empty_service),
                Err(RxError::NonCanonical)
            );
        }
        assert_eq!(receiver.stats().capacity_errors, 0);

        let payload = [0; 14];
        let current: std::vec::Vec<_> =
            TxFrames::new(Metadata::message(8, 1030, 80, 5).unwrap(), RAW, &payload)
                .unwrap()
                .collect();
        let old = TxFrames::new(Metadata::message(8, 1030, 80, 4).unwrap(), RAW, &payload)
            .unwrap()
            .next()
            .unwrap();
        assert!(receiver
            .accept(&current[0], 100, 73, raw_info)
            .unwrap()
            .is_none());
        assert_eq!(
            receiver.accept(&old, 101, 73, raw_info),
            Err(RxError::Duplicate)
        );
        assert!(receiver
            .accept(&current[1], 102, 73, raw_info)
            .unwrap()
            .is_none());
        assert_eq!(
            receiver.accept(&current[0], 102, 73, raw_info),
            Err(RxError::Duplicate)
        );
        assert!(receiver
            .accept(&current[2], 103, 73, raw_info)
            .unwrap()
            .is_some());
    }

    #[test]
    fn service_sessions_cannot_consume_the_message_reserve() {
        fn all_info(_: &Metadata) -> Option<TypeInfo> {
            Some(RAW)
        }

        let mut receiver = RxTransport::new();
        for source in 1..=(RX_SESSION_COUNT - RESERVED_MESSAGE_SESSIONS) as u8 {
            let metadata = Metadata::service(16, 1, true, source, 73, 0).unwrap();
            let frame = TxFrames::new(metadata, RAW, &[]).unwrap().next().unwrap();
            assert!(receiver
                .accept(&frame, u64::from(source), 73, all_info)
                .unwrap()
                .is_some());
        }
        let overflow = TxFrames::new(Metadata::service(16, 1, true, 60, 73, 0).unwrap(), RAW, &[])
            .unwrap()
            .next()
            .unwrap();
        assert_eq!(
            receiver.accept(&overflow, 100, 73, all_info),
            Err(RxError::Capacity)
        );

        let message = TxFrames::new(Metadata::message(16, 1030, 100, 0).unwrap(), RAW, &[])
            .unwrap()
            .next()
            .unwrap();
        assert!(receiver
            .accept(&message, 101, 73, all_info)
            .unwrap()
            .is_some());
    }
}
