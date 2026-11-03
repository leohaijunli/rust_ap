//! Classic-CAN MAVCAN wire fields and bounded forwarding-session policy.
//!
//! Protocol references: MAVLink common.xml and pydronecan's mavcan driver;
//! pins and reference revisions are recorded in docs/mavcan.md.

pub const BUS_NUMBER: u8 = 2;
pub const FRAME_BUS: u8 = BUS_NUMBER - 1;
pub const BITRATE: u32 = 1_000_000;
pub const LEASE_US: u64 = 5_000_000;
pub const FRAME_MAX_AGE_US: u64 = 100_000;
pub const EXTENDED_FLAG: u32 = 1 << 31;
pub const RTR_FLAG: u32 = 1 << 30;
const ID_MASK: u32 = 0x1fff_ffff;
const FILTER_CAPACITY: usize = 32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Peer {
    pub system: u8,
    pub component: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    wire_id: u32,
    len: u8,
    data: [u8; 8],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    Id,
    Length,
}

impl Frame {
    pub fn from_wire(wire_id: u32, len: u8, data: [u8; 8]) -> Result<Self, FrameError> {
        if wire_id & !(ID_MASK | EXTENDED_FLAG | RTR_FLAG) != 0
            || (wire_id & EXTENDED_FLAG == 0 && wire_id & ID_MASK > 0x7ff)
        {
            return Err(FrameError::Id);
        }
        if len > 8 {
            return Err(FrameError::Length);
        }
        let mut frame = Self {
            wire_id,
            len,
            data: [0; 8],
        };
        if !frame.remote() {
            frame.data[..usize::from(len)].copy_from_slice(&data[..usize::from(len)]);
        }
        Ok(frame)
    }

    pub const fn wire_id(self) -> u32 {
        self.wire_id
    }

    pub const fn id(self) -> u32 {
        self.wire_id & ID_MASK
    }

    pub const fn extended(self) -> bool {
        self.wire_id & EXTENDED_FLAG != 0
    }

    pub const fn remote(self) -> bool {
        self.wire_id & RTR_FLAG != 0
    }

    pub const fn len(self) -> u8 {
        self.len
    }

    pub const fn is_empty(self) -> bool {
        self.len == 0
    }

    pub const fn data(self) -> [u8; 8] {
        self.data
    }

    /// STM32 FDCAN Classic TX element: identifier/RTR/IDE, then DLC without FD/BRS.
    pub const fn fdcan_tx_header(self) -> [u32; 2] {
        let id = if self.extended() {
            self.id() | (1 << 30)
        } else {
            self.id() << 18
        };
        [
            id | if self.remote() { 1 << 29 } else { 0 },
            (self.len as u32) << 16,
        ]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FilterOperation {
    Replace,
    Add,
    Remove,
}

#[derive(Clone, Copy)]
struct Filters {
    ids: [u16; FILTER_CAPACITY],
    len: usize,
}

impl Filters {
    const EMPTY: Self = Self {
        ids: [0; FILTER_CAPACITY],
        len: 0,
    };

    fn update(&mut self, operation: FilterOperation, ids: &[u16]) -> Result<(), SessionError> {
        // Commit only a complete update, including capacity validation.
        let mut next = if operation == FilterOperation::Replace {
            Self::EMPTY
        } else {
            *self
        };
        for &id in ids {
            let position = next.ids[..next.len].iter().position(|&value| value == id);
            if operation == FilterOperation::Remove {
                if let Some(position) = position {
                    next.len -= 1;
                    next.ids[position] = next.ids[next.len];
                }
            } else if position.is_none() {
                if next.len == FILTER_CAPACITY {
                    return Err(SessionError::FilterCapacity);
                }
                next.ids[next.len] = id;
                next.len += 1;
            }
        }
        *self = next;
        Ok(())
    }

    fn accepts(&self, frame: Frame) -> bool {
        // CAN_FILTER_MODIFY selects the two ID bytes used by DroneCAN messages.
        self.len == 0 || self.ids[..self.len].contains(&((frame.id() >> 8) as u16))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SessionError {
    Bus,
    Denied,
    Busy,
    Inactive,
    FilterCapacity,
}

pub struct Session {
    peer: Option<Peer>,
    deadline_us: u64,
    epoch: u32,
    filters: Filters,
}

impl Session {
    pub const fn new() -> Self {
        Self {
            peer: None,
            deadline_us: 0,
            epoch: 0,
            filters: Filters::EMPTY,
        }
    }

    pub fn request(
        &mut self,
        peer: Peer,
        bus: f32,
        now_us: u64,
        permitted: bool,
    ) -> Result<(), SessionError> {
        if !permitted || peer.system == 0 {
            if !permitted {
                self.stop();
            }
            return Err(SessionError::Denied);
        }
        if bus != 0.0 && bus != f32::from(BUS_NUMBER) {
            return Err(SessionError::Bus);
        }
        if self.active_peer(now_us).is_some_and(|owner| owner != peer) {
            return Err(SessionError::Busy);
        }
        if bus == 0.0 {
            self.stop();
            return Ok(());
        }
        if self.active_peer(now_us).is_none() {
            self.epoch = self.epoch.wrapping_add(1);
            self.filters = Filters::EMPTY;
        }
        self.peer = Some(peer);
        self.deadline_us = now_us.saturating_add(LEASE_US);
        Ok(())
    }

    pub fn stop(&mut self) {
        self.peer = None;
        self.deadline_us = 0;
        self.epoch = self.epoch.wrapping_add(1);
        self.filters = Filters::EMPTY;
    }

    pub fn expire(&mut self, now_us: u64) -> bool {
        if self.peer.is_some() && now_us >= self.deadline_us {
            self.stop();
            true
        } else {
            false
        }
    }

    pub fn active_peer(&self, now_us: u64) -> Option<Peer> {
        self.peer.filter(|_| now_us < self.deadline_us)
    }

    pub const fn epoch(&self) -> u32 {
        self.epoch
    }

    pub const fn remaining_us(&self, now_us: u64) -> u64 {
        self.deadline_us.saturating_sub(now_us)
    }

    pub const fn filter_count(&self) -> usize {
        self.filters.len
    }

    pub fn accepts_sender(&self, peer: Peer, frame_bus: u8, now_us: u64) -> bool {
        frame_bus == FRAME_BUS && self.active_peer(now_us) == Some(peer)
    }

    pub fn accepts_queued(&self, epoch: u32, timestamp_us: u64, now_us: u64) -> bool {
        self.epoch == epoch
            && self.active_peer(now_us).is_some()
            && now_us >= timestamp_us
            && now_us - timestamp_us <= FRAME_MAX_AGE_US
    }

    pub fn accepts_filter(&self, frame: Frame) -> bool {
        self.filters.accepts(frame)
    }

    pub fn update_filters(
        &mut self,
        peer: Peer,
        bus: u8,
        now_us: u64,
        operation: FilterOperation,
        ids: &[u16],
    ) -> Result<(), SessionError> {
        // pydronecan uses one-based bus numbers in filter requests too.
        if bus != BUS_NUMBER {
            return Err(SessionError::Bus);
        }
        if self.active_peer(now_us) != Some(peer) {
            return Err(SessionError::Inactive);
        }
        self.filters.update(operation, ids)
    }
}

impl Default for Session {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HOST: Peer = Peer {
        system: 250,
        component: 189,
    };

    #[test]
    fn classic_frames_preserve_id_flags_dlc_and_payload() {
        for (id, len) in [(0x7ff, 0), (0x7ff, 8), (EXTENDED_FLAG | ID_MASK, 8)] {
            let frame = Frame::from_wire(id, len, [0xab; 8]).unwrap();
            assert_eq!(frame.wire_id(), id);
            assert_eq!(frame.len(), len);
            assert_eq!(frame.extended(), id & EXTENDED_FLAG != 0);
            assert!(!frame.remote());
            assert_eq!(frame.data(), if len == 0 { [0; 8] } else { [0xab; 8] });
        }
        let remote = Frame::from_wire(EXTENDED_FLAG | RTR_FLAG | 0x123, 7, [0xff; 8]).unwrap();
        assert!(remote.remote());
        assert_eq!(remote.id(), 0x123);
        assert_eq!(remote.len(), 7);
        assert_eq!(remote.data(), [0; 8]);
        let short = Frame::from_wire(0x123, 3, [0xaa; 8]).unwrap();
        assert_eq!(short.data(), [0xaa, 0xaa, 0xaa, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn invalid_frames_are_rejected_not_truncated_or_replaced_with_id_zero() {
        for id in [0x800, 0x2000_0000, 0xa000_0001, 0xffff_ffff] {
            assert_eq!(Frame::from_wire(id, 8, [0; 8]), Err(FrameError::Id));
        }
        for len in [9, 64, 255] {
            assert_eq!(
                Frame::from_wire(EXTENDED_FLAG, len, [0; 8]),
                Err(FrameError::Length)
            );
        }
    }

    #[test]
    fn fdcan_mailbox_preserves_remote_dlc_and_classic_frame_format() {
        let standard = Frame::from_wire(0x7ff, 8, [0; 8]).unwrap();
        assert_eq!(standard.fdcan_tx_header(), [0x1ffc_0000, 0x0008_0000]);
        let extended = Frame::from_wire(EXTENDED_FLAG | ID_MASK, 8, [0; 8]).unwrap();
        assert_eq!(extended.fdcan_tx_header(), [0x5fff_ffff, 0x0008_0000]);
        for len in 0..=8 {
            let standard = Frame::from_wire(RTR_FLAG | 0x123, len, [0; 8]).unwrap();
            let extended = Frame::from_wire(EXTENDED_FLAG | RTR_FLAG | 0x123, len, [0; 8]).unwrap();
            let dlc = u32::from(len) << 16;
            assert_eq!(standard.fdcan_tx_header(), [0x248c_0000, dlc]);
            assert_eq!(extended.fdcan_tx_header(), [0x6000_0123, dlc]);
        }
    }

    #[test]
    fn forwarding_lease_handles_renewal_stop_disconnect_and_reopen() {
        let mut session = Session::new();
        assert_eq!(session.active_peer(0), None);
        session.request(HOST, 2.0, 100, true).unwrap();
        let first_epoch = session.epoch();
        assert!(session.accepts_sender(HOST, 1, 101));
        assert!(!session.accepts_sender(HOST, 0, 101));
        session.request(HOST, 2.0, 1_000_100, true).unwrap();
        assert_eq!(session.epoch(), first_epoch);
        assert_eq!(session.remaining_us(1_000_100), LEASE_US);
        assert!(session.accepts_queued(first_epoch, 1_000_100, 1_000_101));
        assert!(!session.accepts_queued(first_epoch, 1_000_100, 1_100_101));
        assert!(!session.accepts_queued(first_epoch, 1_000_100, 1_000_099));
        assert!(session.expire(1_000_100 + LEASE_US));
        session.request(HOST, 2.0, 7_000_000, true).unwrap();
        assert!(!session.accepts_queued(first_epoch, 7_000_000, 7_000_001));
        session.request(HOST, 0.0, 7_000_001, true).unwrap();
        assert_eq!(session.active_peer(7_000_002), None);
        session.request(HOST, 2.0, 8_000_000, true).unwrap();
        let connected_epoch = session.epoch();
        session.stop(); // USB reset/disconnect
        assert!(!session.accepts_queued(connected_epoch, 8_000_000, 8_000_001));
    }

    #[test]
    fn wrong_bus_peer_and_output_state_cannot_open_or_take_over_forwarding() {
        let mut session = Session::new();
        for bus in [1.0, 3.0, -1.0, 2.5, f32::NAN, f32::INFINITY] {
            assert_eq!(session.request(HOST, bus, 0, true), Err(SessionError::Bus));
        }
        session.request(HOST, 2.0, 1, true).unwrap();
        let other = Peer {
            system: 251,
            ..HOST
        };
        for bus in [0.0, 2.0] {
            assert_eq!(
                session.request(other, bus, 2, true),
                Err(SessionError::Busy)
            );
        }
        assert!(!session.accepts_sender(other, FRAME_BUS, 2));
        assert!(!session.accepts_sender(
            Peer {
                component: 190,
                ..HOST
            },
            FRAME_BUS,
            2
        ));
        assert_eq!(
            session.update_filters(HOST, FRAME_BUS, 2, FilterOperation::Replace, &[341]),
            Err(SessionError::Bus)
        );
        assert_eq!(
            session.update_filters(other, BUS_NUMBER, 2, FilterOperation::Replace, &[341]),
            Err(SessionError::Inactive)
        );
        assert_eq!(
            session.request(HOST, 2.0, 3, false),
            Err(SessionError::Denied)
        );
        assert_eq!(session.active_peer(3), None);
    }

    #[test]
    fn filters_are_per_session_bounded_and_match_dronecan_message_ids() {
        let mut session = Session::new();
        session.request(HOST, 2.0, 100, true).unwrap();
        let status = Frame::from_wire(EXTENDED_FLAG | (341 << 8) | 42, 7, [0; 8]).unwrap();
        let gps = Frame::from_wire(EXTENDED_FLAG | (1063 << 8) | 43, 8, [0; 8]).unwrap();
        session
            .update_filters(HOST, 2, 100, FilterOperation::Replace, &[341])
            .unwrap();
        assert!(session.accepts_filter(status));
        assert!(!session.accepts_filter(gps));
        session
            .update_filters(HOST, 2, 100, FilterOperation::Add, &[1063, 1063])
            .unwrap();
        assert_eq!(session.filter_count(), 2);
        assert!(session.accepts_filter(gps));
        session
            .update_filters(HOST, 2, 100, FilterOperation::Remove, &[341])
            .unwrap();
        assert!(!session.accepts_filter(status));
        let ids = core::array::from_fn::<_, FILTER_CAPACITY, _>(|index| index as u16);
        session
            .update_filters(HOST, 2, 100, FilterOperation::Replace, &ids)
            .unwrap();
        assert_eq!(
            session.update_filters(HOST, 2, 100, FilterOperation::Add, &[341]),
            Err(SessionError::FilterCapacity)
        );
        assert_eq!(session.filter_count(), FILTER_CAPACITY);
        assert!(!session.accepts_filter(status));
        let too_many = core::array::from_fn::<_, { FILTER_CAPACITY + 1 }, _>(|index| index as u16);
        assert_eq!(
            session.update_filters(HOST, 2, 100, FilterOperation::Replace, &too_many),
            Err(SessionError::FilterCapacity)
        );
        assert_eq!(session.filter_count(), FILTER_CAPACITY);
        session.request(HOST, 2.0, 200, true).unwrap();
        assert_eq!(session.filter_count(), FILTER_CAPACITY);
        session.request(HOST, 2.0, 200 + LEASE_US, true).unwrap();
        assert_eq!(session.filter_count(), 0);
        assert!(session.accepts_filter(status));
        assert!(session.accepts_filter(gps));
        session
            .update_filters(HOST, 2, 200 + LEASE_US, FilterOperation::Replace, &[341])
            .unwrap();
        session
            .update_filters(HOST, 2, 200 + LEASE_US, FilterOperation::Remove, &[341])
            .unwrap();
        assert_eq!(session.filter_count(), 0);
        assert!(session.accepts_filter(gps));
    }
}
