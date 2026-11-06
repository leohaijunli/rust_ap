//! Bounded framing and target filtering shared by all byte transports.

use crate::parameters::{Id, COUNT};
use crate::telemetry::MAIN_COMPONENT;
use mavlink::{
    common::MavMessage, MAVLinkV1MessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion,
    Message,
};

pub struct ParamStream {
    next: usize,
}

impl Default for ParamStream {
    fn default() -> Self {
        Self { next: COUNT }
    }
}

impl ParamStream {
    pub fn start(&mut self) {
        self.next = 0;
    }
    pub fn peek(&self) -> Option<Id> {
        Id::from_index(self.next)
    }
    pub fn advance(&mut self) {
        self.next = (self.next + 1).min(COUNT);
    }
}

pub struct Decoded {
    pub header: MavHeader,
    pub message: MavMessage,
}

pub fn addressed_to_vehicle(message: &MavMessage, system: u8) -> bool {
    message
        .target_system_id()
        .is_none_or(|id| id == 0 || id == system)
        && message
            .target_component_id()
            .is_none_or(|id| matches!(id, 0 | MAIN_COMPONENT))
}

pub struct FrameParser {
    bytes: [u8; mavlink::MAX_FRAME_SIZE],
    len: usize,
    expected: usize,
}

impl Default for FrameParser {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameParser {
    pub const fn new() -> Self {
        Self {
            bytes: [0; mavlink::MAX_FRAME_SIZE],
            len: 0,
            expected: 0,
        }
    }

    pub fn receiving(&self) -> bool {
        self.len != 0
    }

    pub fn push(&mut self, byte: u8) -> Option<Decoded> {
        if self.len == 0 && !matches!(byte, 0xfe | 0xfd) {
            return None;
        }
        if self.len == self.bytes.len() {
            self.len = 0;
            self.expected = 0;
            return None;
        }
        self.bytes[self.len] = byte;
        self.len += 1;
        if self.len == 2 {
            self.expected = usize::from(byte) + if self.bytes[0] == 0xfe { 8 } else { 12 };
        }
        if self.len == 3 && self.bytes[0] == 0xfd && byte & 1 != 0 {
            self.expected += 13;
        }
        if self.expected == 0 || self.len < self.expected {
            return None;
        }
        let result = self.parse();
        self.len = 0;
        self.expected = 0;
        result
    }

    fn parse(&self) -> Option<Decoded> {
        if self.bytes[0] == 0xfe {
            let mut raw = MAVLinkV1MessageRaw::new();
            raw.as_mut_slice()[..self.len].copy_from_slice(&self.bytes[..self.len]);
            if !raw.has_valid_crc::<MavMessage>() {
                return None;
            }
            Some(Decoded {
                header: MavHeader {
                    sequence: raw.sequence(),
                    system_id: raw.system_id(),
                    component_id: raw.component_id(),
                },
                message: MavMessage::parse(
                    MavlinkVersion::V1,
                    u32::from(raw.message_id()),
                    raw.payload(),
                )
                .ok()?,
            })
        } else {
            let mut raw = MAVLinkV2MessageRaw::new();
            raw.as_mut_slice()[..self.len].copy_from_slice(&self.bytes[..self.len]);
            // Accept and consume the standard signing trailer. CRC is checked;
            // signature authentication is not advertised by this firmware.
            if raw.incompatibility_flags() & !1 != 0 || !raw.has_valid_crc::<MavMessage>() {
                return None;
            }
            Some(Decoded {
                header: MavHeader {
                    sequence: raw.sequence(),
                    system_id: raw.system_id(),
                    component_id: raw.component_id(),
                },
                message: MavMessage::parse(MavlinkVersion::V2, raw.message_id(), raw.payload())
                    .ok()?,
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::common::*;

    #[test]
    fn fragmented_v1_v2_and_signed_frames_preserve_headers_and_recover_after_bad_crc() {
        let header = MavHeader {
            system_id: 255,
            component_id: 190,
            sequence: 9,
        };
        let message = MavMessage::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA {
            target_system: 1,
            target_component: 1,
        });
        let mut v1 = MAVLinkV1MessageRaw::new();
        v1.serialize_message(header, &message);
        let mut v2 = MAVLinkV2MessageRaw::new();
        v2.serialize_message(header, &message);
        let mut signed = MAVLinkV2MessageRaw::new();
        signed.serialize_message_for_signing(header, &message);
        let mut parser = FrameParser::new();
        for frame in [v1.raw_bytes(), v2.raw_bytes(), signed.raw_bytes()] {
            for &byte in &frame[..frame.len() - 1] {
                assert!(parser.push(byte).is_none());
            }
            let decoded = parser.push(frame[frame.len() - 1]).unwrap();
            assert_eq!(decoded.header, header);
            assert_eq!(decoded.message, message);
        }
        let mut corrupt = v2.raw_bytes().to_vec();
        *corrupt.last_mut().unwrap() ^= 1;
        for byte in corrupt {
            assert!(parser.push(byte).is_none());
        }
        assert!(v2
            .raw_bytes()
            .iter()
            .filter_map(|&byte| parser.push(byte))
            .next()
            .is_some());
    }

    #[test]
    fn all_handlers_share_target_filtering() {
        let mut data = COMMAND_LONG_DATA {
            target_system: 2,
            target_component: 1,
            ..Default::default()
        };
        assert!(!addressed_to_vehicle(
            &MavMessage::COMMAND_LONG(data.clone()),
            1
        ));
        data.target_system = 1;
        data.target_component = 99;
        assert!(!addressed_to_vehicle(
            &MavMessage::COMMAND_LONG(data.clone()),
            1
        ));
        data.target_component = 197;
        assert!(!addressed_to_vehicle(
            &MavMessage::COMMAND_LONG(data.clone()),
            1
        ));
        data.target_component = 1;
        assert!(addressed_to_vehicle(&MavMessage::COMMAND_LONG(data), 1));
    }

    #[test]
    fn simultaneous_parameter_lists_have_independent_cursors_and_reset_cleanly() {
        let mut usb = ParamStream::default();
        let mut uart = ParamStream::default();
        usb.start();
        uart.start();
        usb.advance();
        assert_eq!(usb.peek(), Id::from_index(1));
        assert_eq!(uart.peek(), Id::from_index(0));
        for _ in 0..COUNT {
            uart.advance();
        }
        assert_eq!(uart.peek(), None);
        assert_eq!(ParamStream::default().peek(), None);
    }
}
