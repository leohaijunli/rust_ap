//! Transport-agnostic MAVLink message logic shared by the USB CDC and TELEM1 links.

use mavlink::common::*;
use mavlink::{MAVLinkV1MessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, Message};

use v6x_rust_led::parameters::{Id, COUNT, REGISTRY};

use crate::parameter_manager::ParameterManager;

pub const SYSTEM_ID: u8 = Id::MavSysId.definition().default as u8;
pub const COMPONENT_ID: u8 = 1;

/// Decoded inbound frame plus the header it arrived with.
pub struct Decoded {
    pub header: MavHeader,
    pub message: MavMessage,
}

/// Replies produced for one inbound request, plus whether a reboot is pending.
pub struct Reply {
    messages: [Option<MavMessage>; 4],
    count: usize,
    pub reboot: bool,
}

impl Reply {
    pub const EMPTY: Self = Self {
        messages: [const { None }; 4],
        count: 0,
        reboot: false,
    };

    pub fn push(&mut self, message: MavMessage) {
        if self.count < self.messages.len() {
            self.messages[self.count] = Some(message);
            self.count += 1;
        }
    }

    /// Drain the produced replies, advancing `count`.
    pub fn drain(&mut self) -> impl Iterator<Item = MavMessage> + '_ {
        self.messages[..self.count]
            .iter_mut()
            .filter_map(Option::take)
    }
}

/// Handle one inbound message, filling `reply` with the messages to send back.
// Keep PING and command 520 for ground stations using the older discovery flow.
#[allow(deprecated)]
pub async fn handle(
    header: MavHeader,
    message: MavMessage,
    parameters: &ParameterManager<'_>,
    reply: &mut Reply,
) {
    if message
        .target_system_id()
        .is_some_and(|id| id != 0 && id != SYSTEM_ID)
        || message
            .target_component_id()
            .is_some_and(|id| id != 0 && id != COMPONENT_ID)
    {
        return;
    }

    match message {
        MavMessage::PARAM_REQUEST_LIST(_) => {
            for entry in REGISTRY {
                reply.push(parameter(parameters, entry.id));
            }
        }
        MavMessage::PARAM_REQUEST_READ(data) => {
            let id = if data.param_index >= 0 {
                Id::from_index(data.param_index as usize)
            } else {
                data.param_id.to_str().ok().and_then(Id::from_name)
            };
            if let Some(id) = id {
                reply.push(parameter(parameters, id));
            }
        }
        MavMessage::PARAM_SET(data) => {
            if let Some(id) = data.param_id.to_str().ok().and_then(Id::from_name) {
                let error = if data.param_type == MavParamType::MAV_PARAM_TYPE_REAL32 {
                    parameters
                        .set(id, data.param_value)
                        .err()
                        .map(|error| error.message())
                } else {
                    Some("PARAM_SET: expected REAL32")
                };
                reply.push(parameter(parameters, id));
                if let Some(error) = error {
                    reply.push(parameter_warning(error));
                }
            }
        }
        MavMessage::PING(data) if data.target_system == 0 => {
            reply.push(MavMessage::PING(PING_DATA {
                time_usec: data.time_usec,
                seq: data.seq,
                target_system: header.system_id,
                target_component: header.component_id,
            }));
        }
        MavMessage::TIMESYNC(data) if data.tc1 == 0 => {
            reply.push(MavMessage::TIMESYNC(TIMESYNC_DATA {
                tc1: (embassy_time::Instant::now().as_micros() * 1000) as i64,
                ts1: data.ts1,
                ..Default::default()
            }));
        }
        MavMessage::COMMAND_LONG(data) if data.command == MavCmd::MAV_CMD_PREFLIGHT_STORAGE => {
            let result = if data.param1 == 1.0 {
                parameters.save(crate::OUTPUT_STATE).await.map(|_| ())
            } else if data.param1 == 0.0 {
                parameters.load().await.map(|_| ())
            } else {
                reply.push(command_ack(
                    &header,
                    data.command,
                    MavResult::MAV_RESULT_UNSUPPORTED,
                ));
                return;
            };
            let ack = if result.is_ok() {
                MavResult::MAV_RESULT_ACCEPTED
            } else {
                MavResult::MAV_RESULT_FAILED
            };
            reply.push(command_ack(&header, data.command, ack));
            if result.is_err() {
                reply.push(parameter_warning("Parameter storage failed; see !params"));
            }
        }
        MavMessage::COMMAND_LONG(data) => {
            let mut reboot = false;
            let response = match data.command {
                MavCmd::MAV_CMD_REQUEST_MESSAGE => match data.param1 as u32 {
                    0 => Some(heartbeat()),
                    1 => Some(system_status()),
                    2 => Some(system_time()),
                    148 => Some(autopilot_version()),
                    253 => Some(status_text()),
                    _ => None,
                },
                MavCmd::MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES => Some(autopilot_version()),
                MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN if data.param1 == 3.0 => {
                    reboot = true;
                    None
                }
                _ => None,
            };
            let supported = response.is_some() || reboot;
            if let Some(response) = response {
                reply.push(response);
            }
            reply.push(command_ack(
                &header,
                data.command,
                if supported {
                    MavResult::MAV_RESULT_ACCEPTED
                } else {
                    MavResult::MAV_RESULT_UNSUPPORTED
                },
            ));
            if reboot {
                reply.reboot = true;
            }
        }
        _ => {}
    }
}

pub fn heartbeat() -> MavMessage {
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GENERIC,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: if crate::OUTPUT_STATE.armed {
            MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED
        } else {
            MavModeFlag::empty()
        },
        system_status: MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    })
}

pub fn status_text() -> MavMessage {
    MavMessage::STATUSTEXT(STATUSTEXT_DATA {
        severity: MavSeverity::MAV_SEVERITY_INFO,
        text: concat!(
            "Rust FMUv6X v",
            env!("CARGO_PKG_VERSION"),
            " running; MAVLink OK"
        )
        .into(),
        ..Default::default()
    })
}

pub fn system_status() -> MavMessage {
    MavMessage::SYS_STATUS(SYS_STATUS_DATA {
        // The explicit validity/staleness flag is available in !metrics. Before
        // the first valid window retain the baseline's zero load field.
        load: crate::diagnostics::cpu_load().unwrap_or(0),
        voltage_battery: u16::MAX,
        current_battery: -1,
        battery_remaining: -1,
        ..Default::default()
    })
}

pub fn system_time() -> MavMessage {
    MavMessage::SYSTEM_TIME(SYSTEM_TIME_DATA {
        time_unix_usec: 0,
        time_boot_ms: embassy_time::Instant::now().as_millis() as u32,
    })
}

pub fn autopilot_version() -> MavMessage {
    let uid = embassy_stm32::uid::uid();
    let mut uid2 = [0u8; 18];
    uid2[..uid.len()].copy_from_slice(&uid);
    MavMessage::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
        capabilities: MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2
            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST,
        flight_sw_version: v6x_rust_led::build_info::FLIGHT_SW_VERSION,
        flight_custom_version: v6x_rust_led::build_info::SOURCE_ID.to_le_bytes(),
        vendor_id: 0x1209,
        product_id: 0x0001,
        uid: u64::from_le_bytes(uid[..8].try_into().unwrap()),
        uid2,
        ..Default::default()
    })
}

pub fn parameter(parameters: &ParameterManager<'_>, id: Id) -> MavMessage {
    MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
        param_id: id.definition().name.into(),
        param_value: parameters.get(id),
        param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
        param_count: COUNT as u16,
        param_index: id as u16,
    })
}

pub fn parameter_warning(text: &'static str) -> MavMessage {
    MavMessage::STATUSTEXT(STATUSTEXT_DATA {
        severity: MavSeverity::MAV_SEVERITY_WARNING,
        text: text.into(),
        ..Default::default()
    })
}

pub fn command_ack(header: &MavHeader, command: MavCmd, result: MavResult) -> MavMessage {
    MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
        command,
        result,
        target_system: header.system_id,
        target_component: header.component_id,
        ..Default::default()
    })
}

/// Encode a message into a complete MAVLink v2 frame, advancing the sequence.
pub fn encode(
    sequence: &mut u8,
    message: &MavMessage,
    out: &mut [u8; mavlink::MAX_FRAME_SIZE],
) -> usize {
    let header = MavHeader {
        system_id: SYSTEM_ID,
        component_id: COMPONENT_ID,
        sequence: *sequence,
    };
    *sequence = sequence.wrapping_add(1);
    let mut frame = MAVLinkV2MessageRaw::new();
    frame.serialize_message(header, message);
    let bytes = frame.raw_bytes();
    out[..bytes.len()].copy_from_slice(bytes);
    bytes.len()
}

/// Bounded packet assembly; transport boundaries do not delimit MAVLink frames.
pub struct FrameParser {
    bytes: [u8; mavlink::MAX_FRAME_SIZE],
    len: usize,
    expected: usize,
}

impl FrameParser {
    pub fn new() -> Self {
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
            let header = MavHeader {
                sequence: raw.sequence(),
                system_id: raw.system_id(),
                component_id: raw.component_id(),
            };
            MavMessage::parse(
                MavlinkVersion::V1,
                u32::from(raw.message_id()),
                raw.payload(),
            )
            .ok()
            .map(|message| Decoded { header, message })
        } else {
            let mut raw = MAVLinkV2MessageRaw::new();
            raw.as_mut_slice()[..self.len].copy_from_slice(&self.bytes[..self.len]);
            if raw.incompatibility_flags() != 0 || !raw.has_valid_crc::<MavMessage>() {
                return None;
            }
            let header = MavHeader {
                sequence: raw.sequence(),
                system_id: raw.system_id(),
                component_id: raw.component_id(),
            };
            MavMessage::parse(MavlinkVersion::V2, raw.message_id(), raw.payload())
                .ok()
                .map(|message| Decoded { header, message })
        }
    }
}

impl Default for FrameParser {
    fn default() -> Self {
        Self::new()
    }
}
