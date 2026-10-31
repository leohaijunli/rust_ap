//! MAVLink telemetry over USB CDC, with a small diagnostic line console.

use core::cell::RefCell;

use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_stm32::gpio::Output;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender};
use embassy_usb::driver::{Driver, EndpointError};
use mavlink::common::*;
use mavlink::{MAVLinkV1MessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, Message};

use crate::eth_led::Lan8742;
use crate::usb_console::{self, Command, CommandLine, Response};

const SYSTEM_ID: u8 = 1;
const COMPONENT_ID: u8 = 1;
const PARAMETERS: [(&str, f32); 2] = [("MAV_SYS_ID", SYSTEM_ID as f32), ("LINK_LED_HZ", 1.0)];

// Fixed-size buffers are intentional: this no_std firmware has no allocator.
#[allow(clippy::large_enum_variant)]
enum Outbound {
    Message(MavMessage),
    Console(Response),
    Bootloader { console_ack: bool },
}

type Replies = Channel<NoopRawMutex, Outbound, 4>;

pub type UsbSession = Watch<NoopRawMutex, u32, 2>;

/// Start a fresh transport session whenever the host resets or reconfigures USB.
pub struct UsbSessionEvents<'a> {
    session: &'a UsbSession,
    generation: u32,
}

impl<'a> UsbSessionEvents<'a> {
    pub fn new(session: &'a UsbSession) -> Self {
        Self {
            session,
            generation: 0,
        }
    }

    fn restart(&mut self) {
        self.generation = self.generation.wrapping_add(1);
        self.session.sender().send(self.generation);
    }
}

impl embassy_usb::Handler for UsbSessionEvents<'_> {
    fn reset(&mut self) {
        self.restart();
    }

    fn configured(&mut self, _configured: bool) {
        self.restart();
    }

    fn enabled(&mut self, enabled: bool) {
        if !enabled {
            self.restart();
        }
    }
}

pub async fn run<'d, D: Driver<'d>>(
    class: CdcAcmClass<'d, D>,
    phy: &RefCell<Lan8742<'_>>,
    eth_power: &Output<'_>,
    session: &UsbSession,
) -> ! {
    let (mut sender, mut receiver) = class.split();
    let replies = Replies::new();
    let mut tx_reset = session.receiver().unwrap();
    let mut rx_reset = session.receiver().unwrap();
    tx_reset.get().await;
    rx_reset.get().await;
    join(
        async {
            loop {
                // Poll reset first: never begin a fresh write for an old session.
                select(tx_reset.changed(), transmit(&mut sender, &replies)).await;
                replies.clear();
            }
        },
        async {
            loop {
                select(
                    rx_reset.changed(),
                    receive(&mut receiver, &replies, phy, eth_power),
                )
                .await;
            }
        },
    )
    .await;
    unreachable!()
}

async fn transmit<'d, D: Driver<'d>>(sender: &mut Sender<'d, D>, replies: &Replies) -> ! {
    let mut sequence = 0u8;
    'connection: loop {
        sender.wait_connection().await;
        let mut ticks = 0u32;
        for message in [heartbeat(), status_text(), autopilot_version()] {
            if send_message(sender, &mut sequence, &message).await.is_err() {
                continue 'connection;
            }
        }
        let mut next_heartbeat = Instant::now() + Duration::from_secs(1);

        loop {
            let result = match select(Timer::at(next_heartbeat), replies.receive()).await {
                Either::First(()) => {
                    ticks = ticks.wrapping_add(1);
                    let mut result = send_message(sender, &mut sequence, &heartbeat()).await;
                    if result.is_ok() {
                        result = send_message(sender, &mut sequence, &system_status()).await;
                    }
                    if result.is_ok() {
                        // Generate boot time after earlier USB writes complete,
                        // so opening the port cannot release an old timestamp.
                        result = send_message(sender, &mut sequence, &system_time()).await;
                    }
                    if result.is_ok() && ticks.is_multiple_of(5) {
                        result = send_message(sender, &mut sequence, &status_text()).await;
                    }
                    // USB writes can wait while the host's serial port is closed.
                    // Skip missed periods rather than bursting old heartbeats.
                    next_heartbeat = Instant::now() + Duration::from_secs(1);
                    result
                }
                Either::Second(Outbound::Message(message)) => {
                    send_message(sender, &mut sequence, &message).await
                }
                Either::Second(Outbound::Console(response)) => {
                    write_transfer(sender, response.as_bytes()).await
                }
                Either::Second(Outbound::Bootloader { console_ack }) => {
                    if console_ack {
                        let _ = write_transfer(sender, b"OK bootloader\n").await;
                    }
                    Timer::after_millis(100).await;
                    usb_console::reboot_to_bootloader();
                }
            };
            if result.is_err() {
                break;
            }
        }
    }
}

async fn send_message<'d, D: Driver<'d>>(
    sender: &mut Sender<'d, D>,
    sequence: &mut u8,
    message: &MavMessage,
) -> Result<(), EndpointError> {
    let header = MavHeader {
        system_id: SYSTEM_ID,
        component_id: COMPONENT_ID,
        sequence: *sequence,
    };
    *sequence = sequence.wrapping_add(1);
    let mut frame = MAVLinkV2MessageRaw::new();
    frame.serialize_message(header, message);
    write_transfer(sender, frame.raw_bytes()).await
}

async fn write_transfer<'d, D: Driver<'d>>(
    sender: &mut Sender<'d, D>,
    data: &[u8],
) -> Result<(), EndpointError> {
    let max_packet_size = usize::from(sender.max_packet_size());
    for packet in data.chunks(max_packet_size) {
        sender.write_packet(packet).await?;
    }
    if data.len().is_multiple_of(max_packet_size) {
        sender.write_packet(&[]).await?;
    }
    Ok(())
}

async fn receive<'d, D: Driver<'d>>(
    receiver: &mut Receiver<'d, D>,
    replies: &Replies,
    phy: &RefCell<Lan8742<'_>>,
    eth_power: &Output<'_>,
) -> ! {
    let mut packet = [0u8; 64];
    loop {
        receiver.wait_connection().await;
        let mut parser = FrameParser::new();
        let mut command_line = CommandLine::new();
        let mut last_byte = Instant::now();
        while let Ok(n) = receiver.read_packet(&mut packet).await {
            let now = Instant::now();
            if parser.receiving() && (now - last_byte).as_millis() > 1000 {
                // A host that closed mid-frame must not block later requests.
                parser = FrameParser::new();
                command_line = CommandLine::new();
            }
            last_byte = now;
            for &byte in &packet[..n] {
                if parser.receiving() || matches!(byte, 0xfe | 0xfd) {
                    command_line = CommandLine::new();
                    if let Some((header, message)) = parser.push(byte) {
                        handle_message(header, message, replies).await;
                    }
                } else {
                    match command_line.push(byte) {
                        Some(Command::Status) => {
                            replies
                                .send(Outbound::Console(usb_console::status(phy, eth_power)))
                                .await;
                        }
                        Some(Command::Bootloader) => {
                            replies
                                .send(Outbound::Bootloader { console_ack: true })
                                .await;
                        }
                        None => {}
                    }
                }
            }
        }
    }
}

// Keep PING and command 520 for ground stations using the older discovery flow.
#[allow(deprecated)]
async fn handle_message(header: MavHeader, message: MavMessage, replies: &Replies) {
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
            for index in 0..PARAMETERS.len() {
                replies.send(Outbound::Message(parameter(index))).await;
            }
        }
        MavMessage::PARAM_REQUEST_READ(data) => {
            let index = if data.param_index >= 0 {
                Some(data.param_index as usize)
            } else {
                PARAMETERS
                    .iter()
                    .position(|(name, _)| data.param_id.to_str().ok() == Some(*name))
            };
            if let Some(index) = index.filter(|index| *index < PARAMETERS.len()) {
                replies.send(Outbound::Message(parameter(index))).await;
            }
        }
        MavMessage::PARAM_SET(data) => {
            // These are read-only bring-up parameters. Report the actual value.
            if let Some(index) = PARAMETERS
                .iter()
                .position(|(name, _)| data.param_id.to_str().ok() == Some(*name))
            {
                replies.send(Outbound::Message(parameter(index))).await;
            }
        }
        MavMessage::PING(data) if data.target_system == 0 => {
            replies
                .send(Outbound::Message(MavMessage::PING(PING_DATA {
                    time_usec: data.time_usec,
                    seq: data.seq,
                    target_system: header.system_id,
                    target_component: header.component_id,
                })))
                .await;
        }
        MavMessage::TIMESYNC(data) if data.tc1 == 0 => {
            replies
                .send(Outbound::Message(MavMessage::TIMESYNC(TIMESYNC_DATA {
                    tc1: (Instant::now().as_micros() * 1000) as i64,
                    ts1: data.ts1,
                    ..Default::default()
                })))
                .await;
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
                replies.send(Outbound::Message(response)).await;
            }
            replies
                .send(Outbound::Message(MavMessage::COMMAND_ACK(
                    COMMAND_ACK_DATA {
                        command: data.command,
                        result: if supported {
                            MavResult::MAV_RESULT_ACCEPTED
                        } else {
                            MavResult::MAV_RESULT_UNSUPPORTED
                        },
                        target_system: header.system_id,
                        target_component: header.component_id,
                        ..Default::default()
                    },
                )))
                .await;
            if reboot {
                replies
                    .send(Outbound::Bootloader { console_ack: false })
                    .await;
            }
        }
        _ => {}
    }
}

fn heartbeat() -> MavMessage {
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GENERIC,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    })
}

fn status_text() -> MavMessage {
    MavMessage::STATUSTEXT(STATUSTEXT_DATA {
        severity: MavSeverity::MAV_SEVERITY_INFO,
        text: concat!(
            "Rust FMUv6X v",
            env!("CARGO_PKG_VERSION"),
            " running; USB MAVLink OK"
        )
        .into(),
        ..Default::default()
    })
}

fn system_status() -> MavMessage {
    MavMessage::SYS_STATUS(SYS_STATUS_DATA {
        voltage_battery: u16::MAX,
        current_battery: -1,
        battery_remaining: -1,
        ..Default::default()
    })
}

fn system_time() -> MavMessage {
    MavMessage::SYSTEM_TIME(SYSTEM_TIME_DATA {
        time_unix_usec: 0,
        time_boot_ms: Instant::now().as_millis() as u32,
    })
}

fn autopilot_version() -> MavMessage {
    let uid = embassy_stm32::uid::uid();
    let mut uid2 = [0u8; 18];
    uid2[..uid.len()].copy_from_slice(&uid);
    MavMessage::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
        capabilities: MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2
            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST,
        flight_sw_version: 1 << 16, // 0.1.0 development build.
        flight_custom_version: *b"RUSTFMU1",
        vendor_id: 0x1209,
        product_id: 0x0001,
        uid: u64::from_le_bytes(uid[..8].try_into().unwrap()),
        uid2,
        ..Default::default()
    })
}

fn parameter(index: usize) -> MavMessage {
    let (name, value) = PARAMETERS[index];
    MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
        param_id: name.into(),
        param_value: value,
        param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
        param_count: PARAMETERS.len() as u16,
        param_index: index as u16,
    })
}

/// Bounded packet assembly; USB packet boundaries do not delimit MAVLink frames.
struct FrameParser {
    bytes: [u8; mavlink::MAX_FRAME_SIZE],
    len: usize,
    expected: usize,
}

impl FrameParser {
    fn new() -> Self {
        Self {
            bytes: [0; mavlink::MAX_FRAME_SIZE],
            len: 0,
            expected: 0,
        }
    }

    fn receiving(&self) -> bool {
        self.len != 0
    }

    fn push(&mut self, byte: u8) -> Option<(MavHeader, MavMessage)> {
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

    fn parse(&self) -> Option<(MavHeader, MavMessage)> {
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
            .map(|message| (header, message))
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
                .map(|message| (header, message))
        }
    }
}
