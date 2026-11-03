//! Transport-agnostic MAVLink message logic shared by the USB CDC and TELEM1 links.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use mavlink::common::*;
use mavlink::{MAVLinkV1MessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, Message};

use v6x_rust_led::mavftp;
use v6x_rust_led::parameters::{Id, COUNT, REGISTRY};

use crate::parameter_manager::ParameterManager;
use v6x_rust_led::logger::{LogError, LogRequest, LogResponse};

pub const SYSTEM_ID: u8 = Id::MavSysId.definition().default as u8;
pub const COMPONENT_ID: u8 = 1;

/// Ground stations only hold one FTP session; both links share it.
static FTP: Mutex<CriticalSectionRawMutex, RefCell<mavftp::Server>> =
    Mutex::new(RefCell::new(mavftp::Server::new()));

/// Decoded inbound frame plus the header it arrived with.
pub struct Decoded {
    pub header: MavHeader,
    pub message: MavMessage,
}

/// Replies produced for one inbound request, plus whether a reboot is pending.
pub struct Reply {
    messages: [Option<MavMessage>; Self::CAPACITY],
    count: usize,
    pub reboot: bool,
}

impl Reply {
    /// One burst of FTP frames must fit without being dropped.
    pub const CAPACITY: usize = 4;

    pub const EMPTY: Self = Self {
        messages: [const { None }; Self::CAPACITY],
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

const _: () = assert!(mavftp::BURST_FRAMES <= Reply::CAPACITY);

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
        MavMessage::FILE_TRANSFER_PROTOCOL(data) => {
            if data.target_network == 0 {
                let mut responses = mavftp::Responses::new();
                FTP.lock(|ftp| ftp.borrow_mut().handle(&data.payload, &mut responses));
                for frame in responses.frames() {
                    reply.push(file_transfer(header, frame));
                }
            }
        }
        MavMessage::PARAM_REQUEST_LIST(_) => {
            // The reply buffer holds only part of the registry, so the ground
            // station has to ask for the missing indices, which QGC does.
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
                let outcome = if data.param_type == MavParamType::MAV_PARAM_TYPE_REAL32 {
                    parameters
                        .set(id, data.param_value)
                        .map_err(|error| error.message())
                } else {
                    Err("PARAM_SET: expected REAL32")
                };
                reply.push(parameter(parameters, id));
                match outcome {
                    // Ground stations expect a changed value to survive a power
                    // cycle, so persist it right away when storage allows. Only
                    // disarmed vehicles with outputs closed may write FRAM.
                    Ok(true) => {
                        if parameters.save(crate::OUTPUT_STATE).await.is_err() {
                            reply.push(parameter_warning(
                                "PARAM_SET: RAM only; save blocked while outputs active",
                            ));
                        }
                    }
                    Ok(false) => {}
                    Err(error) => reply.push(parameter_warning(error)),
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
        MavMessage::SYSTEM_TIME(data) => {
            // QGC broadcasts UTC once a second. Only a plausible UNIX time is
            // accepted, and equal values are skipped inside the RTC driver.
            crate::rtc::set_unix_usec(data.time_unix_usec);
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
        MavMessage::COMMAND_LONG(data) if data.command == MavCmd::MAV_CMD_LOGGING_START => {
            reply.push(command_ack(
                &header,
                data.command,
                logger_result(crate::sd_logger::request_start()),
            ));
        }
        MavMessage::COMMAND_LONG(data) if data.command == MavCmd::MAV_CMD_LOGGING_STOP => {
            reply.push(command_ack(
                &header,
                data.command,
                logger_result(crate::sd_logger::request_stop()),
            ));
        }
        MavMessage::COMMAND_LONG(data)
            if data.command == MavCmd::MAV_CMD_REQUEST_MESSAGE && data.param1 == 252.0 =>
        {
            let result = if let Some(message) = ina219_status() {
                reply.push(message);
                MavResult::MAV_RESULT_ACCEPTED
            } else {
                MavResult::MAV_RESULT_TEMPORARILY_REJECTED
            };
            reply.push(command_ack(&header, data.command, result));
        }
        MavMessage::COMMAND_LONG(data) => {
            let mut reboot = false;
            let response = match data.command {
                MavCmd::MAV_CMD_REQUEST_MESSAGE => match data.param1 as u32 {
                    0 => Some(heartbeat()),
                    1 => Some(system_status()),
                    2 => Some(system_time()),
                    105 => highres_imu(),
                    148 => Some(autopilot_version()),
                    253 => Some(status_text()),
                    // QGC asks for this before it can show parameter groups.
                    397 => Some(component_metadata()),
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
        // QGC only saves onboard logs as `.ulg` when the autopilot is PX4;
        // every other identity is stored as `.bin`. We keep the original PX4
        // bootloader, MAVLink2 and no FTP capability, so the message-based
        // LOG_* download path is still used.
        autopilot: MavAutopilot::MAV_AUTOPILOT_PX4,
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
        // 0 means "UTC unknown"; QGC then falls back to boot time.
        time_unix_usec: crate::rtc::valid_unix().map_or(0, |seconds| seconds * 1_000_000),
        time_boot_ms: embassy_time::Instant::now().as_millis() as u32,
    })
}

pub fn highres_imu() -> Option<MavMessage> {
    let sample = crate::imu::latest()?;
    let now = embassy_time::Instant::now().as_micros();
    if sample.is_stale(now, v6x_rust_led::icm42688::Config::DEFAULT) {
        return None;
    }
    let accel_frd = sample.accel_frd();
    let gyro_frd = sample.gyro_frd();
    let fields_updated = HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XACC
        | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_YACC
        | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_ZACC
        | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XGYRO
        | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_YGYRO
        | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_ZGYRO
        | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_TEMPERATURE;
    Some(MavMessage::HIGHRES_IMU(HIGHRES_IMU_DATA {
        time_usec: sample.sample_timestamp_us,
        xacc: accel_frd[0],
        yacc: accel_frd[1],
        zacc: accel_frd[2],
        xgyro: gyro_frd[0],
        ygyro: gyro_frd[1],
        zgyro: gyro_frd[2],
        temperature: sample.temperature_c,
        fields_updated,
        id: v6x_rust_led::icm42688::SOURCE_ID,
        ..Default::default()
    }))
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

pub fn ina219_status() -> Option<MavMessage> {
    let measurement = crate::ina219::latest()?;
    // One standard debug message keeps all three SI values visible together
    // even in QGC versions which merge named scalar messages in Inspector.
    Some(MavMessage::DEBUG_VECT(DEBUG_VECT_DATA {
        time_usec: measurement.timestamp_us,
        name: "INA219".into(),
        x: measurement.bus_voltage_v,
        y: measurement.current_a,
        z: measurement.power_w,
    }))
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

/// One FTP frame addressed back to the requesting ground station.
pub fn file_transfer(header: MavHeader, frame: &mavftp::Frame) -> MavMessage {
    let mut payload = [0u8; mavftp::PAYLOAD_LEN];
    frame.encode(&mut payload);
    MavMessage::FILE_TRANSFER_PROTOCOL(FILE_TRANSFER_PROTOCOL_DATA {
        target_network: 0,
        target_system: header.system_id,
        target_component: header.component_id,
        payload,
    })
}

/// Advertise the static metadata file QGC fetches with MAVLink FTP.
pub fn component_metadata() -> MavMessage {
    MavMessage::COMPONENT_METADATA(COMPONENT_METADATA_DATA {
        time_boot_ms: embassy_time::Instant::now().as_millis() as u32,
        file_crc: v6x_rust_led::component_metadata::GENERAL_CRC,
        uri: v6x_rust_led::component_metadata::GENERAL_URI.into(),
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

fn logger_result(result: Result<(), v6x_rust_led::logger::ControlError>) -> MavResult {
    match result {
        Ok(()) => MavResult::MAV_RESULT_ACCEPTED,
        Err(v6x_rust_led::logger::ControlError::InvalidState) => {
            MavResult::MAV_RESULT_TEMPORARILY_REJECTED
        }
        Err(_) => MavResult::MAV_RESULT_FAILED,
    }
}

pub fn serial_control_reply(header: MavHeader, bytes: &[u8], more: bool) -> MavMessage {
    let mut data = [0u8; 70];
    data[..bytes.len()].copy_from_slice(bytes);
    let mut flags = SerialControlFlag::SERIAL_CONTROL_FLAG_REPLY;
    if more {
        flags |= SerialControlFlag::SERIAL_CONTROL_FLAG_MULTI;
    }
    MavMessage::SERIAL_CONTROL(SERIAL_CONTROL_DATA {
        device: SerialControlDev::SERIAL_CONTROL_DEV_SHELL,
        flags,
        count: bytes.len() as u8,
        data,
        target_system: header.system_id,
        target_component: header.component_id,
        ..Default::default()
    })
}

pub fn log_request(message: &MavMessage) -> Option<LogRequest> {
    match message {
        MavMessage::LOG_REQUEST_LIST(data)
            if targeted(data.target_system, data.target_component) =>
        {
            Some(LogRequest::List {
                start: data.start,
                end: data.end,
            })
        }
        MavMessage::LOG_REQUEST_DATA(data)
            if targeted(data.target_system, data.target_component) =>
        {
            Some(LogRequest::Data {
                id: data.id,
                offset: data.ofs,
                count: data.count,
            })
        }
        MavMessage::LOG_REQUEST_END(data)
            if targeted(data.target_system, data.target_component) =>
        {
            Some(LogRequest::End)
        }
        MavMessage::LOG_ERASE(data) if targeted(data.target_system, data.target_component) => {
            Some(LogRequest::Erase)
        }
        _ => None,
    }
}

fn targeted(system: u8, component: u8) -> bool {
    matches!(system, 0 | SYSTEM_ID) && matches!(component, 0 | COMPONENT_ID)
}

pub fn log_response(response: LogResponse) -> Option<MavMessage> {
    match response {
        LogResponse::Entry {
            id,
            num_logs,
            last_log_num,
            size,
            time_utc,
        } => Some(MavMessage::LOG_ENTRY(LOG_ENTRY_DATA {
            time_utc,
            size,
            id,
            num_logs,
            last_log_num,
        })),
        LogResponse::Data {
            id,
            offset,
            count,
            data,
        } => Some(MavMessage::LOG_DATA(LOG_DATA_DATA {
            ofs: offset,
            id,
            count,
            data,
        })),
        LogResponse::Error(error) => Some(parameter_warning(log_error_text(error))),
        LogResponse::Done => None,
    }
}

fn log_error_text(error: LogError) -> &'static str {
    match error {
        LogError::Busy => "Log service busy",
        LogError::Mount => "Log SD mount failed",
        LogError::Directory => "Log directory read failed",
        LogError::NotFound => "Log id not found",
        LogError::Range => "Log offset out of range",
        LogError::Read => "Log data read failed",
        LogError::EraseDisabled => "Log erase disabled",
    }
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
