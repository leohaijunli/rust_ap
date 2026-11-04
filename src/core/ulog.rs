//! Minimal no-alloc ULog v1 encoder for the fixed M1 topic registry.

use core::mem::size_of;

use crate::ina219::Measurement;
use crate::px4_messages::{ActuatorArmed, SensorAccel, SensorGps, SensorGyro};

pub const FILE_HEADER_LEN: usize = 16;
pub const MAX_RECORD_LEN: usize = 800;
pub const TOPIC_COUNT: usize = 11;

const MAGIC: [u8; 7] = [0x55, 0x4c, 0x6f, 0x67, 0x01, 0x12, 0x35];
const FORMAT_VERSION: u8 = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum TopicId {
    SensorAccel = 0,
    SensorGyro = 1,
    DronecanRawCommand = 2,
    ActuatorArmed = 3,
    DronecanEscStatus = 4,
    DronecanNodeStatus = 5,
    CanStats = 6,
    RuntimeHealth = 7,
    ParameterChange = 8,
    CurrentSensor = 9,
    SensorGps = 10,
}

pub struct Topic {
    pub id: TopicId,
    pub name: &'static str,
    pub format: &'static str,
    pub payload_len: usize,
}

pub const TOPICS: [Topic; TOPIC_COUNT] = [
    Topic {
        id: TopicId::SensorAccel,
        name: "sensor_accel",
        format: "sensor_accel:uint64_t timestamp;uint64_t timestamp_sample;uint32_t device_id;float x;float y;float z;float temperature;uint32_t error_count;uint8_t[3] clip_counter;uint8_t samples;",
        payload_len: 44,
    },
    Topic {
        id: TopicId::SensorGyro,
        name: "sensor_gyro",
        format: "sensor_gyro:uint64_t timestamp;uint64_t timestamp_sample;uint32_t device_id;float x;float y;float z;float temperature;uint32_t error_count;uint8_t[3] clip_counter;uint8_t samples;",
        payload_len: 44,
    },
    Topic {
        id: TopicId::DronecanRawCommand,
        name: "dronecan_raw_command",
        format: "dronecan_raw_command:uint64_t timestamp;uint32_t sequence;uint8_t valid;uint8_t armed;uint8_t outputs_enabled;int16_t[4] command;uint32_t age_us;",
        payload_len: 27,
    },
    Topic {
        id: TopicId::ActuatorArmed,
        name: "actuator_armed",
        format: "actuator_armed:uint64_t timestamp;bool armed;bool prearmed;bool ready_to_arm;bool lockdown;bool kill;bool termination;bool in_esc_calibration_mode;",
        payload_len: 15,
    },
    Topic {
        id: TopicId::DronecanEscStatus,
        name: "dronecan_esc_status",
        format: "dronecan_esc_status:uint64_t timestamp;uint32_t sequence;uint8_t source_node_id;uint8_t esc_index;uint8_t valid;uint8_t stale;uint32_t error_count;float voltage;float current;float temperature;int32_t rpm;uint8_t power_rating_pct;",
        payload_len: 37,
    },
    Topic {
        id: TopicId::DronecanNodeStatus,
        name: "dronecan_node_status",
        format: "dronecan_node_status:uint64_t timestamp;uint8_t source_node_id;uint8_t health;uint8_t mode;uint8_t online;uint32_t uptime_sec;uint16_t vendor_status;",
        payload_len: 18,
    },
    Topic {
        id: TopicId::CanStats,
        name: "rust_can_stats",
        format: "rust_can_stats:uint64_t timestamp;uint32_t[10] counters;",
        payload_len: 48,
    },
    Topic {
        id: TopicId::RuntimeHealth,
        name: "rust_runtime_health",
        format: "rust_runtime_health:uint64_t timestamp;uint8_t cpu_valid;uint16_t cpu_load_permille;uint8_t stack_valid;uint32_t stack_free_bytes;uint8_t schedule_valid;uint32_t release_miss;uint32_t deadline_miss;uint32_t logger_normal_dropped;uint32_t logger_event_dropped;uint32_t logger_high_water_bytes;",
        payload_len: 37,
    },
    Topic {
        id: TopicId::ParameterChange,
        name: "rust_parameter_change",
        format: "rust_parameter_change:uint64_t timestamp;uint16_t index;uint8_t kind;uint8_t dirty;uint8_t persisted;float value;",
        payload_len: 17,
    },
    Topic {
        id: TopicId::CurrentSensor,
        name: "rust_current_sensor",
        format: "rust_current_sensor:uint64_t timestamp;uint64_t timestamp_sample;float voltage_v;float current_a;float power_w;",
        payload_len: 28,
    },
    Topic {
        id: TopicId::SensorGps,
        name: "sensor_gps",
        format: "sensor_gps:uint64_t timestamp;uint64_t timestamp_sample;uint32_t device_id;double latitude_deg;double longitude_deg;double altitude_msl_m;double altitude_ellipsoid_m;float s_variance_m_s;float c_variance_rad;uint8_t fix_type;float eph;float epv;float hdop;float vdop;int32_t noise_per_ms;uint16_t automatic_gain_control;uint8_t jamming_state;int32_t jamming_indicator;uint8_t spoofing_state;uint8_t authentication_state;float vel_m_s;float vel_n_m_s;float vel_e_m_s;float vel_d_m_s;float cog_rad;uint8_t vel_ned_valid;int32_t timestamp_time_relative;uint64_t time_utc_usec;uint8_t satellites_used;uint32_t system_error;float heading;float heading_offset;float heading_accuracy;float rtcm_injection_rate;uint8_t selected_rtcm_instance;uint8_t rtcm_crc_failed;uint8_t rtcm_msg_used;",
        payload_len: 147,
    },
];

impl TopicId {
    pub const fn definition(self) -> &'static Topic {
        &TOPICS[self as usize]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Record {
    bytes: [u8; MAX_RECORD_LEN],
    len: u16,
}

impl Record {
    pub const fn empty() -> Self {
        Self {
            bytes: [0; MAX_RECORD_LEN],
            len: 0,
        }
    }

    #[cfg(test)]
    pub(crate) fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        if bytes.len() > MAX_RECORD_LEN {
            return Err(Error::TooLong);
        }
        let mut record = Self::empty();
        record.bytes[..bytes.len()].copy_from_slice(bytes);
        record.len = bytes.len() as u16;
        Ok(record)
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..usize::from(self.len)]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    TooLong,
    InvalidText,
    InvalidLength,
    InvalidInstance,
    TimestampRegression,
}

impl core::fmt::Display for Error {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter.write_str(match self {
            Self::TooLong => "ULog record exceeds buffer capacity",
            Self::InvalidText => "invalid ULog key or topic name",
            Self::InvalidLength => "ULog payload length does not match schema",
            Self::InvalidInstance => "ULog encoder and estimator instance differ",
            Self::TimestampRegression => "ULog timestamp regressed",
        })
    }
}

impl core::error::Error for Error {}

pub const fn file_header(start_timestamp_us: u64) -> [u8; FILE_HEADER_LEN] {
    let mut bytes = [0; FILE_HEADER_LEN];
    let mut index = 0;
    while index < MAGIC.len() {
        bytes[index] = MAGIC[index];
        index += 1;
    }
    bytes[7] = FORMAT_VERSION;
    let timestamp = start_timestamp_us.to_le_bytes();
    index = 0;
    while index < timestamp.len() {
        bytes[8 + index] = timestamp[index];
        index += 1;
    }
    bytes
}

pub fn flag_bits() -> Record {
    let mut builder = Builder::new(b'B');
    builder.push(&[0; 40]).expect("fixed flag bits fit");
    builder.finish()
}

pub fn format(topic: TopicId) -> Result<Record, Error> {
    message(b'F', topic.definition().format.as_bytes())
}

/// Encode a metadata format into caller-owned storage. Estimator definitions
/// exceed the runtime record capacity; they do not enter the firmware queue.
pub fn format_definition(definition: &str, out: &mut [u8]) -> Result<usize, Error> {
    let length = definition.len();
    if length > usize::from(u16::MAX) || out.len() < length + 3 {
        return Err(Error::TooLong);
    }
    out[..2].copy_from_slice(&(length as u16).to_le_bytes());
    out[2] = b'F';
    out[3..length + 3].copy_from_slice(definition.as_bytes());
    Ok(length + 3)
}

pub fn information(key: &str, value: &[u8]) -> Result<Record, Error> {
    validate_information(key, value.len())?;
    keyed_message(b'I', key.as_bytes(), value)
}

pub fn parameter_i32(name: &str, value: i32) -> Result<Record, Error> {
    parameter(name, b"int32_t ", &value.to_le_bytes())
}

pub fn parameter_f32(name: &str, value: f32) -> Result<Record, Error> {
    parameter(name, b"float ", &value.to_bits().to_le_bytes())
}

pub fn subscription(topic: TopicId) -> Result<Record, Error> {
    raw_subscription(topic as u16, 0, topic.definition().name)
}

pub fn raw_subscription(message_id: u16, multi_id: u8, name: &str) -> Result<Record, Error> {
    validate_name(name)?;
    let mut builder = Builder::new(b'A');
    builder.push_u8(multi_id)?;
    builder.push_u16(message_id)?;
    builder.push(name.as_bytes())?;
    Ok(builder.finish())
}

/// A raw D record; schema length and per-topic monotonicity belong to the
/// caller's registry/encoder, not to a second global firmware topic table.
pub fn data_record(message_id: u16, timestamp_us: u64, fields: &[u8]) -> Result<Record, Error> {
    let mut builder = Builder::new(b'D');
    builder.push_u16(message_id)?;
    builder.push(&timestamp_us.to_le_bytes())?;
    builder.push(fields)?;
    Ok(builder.finish())
}

pub fn dropout(duration_ms: u16) -> Record {
    let mut builder = Builder::new(b'O');
    builder
        .push(&duration_ms.to_le_bytes())
        .expect("dropout fits");
    builder.finish()
}

pub fn synchronization() -> Record {
    message(b'S', &[0x2f, 0x73, 0x13, 0x20, 0x25, 0x0c, 0xbb, 0x12]).expect("synchronization fits")
}

pub fn padding(total_len: usize) -> Result<Record, Error> {
    if !(4..=MAX_RECORD_LEN).contains(&total_len) {
        return Err(Error::InvalidLength);
    }
    let mut builder = Builder::new(b'X');
    let zeros = [0; 32];
    let mut remaining = total_len - 3;
    while remaining != 0 {
        let count = remaining.min(zeros.len());
        builder.push(&zeros[..count])?;
        remaining -= count;
    }
    Ok(builder.finish())
}

pub struct DataEncoder {
    last_timestamp: [u64; TOPIC_COUNT],
    seen: [bool; TOPIC_COUNT],
}

impl DataEncoder {
    pub const fn new() -> Self {
        Self {
            last_timestamp: [0; TOPIC_COUNT],
            seen: [false; TOPIC_COUNT],
        }
    }

    pub fn encode(
        &mut self,
        topic: TopicId,
        timestamp_us: u64,
        fields: &[u8],
    ) -> Result<Record, Error> {
        let index = topic as usize;
        let expected = topic.definition().payload_len - size_of::<u64>();
        if fields.len() != expected {
            return Err(Error::InvalidLength);
        }
        if self.seen[index] && timestamp_us < self.last_timestamp[index] {
            return Err(Error::TimestampRegression);
        }
        let record = data_record(topic as u16, timestamp_us, fields)?;
        self.last_timestamp[index] = timestamp_us;
        self.seen[index] = true;
        Ok(record)
    }
}

impl Default for DataEncoder {
    fn default() -> Self {
        Self::new()
    }
}

pub struct Payload {
    bytes: [u8; MAX_RECORD_LEN - 5 - 8],
    len: usize,
}

impl Payload {
    pub const fn new() -> Self {
        Self {
            bytes: [0; MAX_RECORD_LEN - 5 - 8],
            len: 0,
        }
    }

    pub fn push_u8(&mut self, value: u8) -> Result<(), Error> {
        self.push(&[value])
    }

    pub fn push_u16(&mut self, value: u16) -> Result<(), Error> {
        self.push(&value.to_le_bytes())
    }

    pub fn push_i16(&mut self, value: i16) -> Result<(), Error> {
        self.push(&value.to_le_bytes())
    }

    pub fn push_u32(&mut self, value: u32) -> Result<(), Error> {
        self.push(&value.to_le_bytes())
    }

    pub fn push_i32(&mut self, value: i32) -> Result<(), Error> {
        self.push(&value.to_le_bytes())
    }

    pub fn push_i64(&mut self, value: i64) -> Result<(), Error> {
        self.push(&value.to_le_bytes())
    }

    pub fn push_u64(&mut self, value: u64) -> Result<(), Error> {
        self.push(&value.to_le_bytes())
    }

    pub fn push_f32(&mut self, value: f32) -> Result<(), Error> {
        self.push(&value.to_bits().to_le_bytes())
    }

    pub fn push_f64(&mut self, value: f64) -> Result<(), Error> {
        self.push(&value.to_bits().to_le_bytes())
    }

    pub fn push(&mut self, bytes: &[u8]) -> Result<(), Error> {
        let end = self.len + bytes.len();
        if end > self.bytes.len() {
            return Err(Error::TooLong);
        }
        self.bytes[self.len..end].copy_from_slice(bytes);
        self.len = end;
        Ok(())
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

impl Default for Payload {
    fn default() -> Self {
        Self::new()
    }
}

pub fn sensor_accel_payload(report: SensorAccel) -> Payload {
    sensor_payload(
        report.timestamp_sample,
        report.device_id,
        [report.x, report.y, report.z, report.temperature],
        report.error_count,
        report.clip_counter,
        report.samples,
    )
}

pub fn sensor_gyro_payload(report: SensorGyro) -> Payload {
    sensor_payload(
        report.timestamp_sample,
        report.device_id,
        [report.x, report.y, report.z, report.temperature],
        report.error_count,
        report.clip_counter,
        report.samples,
    )
}

fn sensor_payload(
    timestamp_sample: u64,
    device_id: u32,
    xyz_temperature: [f32; 4],
    error_count: u32,
    clip_counter: [u8; 3],
    samples: u8,
) -> Payload {
    let mut payload = Payload::new();
    payload.push_u64(timestamp_sample).unwrap();
    payload.push_u32(device_id).unwrap();
    for value in xyz_temperature {
        payload.push_f32(value).unwrap();
    }
    payload.push_u32(error_count).unwrap();
    payload.push(&clip_counter).unwrap();
    payload.push_u8(samples).unwrap();
    payload
}

pub fn actuator_armed_payload(report: ActuatorArmed) -> Payload {
    let mut payload = Payload::new();
    for value in [
        report.armed,
        report.prearmed,
        report.ready_to_arm,
        report.lockdown,
        report.kill,
        report.termination,
        report.in_esc_calibration_mode,
    ] {
        payload.push_u8(u8::from(value)).unwrap();
    }
    payload
}

pub fn current_sensor_payload(measurement: Measurement) -> Payload {
    let mut payload = Payload::new();
    payload.push_u64(measurement.timestamp_us).unwrap();
    for value in [
        measurement.bus_voltage_v,
        measurement.current_a,
        measurement.power_w,
    ] {
        payload.push_f32(value).unwrap();
    }
    payload
}

/// Fixed SensorGps.msg field order (payload starts at timestamp_sample; the
/// leading timestamp is written by the logger).
pub fn sensor_gps_payload(report: SensorGps) -> Payload {
    let mut payload = Payload::new();
    payload.push_u64(report.timestamp_sample).unwrap();
    payload.push_u32(report.device_id).unwrap();
    payload.push_f64(report.latitude_deg).unwrap();
    payload.push_f64(report.longitude_deg).unwrap();
    payload.push_f64(report.altitude_msl_m).unwrap();
    payload.push_f64(report.altitude_ellipsoid_m).unwrap();
    payload.push_f32(report.s_variance_m_s).unwrap();
    payload.push_f32(report.c_variance_rad).unwrap();
    payload.push_u8(report.fix_type).unwrap();
    for value in [report.eph, report.epv, report.hdop, report.vdop] {
        payload.push_f32(value).unwrap();
    }
    payload.push_i32(report.noise_per_ms).unwrap();
    payload.push_u16(report.automatic_gain_control).unwrap();
    payload.push_u8(report.jamming_state).unwrap();
    payload.push_i32(report.jamming_indicator).unwrap();
    payload.push_u8(report.spoofing_state).unwrap();
    payload.push_u8(report.authentication_state).unwrap();
    for value in [
        report.vel_m_s,
        report.vel_n_m_s,
        report.vel_e_m_s,
        report.vel_d_m_s,
        report.cog_rad,
    ] {
        payload.push_f32(value).unwrap();
    }
    payload.push_u8(u8::from(report.vel_ned_valid)).unwrap();
    payload.push_i32(report.timestamp_time_relative).unwrap();
    payload.push_u64(report.time_utc_usec).unwrap();
    payload.push_u8(report.satellites_used).unwrap();
    payload.push_u32(report.system_error).unwrap();
    for value in [
        report.heading,
        report.heading_offset,
        report.heading_accuracy,
        report.rtcm_injection_rate,
    ] {
        payload.push_f32(value).unwrap();
    }
    payload.push_u8(report.selected_rtcm_instance).unwrap();
    payload.push_u8(u8::from(report.rtcm_crc_failed)).unwrap();
    payload.push_u8(report.rtcm_msg_used).unwrap();
    payload
}

fn parameter(name: &str, prefix: &[u8], value: &[u8; 4]) -> Result<Record, Error> {
    validate_name(name)?;
    let key_len = prefix.len() + name.len();
    if key_len > usize::from(u8::MAX) {
        return Err(Error::TooLong);
    }
    let mut builder = Builder::new(b'P');
    builder.push_u8(key_len as u8)?;
    builder.push(prefix)?;
    builder.push(name.as_bytes())?;
    builder.push(value)?;
    Ok(builder.finish())
}

fn keyed_message(kind: u8, key: &[u8], value: &[u8]) -> Result<Record, Error> {
    if key.len() > usize::from(u8::MAX) {
        return Err(Error::TooLong);
    }
    let mut builder = Builder::new(kind);
    builder.push_u8(key.len() as u8)?;
    builder.push(key)?;
    builder.push(value)?;
    Ok(builder.finish())
}

fn message(kind: u8, payload: &[u8]) -> Result<Record, Error> {
    let mut builder = Builder::new(kind);
    builder.push(payload)?;
    Ok(builder.finish())
}

fn validate_key(key: &str) -> Result<(), Error> {
    let Some((data_type, name)) = key.split_once(' ') else {
        return Err(Error::InvalidText);
    };
    if data_type.is_empty() || name.is_empty() || name.contains(' ') {
        return Err(Error::InvalidText);
    }
    validate_name(name)
}

fn validate_information(key: &str, value_len: usize) -> Result<(), Error> {
    validate_key(key)?;
    let data_type = key.split_once(' ').unwrap().0;
    let (base, count) = if let Some((base, suffix)) = data_type.split_once('[') {
        let count = suffix
            .strip_suffix(']')
            .and_then(|value| value.parse::<usize>().ok())
            .filter(|&value| value != 0)
            .ok_or(Error::InvalidText)?;
        (base, count)
    } else {
        (data_type, 1)
    };
    let element_size = match base {
        "int8_t" | "uint8_t" | "bool" | "char" => 1,
        "int16_t" | "uint16_t" => 2,
        "int32_t" | "uint32_t" | "float" => 4,
        "int64_t" | "uint64_t" | "double" => 8,
        _ => return Err(Error::InvalidText),
    };
    if element_size * count != value_len {
        return Err(Error::InvalidLength);
    }
    Ok(())
}

fn validate_name(name: &str) -> Result<(), Error> {
    if name.is_empty()
        || !name
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'/'))
    {
        return Err(Error::InvalidText);
    }
    Ok(())
}

struct Builder {
    record: Record,
    len: usize,
}

impl Builder {
    fn new(kind: u8) -> Self {
        let mut record = Record::empty();
        record.bytes[2] = kind;
        Self { record, len: 3 }
    }

    fn push_u8(&mut self, value: u8) -> Result<(), Error> {
        self.push(&[value])
    }

    fn push_u16(&mut self, value: u16) -> Result<(), Error> {
        self.push(&value.to_le_bytes())
    }

    fn push(&mut self, bytes: &[u8]) -> Result<(), Error> {
        let end = self.len + bytes.len();
        if end > self.record.bytes.len() || bytes.len() > usize::from(u16::MAX) {
            return Err(Error::TooLong);
        }
        self.record.bytes[self.len..end].copy_from_slice(bytes);
        self.len = end;
        Ok(())
    }

    fn finish(mut self) -> Record {
        let payload_len = (self.len - 3) as u16;
        self.record.bytes[..2].copy_from_slice(&payload_len.to_le_bytes());
        self.record.len = self.len as u16;
        self.record
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn minimal_messages_match_the_ulog_wire_layout() {
        assert_eq!(
            file_header(1_000_000),
            [0x55, 0x4c, 0x6f, 0x67, 0x01, 0x12, 0x35, 0x01, 0x40, 0x42, 0x0f, 0, 0, 0, 0, 0,]
        );
        let flags = flag_bits();
        assert_eq!(&flags.as_bytes()[..3], &[40, 0, b'B']);
        assert!(flags.as_bytes()[3..].iter().all(|&byte| byte == 0));
        assert_eq!(dropout(25).as_bytes(), &[2, 0, b'O', 25, 0]);
        assert_eq!(
            synchronization().as_bytes(),
            &[8, 0, b'S', 0x2f, 0x73, 0x13, 0x20, 0x25, 0x0c, 0xbb, 0x12]
        );
        assert_eq!(padding(5).unwrap().as_bytes(), &[2, 0, b'X', 0, 0]);
        assert_eq!(padding(3), Err(Error::InvalidLength));
    }

    #[test]
    fn registry_is_unique_bounded_and_self_consistent() {
        for (index, topic) in TOPICS.iter().enumerate() {
            assert_eq!(topic.id as usize, index);
            assert!(!topic.name.is_empty());
            assert!(topic.format.starts_with(topic.name));
            assert!(topic.format.contains(":uint64_t timestamp;"));
            assert!(topic.payload_len + 5 <= crate::log_queue::RUNTIME_RECORD_LEN);
            assert_eq!(
                topic.payload_len,
                size_of::<u64>() + fields_size(topic.format)
            );
            assert!(format(topic.id).unwrap().as_bytes().len() <= MAX_RECORD_LEN);
            assert_eq!(subscription(topic.id).unwrap().as_bytes()[3], 0);
            assert!(!TOPICS[..index].iter().any(|other| other.name == topic.name));
        }
    }

    /// Sum of every field's size in a format string, after the leading name.
    fn fields_size(format: &str) -> usize {
        format
            .split(';')
            .filter(|field| !field.is_empty())
            .skip(1) // the leading "name:"
            .map(|field| {
                let (ty, _name) = field.split_once(' ').expect("type name");
                let (base, count) = ty.split_once('[').map_or((ty, 1), |(base, suffix)| {
                    (
                        base,
                        suffix.trim_end_matches(']').parse().expect("array count"),
                    )
                });
                element_size(base) * count
            })
            .sum()
    }

    fn element_size(base: &str) -> usize {
        match base {
            "int8_t" | "uint8_t" | "bool" | "char" => 1,
            "int16_t" | "uint16_t" => 2,
            "int32_t" | "uint32_t" | "float" => 4,
            "int64_t" | "uint64_t" | "double" => 8,
            _ => panic!("unknown base type {base}"),
        }
    }

    #[test]
    fn data_length_and_per_topic_monotonicity_are_enforced() {
        let mut encoder = DataEncoder::new();
        let topic = TopicId::ActuatorArmed;
        let fields = [0; 7];
        let record = encoder.encode(topic, 100, &fields).unwrap();
        assert_eq!(&record.as_bytes()[..5], &[17, 0, b'D', 3, 0]);
        assert_eq!(&record.as_bytes()[5..13], &100_u64.to_le_bytes());
        assert_eq!(
            encoder.encode(topic, 99, &fields),
            Err(Error::TimestampRegression)
        );
        assert_eq!(
            encoder.encode(topic, 101, &[0; 6]),
            Err(Error::InvalidLength)
        );
    }

    #[test]
    fn information_and_parameters_have_complete_keys() {
        let info = information("char[7] sys_name", b"rust_ap").unwrap();
        assert_eq!(&info.as_bytes()[..4], &[24, 0, b'I', 16]);
        let parameter = parameter_f32("TEST_GAIN", 1.5).unwrap();
        assert_eq!(&parameter.as_bytes()[..4], &[20, 0, b'P', 15]);
        assert_eq!(&parameter.as_bytes()[19..], &1.5_f32.to_le_bytes());
        assert_eq!(information("missing_type", b"x"), Err(Error::InvalidText));
        assert_eq!(
            information("char[8] sys_name", b"rust_ap"),
            Err(Error::InvalidLength)
        );
        assert_eq!(parameter_i32("BAD NAME", 1), Err(Error::InvalidText));
    }
}
