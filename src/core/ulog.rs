//! Minimal no-alloc ULog v1 encoder for the fixed M1 topic registry.

use core::mem::size_of;

pub const FILE_HEADER_LEN: usize = 16;
pub const MAX_RECORD_LEN: usize = 512;
pub const TOPIC_COUNT: usize = 13;

const MAGIC: [u8; 7] = [0x55, 0x4c, 0x6f, 0x67, 0x01, 0x12, 0x35];
const FORMAT_VERSION: u8 = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum TopicId {
    Imu = 0,
    Ahrs = 1,
    MotorCommand = 2,
    ArmingStatus = 3,
    EscStatus = 4,
    GnssFix = 5,
    GnssAuxiliary = 6,
    NodeStatus = 7,
    CanStats = 8,
    RuntimeHealth = 9,
    Event = 10,
    ParameterChange = 11,
    MavlinkStats = 12,
}

pub struct Topic {
    pub id: TopicId,
    pub name: &'static str,
    pub format: &'static str,
    pub payload_len: usize,
}

pub const TOPICS: [Topic; TOPIC_COUNT] = [
    Topic {
        id: TopicId::Imu,
        name: "imu_icm42688",
        format: "imu_icm42688:uint64_t timestamp;uint64_t sample_timestamp;uint32_t sequence;uint8_t valid;uint8_t source_id;uint32_t age_us;float[3] gyro_rad_s;float[3] accel_m_s2;float temperature_c;",
        payload_len: 54,
    },
    Topic {
        id: TopicId::Ahrs,
        name: "ahrs",
        format: "ahrs:uint64_t timestamp;uint64_t sample_timestamp;uint32_t sequence;uint8_t valid;uint8_t yaw_relative;uint32_t age_us;float[4] q_body_to_world;float[3] angular_velocity_rad_s;",
        payload_len: 54,
    },
    Topic {
        id: TopicId::MotorCommand,
        name: "motor_command_4_slots",
        format: "motor_command_4_slots:uint64_t timestamp;uint32_t sequence;uint8_t valid;uint8_t armed;uint8_t outputs_enabled;int16_t[4] command;uint32_t age_us;",
        payload_len: 27,
    },
    Topic {
        id: TopicId::ArmingStatus,
        name: "arming_status",
        format: "arming_status:uint64_t timestamp;uint32_t sequence;uint8_t status;",
        payload_len: 13,
    },
    Topic {
        id: TopicId::EscStatus,
        name: "esc_status",
        format: "esc_status:uint64_t timestamp;uint32_t sequence;uint8_t source_node_id;uint8_t esc_index;uint8_t valid;uint8_t stale;uint32_t error_count;float voltage;float current;float temperature;int32_t rpm;uint8_t power_rating_pct;",
        payload_len: 37,
    },
    Topic {
        id: TopicId::GnssFix,
        name: "gnss_fix",
        format: "gnss_fix:uint64_t timestamp;uint64_t source_timestamp;uint32_t sequence;uint8_t valid;uint8_t status;uint8_t time_standard;uint8_t sats_used;uint8_t leap_seconds;uint8_t mode;uint8_t sub_mode;int64_t longitude_deg_1e8;int64_t latitude_deg_1e8;int32_t height_ellipsoid_mm;int32_t height_msl_mm;float[3] ned_velocity_m_s;float pdop;float[6] covariance;",
        payload_len: 91,
    },
    Topic {
        id: TopicId::GnssAuxiliary,
        name: "gnss_auxiliary",
        format: "gnss_auxiliary:uint64_t timestamp;uint32_t sequence;uint8_t valid;uint8_t sats_visible;uint8_t sats_used;float[7] dop;",
        payload_len: 43,
    },
    Topic {
        id: TopicId::NodeStatus,
        name: "node_status",
        format: "node_status:uint64_t timestamp;uint8_t source_node_id;uint8_t health;uint8_t mode;uint8_t online;uint32_t uptime_sec;uint16_t vendor_status;",
        payload_len: 18,
    },
    Topic {
        id: TopicId::CanStats,
        name: "can_stats",
        format: "can_stats:uint64_t timestamp;uint32_t[10] counters;",
        payload_len: 48,
    },
    Topic {
        id: TopicId::RuntimeHealth,
        name: "runtime_health",
        format: "runtime_health:uint64_t timestamp;uint8_t cpu_valid;uint16_t cpu_load_permille;uint8_t stack_valid;uint32_t stack_free_bytes;uint8_t schedule_valid;uint32_t release_miss;uint32_t deadline_miss;uint32_t logger_normal_dropped;uint32_t logger_event_dropped;uint32_t logger_high_water_bytes;",
        payload_len: 37,
    },
    Topic {
        id: TopicId::Event,
        name: "event",
        format: "event:uint64_t timestamp;uint8_t severity;uint16_t code;int32_t argument;uint32_t count;",
        payload_len: 19,
    },
    Topic {
        id: TopicId::ParameterChange,
        name: "parameter_change",
        format: "parameter_change:uint64_t timestamp;uint16_t index;uint8_t kind;uint8_t dirty;uint8_t persisted;float value;",
        payload_len: 17,
    },
    Topic {
        id: TopicId::MavlinkStats,
        name: "mavlink_stats",
        format: "mavlink_stats:uint64_t timestamp;uint8_t link;uint32_t[4] counters;",
        payload_len: 25,
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
    TimestampRegression,
}

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
    let definition = topic.definition();
    let mut builder = Builder::new(b'A');
    builder.push_u8(0)?;
    builder.push_u16(topic as u16)?;
    builder.push(definition.name.as_bytes())?;
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
        let mut builder = Builder::new(b'D');
        builder.push_u16(topic as u16)?;
        builder.push(&timestamp_us.to_le_bytes())?;
        builder.push(fields)?;
        self.last_timestamp[index] = timestamp_us;
        self.seen[index] = true;
        Ok(builder.finish())
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
            assert!(topic.payload_len <= 256);
            assert!(format(topic.id).unwrap().as_bytes().len() <= MAX_RECORD_LEN);
            assert_eq!(subscription(topic.id).unwrap().as_bytes()[3], 0);
            assert!(!TOPICS[..index].iter().any(|other| other.name == topic.name));
        }
    }

    #[test]
    fn data_length_and_per_topic_monotonicity_are_enforced() {
        let mut encoder = DataEncoder::new();
        let topic = TopicId::ArmingStatus;
        let fields = [0; 5];
        let record = encoder.encode(topic, 100, &fields).unwrap();
        assert_eq!(&record.as_bytes()[..5], &[15, 0, b'D', 3, 0]);
        assert_eq!(&record.as_bytes()[5..13], &100_u64.to_le_bytes());
        assert_eq!(
            encoder.encode(topic, 99, &fields),
            Err(Error::TimestampRegression)
        );
        assert_eq!(
            encoder.encode(topic, 101, &[0; 4]),
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
