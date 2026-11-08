//! Hand-written encoders for the fixed DroneCAN v0 DSDL set used before M1.

use super::transport::{Metadata, TransferKind, TypeInfo};

pub const NODE_STATUS_ID: u16 = 341;
pub const ESC_RAW_COMMAND_ID: u16 = 1030;
pub const ESC_STATUS_ID: u16 = 1034;
pub const ARMING_STATUS_ID: u16 = 1100;
pub const GET_NODE_INFO_ID: u8 = 1;
pub const RESTART_NODE_ID: u8 = 5;
pub const EXECUTE_OPCODE_ID: u8 = 10;
pub const GET_SET_ID: u8 = 11;

pub const NODE_STATUS_SIGNATURE: u64 = 0x0f08_68d0_c1a7_c6f1;
pub const ESC_RAW_COMMAND_SIGNATURE: u64 = 0x217f_5c87_d7ec_951d;
pub const ESC_STATUS_SIGNATURE: u64 = 0xa9af_28ae_a2fb_b254;
pub const ARMING_STATUS_SIGNATURE: u64 = 0x8700_f375_556a_8003;
pub const GET_NODE_INFO_SIGNATURE: u64 = 0xee46_8a81_21c4_6a9e;
pub const RESTART_NODE_SIGNATURE: u64 = 0x569e_0539_4a30_17f0;
pub const EXECUTE_OPCODE_SIGNATURE: u64 = 0x3b13_1ac5_eb69_d2cd;
pub const GET_SET_SIGNATURE: u64 = 0xa7b6_22f9_39d1_a4d5;

pub const RESTART_MAGIC: u64 = 0x00ac_ce55_1b1e;
pub const OPCODE_SAVE: u8 = 0;
pub const OPCODE_ERASE: u8 = 1;

pub fn type_info(metadata: &Metadata) -> Option<TypeInfo> {
    let (signature, max_payload) = match metadata.kind {
        TransferKind::Message {
            data_type_id: NODE_STATUS_ID,
        } => (NODE_STATUS_SIGNATURE, 7),
        TransferKind::Message {
            data_type_id: ESC_RAW_COMMAND_ID,
        } => (ESC_RAW_COMMAND_SIGNATURE, 35),
        TransferKind::Message {
            data_type_id: ESC_STATUS_ID,
        } => (ESC_STATUS_SIGNATURE, 14),
        TransferKind::Message {
            data_type_id: ARMING_STATUS_ID,
        } => (ARMING_STATUS_SIGNATURE, 1),
        TransferKind::Message {
            data_type_id: super::gnss::GNSS_FIX2_ID,
        } => (super::gnss::GNSS_FIX2_SIGNATURE, 221),
        TransferKind::Message {
            data_type_id: super::gnss::GNSS_AUXILIARY_ID,
        } => (super::gnss::GNSS_AUXILIARY_SIGNATURE, 16),
        TransferKind::Service {
            service_id: GET_NODE_INFO_ID,
            request,
            ..
        } => (GET_NODE_INFO_SIGNATURE, if request { 0 } else { 376 }),
        TransferKind::Service {
            service_id: RESTART_NODE_ID,
            request,
            ..
        } => (RESTART_NODE_SIGNATURE, if request { 5 } else { 1 }),
        TransferKind::Service {
            service_id: EXECUTE_OPCODE_ID,
            ..
        } => (EXECUTE_OPCODE_SIGNATURE, 7),
        TransferKind::Service {
            service_id: GET_SET_ID,
            request,
            ..
        } => (GET_SET_SIGNATURE, if request { 223 } else { 370 }),
        _ => return None,
    };
    Some(TypeInfo {
        signature,
        max_payload,
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NodeStatus {
    pub uptime_sec: u32,
    pub health: u8,
    pub mode: u8,
    pub sub_mode: u8,
    pub vendor_status: u16,
}

impl NodeStatus {
    pub const fn operational(uptime_sec: u32) -> Self {
        Self {
            uptime_sec,
            health: 0,
            mode: 0,
            sub_mode: 0,
            vendor_status: 0,
        }
    }
}

pub fn encode_node_status(status: NodeStatus, output: &mut [u8]) -> Result<usize, CodecError> {
    let mut writer = BitWriter::new(output);
    write_node_status(&mut writer, status)?;
    Ok(writer.finish())
}

pub fn decode_node_status(payload: &[u8]) -> Result<NodeStatus, CodecError> {
    if payload.len() != 7 {
        return Err(CodecError::Length);
    }
    let mut reader = BitReader::new(payload);
    read_node_status(&mut reader)
}

pub fn encode_raw_command(commands: &[i16], output: &mut [u8]) -> Result<usize, CodecError> {
    if commands.len() > 20 {
        return Err(CodecError::Length);
    }
    let mut writer = BitWriter::new(output);
    for &command in commands {
        if !(-8192..=8191).contains(&command) {
            return Err(CodecError::Range);
        }
        writer.write_signed(i64::from(command), 14)?;
    }
    Ok(writer.finish())
}

pub fn encode_zero_raw_command(output: &mut [u8]) -> Result<usize, CodecError> {
    encode_raw_command(&[0; 4], output)
}

pub fn encode_disarmed(output: &mut [u8]) -> Result<usize, CodecError> {
    if output.is_empty() {
        return Err(CodecError::Buffer);
    }
    output[0] = 0;
    Ok(1)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EscStatus {
    pub error_count: u32,
    pub voltage: f32,
    pub current: f32,
    pub temperature: f32,
    pub rpm: i32,
    pub power_rating_pct: u8,
    pub esc_index: u8,
}

pub fn decode_esc_status(payload: &[u8]) -> Result<EscStatus, CodecError> {
    if payload.len() != 14 {
        return Err(CodecError::Length);
    }
    let mut reader = BitReader::new(payload);
    let status = EscStatus {
        error_count: reader.read_unsigned(32)? as u32,
        voltage: float16_to_float32(reader.read_unsigned(16)? as u16),
        current: float16_to_float32(reader.read_unsigned(16)? as u16),
        temperature: float16_to_float32(reader.read_unsigned(16)? as u16),
        rpm: reader.read_signed(18)? as i32,
        power_rating_pct: reader.read_unsigned(7)? as u8,
        esc_index: reader.read_unsigned(5)? as u8,
    };
    if reader.read_unsigned(2)? != 0 {
        return Err(CodecError::Padding);
    }
    Ok(status)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SoftwareVersion {
    pub major: u8,
    pub minor: u8,
    pub optional_field_flags: u8,
    pub vcs_commit: u32,
    pub image_crc: u64,
}

impl SoftwareVersion {
    pub const fn basic(major: u8, minor: u8) -> Self {
        Self {
            major,
            minor,
            optional_field_flags: 0,
            vcs_commit: 0,
            image_crc: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NodeInfo {
    pub status: NodeStatus,
    pub software: SoftwareVersion,
    pub hardware_major: u8,
    pub hardware_minor: u8,
    pub unique_id: [u8; 16],
    pub name: [u8; 80],
    pub name_len: u8,
}

impl NodeInfo {
    pub fn name(&self) -> &[u8] {
        &self.name[..usize::from(self.name_len)]
    }
}

pub struct NodeInfoResponse<'a> {
    pub status: NodeStatus,
    pub software: SoftwareVersion,
    pub hardware_major: u8,
    pub hardware_minor: u8,
    pub unique_id: [u8; 16],
    pub certificate: &'a [u8],
    pub name: &'a [u8],
}

pub fn encode_node_info_response(
    response: &NodeInfoResponse<'_>,
    output: &mut [u8],
) -> Result<usize, CodecError> {
    if response.certificate.len() > 255 || response.name.is_empty() || response.name.len() > 80 {
        return Err(CodecError::Length);
    }
    if !valid_node_name(response.name) {
        return Err(CodecError::Value);
    }
    let mut writer = BitWriter::new(output);
    write_node_status(&mut writer, response.status)?;
    writer.write_unsigned(u64::from(response.software.major), 8)?;
    writer.write_unsigned(u64::from(response.software.minor), 8)?;
    writer.write_unsigned(u64::from(response.software.optional_field_flags), 8)?;
    writer.write_unsigned(u64::from(response.software.vcs_commit), 32)?;
    writer.write_unsigned(response.software.image_crc, 64)?;
    writer.write_unsigned(u64::from(response.hardware_major), 8)?;
    writer.write_unsigned(u64::from(response.hardware_minor), 8)?;
    writer.write_bytes(&response.unique_id)?;
    writer.write_unsigned(response.certificate.len() as u64, 8)?;
    writer.write_bytes(response.certificate)?;
    writer.write_bytes(response.name)?;
    Ok(writer.finish())
}

pub fn decode_node_info_response(payload: &[u8]) -> Result<NodeInfo, CodecError> {
    let mut reader = BitReader::new(payload);
    let status = read_node_status(&mut reader)?;
    let software = SoftwareVersion {
        major: reader.read_unsigned(8)? as u8,
        minor: reader.read_unsigned(8)? as u8,
        optional_field_flags: reader.read_unsigned(8)? as u8,
        vcs_commit: reader.read_unsigned(32)? as u32,
        image_crc: reader.read_unsigned(64)?,
    };
    let hardware_major = reader.read_unsigned(8)? as u8;
    let hardware_minor = reader.read_unsigned(8)? as u8;
    let mut unique_id = [0; 16];
    reader.read_bytes(&mut unique_id)?;
    let certificate_len = reader.read_unsigned(8)? as usize;
    reader.skip_bytes(certificate_len)?;
    let name_bytes = reader.remaining_bytes()?;
    if name_bytes.is_empty() || name_bytes.len() > 80 || !valid_node_name(name_bytes) {
        return Err(CodecError::Value);
    }
    let mut name = [0; 80];
    name[..name_bytes.len()].copy_from_slice(name_bytes);
    Ok(NodeInfo {
        status,
        software,
        hardware_major,
        hardware_minor,
        unique_id,
        name,
        name_len: name_bytes.len() as u8,
    })
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Value<'a> {
    Empty,
    Integer(i64),
    Real(f32),
    Boolean(u8),
    String(&'a [u8]),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum NumericValue {
    Empty,
    Integer(i64),
    Real(f32),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GetSetRequest<'a> {
    pub index: u16,
    pub value: Value<'a>,
    pub name: &'a [u8],
}

pub fn decode_get_set_request(payload: &[u8]) -> Result<GetSetRequest<'_>, CodecError> {
    let mut reader = BitReader::new(payload);
    let index = reader.read_unsigned(13)? as u16;
    let value = read_value(&mut reader)?;
    let name = reader.remaining_bytes()?;
    if name.len() > 92 {
        return Err(CodecError::Length);
    }
    Ok(GetSetRequest { index, value, name })
}

pub struct GetSetResponse<'a> {
    pub value: Value<'a>,
    pub default_value: Value<'a>,
    pub max_value: NumericValue,
    pub min_value: NumericValue,
    pub name: &'a [u8],
}

pub fn encode_get_set_response(
    response: &GetSetResponse<'_>,
    output: &mut [u8],
) -> Result<usize, CodecError> {
    if response.name.len() > 92 {
        return Err(CodecError::Length);
    }
    let mut writer = BitWriter::new(output);
    writer.write_unsigned(0, 5)?;
    write_value(&mut writer, response.value)?;
    writer.write_unsigned(0, 5)?;
    write_value(&mut writer, response.default_value)?;
    writer.write_unsigned(0, 6)?;
    write_numeric_value(&mut writer, response.max_value)?;
    writer.write_unsigned(0, 6)?;
    write_numeric_value(&mut writer, response.min_value)?;
    writer.write_bytes(response.name)?;
    Ok(writer.finish())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExecuteOpcodeRequest {
    pub opcode: u8,
    pub argument: i64,
}

pub fn decode_execute_opcode_request(payload: &[u8]) -> Result<ExecuteOpcodeRequest, CodecError> {
    if payload.len() != 7 {
        return Err(CodecError::Length);
    }
    let mut reader = BitReader::new(payload);
    Ok(ExecuteOpcodeRequest {
        opcode: reader.read_unsigned(8)? as u8,
        argument: reader.read_signed(48)?,
    })
}

pub fn encode_execute_opcode_response(
    argument: i64,
    ok: bool,
    output: &mut [u8],
) -> Result<usize, CodecError> {
    if !(-(1_i64 << 47)..(1_i64 << 47)).contains(&argument) {
        return Err(CodecError::Range);
    }
    let mut writer = BitWriter::new(output);
    writer.write_signed(argument, 48)?;
    writer.write_unsigned(u64::from(ok), 1)?;
    Ok(writer.finish())
}

pub fn decode_restart_node_request(payload: &[u8]) -> Result<u64, CodecError> {
    if payload.len() != 5 {
        return Err(CodecError::Length);
    }
    BitReader::new(payload).read_unsigned(40)
}

pub fn encode_restart_node_response(ok: bool, output: &mut [u8]) -> Result<usize, CodecError> {
    let mut writer = BitWriter::new(output);
    writer.write_unsigned(u64::from(ok), 1)?;
    Ok(writer.finish())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CodecError {
    Buffer,
    Length,
    Range,
    Tag,
    Padding,
    Value,
}

fn write_node_status(writer: &mut BitWriter<'_>, status: NodeStatus) -> Result<(), CodecError> {
    if status.health > 3 || status.mode > 7 || status.sub_mode > 7 {
        return Err(CodecError::Range);
    }
    writer.write_unsigned(u64::from(status.uptime_sec), 32)?;
    writer.write_unsigned(u64::from(status.health), 2)?;
    writer.write_unsigned(u64::from(status.mode), 3)?;
    writer.write_unsigned(u64::from(status.sub_mode), 3)?;
    writer.write_unsigned(u64::from(status.vendor_status), 16)
}

fn read_node_status(reader: &mut BitReader<'_>) -> Result<NodeStatus, CodecError> {
    Ok(NodeStatus {
        uptime_sec: reader.read_unsigned(32)? as u32,
        health: reader.read_unsigned(2)? as u8,
        mode: reader.read_unsigned(3)? as u8,
        sub_mode: reader.read_unsigned(3)? as u8,
        vendor_status: reader.read_unsigned(16)? as u16,
    })
}

fn read_value<'a>(reader: &mut BitReader<'a>) -> Result<Value<'a>, CodecError> {
    Ok(match reader.read_unsigned(3)? {
        0 => Value::Empty,
        1 => Value::Integer(reader.read_signed(64)?),
        2 => Value::Real(f32::from_bits(reader.read_unsigned(32)? as u32)),
        3 => Value::Boolean(reader.read_unsigned(8)? as u8),
        4 => {
            let len = reader.read_unsigned(8)? as usize;
            if len > 128 {
                return Err(CodecError::Length);
            }
            Value::String(reader.read_byte_slice(len)?)
        }
        _ => return Err(CodecError::Tag),
    })
}

fn write_value(writer: &mut BitWriter<'_>, value: Value<'_>) -> Result<(), CodecError> {
    match value {
        Value::Empty => writer.write_unsigned(0, 3),
        Value::Integer(value) => {
            writer.write_unsigned(1, 3)?;
            writer.write_signed(value, 64)
        }
        Value::Real(value) => {
            writer.write_unsigned(2, 3)?;
            writer.write_unsigned(u64::from(value.to_bits()), 32)
        }
        Value::Boolean(value) => {
            writer.write_unsigned(3, 3)?;
            writer.write_unsigned(u64::from(value), 8)
        }
        Value::String(value) => {
            if value.len() > 128 {
                return Err(CodecError::Length);
            }
            writer.write_unsigned(4, 3)?;
            writer.write_unsigned(value.len() as u64, 8)?;
            writer.write_bytes(value)
        }
    }
}

fn write_numeric_value(writer: &mut BitWriter<'_>, value: NumericValue) -> Result<(), CodecError> {
    match value {
        NumericValue::Empty => writer.write_unsigned(0, 2),
        NumericValue::Integer(value) => {
            writer.write_unsigned(1, 2)?;
            writer.write_signed(value, 64)
        }
        NumericValue::Real(value) => {
            writer.write_unsigned(2, 2)?;
            writer.write_unsigned(u64::from(value.to_bits()), 32)
        }
    }
}

fn valid_node_name(name: &[u8]) -> bool {
    name.iter().all(|byte| {
        byte.is_ascii_lowercase() || byte.is_ascii_digit() || matches!(byte, b'.' | b'-' | b'_')
    })
}

fn float16_to_float32(value: u16) -> f32 {
    let sign = u32::from(value & 0x8000) << 16;
    let exponent = (value >> 10) & 0x1f;
    let fraction = u32::from(value & 0x03ff);
    let bits = match exponent {
        0 if fraction == 0 => sign,
        0 => {
            let mut fraction = fraction;
            let mut exponent = 113_u32;
            while fraction & 0x0400 == 0 {
                fraction <<= 1;
                exponent -= 1;
            }
            sign | exponent << 23 | (fraction & 0x03ff) << 13
        }
        31 => sign | 0x7f80_0000 | fraction << 13,
        _ => sign | (u32::from(exponent) + 112) << 23 | fraction << 13,
    };
    f32::from_bits(bits)
}

struct BitWriter<'a> {
    bytes: &'a mut [u8],
    bit_offset: usize,
}

impl<'a> BitWriter<'a> {
    fn new(bytes: &'a mut [u8]) -> Self {
        bytes.fill(0);
        Self {
            bytes,
            bit_offset: 0,
        }
    }

    fn write_unsigned(&mut self, value: u64, bits: usize) -> Result<(), CodecError> {
        if bits == 0 || bits > 64 || self.bit_offset + bits > self.bytes.len() * 8 {
            return Err(CodecError::Buffer);
        }
        if bits < 64 && value >= 1_u64 << bits {
            return Err(CodecError::Range);
        }
        let mut consumed = 0;
        while consumed < bits {
            let width = (bits - consumed).min(8);
            let mask = if width == 8 {
                0xff
            } else {
                (1_u64 << width) - 1
            };
            let chunk = (value >> consumed) & mask;
            for bit in (0..width).rev() {
                self.write_bit(chunk & (1 << bit) != 0);
            }
            consumed += width;
        }
        Ok(())
    }

    fn write_signed(&mut self, value: i64, bits: usize) -> Result<(), CodecError> {
        if bits < 64 {
            let min = -(1_i64 << (bits - 1));
            let max = (1_i64 << (bits - 1)) - 1;
            if !(min..=max).contains(&value) {
                return Err(CodecError::Range);
            }
        }
        let encoded = if bits == 64 {
            value as u64
        } else {
            value as u64 & ((1_u64 << bits) - 1)
        };
        self.write_unsigned(encoded, bits)
    }

    fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), CodecError> {
        for &byte in bytes {
            self.write_unsigned(u64::from(byte), 8)?;
        }
        Ok(())
    }

    fn write_bit(&mut self, value: bool) {
        if value {
            self.bytes[self.bit_offset / 8] |= 1 << (7 - self.bit_offset % 8);
        }
        self.bit_offset += 1;
    }

    fn finish(&self) -> usize {
        self.bit_offset.div_ceil(8)
    }
}

struct BitReader<'a> {
    bytes: &'a [u8],
    bit_offset: usize,
}

impl<'a> BitReader<'a> {
    const fn new(bytes: &'a [u8]) -> Self {
        Self {
            bytes,
            bit_offset: 0,
        }
    }

    fn read_unsigned(&mut self, bits: usize) -> Result<u64, CodecError> {
        if bits == 0 || bits > 64 || self.bit_offset + bits > self.bytes.len() * 8 {
            return Err(CodecError::Length);
        }
        let mut value = 0_u64;
        let mut consumed = 0;
        while consumed < bits {
            let width = (bits - consumed).min(8);
            let mut chunk = 0_u64;
            for _ in 0..width {
                chunk = chunk << 1 | u64::from(self.read_bit());
            }
            value |= chunk << consumed;
            consumed += width;
        }
        Ok(value)
    }

    fn read_signed(&mut self, bits: usize) -> Result<i64, CodecError> {
        let value = self.read_unsigned(bits)?;
        if bits == 64 {
            return Ok(value as i64);
        }
        let shift = 64 - bits;
        Ok(((value << shift) as i64) >> shift)
    }

    fn read_bytes(&mut self, output: &mut [u8]) -> Result<(), CodecError> {
        for byte in output {
            *byte = self.read_unsigned(8)? as u8;
        }
        Ok(())
    }

    fn read_byte_slice(&mut self, len: usize) -> Result<&'a [u8], CodecError> {
        if !self.bit_offset.is_multiple_of(8) || self.bit_offset + len * 8 > self.bytes.len() * 8 {
            return Err(CodecError::Length);
        }
        let start = self.bit_offset / 8;
        self.bit_offset += len * 8;
        Ok(&self.bytes[start..start + len])
    }

    fn skip_bytes(&mut self, len: usize) -> Result<(), CodecError> {
        self.read_byte_slice(len).map(|_| ())
    }

    fn remaining_bytes(&mut self) -> Result<&'a [u8], CodecError> {
        if !self.bit_offset.is_multiple_of(8) {
            return Err(CodecError::Length);
        }
        let start = self.bit_offset / 8;
        self.bit_offset = self.bytes.len() * 8;
        Ok(&self.bytes[start..])
    }

    fn read_bit(&mut self) -> bool {
        let value = self.bytes[self.bit_offset / 8] & (1 << (7 - self.bit_offset % 8)) != 0;
        self.bit_offset += 1;
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bytes(hex: &str) -> std::vec::Vec<u8> {
        hex.as_bytes()
            .as_chunks::<2>()
            .0
            .iter()
            .map(|pair| {
                let text = core::str::from_utf8(pair).unwrap();
                u8::from_str_radix(text, 16).unwrap()
            })
            .collect()
    }

    #[test]
    fn raw_command_and_status_match_fixed_dsdl_vectors() {
        let mut output = [0; 64];
        let len = encode_zero_raw_command(&mut output).unwrap();
        assert_eq!(&output[..len], &[0; 7]);
        let len = encode_raw_command(&[-8192, -1, 0, 8191], &mut output).unwrap();
        assert_eq!(&output[..len], bytes("0083fff0003fdf"));

        let status = decode_esc_status(&bytes("7b000000204e003db05ce02e190c")).unwrap();
        assert_eq!(status.error_count, 123);
        assert_eq!(status.voltage, 24.5);
        assert_eq!(status.current, 1.25);
        assert_eq!(status.temperature, 300.0);
        assert_eq!(status.rpm, 12_000);
        assert_eq!(status.power_rating_pct, 50);
        assert_eq!(status.esc_index, 3);

        let unknown = decode_esc_status(&bytes("00000000ff7fff7fff7f0000807c")).unwrap();
        assert!(unknown.voltage.is_nan());
        assert!(unknown.current.is_nan());
        assert!(unknown.temperature.is_nan());
        assert_eq!(unknown.rpm, -131_072);
        assert_eq!(unknown.esc_index, 31);
    }

    #[test]
    fn node_status_and_info_match_fixed_dsdl_vectors() {
        let status = NodeStatus {
            uptime_sec: 123_456,
            health: 0,
            mode: 0,
            sub_mode: 0,
            vendor_status: 0x1234,
        };
        let mut output = [0; 512];
        let len = encode_node_status(status, &mut output).unwrap();
        assert_eq!(&output[..len], bytes("40e20100003412"));
        assert_eq!(decode_node_status(&output[..len]), Ok(status));

        let response = NodeInfoResponse {
            status: NodeStatus {
                vendor_status: 0,
                ..status
            },
            software: SoftwareVersion::basic(1, 0),
            hardware_major: 0,
            hardware_minor: 0,
            unique_id: core::array::from_fn(|index| index as u8),
            certificate: &[],
            name: b"org.rust-ap.fmuv6x",
        };
        let len = encode_node_info_response(&response, &mut output).unwrap();
        let expected = bytes(concat!(
            "40e20100000000010000000000000000000000000000000000",
            "0102030405060708090a0b0c0d0e0f00",
            "6f72672e727573742d61702e666d75763678"
        ));
        assert_eq!(&output[..len], expected);
        let decoded = decode_node_info_response(&output[..len]).unwrap();
        assert_eq!(decoded.name(), b"org.rust-ap.fmuv6x");
        assert_eq!(decoded.unique_id, response.unique_id);
    }

    #[test]
    fn parameter_requests_and_response_match_fixed_vectors() {
        let integer = bytes("00012a0000000000000043414e5f4e4f44455f4944");
        assert_eq!(
            decode_get_set_request(&integer),
            Ok(GetSetRequest {
                index: 0,
                value: Value::Integer(42),
                name: b"CAN_NODE_ID",
            })
        );
        let real = bytes("00020000284243414e5f4e4f44455f4944");
        assert_eq!(
            decode_get_set_request(&real),
            Ok(GetSetRequest {
                index: 0,
                value: Value::Real(42.0),
                name: b"CAN_NODE_ID",
            })
        );

        let mut output = [0; 512];
        let response = GetSetResponse {
            value: Value::Real(42.0),
            default_value: Value::Real(42.0),
            max_value: NumericValue::Real(127.0),
            min_value: NumericValue::Real(1.0),
            name: b"CAN_NODE_ID",
        };
        let len = encode_get_set_response(&response, &mut output).unwrap();
        assert_eq!(
            &output[..len],
            bytes("02000028420200002842020000fe42020000803f43414e5f4e4f44455f4944")
        );
    }

    #[test]
    fn save_and_restart_services_match_fixed_vectors() {
        assert_eq!(
            decode_execute_opcode_request(&bytes("00000000000000")),
            Ok(ExecuteOpcodeRequest {
                opcode: OPCODE_SAVE,
                argument: 0,
            })
        );
        let mut output = [0; 8];
        let len = encode_execute_opcode_response(0, true, &mut output).unwrap();
        assert_eq!(&output[..len], bytes("00000000000080"));
        assert_eq!(
            decode_restart_node_request(&bytes("1e1b55ceac")),
            Ok(RESTART_MAGIC)
        );
        let len = encode_restart_node_response(true, &mut output).unwrap();
        assert_eq!(&output[..len], &[0x80]);
    }
}
