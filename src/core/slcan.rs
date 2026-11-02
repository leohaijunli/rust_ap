//! SLCAN (ASCII serial-CAN) codec for the DroneCAN USB bridge.
//!
//! Zubax-compatible framing used by the UAVCAN GUI Tool: CAN frames and adapter
//! commands are ASCII over a byte stream, each line terminated by `\r`. The
//! bridge is one source on the FDCAN2 bus; it is not a DroneCAN node.

pub const MAX_DATA: usize = 8;
pub const MAX_FRAME_LINE: usize = 1 + 8 + 1 + 2 * MAX_DATA; // "T" + id + dlc + data + \r

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    LineTooLong,
    Invalid,
    BufferTooSmall,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    pub extended: bool,
    pub id: u32,
    pub data: [u8; MAX_DATA],
    pub len: u8,
}

impl Frame {
    /// A DroneCAN/UAVCAN message is a 29-bit extended CAN frame.
    pub const fn extended(id: u32, data: [u8; MAX_DATA], len: u8) -> Self {
        Self {
            extended: true,
            id,
            data,
            len,
        }
    }
}

/// Outcome of decoding one complete SLCAN line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    Frame(Frame),
    Open,
    Close,
    SetBitrate(u32),
    Version,
    Flags,
    Timestamp,
    Unknown,
}

/// Incremental SLCAN line decoder. Feed bytes with `push`; it returns the
/// command when a complete `\r`-terminated line has been consumed.
pub struct Codec {
    line: [u8; MAX_FRAME_LINE],
    len: usize,
}

impl Codec {
    pub const fn new() -> Self {
        Self {
            line: [0; MAX_FRAME_LINE],
            len: 0,
        }
    }

    pub fn push(&mut self, byte: u8) -> Result<Option<Command>, Error> {
        if byte == b'\r' {
            let command = decode(&self.line[..self.len]);
            self.len = 0;
            command
        } else {
            if self.len >= MAX_FRAME_LINE {
                self.len = 0;
                return Err(Error::LineTooLong);
            }
            self.line[self.len] = byte;
            self.len += 1;
            Ok(None)
        }
    }

    pub fn reset(&mut self) {
        self.len = 0;
    }
}

impl Default for Codec {
    fn default() -> Self {
        Self::new()
    }
}

fn decode(line: &[u8]) -> Result<Option<Command>, Error> {
    match line.first().copied() {
        None => Ok(None),
        Some(b'O') if line.len() == 1 => Ok(Some(Command::Open)),
        Some(b'C') if line.len() == 1 => Ok(Some(Command::Close)),
        Some(b'V' | b'v') if line.len() == 1 => Ok(Some(Command::Version)),
        Some(b'F') if line.len() == 1 => Ok(Some(Command::Flags)),
        Some(b'Z') if line.len() == 1 => Ok(Some(Command::Timestamp)),
        Some(b'S') if line.len() == 2 => {
            decode_bitrate(line[1]).map(|bits| Some(Command::SetBitrate(bits)))
        }
        Some(b'T') => decode_frame(line, true).map(Some),
        Some(b't') => decode_frame(line, false).map(Some),
        Some(_) => Err(Error::Invalid),
    }
}

fn decode_bitrate(digit: u8) -> Result<u32, Error> {
    Ok(match digit {
        b'0' => 10_000,
        b'1' => 20_000,
        b'2' => 50_000,
        b'3' => 100_000,
        b'4' => 125_000,
        b'5' => 250_000,
        b'6' => 500_000,
        b'7' => 800_000,
        b'8' => 1_000_000,
        _ => return Err(Error::Invalid),
    })
}

fn decode_frame(line: &[u8], extended: bool) -> Result<Command, Error> {
    let id_len = if extended { 8 } else { 3 };
    if line.len() < 1 + id_len + 1 {
        return Err(Error::Invalid);
    }
    let id = parse_hex(&line[1..1 + id_len])?;
    let dlc = hex_val(line[1 + id_len])?;
    if dlc > MAX_DATA as u8 {
        return Err(Error::Invalid);
    }
    let mut data = [0; MAX_DATA];
    for (index, byte) in data[..usize::from(dlc)].iter_mut().enumerate() {
        let offset = 1 + id_len + 1 + 2 * index;
        if offset + 1 >= line.len() {
            return Err(Error::Invalid);
        }
        *byte = (hex_val(line[offset])? << 4) | hex_val(line[offset + 1])?;
    }
    Ok(Command::Frame(Frame {
        extended,
        id,
        data,
        len: dlc,
    }))
}

/// Encode a CAN frame as an SLCAN `T...\r` line (DroneCAN always uses extended).
pub fn encode_frame(frame: &Frame, out: &mut [u8]) -> Result<usize, Error> {
    let (prefix, id_len) = if frame.extended { (b'T', 8) } else { (b't', 3) };
    let len = 1 + id_len + 1 + 2 * usize::from(frame.len) + 1;
    if len > out.len() || frame.len > MAX_DATA as u8 {
        return Err(Error::BufferTooSmall);
    }
    out[0] = prefix;
    write_hex(&mut out[1..1 + id_len], frame.id, id_len);
    out[1 + id_len] = hex_char(frame.len);
    for (index, byte) in frame.data[..usize::from(frame.len)].iter().enumerate() {
        let offset = 1 + id_len + 1 + 2 * index;
        out[offset] = hex_char(byte >> 4);
        out[offset + 1] = hex_char(byte & 0x0f);
    }
    out[len - 1] = b'\r';
    Ok(len)
}

/// Adapter acknowledgement: success `\r`, failure BEL.
pub const fn ack(ok: bool) -> u8 {
    if ok {
        b'\r'
    } else {
        b'\x07'
    }
}

/// Version reply the GUI reads on `V`/`v`.
pub fn version_reply(out: &mut [u8]) -> usize {
    let text = b"V20231102 v0.1\r";
    let len = text.len();
    out[..len].copy_from_slice(text);
    len
}

fn parse_hex(bytes: &[u8]) -> Result<u32, Error> {
    let mut value = 0u32;
    for &byte in bytes {
        value = value.checked_shl(4).ok_or(Error::Invalid)? | u32::from(hex_val(byte)?);
    }
    Ok(value)
}

fn hex_val(byte: u8) -> Result<u8, Error> {
    match byte {
        b'0'..=b'9' => Ok(byte - b'0'),
        b'a'..=b'f' => Ok(byte - b'a' + 10),
        b'A'..=b'F' => Ok(byte - b'A' + 10),
        _ => Err(Error::Invalid),
    }
}

const fn hex_char(value: u8) -> u8 {
    match value & 0x0f {
        0..=9 => b'0' + (value & 0x0f),
        _ => b'A' + (value & 0x0f) - 10,
    }
}

fn write_hex(out: &mut [u8], value: u32, digits: usize) {
    for (index, slot) in out.iter_mut().enumerate().take(digits) {
        *slot = hex_char((value >> (4 * (digits - 1 - index))) as u8);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extended_dronecan_frame_round_trips() {
        let frame = Frame::extended(0x1250a2, [1, 2, 3, 4, 0, 0, 0, 0], 4);
        let mut out = [0u8; MAX_FRAME_LINE];
        let len = encode_frame(&frame, &mut out).unwrap();
        assert_eq!(&out[..len], b"T001250A2401020304\r");
        let mut codec = Codec::new();
        for &byte in &out[..len] {
            let decoded = codec.push(byte).unwrap();
            if let Some(Command::Frame(parsed)) = decoded {
                assert_eq!(parsed, frame);
                return;
            }
        }
        panic!("frame not decoded");
    }

    #[test]
    fn commands_and_bitrate_are_parsed() {
        let commands: &[(&[u8], Command)] = &[
            (b"O\r", Command::Open),
            (b"C\r", Command::Close),
            (b"S8\r", Command::SetBitrate(1_000_000)),
            (b"V\r", Command::Version),
            (b"v\r", Command::Version),
            (b"Z\r", Command::Timestamp),
            (b"F\r", Command::Flags),
        ];
        for (line, expected) in commands {
            let mut codec = Codec::new();
            let mut decoded = None;
            for &byte in *line {
                if let Some(command) = codec.push(byte).unwrap() {
                    decoded = Some(command);
                }
            }
            assert_eq!(decoded, Some(*expected), "line {:?}", line);
        }
        // A lone 'S' is incomplete; an unknown bitrate digit is rejected on '\r'.
        let mut codec = Codec::new();
        assert_eq!(codec.push(b'S').unwrap(), None);
        assert_eq!(codec.push(b'9').unwrap(), None);
        assert_eq!(codec.push(b'\r'), Err(Error::Invalid));
    }

    #[test]
    fn standard_frame_and_bad_line_are_handled() {
        let frame = Frame {
            extended: false,
            id: 0x123,
            data: [0xab, 0, 0, 0, 0, 0, 0, 0],
            len: 1,
        };
        let mut out = [0u8; MAX_FRAME_LINE];
        let len = encode_frame(&frame, &mut out).unwrap();
        assert_eq!(&out[..len], b"t1231AB\r");
        let mut codec = Codec::new();
        for &byte in &out[..len] {
            let decoded = codec.push(byte).unwrap();
            if let Some(Command::Frame(parsed)) = decoded {
                assert_eq!(parsed, frame);
                return;
            }
        }
        panic!("standard frame not decoded");
    }

    #[test]
    fn overflow_is_rejected_and_the_line_recovers() {
        let mut codec = Codec::new();
        for _ in 0..MAX_FRAME_LINE {
            assert_eq!(codec.push(b'0').unwrap(), None);
        }
        assert_eq!(codec.push(b'0'), Err(Error::LineTooLong));
        assert_eq!(codec.push(b'O').unwrap(), None);
        assert_eq!(codec.push(b'\r').unwrap(), Some(Command::Open));
    }
}
