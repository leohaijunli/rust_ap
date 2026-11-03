//! Read-only MAVLink FTP server for the component metadata files.
//!
//! QGroundControl downloads component metadata with `OpenFileRO` followed by
//! `BurstReadFile` and finishes with `ResetSessions`, so only those commands
//! (plus `ReadFile` for retries) are implemented.

use crate::component_metadata;

/// Payload bytes after the 12-byte FTP header.
pub const MAX_DATA: usize = 239;
/// Complete `FILE_TRANSFER_PROTOCOL.payload` length.
pub const PAYLOAD_LEN: usize = 251;
/// Frames produced per burst request; bounded by the MAVLink reply buffer.
pub const BURST_FRAMES: usize = 4;

const HEADER_LEN: usize = 12;

pub const OP_TERMINATE_SESSION: u8 = 1;
pub const OP_RESET_SESSIONS: u8 = 2;
pub const OP_OPEN_FILE_RO: u8 = 4;
pub const OP_READ_FILE: u8 = 5;
pub const OP_BURST_READ_FILE: u8 = 15;

pub const RSP_ACK: u8 = 128;
pub const RSP_NAK: u8 = 129;

pub const ERR_INVALID_DATA_SIZE: u8 = 3;
pub const ERR_INVALID_SESSION: u8 = 4;
pub const ERR_EOF: u8 = 6;
pub const ERR_UNKNOWN_COMMAND: u8 = 7;
pub const ERR_FILE_NOT_FOUND: u8 = 10;

/// One wire-level FTP frame, before it is copied into a MAVLink message.
#[derive(Clone, Copy)]
pub struct Frame {
    pub seq: u16,
    pub session: u8,
    pub opcode: u8,
    pub size: u8,
    pub req_opcode: u8,
    pub burst_complete: bool,
    pub offset: u32,
    pub data: [u8; MAX_DATA],
}

const EMPTY_FRAME: Frame = Frame {
    seq: 0,
    session: 0,
    opcode: 0,
    size: 0,
    req_opcode: 0,
    burst_complete: false,
    offset: 0,
    data: [0; MAX_DATA],
};

impl Frame {
    /// Serialize the header and data into a `FILE_TRANSFER_PROTOCOL` payload.
    pub fn encode(&self, payload: &mut [u8; PAYLOAD_LEN]) {
        payload[..2].copy_from_slice(&self.seq.to_le_bytes());
        payload[2] = self.session;
        payload[3] = self.opcode;
        payload[4] = self.size;
        payload[5] = self.req_opcode;
        payload[6] = u8::from(self.burst_complete);
        payload[7] = 0;
        payload[8..12].copy_from_slice(&self.offset.to_le_bytes());
        let size = usize::from(self.size);
        payload[HEADER_LEN..HEADER_LEN + size].copy_from_slice(&self.data[..size]);
    }
}

/// Frames produced for one request; never larger than [`BURST_FRAMES`].
pub struct Responses {
    frames: [Frame; BURST_FRAMES],
    len: usize,
}

impl Responses {
    pub const fn new() -> Self {
        Self {
            frames: [EMPTY_FRAME; BURST_FRAMES],
            len: 0,
        }
    }

    pub fn frames(&self) -> &[Frame] {
        &self.frames[..self.len]
    }

    fn push(&mut self, frame: Frame) {
        if self.len < self.frames.len() {
            self.frames[self.len] = frame;
            self.len += 1;
        }
    }
}

impl Default for Responses {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy)]
struct Request {
    seq: u16,
    session: u8,
    opcode: u8,
    size: u8,
    offset: u32,
}

/// Single-session server: ground stations only ever hold one open file.
pub struct Server {
    session: u8,
    active: bool,
    file: Option<&'static [u8]>,
}

impl Server {
    pub const fn new() -> Self {
        Self {
            session: 0,
            active: false,
            file: None,
        }
    }

    /// Handle one request payload, appending the frames to send back.
    pub fn handle(&mut self, payload: &[u8; PAYLOAD_LEN], responses: &mut Responses) {
        let request = parse(payload);
        match request.opcode {
            OP_OPEN_FILE_RO => self.open(payload, request, responses),
            OP_READ_FILE => self.read(request, responses, false),
            OP_BURST_READ_FILE => self.read(request, responses, true),
            OP_TERMINATE_SESSION => {
                if self.active && request.session == self.session {
                    self.close();
                    responses.ack(&request, self.session, 0, &[], false);
                } else {
                    responses.nak(&request, ERR_INVALID_SESSION);
                }
            }
            OP_RESET_SESSIONS => {
                self.close();
                responses.ack(&request, self.session, 0, &[], false);
            }
            _ => responses.nak(&request, ERR_UNKNOWN_COMMAND),
        }
    }

    fn open(&mut self, payload: &[u8; PAYLOAD_LEN], request: Request, responses: &mut Responses) {
        let Some(path) = path(payload, usize::from(request.size)) else {
            responses.nak(&request, ERR_INVALID_DATA_SIZE);
            return;
        };
        let Some(file) = component_metadata::file(path) else {
            responses.nak(&request, ERR_FILE_NOT_FOUND);
            return;
        };
        self.close();
        self.session = self.session.wrapping_add(1).max(1);
        self.active = true;
        self.file = Some(file);
        responses.ack(
            &request,
            self.session,
            0,
            &(file.len() as u32).to_le_bytes(),
            false,
        );
    }

    fn read(&mut self, request: Request, responses: &mut Responses, burst: bool) {
        if !self.active || request.session != self.session {
            responses.nak(&request, ERR_INVALID_SESSION);
            return;
        }
        let Some(file) = self.file else {
            responses.nak(&request, ERR_INVALID_SESSION);
            return;
        };
        let start = request.offset as usize;
        if start >= file.len() {
            responses.nak(&request, ERR_EOF);
            return;
        }

        let window = chunk_size(request.size);
        let mut offset = start;
        let mut seq = request.seq.wrapping_add(1);
        let mut frames_left = if burst { BURST_FRAMES } else { 1 };
        while frames_left > 0 {
            let count = window.min(file.len() - offset);
            frames_left -= 1;
            let last = frames_left == 0 || offset + count == file.len();
            responses.push(Frame {
                seq,
                session: self.session,
                opcode: RSP_ACK,
                size: count as u8,
                req_opcode: request.opcode,
                burst_complete: burst && last,
                offset: offset as u32,
                data: padded(&file[offset..offset + count]),
            });
            offset += count;
            seq = seq.wrapping_add(1);
            if last {
                return;
            }
        }
    }

    fn close(&mut self) {
        self.active = false;
        self.file = None;
    }
}

impl Default for Server {
    fn default() -> Self {
        Self::new()
    }
}

impl Responses {
    fn ack(
        &mut self,
        request: &Request,
        session: u8,
        offset: u32,
        data: &[u8],
        burst_complete: bool,
    ) {
        self.push(Frame {
            seq: request.seq.wrapping_add(1),
            session,
            opcode: RSP_ACK,
            size: data.len() as u8,
            req_opcode: request.opcode,
            burst_complete,
            offset,
            data: padded(data),
        });
    }

    fn nak(&mut self, request: &Request, error: u8) {
        let mut data = [0u8; MAX_DATA];
        data[0] = error;
        self.push(Frame {
            seq: request.seq.wrapping_add(1),
            session: request.session,
            opcode: RSP_NAK,
            size: 1,
            req_opcode: request.opcode,
            burst_complete: false,
            offset: 0,
            data,
        });
    }
}

fn padded(data: &[u8]) -> [u8; MAX_DATA] {
    let mut out = [0u8; MAX_DATA];
    out[..data.len()].copy_from_slice(data);
    out
}

fn chunk_size(requested: u8) -> usize {
    match usize::from(requested) {
        0 => MAX_DATA,
        size => size.min(MAX_DATA),
    }
}

fn parse(payload: &[u8; PAYLOAD_LEN]) -> Request {
    Request {
        seq: u16::from_le_bytes([payload[0], payload[1]]),
        session: payload[2],
        opcode: payload[3],
        size: payload[4],
        offset: u32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]),
    }
}

fn path(payload: &[u8; PAYLOAD_LEN], size: usize) -> Option<&str> {
    if size > MAX_DATA {
        return None;
    }
    let bytes = &payload[HEADER_LEN..HEADER_LEN + size];
    let end = bytes.iter().position(|&byte| byte == 0).unwrap_or(size);
    core::str::from_utf8(&bytes[..end]).ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::component_metadata;
    use crate::component_metadata::PARAMETERS_JSON;

    fn request(seq: u16, session: u8, opcode: u8, offset: u32, data: &[u8]) -> [u8; PAYLOAD_LEN] {
        let mut payload = [0u8; PAYLOAD_LEN];
        payload[..2].copy_from_slice(&seq.to_le_bytes());
        payload[2] = session;
        payload[3] = opcode;
        payload[4] = data.len() as u8;
        payload[8..12].copy_from_slice(&offset.to_le_bytes());
        payload[HEADER_LEN..HEADER_LEN + data.len()].copy_from_slice(data);
        payload
    }

    fn open(server: &mut Server, path: &str) -> Response {
        let mut responses = Responses::new();
        server.handle(
            &request(1, 0, OP_OPEN_FILE_RO, 0, path.as_bytes()),
            &mut responses,
        );
        Response::from(responses.frames())
    }

    struct Response {
        frames: [Frame; BURST_FRAMES],
        len: usize,
    }

    impl Response {
        fn from(frames: &[Frame]) -> Self {
            let mut copy = [EMPTY_FRAME; BURST_FRAMES];
            copy[..frames.len()].copy_from_slice(frames);
            Self {
                frames: copy,
                len: frames.len(),
            }
        }

        fn one(&self) -> &Frame {
            assert_eq!(self.len, 1);
            &self.frames[0]
        }
    }

    #[test]
    fn unknown_paths_are_reported_as_not_found() {
        let mut server = Server::new();
        let response = open(&mut server, "/component_metadata/missing.json");
        let frame = response.one();
        assert_eq!(frame.opcode, RSP_NAK);
        assert_eq!(frame.size, 1);
        assert_eq!(frame.data[0], ERR_FILE_NOT_FOUND);
        assert_eq!(frame.seq, 2);
        assert_eq!(frame.req_opcode, OP_OPEN_FILE_RO);
    }

    #[test]
    fn open_reports_the_file_length_and_session() {
        let mut server = Server::new();
        let response = open(&mut server, "//component_metadata/general.json");
        let frame = response.one();
        assert_eq!(frame.opcode, RSP_ACK);
        assert_eq!(frame.size, 4);
        assert_ne!(frame.session, 0);
        let length = u32::from_le_bytes(frame.data[..4].try_into().unwrap());
        assert_eq!(length as usize, component_metadata::GENERAL_JSON.len());
    }

    #[test]
    fn read_returns_data_then_eof() {
        let mut server = Server::new();
        let session = open(&mut server, "parameters.json").one().session;
        let size = PARAMETERS_JSON.len();

        let mut responses = Responses::new();
        server.handle(&request(3, session, OP_READ_FILE, 0, &[]), &mut responses);
        let first = responses.frames()[0];
        assert_eq!(first.opcode, RSP_ACK);
        assert_eq!(first.size as usize, MAX_DATA.min(size));
        assert_eq!(first.offset, 0);
        assert_eq!(
            &first.data[..first.size as usize],
            &PARAMETERS_JSON.as_bytes()[..first.size as usize]
        );

        let mut responses = Responses::new();
        server.handle(
            &request(5, session, OP_READ_FILE, size as u32, &[]),
            &mut responses,
        );
        let last = responses.frames()[0];
        assert_eq!(last.opcode, RSP_NAK);
        assert_eq!(last.data[0], ERR_EOF);
        assert_eq!(last.session, session);
    }

    #[test]
    fn burst_download_reassembles_the_parameter_file() {
        let mut server = Server::new();
        let session = open(&mut server, "/component_metadata/parameters.json")
            .one()
            .session;
        let mut collected = std::vec::Vec::new();
        let mut seq = 3u16;

        loop {
            let mut responses = Responses::new();
            server.handle(
                &request(
                    seq,
                    session,
                    OP_BURST_READ_FILE,
                    collected.len() as u32,
                    &[],
                ),
                &mut responses,
            );
            let frames = responses.frames();
            assert!(!frames.is_empty());
            let last = *frames.last().unwrap();
            if last.opcode == RSP_NAK {
                assert_eq!(last.data[0], ERR_EOF);
                break;
            }
            for (index, frame) in frames.iter().enumerate() {
                assert_eq!(frame.opcode, RSP_ACK);
                assert_eq!(frame.seq, seq.wrapping_add(1 + index as u16));
                assert_eq!(frame.session, session);
                assert_eq!(frame.offset as usize, collected.len());
                assert_eq!(frame.burst_complete, index + 1 == frames.len());
                collected.extend_from_slice(&frame.data[..usize::from(frame.size)]);
            }
            seq = last.seq.wrapping_add(1);
        }

        assert_eq!(collected, PARAMETERS_JSON.as_bytes());
    }

    #[test]
    fn sessions_are_validated_and_can_be_reset() {
        let mut server = Server::new();
        let session = open(&mut server, "general.json").one().session;

        let mut responses = Responses::new();
        server.handle(
            &request(3, session.wrapping_add(1), OP_READ_FILE, 0, &[]),
            &mut responses,
        );
        assert_eq!(responses.frames()[0].data[0], ERR_INVALID_SESSION);

        let mut responses = Responses::new();
        server.handle(&request(4, 0, OP_RESET_SESSIONS, 0, &[]), &mut responses);
        assert_eq!(responses.frames()[0].opcode, RSP_ACK);

        let mut responses = Responses::new();
        server.handle(&request(6, session, OP_READ_FILE, 0, &[]), &mut responses);
        assert_eq!(responses.frames()[0].data[0], ERR_INVALID_SESSION);
    }

    #[test]
    fn unsupported_commands_are_rejected() {
        let mut server = Server::new();
        let mut responses = Responses::new();
        server.handle(&request(1, 0, 3, 0, b"/"), &mut responses);
        let frame = responses.frames()[0];
        assert_eq!(frame.opcode, RSP_NAK);
        assert_eq!(frame.data[0], ERR_UNKNOWN_COMMAND);
    }

    #[test]
    fn frames_serialize_into_the_wire_payload() {
        let frame = Frame {
            seq: 0x1234,
            session: 7,
            opcode: RSP_ACK,
            size: 3,
            req_opcode: OP_READ_FILE,
            burst_complete: true,
            offset: 0x0102_0304,
            data: padded(&[9, 8, 7]),
        };
        let mut payload = [0u8; PAYLOAD_LEN];
        frame.encode(&mut payload);
        assert_eq!(&payload[..2], &[0x34, 0x12]);
        assert_eq!(payload[2], 7);
        assert_eq!(payload[3], RSP_ACK);
        assert_eq!(payload[4], 3);
        assert_eq!(payload[5], OP_READ_FILE);
        assert_eq!(payload[6], 1);
        assert_eq!(&payload[8..12], &[0x04, 0x03, 0x02, 0x01]);
        assert_eq!(&payload[HEADER_LEN..HEADER_LEN + 3], &[9, 8, 7]);
    }
}
