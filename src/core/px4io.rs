//! PX4IO v5 packets and the output-disabled FMUv6X bring-up policy.
//!
//! Protocol definitions follow PX4 v1.17.0, commit
//! d6f12ad1c4f70ad3230afd7d86e971421e02fef4:
//! src/modules/px4iofirmware/{protocol.h,registers.c}.
//! No C/C++ runtime or packed-memory casts are used.

pub const PROTOCOL_VERSION: u16 = 5;
pub const MAX_REGISTERS: usize = 32;
pub const MAX_ACTUATORS: usize = 8;
pub const HEADER_LEN: usize = 4;
pub const MAX_PACKET_LEN: usize = HEADER_LEN + MAX_REGISTERS * 2;
pub const STATUS_PERIOD_MS: u64 = 20;
pub const STALE_AFTER_US: u64 = 100_000;

pub mod page {
    pub const CONFIG: u8 = 0;
    pub const STATUS: u8 = 1;
    pub const SERVOS: u8 = 3;
    pub const SETUP: u8 = 50;
    pub const DISARMED_PWM: u8 = 109;
}

pub mod setup {
    pub const FEATURES: u8 = 0;
    pub const ARMING: u8 = 1;
    pub const CRC: u8 = 11;
    pub const SAFETY_BUTTON_ACK: u8 = 14;
    pub const SAFETY_OFF: u8 = 15;
}

pub mod flags {
    pub const OUTPUTS_ARMED: u16 = 1 << 0;
    pub const RC_OK: u16 = 1 << 1;
    pub const FMU_OK: u16 = 1 << 5;
    pub const ARM_SYNC: u16 = 1 << 7;
    pub const INIT_OK: u16 = 1 << 8;
    pub const FAILSAFE: u16 = 1 << 9;
    pub const SAFETY_OFF: u16 = 1 << 10;
    pub const SAFETY_BUTTON_EVENT: u16 = 1 << 14;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PacketError {
    RegisterCount,
    Length,
    Crc,
    CorruptRequest,
    RemoteError,
    ReplyCode,
    Address,
}

pub struct Request {
    bytes: [u8; MAX_PACKET_LEN],
    len: usize,
    reply_count: usize,
}

impl Request {
    pub fn read(page: u8, offset: u8, count: usize) -> Result<Self, PacketError> {
        // PX4 reads transmit count registers too. Their values have no meaning,
        // but are part of the packet CRC; initialize them deterministically.
        Self::new(page, offset, count, false)
    }

    /// Only zero-valued, output-disabling writes and the button ACK are exposed.
    pub fn safe_write(operation: SafeWrite) -> Result<Self, PacketError> {
        let (page, offset, count) = match operation {
            SafeWrite::Disarm => (page::SETUP, setup::ARMING, 1),
            SafeWrite::DisableSbus => (page::SETUP, setup::FEATURES, 1),
            SafeWrite::SafetyOn => (page::SETUP, setup::SAFETY_OFF, 1),
            SafeWrite::AcknowledgeButton => (page::SETUP, setup::SAFETY_BUTTON_ACK, 1),
            SafeWrite::DisableDisarmedPwm(count) => {
                if !(1..=MAX_ACTUATORS).contains(&count) {
                    return Err(PacketError::RegisterCount);
                }
                (page::DISARMED_PWM, 0, count)
            }
        };
        Self::new(page, offset, count, true)
    }

    fn new(page: u8, offset: u8, count: usize, write: bool) -> Result<Self, PacketError> {
        if !(1..=MAX_REGISTERS).contains(&count) {
            return Err(PacketError::RegisterCount);
        }
        let len = HEADER_LEN + count * 2;
        let mut bytes = [0; MAX_PACKET_LEN];
        bytes[0] = count as u8 | if write { 0x40 } else { 0 };
        bytes[2] = page;
        bytes[3] = offset;
        bytes[1] = packet_crc(&bytes[..len]);
        Ok(Self {
            bytes,
            len,
            reply_count: if write { 0 } else { count },
        })
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    pub fn decode_reply<'a>(&self, bytes: &'a [u8]) -> Result<Reply<'a>, PacketError> {
        if bytes.len() < HEADER_LEN {
            return Err(PacketError::Length);
        }
        let count = usize::from(bytes[0] & 0x3f);
        if count > MAX_REGISTERS {
            return Err(PacketError::RegisterCount);
        }
        if bytes.len() != HEADER_LEN + count * 2 {
            return Err(PacketError::Length);
        }
        if bytes[1] != packet_crc(bytes) {
            return Err(PacketError::Crc);
        }
        match bytes[0] & 0xc0 {
            0 => {}
            0x40 => return Err(PacketError::CorruptRequest),
            0x80 => return Err(PacketError::RemoteError),
            _ => return Err(PacketError::ReplyCode),
        }
        if bytes[2..HEADER_LEN] != self.bytes[2..HEADER_LEN] {
            return Err(PacketError::Address);
        }
        if count != self.reply_count {
            return Err(PacketError::RegisterCount);
        }
        Ok(Reply {
            registers: &bytes[HEADER_LEN..],
        })
    }
}

pub struct Reply<'a> {
    registers: &'a [u8],
}

impl Reply<'_> {
    pub fn copy_registers(&self, output: &mut [u16]) -> Result<(), PacketError> {
        if self.registers.len() != output.len() * 2 {
            return Err(PacketError::RegisterCount);
        }
        for (dest, &bytes) in output.iter_mut().zip(self.registers.as_chunks::<2>().0) {
            *dest = u16::from_le_bytes(bytes);
        }
        Ok(())
    }
}

/// CRC-8/0x07, initial zero; the CRC byte itself is zero during calculation.
fn packet_crc(bytes: &[u8]) -> u8 {
    let mut crc = 0u8;
    for (index, &byte) in bytes.iter().enumerate() {
        crc ^= if index == 1 { 0 } else { byte };
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SafeWrite {
    Disarm,
    DisableSbus,
    SafetyOn,
    AcknowledgeButton,
    DisableDisarmedPwm(usize),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Configuration {
    pub protocol: u16,
    pub hardware: u16,
    pub bootloader: u16,
    pub max_transfer: u16,
    pub controls: u16,
    pub actuators: u16,
    pub rc_inputs: u16,
    pub adc_inputs: u16,
}

impl Configuration {
    pub const fn from_registers(regs: [u16; 8]) -> Self {
        Self {
            protocol: regs[0],
            hardware: regs[1],
            bootloader: regs[2],
            max_transfer: regs[3],
            controls: regs[4],
            actuators: regs[5],
            rc_inputs: regs[6],
            adc_inputs: regs[7],
        }
    }

    pub fn is_supported(self) -> bool {
        self.protocol == PROTOCOL_VERSION
            && self.hardware == 2
            && (18..=64).contains(&self.max_transfer)
            && (1..=8).contains(&self.controls)
            && (1..=8).contains(&self.actuators)
            && (1..=32).contains(&self.rc_inputs)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Status {
    pub timestamp_us: u64,
    pub flags: u16,
    pub alarms: u16,
    pub features: u16,
    pub arming: u16,
    pub servo_mv: u16,
    pub rssi_mv: u16,
    pub pwm: [u16; MAX_ACTUATORS],
    pub disarmed_pwm: [u16; MAX_ACTUATORS],
}

impl Status {
    /// OUTPUTS_ARMED describes IO timer enable, not aircraft arming. A retained
    /// RAW_PWM flag can keep that bit set even when all pulse widths are zero.
    pub fn outputs_inhibited(self) -> bool {
        self.arming == 0
            && self.features == 0
            && self.flags & flags::SAFETY_OFF == 0
            && self
                .pwm
                .iter()
                .chain(&self.disarmed_pwm)
                .all(|&value| value == 0)
    }

    pub fn is_fresh(self, now_us: u64) -> bool {
        now_us >= self.timestamp_us && now_us - self.timestamp_us <= STALE_AFTER_US
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reply(page: u8, offset: u8, code: u8, registers: &[u16]) -> std::vec::Vec<u8> {
        let mut bytes = std::vec![registers.len() as u8 | code, 0, page, offset];
        for value in registers {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        bytes[1] = packet_crc(&bytes);
        bytes
    }

    #[test]
    fn bytes_match_unmodified_px4_crc_packet_reference() {
        // Generated by tools/px4io_reference.cpp against the fixed upstream
        // protocol.h, rather than by this Rust encoder's own CRC function.
        assert_eq!(
            Request::read(page::CONFIG, 0, 8).unwrap().as_bytes(),
            &[8, 0x67, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        );
        assert_eq!(
            Request::safe_write(SafeWrite::Disarm).unwrap().as_bytes(),
            &[0x41, 0xb1, 50, 1, 0, 0]
        );
        assert_eq!(
            Request::safe_write(SafeWrite::SafetyOn).unwrap().as_bytes(),
            &[0x41, 0x9d, 50, 15, 0, 0]
        );
        assert_eq!(
            Request::safe_write(SafeWrite::AcknowledgeButton)
                .unwrap()
                .as_bytes(),
            &[0x41, 0xf6, 50, 14, 0, 0]
        );
        let mut max_read = [0; MAX_PACKET_LEN];
        max_read[0] = 32;
        max_read[1] = 0xc4;
        assert_eq!(
            Request::read(page::CONFIG, 0, 32).unwrap().as_bytes(),
            max_read
        );
        let mut disable_pwm = [0; 20];
        disable_pwm[..4].copy_from_slice(&[0x48, 0x6e, 109, 0]);
        assert_eq!(
            Request::safe_write(SafeWrite::DisableDisarmedPwm(8))
                .unwrap()
                .as_bytes(),
            disable_pwm
        );

        let mut config = [0; 8];
        Request::read(page::CONFIG, 0, 8)
            .unwrap()
            .decode_reply(&[
                8, 0xb7, 0, 0, 5, 0, 2, 0, 3, 0, 64, 0, 8, 0, 8, 0, 18, 0, 2, 0,
            ])
            .unwrap()
            .copy_registers(&mut config)
            .unwrap();
        assert_eq!(config, [5, 2, 3, 64, 8, 8, 18, 2]);
        Request::safe_write(SafeWrite::AcknowledgeButton)
            .unwrap()
            .decode_reply(&[0, 0xf9, 50, 14])
            .unwrap();
    }

    #[test]
    fn packet_counts_and_read_padding_are_bounded() {
        for count in [0, MAX_REGISTERS + 1, usize::MAX] {
            assert!(matches!(
                Request::read(0, 0, count),
                Err(PacketError::RegisterCount)
            ));
        }
        let request = Request::read(page::CONFIG, 0, MAX_REGISTERS).unwrap();
        assert_eq!(request.as_bytes().len(), MAX_PACKET_LEN);
        assert!(request.as_bytes()[HEADER_LEN..]
            .iter()
            .all(|&byte| byte == 0));
    }

    #[test]
    fn reads_and_zero_count_write_ack_match_px4() {
        let request = Request::read(page::CONFIG, 0, 2).unwrap();
        let bytes = reply(page::CONFIG, 0, 0, &[5, 2]);
        let mut registers = [0; 2];
        request
            .decode_reply(&bytes)
            .unwrap()
            .copy_registers(&mut registers)
            .unwrap();
        assert_eq!(registers, [5, 2]);

        let request = Request::safe_write(SafeWrite::AcknowledgeButton).unwrap();
        let bytes = reply(page::SETUP, setup::SAFETY_BUTTON_ACK, 0, &[]);
        request
            .decode_reply(&bytes)
            .unwrap()
            .copy_registers(&mut [])
            .unwrap();
    }

    #[test]
    fn malformed_corrupt_wrong_address_and_remote_errors_are_rejected() {
        let request = Request::read(page::CONFIG, 0, 2).unwrap();
        let mut bytes = reply(page::CONFIG, 0, 0, &[5, 2]);
        bytes[4] ^= 1;
        assert!(matches!(
            request.decode_reply(&bytes),
            Err(PacketError::Crc)
        ));
        bytes = reply(page::CONFIG, 0, 0, &[5, 2]);
        assert!(matches!(
            request.decode_reply(&bytes[..7]),
            Err(PacketError::Length)
        ));
        assert!(matches!(
            request.decode_reply(&reply(1, 0, 0, &[5, 2])),
            Err(PacketError::Address)
        ));
        assert!(matches!(
            request.decode_reply(&reply(0, 0, 0x40, &[])),
            Err(PacketError::CorruptRequest)
        ));
        assert!(matches!(
            request.decode_reply(&reply(0, 0, 0x80, &[])),
            Err(PacketError::RemoteError)
        ));
        assert!(matches!(
            request.decode_reply(&reply(0, 0, 0, &[5])),
            Err(PacketError::RegisterCount)
        ));
        assert!(matches!(
            request.decode_reply(&[63, 0, 0, 0]),
            Err(PacketError::RegisterCount)
        ));
    }

    #[test]
    fn exposed_writes_cannot_arm_enable_sbus_or_send_nonzero_pwm() {
        for operation in [
            SafeWrite::Disarm,
            SafeWrite::DisableSbus,
            SafeWrite::SafetyOn,
            SafeWrite::AcknowledgeButton,
            SafeWrite::DisableDisarmedPwm(MAX_ACTUATORS),
        ] {
            let request = Request::safe_write(operation).unwrap();
            let bytes = request.as_bytes();
            assert_eq!(bytes[0] & 0xc0, 0x40);
            assert!(bytes[HEADER_LEN..].iter().all(|&byte| byte == 0));
            assert!(bytes[2] == page::SETUP || bytes[2] == page::DISARMED_PWM);
            assert_ne!(bytes[3], 10); // No reboot-to-bootloader operation.
        }
        assert!(Request::safe_write(SafeWrite::DisableDisarmedPwm(9)).is_err());
    }

    #[test]
    fn unknown_configuration_and_stale_or_active_outputs_fail_closed() {
        let mut config = Configuration::from_registers([5, 2, 3, 64, 8, 8, 18, 2]);
        assert!(config.is_supported());
        config.protocol = 4;
        assert!(!config.is_supported());
        config.protocol = 5;
        config.actuators = 9;
        assert!(!config.is_supported());

        let mut status = Status {
            timestamp_us: 100,
            flags: flags::OUTPUTS_ARMED,
            alarms: 0,
            features: 0,
            arming: 0,
            servo_mv: 0,
            rssi_mv: 0,
            pwm: [0; MAX_ACTUATORS],
            disarmed_pwm: [0; MAX_ACTUATORS],
        };
        assert!(status.outputs_inhibited());
        assert!(status.is_fresh(100));
        assert!(!status.is_fresh(99));
        assert!(!status.is_fresh(STALE_AFTER_US + 101));
        status.arming = 2;
        assert!(!status.outputs_inhibited());
        status.arming = 0;
        status.pwm[0] = 1_000;
        assert!(!status.outputs_inhibited());
        status.pwm[0] = 0;
        status.flags |= flags::SAFETY_OFF;
        assert!(!status.outputs_inhibited());
    }
}
