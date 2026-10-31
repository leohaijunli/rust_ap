use core::cell::RefCell;
use core::fmt::{self, Write};

use crate::eth_led::Lan8742;
use embassy_stm32::gpio::Output;

pub(crate) fn status(phy: &RefCell<Lan8742<'_>>, eth_power: &Output<'_>) -> Response {
    let mut response = Response::new();
    let pwr = embassy_stm32::pac::PWR.cr3().read();
    let _ = writeln!(
        response,
        "Rust FMUv6X v{} USB=MAVLink2 clock=PLL3Q/48MHz ZLP=on",
        env!("CARGO_PKG_VERSION")
    );
    let _ = writeln!(
        response,
        "USB33RDY={} ETH_POWER={}",
        pwr.usb33rdy(),
        eth_power.is_set_high()
    );
    let _ = phy.borrow_mut().write_diagnostics(&mut response);
    response.finish();
    response
}

pub(crate) fn report(command: Command) -> Response {
    let mut response = Response::new();
    let _ = match command {
        Command::Platform => crate::platform::write_diagnostics(&mut response),
        Command::Metrics => crate::diagnostics::write_diagnostics(&mut response),
        Command::MetricsReset => {
            crate::diagnostics::request_reset();
            writeln!(response, "OK metrics-reset-requested")
        }
        Command::Fault => crate::fault::write_diagnostics(&mut response),
        Command::FaultClear => writeln!(response, "FAULT_CLEAR ok={}", crate::fault::clear()),
        _ => writeln!(response, "ERR unsupported-report"),
    };
    response.finish();
    response
}

pub(crate) enum Command {
    Status,
    Bootloader,
    Platform,
    Metrics,
    MetricsReset,
    Fault,
    FaultClear,
    #[cfg(feature = "fault-injection")]
    Panic,
    #[cfg(feature = "fault-injection")]
    HardFault,
}

pub(crate) struct CommandLine {
    bytes: [u8; 16],
    len: usize,
    overflow: bool,
}

impl CommandLine {
    pub(crate) fn new() -> Self {
        Self {
            bytes: [0; 16],
            len: 0,
            overflow: false,
        }
    }

    pub(crate) fn push(&mut self, byte: u8) -> Option<Command> {
        if byte == b'\n' {
            let line = &self.bytes[..self.len];
            let line = line.strip_suffix(b"\r").unwrap_or(line);
            let command = if self.overflow {
                None
            } else {
                match line {
                    b"!status" => Some(Command::Status),
                    b"!bootloader" => Some(Command::Bootloader),
                    b"!platform" => Some(Command::Platform),
                    b"!metrics" => Some(Command::Metrics),
                    b"!metrics-reset" => Some(Command::MetricsReset),
                    b"!fault" => Some(Command::Fault),
                    b"!fault-clear" => Some(Command::FaultClear),
                    #[cfg(feature = "fault-injection")]
                    b"!panic" => Some(Command::Panic),
                    #[cfg(feature = "fault-injection")]
                    b"!hardfault" => Some(Command::HardFault),
                    _ => None,
                }
            };
            self.len = 0;
            self.overflow = false;
            return command;
        }

        if self.len < self.bytes.len() {
            self.bytes[self.len] = byte;
            self.len += 1;
        } else {
            // Ignore an overlong line until its newline; never match a suffix.
            self.overflow = true;
        }
        None
    }
}

pub(crate) struct Response {
    bytes: [u8; 2048],
    len: usize,
    overflow: bool,
}

impl Response {
    fn new() -> Self {
        Self {
            bytes: [0; 2048],
            len: 0,
            overflow: false,
        }
    }

    pub(crate) fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    fn finish(&mut self) {
        let _ = writeln!(self, "END");
        if self.overflow {
            self.len = 0;
            self.overflow = false;
            let _ = self.write_str("ERR response-overflow\nEND\n");
        }
    }
}

impl Write for Response {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let end = self.len + text.len();
        if end > self.bytes.len() {
            self.overflow = true;
            return Err(fmt::Error);
        }
        self.bytes[self.len..end].copy_from_slice(text.as_bytes());
        self.len = end;
        Ok(())
    }
}
