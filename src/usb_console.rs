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
    let _ = writeln!(response, "END");
    response
}

pub(crate) fn reboot_to_bootloader() -> ! {
    use embassy_stm32::pac::{PWR, RCC, RTC};

    // PX4's FMUv6X bootloader checks RTC backup register 0 for this signature.
    RCC.apb4enr().modify(|w| w.set_rtcapben(true));
    PWR.cr1().modify(|w| w.set_dbp(true));
    while !PWR.cr1().read().dbp() {}
    RTC.bkpr(0).write(|w| w.set_bkp(0xb007_b007));
    cortex_m::asm::dsb();
    cortex_m::peripheral::SCB::sys_reset();
}

pub(crate) enum Command {
    Status,
    Bootloader,
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
    bytes: [u8; 1536],
    len: usize,
}

impl Response {
    fn new() -> Self {
        Self {
            bytes: [0; 1536],
            len: 0,
        }
    }

    pub(crate) fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

impl Write for Response {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let end = self.len + text.len();
        if end > self.bytes.len() {
            return Err(fmt::Error);
        }
        self.bytes[self.len..end].copy_from_slice(text.as_bytes());
        self.len = end;
        Ok(())
    }
}
