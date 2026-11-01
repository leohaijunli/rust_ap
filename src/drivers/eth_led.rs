use core::fmt;

use embassy_stm32::gpio::{Output, OutputOpenDrain};

// Preserve the baseline's >=4.6875 us bit phase at the configured CPU clock.
// Long power-up/retry delays use embassy-time, not CPU cycle guesses.
const MDIO_DELAY_CYCLES: u32 = (crate::platform::CPU_HZ as u64 * 300 / 64_000_000) as u32;

const BMCR: u8 = 0x00;
const BMSR: u8 = 0x01;
const PHYID1: u8 = 0x02;
const PHYID2: u8 = 0x03;
const MMD_CONTROL: u8 = 0x0d;
const MMD_DATA: u8 = 0x0e;
const INTERRUPT_MASK: u8 = 0x1e;
const PCS_MMD: u16 = 3;
const WUCSR: u16 = 0x8010;
const LED1_FUNCTION: u16 = 3 << 13;
const LED2_FUNCTION: u16 = 3 << 11;
const WOL_RECEIVED_FLAGS: u16 = 0x00f0;

const BMCR_FULL_DUPLEX: u16 = 1 << 8;
const BMCR_SPEED_100: u16 = 1 << 13;
const LAN8742_ID1: u16 = 0x0007;
const LAN8742_ID2: u16 = 0xc130;

/// Minimal bit-banged MDIO driver for the FMUv6X LAN8742A Ethernet PHY.
///
/// LINK selects deasserted nINT (high) to light the LED, or the inactive
/// link/activity indication (low) to turn it off. SPD retains its native PHY
/// speed indication. PHY power stays on throughout.
pub struct Lan8742<'d> {
    mdio: OutputOpenDrain<'d>,
    mdc: Output<'d>,
    addr: Option<u8>,
    scan_ids: [[u16; 2]; 32],
    led_config: Option<u16>,
    led_state: Option<bool>,
}

impl<'d> Lan8742<'d> {
    pub fn new(mdio: OutputOpenDrain<'d>, mdc: Output<'d>) -> Self {
        Self {
            mdio,
            mdc,
            addr: None,
            scan_ids: [[0xffff; 2]; 32],
            led_config: None,
            led_state: None,
        }
    }

    pub fn init(&mut self) -> bool {
        self.addr = self.scan();
        self.led_config = None;
        self.led_state = None;
        self.addr.is_some()
    }

    pub fn found(&self) -> bool {
        self.addr.is_some()
    }

    /// Report the actual MDIO responses, including unsupported PHY IDs.
    pub fn write_diagnostics(&mut self, out: &mut impl fmt::Write) -> fmt::Result {
        writeln!(
            out,
            "PHY found={} mdio_high={}",
            self.found(),
            self.mdio.is_high()
        )?;
        for (addr, ids) in self.scan_ids.iter().enumerate() {
            if addr < 2 || *ids != [0xffff; 2] {
                writeln!(
                    out,
                    "PHY addr={addr:02} id1={:04x} id2={:04x}",
                    ids[0], ids[1]
                )?;
            }
        }
        if let Some(addr) = self.addr {
            let bmcr = self.read_reg(addr, BMCR);
            // Link status is latched low; a second read gives the current state.
            self.read_reg(addr, BMSR);
            let bmsr = self.read_reg(addr, BMSR);
            writeln!(
                out,
                "LAN8742 addr={addr} BMCR={bmcr:04x} BMSR={bmsr:04x} link={}",
                bmsr & (1 << 2) != 0
            )?;
            let wucsr = self.read_mmd(WUCSR);
            let interrupt_mask = self.read_reg(addr, INTERRUPT_MASK);
            writeln!(
                out,
                "LED ready={} WUCSR={wucsr:04x} IMR={interrupt_mask:04x} high=nINT",
                self.led_control_ready()
            )?;
            if let Some(link) = self.led_state {
                writeln!(out, "LED requested LINK={link} SPD=PHY")?;
            }
        }
        Ok(())
    }

    pub fn led_control_ready(&self) -> bool {
        self.led_config.is_some()
    }

    /// Prepare LINK for the cable-disconnected heartbeat demo.
    pub fn configure_leds(&mut self) -> bool {
        if self.addr.is_none() {
            return false;
        }

        // Keep the PHY active and nINT deasserted for the verified active-high
        // LINK LED wiring. LED2 function 00 retains native speed indication.
        self.write_reg(BMCR, BMCR_FULL_DUPLEX | BMCR_SPEED_100);
        self.write_reg(INTERRUPT_MASK, 0);
        let config = self.read_mmd(WUCSR);
        if config == 0xffff {
            return false;
        }

        // Received-event bits are write-one-to-clear: do not echo them back.
        self.led_config = Some(config & !(LED1_FUNCTION | LED2_FUNCTION | WOL_RECEIVED_FLAGS));
        self.led_state = None;
        if !self.set_link_led(true) {
            self.led_config = None;
            return false;
        }
        true
    }

    /// Set LINK's signal and verify the register write.
    pub fn set_link_led(&mut self, link: bool) -> bool {
        let Some(base) = self.led_config else {
            return false;
        };
        if self.led_state == Some(link) {
            return true;
        }

        // LED1 function 01: nINT high = on. Function 00: no-link low = off.
        let config = base | if link { 1 << 13 } else { 0 };
        self.write_mmd(WUCSR, config);
        let readback = self.read_mmd(WUCSR);
        if readback == 0xffff
            || readback & (LED1_FUNCTION | LED2_FUNCTION)
                != config & (LED1_FUNCTION | LED2_FUNCTION)
        {
            return false;
        }
        self.led_state = Some(link);
        true
    }

    fn select_mmd(&mut self, reg: u16) {
        self.write_reg(MMD_CONTROL, PCS_MMD);
        self.write_reg(MMD_DATA, reg);
        self.write_reg(MMD_CONTROL, PCS_MMD | (1 << 14));
    }

    fn read_mmd(&mut self, reg: u16) -> u16 {
        let Some(addr) = self.addr else {
            return 0xffff;
        };
        self.select_mmd(reg);
        self.read_reg(addr, MMD_DATA)
    }

    fn write_mmd(&mut self, reg: u16, value: u16) {
        self.select_mmd(reg);
        self.write_reg(MMD_DATA, value);
    }

    fn scan(&mut self) -> Option<u8> {
        let mut found = None;
        for addr in 0..32u8 {
            let id1 = self.read_reg(addr, PHYID1);
            let id2 = self.read_reg(addr, PHYID2);
            self.scan_ids[usize::from(addr)] = [id1, id2];
            // The low four PHYID2 bits are the silicon revision.
            if id1 == LAN8742_ID1 && id2 & 0xfff0 == LAN8742_ID2 {
                found = Some(addr);
            }
        }
        found
    }

    fn read_reg(&mut self, addr: u8, reg: u8) -> u16 {
        for _ in 0..32 {
            self.write_bit(true);
        }
        self.write_bit(false);
        self.write_bit(true);
        self.write_bit(true);
        self.write_bit(false);
        self.write_bits(u16::from(addr), 5);
        self.write_bits(u16::from(reg), 5);
        // The PHY advances MDIO after MDC rises. With sampling at the end of
        // the high phase, the first read clock returns the driven TA=0 bit.
        // Discarding another bit here would skip D15 (0007 becomes 000f).
        if self.read_bit() {
            for _ in 0..32 {
                self.read_bit();
            }
            return 0xffff;
        }

        let mut value = 0u16;
        for _ in 0..16 {
            value = (value << 1) | u16::from(self.read_bit());
        }
        self.read_bit(); // Idle clock: let the PHY release MDIO.
        value
    }

    fn write_reg(&mut self, reg: u8, value: u16) {
        let Some(addr) = self.addr else {
            return;
        };

        for _ in 0..32 {
            self.write_bit(true);
        }
        self.write_bit(false);
        self.write_bit(true);
        self.write_bit(false);
        self.write_bit(true);
        self.write_bits(u16::from(addr), 5);
        self.write_bits(u16::from(reg), 5);
        self.write_bit(true);
        self.write_bit(false);
        self.write_bits(value, 16);
        self.write_bit(true);
    }

    fn write_bits(&mut self, value: u16, count: u8) {
        for shift in (0..count).rev() {
            self.write_bit((value >> shift) & 1 != 0);
        }
    }

    fn write_bit(&mut self, bit: bool) {
        if bit {
            self.mdio.set_high();
        } else {
            self.mdio.set_low();
        }
        Self::delay();
        self.mdc.set_high();
        Self::delay();
        self.mdc.set_low();
        Self::delay();
    }

    fn read_bit(&mut self) -> bool {
        self.mdio.set_high();
        Self::delay();
        self.mdc.set_high();
        Self::delay();
        self.mdc.set_low();
        let bit = self.mdio.is_high();
        Self::delay();
        bit
    }

    fn delay() {
        cortex_m::asm::delay(MDIO_DELAY_CYCLES);
    }
}
