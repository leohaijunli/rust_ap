//! Preserve the RTC configuration inherited from PX4 across Embassy init.
//!
//! embassy-stm32 0.5.0 resets the backup domain when LsConfig differs from
//! BDCR, then the generated ClockMux writes RTCSEL again. Both must match.
//! References and supported retained states: docs/fmuv6x-hardware.md.

use core::cell::Cell;
use core::fmt;

use cortex_m::interrupt::{free, Mutex};
use embassy_stm32::pac::rcc::regs::Bdcr;
use embassy_stm32::pac::rcc::vals::Lsedrv;
use embassy_stm32::pac::{RCC, RTC};
use embassy_stm32::rcc::{LsConfig, LseConfig, LseDrive, LseMode, RtcClockSource};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, Peripherals};

// H753 BDCR configuration bits: LSEON/BYP/DRV/CSSON, RTCSEL/EN, BDRST.
// LSERDY and LSECSSD are live status, not configuration to restore.
const CONFIG_MASK: u32 = 0x0001_833d;

#[derive(Clone, Copy)]
struct Status {
    bdcr_before: u32,
    bdcr_after: u32,
    csr_before: u32,
    csr_after: u32,
    bkp_preserved: bool,
}

impl Status {
    fn config_preserved(self) -> bool {
        (self.bdcr_before ^ self.bdcr_after) & CONFIG_MASK == 0
    }
}

static STATUS: Mutex<Cell<Option<Status>>> = Mutex::new(Cell::new(None));

fn configure(config: &mut Config, bdcr: Bdcr) {
    let rtc = bdcr.rtcsel();
    // These states cannot be represented by this fixed HAL without a reset
    // or its HSE-RTC todo!(). Reject BEFORE calling it, leaving BDCR intact.
    assert!(!bdcr.bdrst(), "backup domain held in reset");
    assert!(rtc != RtcClockSource::HSE, "HAL cannot preserve HSE RTC");
    assert_eq!(
        bdcr.rtcen(),
        rtc != RtcClockSource::DISABLE,
        "HAL cannot preserve this RTCSEL/RTCEN combination"
    );
    assert!(
        bdcr.lseon() || (!bdcr.lsebyp() && rtc != RtcClockSource::LSE),
        "HAL cannot preserve this stopped LSE configuration"
    );

    let mut ls = LsConfig::off();
    ls.rtc = rtc;
    // The HAL does not disable LSI when false. Enable it only when selected
    // by the retained RTC; an unrelated running LSI is left alone.
    ls.lsi = rtc == RtcClockSource::LSI;
    if bdcr.lseon() {
        let mode = if bdcr.lsebyp() {
            LseMode::Bypass
        } else {
            // Match the selected H753 PAC, whose two medium encodings differ
            // from the generic H7 PAC; HAL converts these variants back to it.
            let drive = match bdcr.lsedrv() {
                Lsedrv::LOW => LseDrive::Low,
                Lsedrv::MEDIUM_LOW => LseDrive::MediumLow,
                Lsedrv::MEDIUM_HIGH => LseDrive::MediumHigh,
                Lsedrv::HIGH => LseDrive::High,
            };
            LseMode::Oscillator(drive)
        };
        ls.lse = Some(LseConfig {
            frequency: Hertz(32_768), // fixed FMUv6X board.h, not a measurement
            mode,
        });
    }
    config.rcc.ls = ls;
    config.rcc.mux.rtcsel = rtc;
}

fn backup_words() -> [u32; 32] {
    let mut words = [0; 32];
    for (index, word) in words.iter_mut().enumerate() {
        *word = RTC.bkpr(index).read().bkp();
    }
    words
}

// A single audited startup boundary shared by normal and diagnostic images.
#[inline(never)]
pub fn init(mut config: Config) -> Peripherals {
    RCC.apb4enr().modify(|w| w.set_rtcapben(true));
    let _ = RCC.apb4enr().read();
    let bdcr = RCC.bdcr().read();
    let csr = RCC.csr().read().0;
    let words = backup_words();
    configure(&mut config, bdcr);

    let p = embassy_stm32::init(config);
    let status = Status {
        bdcr_before: bdcr.0,
        bdcr_after: RCC.bdcr().read().0,
        csr_before: csr,
        csr_after: RCC.csr().read().0,
        bkp_preserved: words == backup_words(),
    };
    // Do not repair BKP words after a reset and report that as preservation:
    // it would hide a lost RTC calendar/configuration. Dump can report failure.
    free(|cs| STATUS.borrow(cs).set(Some(status)));
    p
}

#[inline(never)]
pub fn preserved() -> bool {
    free(|cs| {
        STATUS
            .borrow(cs)
            .get()
            .is_some_and(|status| status.config_preserved() && status.bkp_preserved)
    })
}

pub fn write_diagnostics(out: &mut impl fmt::Write, prefix: &str) -> fmt::Result {
    if let Some(status) = free(|cs| STATUS.borrow(cs).get()) {
        writeln!(
            out,
            "{prefix} bdcr_before={:08x} bdcr_after={:08x} csr_before={:08x} csr_after={:08x} config_preserved={} bkp_preserved={}",
            status.bdcr_before,
            status.bdcr_after,
            status.csr_before,
            status.csr_after,
            status.config_preserved(),
            status.bkp_preserved
        )
    } else {
        writeln!(out, "{prefix} unavailable=true")
    }
}
