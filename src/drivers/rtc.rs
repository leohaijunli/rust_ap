//! RTC calendar on the raw STM32H753 registers.
//!
//! `embassy_stm32::rtc::Rtc::new` calls `enable_and_reset`, which resets the
//! whole backup domain and would clear the PX4 backup words that
//! `backup_domain` preserves. This driver therefore touches only the RTC/BDCR
//! registers, keeps the inherited clock source, and never asserts BDRST.
//!
//! BDCR reporting LSEON/LSERDY/RTCSEL=LSE/RTCEN is not proof that RTCCLK
//! reaches the RTC: a stalled crystal leaves those bits set while the calendar
//! freezes. `init` therefore validates both the sub-second counter and calendar
//! progression, and only then falls back to a stronger LSE drive and LSI.
//!
//! The calendar is treated as "not set" until it reads at least 2020-01-01
//! (`wall_clock::MIN_PLAUSIBLE_UNIX`); a power-on reset leaves 2000-01-01.

use core::cell::Cell;
use core::fmt;

use cortex_m::interrupt::{free, Mutex};
use embassy_stm32::pac::rcc::vals::{Lsedrv, Rtcsel};
use embassy_stm32::pac::rtc::vals::Ampm;
use embassy_stm32::pac::{PWR, RCC, RTC};
use embassy_time::{Duration, Instant};
use v6x_rust_led::wall_clock::{Utc, MAX_RTC_UNIX, MIN_PLAUSIBLE_UNIX};

const LSE_HZ: u32 = 32_768;
const LSI_HZ: u32 = 32_000;
const ASYNC_DIV: u32 = 128;
const LSE_START_TIMEOUT: Duration = Duration::from_millis(2_000);
const LSE_STOP_TIMEOUT: Duration = Duration::from_millis(200);
const LSI_START_TIMEOUT: Duration = Duration::from_millis(100);
/// Clearing a stale INIT takes two RTCCLK cycles (~61 us).
const INIT_RELEASE_TIMEOUT: Duration = Duration::from_millis(10);
/// RTCCLK is 32.768 kHz, so two cycles are ~61 us; allow a wide margin.
const SYNC_SPINS: u32 = 20_000;
/// Both LSE/128 and LSI/128 advance the sub-second counter every ~4 ms.
const TICK_STEP: Duration = Duration::from_millis(4);
const TICK_SAMPLES: u32 = 4;
/// A 1 Hz calendar must show a new second well inside this window.
const COUNT_WINDOW: Duration = Duration::from_millis(1_500);
/// Bounded read-back wait after a calendar write.
const VERIFY_WINDOW: Duration = Duration::from_millis(200);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Source {
    Lse,
    Lsi,
    None,
}

impl Source {
    fn as_str(self) -> &'static str {
        match self {
            Self::Lse => "lse",
            Self::Lsi => "lsi",
            Self::None => "none",
        }
    }

    /// Asynchronous/synchronous prescaler pair producing a 1 Hz calendar.
    fn prescaler(self) -> (u8, u16) {
        let hz = match self {
            Self::Lsi => LSI_HZ,
            Self::Lse | Self::None => LSE_HZ,
        };
        // async = 128, sync = hz / 128: 256 for LSE, 250 for LSI.
        let sync = u16::try_from(hz / ASYNC_DIV - 1).unwrap_or(u16::MAX);
        (u8::try_from(ASYNC_DIV - 1).unwrap_or(127), sync)
    }
}

/// Raw snapshot of the calendar registers, printed once per `!time`.
///
/// `isr` carries INIT/INITF/RSF: a stuck INIT freezes the calendar while the
/// sub-second counter keeps running, which is exactly the symptom that made
/// the earlier BDCR-only diagnosis misleading.
#[derive(Clone, Copy)]
struct RawRegisters {
    isr: u32,
    prer: u32,
    tr: u32,
    dr: u32,
    ssr: u32,
}

impl RawRegisters {
    fn read() -> Self {
        // Reading SSR or TR locks the calendar shadow registers until DR is
        // read. Keep DR last so diagnostics never leave the calendar locked.
        let ssr = RTC.ssr().read().0;
        let tr = RTC.tr().read().0;
        let dr = RTC.dr().read().0;
        Self {
            isr: RTC.isr().read().0,
            prer: RTC.prer().read().0,
            tr,
            dr,
            ssr,
        }
    }

    fn init(self) -> bool {
        self.isr & (1 << 7) != 0
    }
}

impl fmt::Display for RawRegisters {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "isr={:08x} prer={:08x} tr={:08x} dr={:08x} ssr={:08x}",
            self.isr, self.prer, self.tr, self.dr, self.ssr
        )
    }
}

#[derive(Clone, Copy)]
struct Status {
    bdcr_before: u32,
    bdcr_after: u32,
    source: Source,
    /// How the source was chosen: `inherited`, `inherited-frozen`,
    /// `lse-high`, `lsi`, or `none`.
    probe: &'static str,
    prescaler_ok: bool,
    /// A recovery path changed or restarted the RTC clock source.
    recovered: bool,
    /// INIT was still set from an earlier boot when this one started.
    stale_init: bool,
}

static STATUS: Mutex<Cell<Option<Status>>> = Mutex::new(Cell::new(None));

#[inline(never)]
pub fn init() {
    unlock_backup_domain();
    RCC.apb4enr().modify(|w| w.set_rtcapben(true));
    let _ = RCC.apb4enr().read();
    let bdcr_before = RCC.bdcr().read().0;
    // INIT lives in the RTC domain and survives an MCU reset: a write attempt
    // that failed while RTCCLK was dead leaves the calendar frozen forever,
    // with the sub-second counter still running. Release it first.
    let stale_init = RawRegisters::read().init();
    unlock_rtc_registers();
    release_init_mode();
    lock_rtc_registers();

    let inherited = source();
    let mut prescaler_ok = inherited != Source::None && configure_prescaler(inherited);
    let mut probe = "inherited";
    let mut recovered = false;

    // `ticking` only proves RTCCLK flows; `counting` proves the calendar
    // itself advances. Both are needed: SSR keeps counting while TR/DR stay
    // frozen when the shadow registers are never refreshed.
    if !(prescaler_ok && ticking() && counting()) {
        if let Some(name) = recover() {
            prescaler_ok = true;
            probe = name;
            recovered = true;
        } else if now().and_then(Utc::plausible_unix).is_some() {
            // Recovery failed but the calendar carries a usable date: report
            // it instead of overwriting it with an implausible one.
            probe = "inherited-frozen";
        } else {
            probe = "none";
            prescaler_ok = false;
        }
    }

    let status = Status {
        bdcr_before,
        bdcr_after: RCC.bdcr().read().0,
        source: source(),
        probe,
        prescaler_ok,
        recovered,
        stale_init,
    };
    free(|cs| STATUS.borrow(cs).set(Some(status)));
}

/// Current calendar, or `None` while the RTC is stopped/invalid.
pub fn now() -> Option<Utc> {
    free(|_| read_calendar())
}

/// Raw UNIX seconds of the calendar, including pre-2020 power-on dates.
pub fn now_unix() -> Option<u64> {
    now().map(Utc::unix)
}

/// UNIX seconds only when the calendar is plausibly set (>= 2020).
pub fn valid_unix() -> Option<u64> {
    now().and_then(Utc::plausible_unix)
}

/// Write an absolute time. Returns whether the read-back agrees within 1 s.
pub fn set_unix_seconds(seconds: u64) -> bool {
    let Some(utc) = Utc::from_unix(seconds).filter(|utc| utc.is_rtc_range()) else {
        return false;
    };
    let written = free(|_| {
        unlock_backup_domain();
        write_calendar(utc)
    });
    if !written {
        return false;
    }
    // The shadow refresh can land on the next calendar tick, so poll briefly
    // outside the interrupt-free section instead of failing on a stale read.
    let start = Instant::now();
    while start.elapsed() < VERIFY_WINDOW {
        if now_unix().is_some_and(|read| read.abs_diff(seconds) <= 1) {
            return true;
        }
        wait_for(Duration::from_millis(20));
    }
    false
}

/// Apply an inbound `SYSTEM_TIME.time_unix_usec`.
///
/// Values that do not look like a UNIX time after 2020 are ignored, and a
/// write is skipped when the RTC already agrees within one second so a
/// once-per-second telemetry stream does not hammer the calendar registers.
pub fn set_unix_usec(usec: u64) -> bool {
    let seconds = usec / 1_000_000;
    if !(MIN_PLAUSIBLE_UNIX..=MAX_RTC_UNIX).contains(&seconds) {
        return false;
    }
    if now_unix().is_some_and(|current| current.abs_diff(seconds) <= 1) {
        return true;
    }
    set_unix_seconds(seconds)
}

pub fn write_diagnostics(out: &mut impl fmt::Write, prefix: &str) -> fmt::Result {
    let Some(status) = free(|cs| STATUS.borrow(cs).get()) else {
        return writeln!(out, "{prefix} unavailable=true");
    };
    let ticking = status.source != Source::None && ticking();
    let counting = ticking && counting();
    let regs = RawRegisters::read();
    match now() {
        Some(calendar) => writeln!(
            out,
            "{prefix} schema=3 source={} ticking={} counting={} probe={} prescaler_ok={} recovered={} stale_init={} calendar={calendar} unix={} valid={} bdcr_before={:08x} bdcr_after={:08x} {regs}",
            status.source.as_str(),
            ticking,
            counting,
            status.probe,
            status.prescaler_ok,
            status.recovered,
            status.stale_init,
            calendar.unix(),
            calendar.plausible_unix().is_some(),
            status.bdcr_before,
            status.bdcr_after
        ),
        None => writeln!(
            out,
            "{prefix} schema=3 source={} ticking={} counting={} probe={} prescaler_ok={} recovered={} stale_init={} calendar=invalid unix=0 valid=false bdcr_before={:08x} bdcr_after={:08x} {regs}",
            status.source.as_str(),
            ticking,
            counting,
            status.probe,
            status.prescaler_ok,
            status.recovered,
            status.stale_init,
            status.bdcr_before,
            status.bdcr_after
        ),
    }
}

/// Repair an unusable clock: retry LSE at maximum drive, then fall back to LSI.
///
/// Returns the probe label of the source that produced an advancing calendar.
fn recover() -> Option<&'static str> {
    if restart_lse(Lsedrv::HIGH) {
        select_clock(Rtcsel::LSE);
        if activate(Source::Lse) {
            return Some("lse-high");
        }
    }
    if start_lsi() {
        select_clock(Rtcsel::LSI);
        if activate(Source::Lsi) {
            // LSI is the final source, so stop a crystal that cannot drive the
            // clock: a stopped LSE would otherwise stall the next boot's HAL
            // wait for LSERDY.
            RCC.bdcr().modify(|w| w.set_lseon(false));
            return Some("lsi");
        }
    }
    None
}

/// Configure the prescaler and prove both RTCCLK and the calendar advance.
fn activate(source: Source) -> bool {
    configure_prescaler(source)
        && (now().is_some()
            || write_calendar(Utc::new(2000, 1, 1, 0, 0, 0).expect("epoch is valid")))
        && ticking()
        && counting()
}

fn source() -> Source {
    let bdcr = RCC.bdcr().read();
    if !bdcr.rtcen() {
        return Source::None;
    }
    match bdcr.rtcsel() {
        Rtcsel::LSE => Source::Lse,
        Rtcsel::LSI => Source::Lsi,
        _ => Source::None,
    }
}

/// Restart LSE with an explicit drive strength (LSEDRV is writable only while
/// LSEON is clear).
fn restart_lse(drive: Lsedrv) -> bool {
    RCC.bdcr().modify(|w| w.set_lseon(false));
    let _ = wait_flag(|| !RCC.bdcr().read().lserdy(), LSE_STOP_TIMEOUT);
    RCC.bdcr().modify(|w| {
        w.set_lsedrv(drive);
        w.set_lseon(true);
    });
    wait_flag(|| RCC.bdcr().read().lserdy(), LSE_START_TIMEOUT)
}

fn start_lsi() -> bool {
    RCC.csr().modify(|w| w.set_lsion(true));
    wait_flag(|| RCC.csr().read().lsirdy(), LSI_START_TIMEOUT)
}

fn select_clock(clock: Rtcsel) {
    // RTCSEL is only writable while the RTC is disabled, so write RTCEN, RTCSEL
    // and RTCEN again as three separate stores. The backup domain is never
    // reset here, so PX4's backup words survive.
    RCC.bdcr().modify(|w| w.set_rtcen(false));
    RCC.bdcr().modify(|w| w.set_rtcsel(clock));
    RCC.bdcr().modify(|w| w.set_rtcen(true));
}

fn configure_prescaler(source: Source) -> bool {
    let (async_psc, sync_psc) = source.prescaler();
    let current = RTC.prer().read();
    if current.prediv_a() == async_psc && current.prediv_s() == sync_psc {
        return true;
    }
    unlock_rtc_registers();
    let written = if enter_init() {
        RTC.prer().write(|w| {
            w.set_prediv_a(async_psc);
            w.set_prediv_s(sync_psc);
        });
        exit_init();
        true
    } else {
        // Leave init mode even if INITF never appeared, so a failed write does
        // not freeze the calendar.
        exit_init();
        false
    };
    lock_rtc_registers();
    let current = RTC.prer().read();
    written && current.prediv_a() == async_psc && current.prediv_s() == sync_psc
}

/// Prove RTCCLK reaches the RTC by watching the sub-second counter.
fn ticking() -> bool {
    if !wait_synchronized() {
        return false;
    }
    let mut last = read_subsecond();
    for _ in 0..TICK_SAMPLES {
        wait_for(TICK_STEP);
        let current = read_subsecond();
        if current != last {
            return true;
        }
        last = current;
    }
    false
}

/// Read SSR without leaving the calendar shadow registers locked.
fn read_subsecond() -> u16 {
    let subsecond = RTC.ssr().read().ss();
    let _ = RTC.tr().read();
    let _ = RTC.dr().read();
    subsecond
}

fn read_calendar() -> Option<Utc> {
    if source() == Source::None || !refresh_shadow() {
        return None;
    }
    // TR/DR are shadow registers that only refresh after RSF is cleared.
    // Reading them without clearing RSF returns the value frozen at the last
    // refresh, which looks exactly like a stopped calendar.
    // SSR/TR shadow reads are coherent because hardware locks the higher-order
    // calendar values until DR is read. DR must remain the final calendar
    // access; another SSR read here would lock the shadow again.
    let _ = RTC.ssr().read();
    let time = RTC.tr().read();
    let date = RTC.dr().read();
    let second = bcd(time.st(), time.su());
    let minute = bcd(time.mnt(), time.mnu());
    let mut hour = bcd(time.ht(), time.hu());
    if matches!(time.pm(), Ampm::PM) && hour < 12 {
        hour += 12;
    }
    let day = bcd(date.dt(), date.du());
    let month = bcd(u8::from(date.mt()), date.mu());
    let year = 2000 + u16::from(bcd(date.yt(), date.yu()));
    Utc::new(year, month, day, hour, minute, second)
}

fn write_calendar(utc: Utc) -> bool {
    if !utc.is_rtc_range() {
        return false;
    }
    let (su, st) = bcd_parts(utc.second);
    let (mnu, mnt) = bcd_parts(utc.minute);
    let (hu, ht) = bcd_parts(utc.hour);
    let (du, dt) = bcd_parts(utc.day);
    let (mu, mt) = bcd_parts(utc.month);
    let (yu, yt) = bcd_parts(u8::try_from(utc.year - 2000).expect("RTC range checked"));

    unlock_rtc_registers();
    if !enter_init() {
        exit_init();
        lock_rtc_registers();
        return false;
    }
    RTC.tr().write(|w| {
        w.set_su(su);
        w.set_st(st);
        w.set_mnu(mnu);
        w.set_mnt(mnt);
        w.set_hu(hu);
        w.set_ht(ht);
        w.set_pm(Ampm::AM);
    });
    RTC.dr().write(|w| {
        w.set_du(du);
        w.set_dt(dt);
        w.set_mu(mu);
        w.set_mt(mt != 0);
        w.set_wdu(utc.weekday());
        w.set_yu(yu);
        w.set_yt(yt);
    });
    exit_init();
    lock_rtc_registers();
    refresh_shadow()
}

/// Wait for the shadow registers to refresh after RSF is cleared.
///
/// Clearing RSF is a write to the write-protected RTC_ISR: it only takes effect
/// while WPR is unlocked. Without the unlock the flag stays set, the shadow
/// registers never update, and every calendar read keeps returning the value
/// frozen at the last successful refresh.
fn refresh_shadow() -> bool {
    unlock_rtc_registers();
    RTC.isr().modify(|w| w.set_rsf(false));
    let mut refreshed = false;
    for _ in 0..SYNC_SPINS {
        if RTC.isr().read().rsf() {
            refreshed = true;
            break;
        }
    }
    lock_rtc_registers();
    refreshed
}

fn wait_synchronized() -> bool {
    for _ in 0..SYNC_SPINS {
        if RTC.isr().read().rsf() {
            return true;
        }
    }
    false
}

/// Prove the calendar itself advances, not just the sub-second counter.
fn counting() -> bool {
    let Some(first) = read_calendar() else {
        return false;
    };
    let start = Instant::now();
    while start.elapsed() < COUNT_WINDOW {
        wait_for(TICK_STEP);
        if read_calendar().is_some_and(|now| now.unix() != first.unix()) {
            return true;
        }
    }
    false
}

fn enter_init() -> bool {
    release_init_mode();
    RTC.isr().modify(|w| w.set_init(true));
    for _ in 0..SYNC_SPINS {
        if RTC.isr().read().initf() {
            return true;
        }
    }
    false
}

fn exit_init() {
    RTC.isr().modify(|w| w.set_init(false));
}

/// Clear an INIT left set by an earlier boot.
///
/// While INIT stays set the calendar is frozen but the sub-second counter keeps
/// counting, so a stale INIT looks like a stopped crystal. INIT lives in the
/// RTC domain and is not cleared by an MCU reset.
fn release_init_mode() -> bool {
    if !RTC.isr().read().init() {
        return true;
    }
    RTC.isr().modify(|w| w.set_init(false));
    let _ = wait_flag(|| !RTC.isr().read().initf(), INIT_RELEASE_TIMEOUT);
    !RTC.isr().read().init()
}

fn unlock_rtc_registers() {
    unlock_backup_domain();
    RTC.wpr().write(|w| w.set_key(0xca));
    RTC.wpr().write(|w| w.set_key(0x53));
}

fn lock_rtc_registers() {
    RTC.wpr().write(|w| w.set_key(0xff));
}

fn unlock_backup_domain() {
    PWR.cr1().modify(|w| w.set_dbp(true));
    for _ in 0..1_000 {
        if PWR.cr1().read().dbp() {
            return;
        }
    }
}

fn wait_flag(mut ready: impl FnMut() -> bool, timeout: Duration) -> bool {
    let start = Instant::now();
    loop {
        if ready() {
            return true;
        }
        if start.elapsed() >= timeout {
            return false;
        }
    }
}

fn wait_for(duration: Duration) {
    let start = Instant::now();
    while start.elapsed() < duration {}
}

fn bcd(tens: u8, units: u8) -> u8 {
    tens * 10 + units
}

fn bcd_parts(value: u8) -> (u8, u8) {
    (value % 10, value / 10)
}
