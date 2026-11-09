//! Single FMU owner for the observed USL RGB at I2C1 PB8/PB9, address 0x39.
//! The IO safety LED is untouched. Blocking transfers are 10-ms bounded, not
//! async preemption; failed writes revoke readiness instead of hiding errors.

use core::{
    fmt,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use embassy_stm32::{
    i2c::{mode::Master, Config, Error as I2cError, I2c},
    mode::Blocking,
    peripherals,
    time::Hertz,
    Peri,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Instant, Timer};
use v6x_rust_ap::rgb_led::{self, OFF, TEST_COLORS};

type Bus = I2c<'static, Blocking, Master>;

static LATEST: Watch<CriticalSectionRawMutex, Snapshot, 0> = Watch::new();
static TEST_REQUESTED: AtomicBool = AtomicBool::new(false);
static TRANSFERS: AtomicU32 = AtomicU32::new(0);
static ERRORS: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy)]
struct Snapshot {
    probes: u32,
    ready: bool,
    in_test: bool,
    tests: u32,
    color: [u8; 3],
    error: Option<I2cError>,
}

impl Snapshot {
    const EMPTY: Self = Self {
        probes: 0,
        ready: false,
        in_test: false,
        tests: 0,
        color: OFF,
        error: None,
    };
}

fn set_color(bus: &mut Bus, state: &mut Snapshot, color: [u8; 3]) -> bool {
    for command in rgb_led::writes(color) {
        TRANSFERS.fetch_add(1, Ordering::Relaxed);
        if let Err(error) = bus.blocking_write(rgb_led::ADDRESS, &[command]) {
            state.ready = false;
            state.error = Some(error);
            ERRORS.fetch_add(1, Ordering::Relaxed);
            return false;
        }
    }
    state.color = color;
    state.ready = true;
    state.error = None;
    true
}

fn test_allowed() -> bool {
    v6x_rust_ap::logger_switch::input_enabled(
        crate::OUTPUT_STATE.armed,
        crate::OUTPUT_STATE.outputs_enabled,
    )
}

#[embassy_executor::task]
pub async fn run(
    i2c1: Peri<'static, peripherals::I2C1>,
    scl1: Peri<'static, peripherals::PB8>,
    sda1: Peri<'static, peripherals::PB9>,
) -> ! {
    let mut config = Config::default();
    config.frequency = Hertz(100_000);
    config.timeout = Duration::from_millis(10);
    config.scl_pullup = true;
    config.sda_pullup = true;
    let mut bus = I2c::new_blocking(i2c1, scl1, sda1, config);
    let mut state = Snapshot::EMPTY;
    let mut startup_test = true;
    loop {
        if !state.ready {
            state.probes = state.probes.wrapping_add(1);
            set_color(&mut bus, &mut state, OFF);
            LATEST.sender().send(state);
            if !state.ready {
                Timer::after_millis(1_000).await;
                continue;
            }
        }
        let requested = TEST_REQUESTED.swap(false, Ordering::AcqRel);
        let test = startup_test || requested;
        if test && test_allowed() {
            startup_test = false;
            state.in_test = true;
            for color in TEST_COLORS {
                if !set_color(&mut bus, &mut state, color) {
                    break;
                }
                LATEST.sender().send(state);
                Timer::after_millis(1_000).await;
            }
            state.in_test = false;
            if state.ready {
                state.tests = state.tests.wrapping_add(1);
            }
        }
        if state.ready {
            let color = rgb_led::logger_color(
                crate::sd_logger::snapshot().status,
                Instant::now().as_millis(),
            );
            if color != state.color {
                set_color(&mut bus, &mut state, color);
            }
        }
        LATEST.sender().send(state);
        Timer::after_millis(20).await;
    }
}

pub fn request_test() -> bool {
    if !test_allowed()
        || !LATEST
            .try_get()
            .is_some_and(|state| state.ready && !state.in_test)
    {
        return false;
    }
    !TEST_REQUESTED.swap(true, Ordering::AcqRel)
}

pub fn write_diagnostics(output: &mut impl fmt::Write) -> fmt::Result {
    let state = LATEST.try_get().unwrap_or(Snapshot::EMPTY);
    writeln!(output, "USL_RGB schema=2 owner=FMU role=logger bus=I2C1 scl=PB8 sda=PB9 address={:02x} protocol=NCP5623-compatible ready={} probes={} transfers={} errors={} in_test={} tests={} rgb=[{},{},{}] last_error={:?}",
        rgb_led::ADDRESS, state.ready, state.probes, TRANSFERS.load(Ordering::Relaxed), ERRORS.load(Ordering::Relaxed), state.in_test, state.tests, state.color[0], state.color[1], state.color[2], state.error)
}
