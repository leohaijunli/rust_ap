//! GPS1 safety-switch hardware repurposed as an SD logger control.

use core::fmt;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use embassy_stm32::gpio::{Input, OutputType, Pull, Speed};
use embassy_stm32::peripherals;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, PwmPinConfig, SimplePwm};
use embassy_stm32::Peri;
use embassy_time::{Instant, Timer};
use v6x_rust_ap::logger::Status;
use v6x_rust_ap::logger_switch::{self, Action, BeepSequence, ButtonEvent, Debouncer};

const BUZZER_HZ: u32 = 4_000;

static BUTTON_PRESSED: AtomicBool = AtomicBool::new(false);
static BUTTON_RAW_HIGH: AtomicBool = AtomicBool::new(false);
static LED_REQUESTED: AtomicBool = AtomicBool::new(false);
static BUZZER_ON: AtomicBool = AtomicBool::new(false);
static BUZZER_READY: AtomicBool = AtomicBool::new(false);
static BUZZER_TEST_REQUESTED: AtomicBool = AtomicBool::new(false);
static BUZZER_TESTS: AtomicU32 = AtomicU32::new(0);
static PRESSES: AtomicU32 = AtomicU32::new(0);
static SHORT_PRESSES: AtomicU32 = AtomicU32::new(0);
static LONG_PRESSES: AtomicU32 = AtomicU32::new(0);
static START_REQUESTS: AtomicU32 = AtomicU32::new(0);
static STOP_REQUESTS: AtomicU32 = AtomicU32::new(0);
static REJECTED: AtomicU32 = AtomicU32::new(0);
static PENDING: AtomicU8 = AtomicU8::new(0);
static LAST_EVENT: AtomicU8 = AtomicU8::new(Event::None as u8);

#[derive(Clone, Copy)]
#[repr(u8)]
enum Event {
    None,
    StartRequested,
    Started,
    StopRequested,
    Stopped,
    Rejected,
    LoggerError,
    LongPress,
}

#[embassy_executor::task]
pub async fn run(
    button_pin: Peri<'static, peripherals::PF5>,
    buzzer_timer: Peri<'static, peripherals::TIM14>,
    buzzer_pin: Peri<'static, peripherals::PF9>,
) -> ! {
    // Follow the fixed PX4 board GPIO definition; actual CUAV idle/pressed
    // levels still require H-12. A held input is suppressed at boot/reconnect.
    let button = Input::new(button_pin, Pull::Up);
    // PF9 drives the board's buzzer transistor, not the external BUZZER- net
    // directly. PX4 uses push-pull at the MCU and GPIO low when idle.
    let buzzer_pin = PwmPin::new_with_config(
        buzzer_pin,
        PwmPinConfig {
            output_type: OutputType::PushPull,
            speed: Speed::Low,
            pull: Pull::None,
        },
    );
    let mut pwm = SimplePwm::new(
        buzzer_timer,
        Some(buzzer_pin),
        None,
        None,
        None,
        Hertz(BUZZER_HZ),
        CountingMode::EdgeAlignedUp,
    );
    let mut buzzer = pwm.ch1();
    // Enabled 0% PWM gives a defined low idle level, unlike a floating channel.
    buzzer.set_duty_cycle_fully_off();
    buzzer.enable();
    BUZZER_READY.store(true, Ordering::Release);

    let initial_pressed = button.is_high();
    let mut debouncer = Debouncer::new(initial_pressed, Instant::now().as_millis());
    BUTTON_PRESSED.store(initial_pressed, Ordering::Release);
    BUTTON_RAW_HIGH.store(initial_pressed, Ordering::Release);
    let started_at = Instant::now();
    let mut pending = None;
    let mut beeper = BeepSequence::new();
    let mut buzzer_enabled = false;

    loop {
        let now_ms = Instant::now().as_millis();
        let status = crate::sd_logger::snapshot().status;
        if let Some(action) = pending {
            let result = match (action, status) {
                (Action::Start, Status::Ready) => Some((Event::Started, 1)),
                (Action::Stop, Status::Disabled) => Some((Event::Stopped, 2)),
                (_, Status::Error) => Some((Event::LoggerError, 3)),
                _ => None,
            };
            if let Some((event, pulses)) = result {
                pending = None;
                PENDING.store(0, Ordering::Release);
                LAST_EVENT.store(event as u8, Ordering::Release);
                beeper.play(pulses, now_ms);
            }
        }

        let pressed = button.is_high();
        BUTTON_RAW_HIGH.store(pressed, Ordering::Release);
        // PF5 and SD logging belong to FMU. IO readiness still protects IO
        // outputs, but must not disable this independent data-only control.
        let button_event = debouncer.update(pressed, now_ms, input_enabled());
        if let Some(event) = button_event {
            PRESSES.fetch_add(1, Ordering::Relaxed);
            match event {
                ButtonEvent::ShortPress => {
                    SHORT_PRESSES.fetch_add(1, Ordering::Relaxed);
                    request_logger_action(status, &mut pending, &mut beeper, now_ms);
                }
                ButtonEvent::LongPress => {
                    // Reserved for a future user-selected action. It must not
                    // fall through to the short-press logger toggle.
                    LONG_PRESSES.fetch_add(1, Ordering::Relaxed);
                    LAST_EVENT.store(Event::LongPress as u8, Ordering::Release);
                }
            }
        }
        BUTTON_PRESSED.store(debouncer.is_pressed(), Ordering::Release);

        if BUZZER_TEST_REQUESTED.swap(false, Ordering::AcqRel)
            && input_enabled()
            && status == Status::Disabled
            && pending.is_none()
        {
            beeper.play_test(now_ms);
            BUZZER_TESTS.fetch_add(1, Ordering::Relaxed);
        }

        let elapsed_ms = started_at.elapsed().as_millis();
        let led_on = logger_switch::indicator_pattern(status).is_on(elapsed_ms);
        // Desired logger pattern is observable but cannot be applied to the
        // IO-owned safety LED without a separate, explicit override protocol.
        LED_REQUESTED.store(led_on, Ordering::Release);

        let should_buzz = beeper.is_on(now_ms);
        if should_buzz != buzzer_enabled {
            if should_buzz {
                buzzer.set_duty_cycle_percent(50);
            } else {
                buzzer.set_duty_cycle_fully_off();
            }
            buzzer_enabled = should_buzz;
            BUZZER_ON.store(should_buzz, Ordering::Release);
        }

        Timer::after_millis(logger_switch::SAMPLE_PERIOD_MS).await;
    }
}

fn input_enabled() -> bool {
    logger_switch::input_enabled(
        crate::OUTPUT_STATE.armed,
        crate::OUTPUT_STATE.outputs_enabled,
    )
}

pub fn request_buzzer_test() -> bool {
    if !input_enabled()
        || !BUZZER_READY.load(Ordering::Acquire)
        || crate::sd_logger::snapshot().status != Status::Disabled
        || PENDING.load(Ordering::Acquire) != 0
    {
        return false;
    }
    !BUZZER_TEST_REQUESTED.swap(true, Ordering::AcqRel)
}

pub fn write_buzzer_diagnostics(output: &mut impl fmt::Write) -> fmt::Result {
    let timer = embassy_stm32::pac::TIM14;
    let gpio = embassy_stm32::pac::GPIOF;
    writeln!(output, "BUZZER schema=1 pin=PF9 timer=TIM14 frequency_hz={} ready={} on={} test_pending={} tests={} gpio_mode={:?} gpio_af={} timer_enabled={} channel_enabled={} psc={} arr={} ccr={}",
        BUZZER_HZ, BUZZER_READY.load(Ordering::Acquire), BUZZER_ON.load(Ordering::Acquire),
        BUZZER_TEST_REQUESTED.load(Ordering::Acquire), BUZZER_TESTS.load(Ordering::Acquire),
        gpio.moder().read().moder(9), gpio.afr(1).read().afr(1),
        timer.cr1().read().cen(), timer.ccer().read().cce(0),
        timer.psc().read(), timer.arr().read().arr(), timer.ccr(0).read().ccr())
}

fn request_logger_action(
    status: Status,
    pending: &mut Option<Action>,
    beeper: &mut BeepSequence,
    now_ms: u64,
) {
    let Some(action) = logger_switch::action_for(status) else {
        reject(beeper, now_ms);
        return;
    };
    let result = match action {
        Action::Start => crate::sd_logger::request_start(),
        Action::Stop => crate::sd_logger::request_stop(),
    };
    if result.is_err() {
        reject(beeper, now_ms);
        return;
    }

    *pending = Some(action);
    match action {
        Action::Start => {
            START_REQUESTS.fetch_add(1, Ordering::Relaxed);
            PENDING.store(1, Ordering::Release);
            LAST_EVENT.store(Event::StartRequested as u8, Ordering::Release);
        }
        Action::Stop => {
            STOP_REQUESTS.fetch_add(1, Ordering::Relaxed);
            PENDING.store(2, Ordering::Release);
            LAST_EVENT.store(Event::StopRequested as u8, Ordering::Release);
        }
    }
}

fn reject(beeper: &mut BeepSequence, now_ms: u64) {
    REJECTED.fetch_add(1, Ordering::Relaxed);
    LAST_EVENT.store(Event::Rejected as u8, Ordering::Release);
    beeper.play(3, now_ms);
}

pub fn write_diagnostics(output: &mut impl fmt::Write) -> fmt::Result {
    writeln!(
        output,
        "LOGGER_SWITCH schema=3 role=logger button=PF5 active_high=true enabled={} io_ready={} raw_high={} pressed={} presses={} short_presses={} long_presses={} long_press_ms={} led_owner=PX4IO led_override=false requested_on={} buzzer=PF9 timer=TIM14 push_pull=true frequency_hz={} on={} pending={} last_event={} start_requests={} stop_requests={} rejected={}",
        input_enabled(),
        crate::px4io::ready(),
        BUTTON_RAW_HIGH.load(Ordering::Acquire),
        BUTTON_PRESSED.load(Ordering::Acquire),
        PRESSES.load(Ordering::Relaxed),
        SHORT_PRESSES.load(Ordering::Relaxed),
        LONG_PRESSES.load(Ordering::Relaxed),
        logger_switch::LONG_PRESS_MS,
        LED_REQUESTED.load(Ordering::Acquire),
        BUZZER_HZ,
        BUZZER_ON.load(Ordering::Acquire),
        pending_name(PENDING.load(Ordering::Acquire)),
        event_name(LAST_EVENT.load(Ordering::Acquire)),
        START_REQUESTS.load(Ordering::Relaxed),
        STOP_REQUESTS.load(Ordering::Relaxed),
        REJECTED.load(Ordering::Relaxed),
    )
}

fn pending_name(value: u8) -> &'static str {
    match value {
        1 => "start",
        2 => "stop",
        _ => "none",
    }
}

fn event_name(value: u8) -> &'static str {
    match value {
        value if value == Event::StartRequested as u8 => "start_requested",
        value if value == Event::Started as u8 => "started",
        value if value == Event::StopRequested as u8 => "stop_requested",
        value if value == Event::Stopped as u8 => "stopped",
        value if value == Event::Rejected as u8 => "rejected",
        value if value == Event::LoggerError as u8 => "logger_error",
        value if value == Event::LongPress as u8 => "long_press_reserved",
        _ => "none",
    }
}
