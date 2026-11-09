//! Pure button and indicator policy for the GPS1 logger switch.

use crate::logger::Status;

pub const SAMPLE_PERIOD_MS: u64 = 10;
pub const LONG_PRESS_MS: u64 = 1_500;
pub const DEBOUNCE_MS: u64 = 40;
const BEEP_TONE_MS: u64 = 100;
const BEEP_GAP_MS: u64 = 80;
const TEST_DELAY_MS: u64 = 500;
const TEST_TONE_MS: u64 = 700;
const TEST_GAP_MS: u64 = 400;

/// Logging input is independent of PX4IO communication, but must not change
/// the recording state while the vehicle is armed or outputs are enabled.
pub const fn input_enabled(armed: bool, outputs_enabled: bool) -> bool {
    !armed && !outputs_enabled
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    Start,
    Stop,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ButtonEvent {
    ShortPress,
    LongPress,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IndicatorPattern {
    Off,
    On,
    Blink { half_period_ms: u64 },
}

impl IndicatorPattern {
    pub fn is_on(self, elapsed_ms: u64) -> bool {
        match self {
            Self::Off => false,
            Self::On => true,
            Self::Blink { half_period_ms } => {
                half_period_ms != 0 && (elapsed_ms / half_period_ms).is_multiple_of(2)
            }
        }
    }
}

pub struct Debouncer {
    stable_pressed: bool,
    candidate_pressed: bool,
    candidate_since_ms: u64,
    pressed_since_ms: u64,
    long_press_emitted: bool,
    events_enabled: bool,
}

impl Debouncer {
    pub const fn new(initial_pressed: bool, now_ms: u64) -> Self {
        Self {
            stable_pressed: initial_pressed,
            candidate_pressed: initial_pressed,
            candidate_since_ms: now_ms,
            pressed_since_ms: now_ms,
            long_press_emitted: false,
            events_enabled: !initial_pressed,
        }
    }

    /// Emits a short press on release or one long press after the hold time.
    /// An initially held button emits nothing until first released.
    pub fn update(&mut self, pressed: bool, now_ms: u64, permitted: bool) -> Option<ButtonEvent> {
        if !permitted {
            *self = Self::new(pressed, now_ms);
            return None;
        }
        if pressed != self.candidate_pressed {
            self.candidate_pressed = pressed;
            self.candidate_since_ms = now_ms;
        }

        if pressed != self.stable_pressed {
            if now_ms.saturating_sub(self.candidate_since_ms) < DEBOUNCE_MS {
                return None;
            }
            self.stable_pressed = pressed;
            if pressed {
                self.pressed_since_ms = self.candidate_since_ms;
                self.long_press_emitted = false;
            } else {
                let event = if !self.events_enabled || self.long_press_emitted {
                    None
                } else if self
                    .candidate_since_ms
                    .saturating_sub(self.pressed_since_ms)
                    >= LONG_PRESS_MS
                {
                    Some(ButtonEvent::LongPress)
                } else {
                    Some(ButtonEvent::ShortPress)
                };
                self.events_enabled = true;
                self.long_press_emitted = false;
                return event;
            }
        }
        self.update_hold(pressed, now_ms)
    }

    pub const fn is_pressed(&self) -> bool {
        self.stable_pressed
    }

    fn update_hold(&mut self, pressed: bool, now_ms: u64) -> Option<ButtonEvent> {
        if !pressed || !self.events_enabled || self.long_press_emitted {
            return None;
        }
        if now_ms.saturating_sub(self.pressed_since_ms) < LONG_PRESS_MS {
            return None;
        }
        self.long_press_emitted = true;
        Some(ButtonEvent::LongPress)
    }
}

/// A bounded, non-blocking tone envelope. Scheduling delays never replay old
/// pulses or extend the requested sequence by counting slow task polls.
pub struct BeepSequence {
    started_ms: u64,
    pulses: u8,
    tone_ms: u64,
    gap_ms: u64,
}

impl BeepSequence {
    pub const fn new() -> Self {
        Self {
            started_ms: 0,
            pulses: 0,
            tone_ms: BEEP_TONE_MS,
            gap_ms: BEEP_GAP_MS,
        }
    }

    pub fn play(&mut self, pulses: u8, now_ms: u64) {
        self.started_ms = now_ms;
        self.pulses = pulses;
        self.tone_ms = BEEP_TONE_MS;
        self.gap_ms = BEEP_GAP_MS;
    }

    /// Audible, bounded test independent of SD requests; starts after 500 ms.
    pub fn play_test(&mut self, now_ms: u64) {
        self.started_ms = now_ms.saturating_add(TEST_DELAY_MS);
        self.pulses = 3;
        self.tone_ms = TEST_TONE_MS;
        self.gap_ms = TEST_GAP_MS;
    }

    pub fn is_on(&self, now_ms: u64) -> bool {
        if now_ms < self.started_ms {
            return false;
        }
        let elapsed_ms = now_ms - self.started_ms;
        let period_ms = self.tone_ms + self.gap_ms;
        elapsed_ms / period_ms < u64::from(self.pulses) && elapsed_ms % period_ms < self.tone_ms
    }
}

impl Default for BeepSequence {
    fn default() -> Self {
        Self::new()
    }
}

pub const fn action_for(status: Status) -> Option<Action> {
    match status {
        Status::Disabled | Status::Error => Some(Action::Start),
        Status::Ready => Some(Action::Stop),
        Status::Initializing | Status::Stopping | Status::Managing => None,
    }
}

pub const fn indicator_pattern(status: Status) -> IndicatorPattern {
    match status {
        Status::Disabled => IndicatorPattern::Off,
        Status::Ready => IndicatorPattern::On,
        Status::Initializing | Status::Stopping | Status::Managing => IndicatorPattern::Blink {
            half_period_ms: 250,
        },
        Status::Error => IndicatorPattern::Blink {
            half_period_ms: 100,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settle(debouncer: &mut Debouncer, pressed: bool, now_ms: u64) -> Option<ButtonEvent> {
        let first = debouncer.update(pressed, now_ms, true);
        first.or(debouncer.update(pressed, now_ms + DEBOUNCE_MS, true))
    }

    #[test]
    fn press_is_debounced_and_emitted_once() {
        let mut debouncer = Debouncer::new(false, 0);

        assert_eq!(debouncer.update(true, 0, true), None);
        assert_eq!(debouncer.update(false, 10, true), None);
        assert_eq!(settle(&mut debouncer, true, 20), None);
        assert!(debouncer.is_pressed());
        assert_eq!(debouncer.update(true, 70, true), None);

        assert_eq!(
            settle(&mut debouncer, false, 80),
            Some(ButtonEvent::ShortPress)
        );
        assert!(!debouncer.is_pressed());
        assert_eq!(settle(&mut debouncer, true, 130), None);
    }

    #[test]
    fn initially_held_button_does_not_trigger_on_boot() {
        let mut debouncer = Debouncer::new(true, 0);

        assert_eq!(settle(&mut debouncer, true, 0), None);
        assert_eq!(debouncer.update(true, LONG_PRESS_MS, true), None);
        assert_eq!(settle(&mut debouncer, false, LONG_PRESS_MS + 10), None);
        assert_eq!(settle(&mut debouncer, true, LONG_PRESS_MS + 60), None);
        assert_eq!(
            settle(&mut debouncer, false, LONG_PRESS_MS + 110),
            Some(ButtonEvent::ShortPress)
        );
    }

    #[test]
    fn long_press_is_emitted_once_and_suppresses_short_press() {
        let mut debouncer = Debouncer::new(false, 0);
        assert_eq!(settle(&mut debouncer, true, 0), None);
        assert_eq!(debouncer.update(true, LONG_PRESS_MS - 1, true), None);
        assert_eq!(
            debouncer.update(true, LONG_PRESS_MS, true),
            Some(ButtonEvent::LongPress)
        );
        assert_eq!(debouncer.update(true, LONG_PRESS_MS + 10, true), None);
        assert_eq!(settle(&mut debouncer, false, LONG_PRESS_MS + 20), None);
    }

    #[test]
    fn delayed_poll_does_not_turn_a_long_hold_into_a_short_press() {
        let mut debouncer = Debouncer::new(false, 0);
        assert_eq!(settle(&mut debouncer, true, 0), None);
        assert_eq!(
            settle(&mut debouncer, false, 2_000),
            Some(ButtonEvent::LongPress)
        );
    }

    #[test]
    fn logger_input_requires_disarmed_and_outputs_disabled() {
        assert!(input_enabled(false, false));
        assert!(!input_enabled(true, false));
        assert!(!input_enabled(false, true));
        assert!(!input_enabled(true, true));
    }

    #[test]
    fn withdrawing_permission_requires_a_new_release_and_press() {
        let mut debouncer = Debouncer::new(false, 0);
        assert_eq!(settle(&mut debouncer, true, 10), None);
        assert_eq!(debouncer.update(true, 100, false), None);
        assert_eq!(debouncer.update(true, 2_000, true), None);
        assert_eq!(settle(&mut debouncer, false, 2_010), None);
        assert_eq!(settle(&mut debouncer, true, 2_060), None);
        assert_eq!(
            settle(&mut debouncer, false, 2_140),
            Some(ButtonEvent::ShortPress)
        );
    }

    #[test]
    fn tone_envelope_is_bounded_and_does_not_catch_up_after_delays() {
        let mut beeps = BeepSequence::new();
        assert!(!beeps.is_on(0));
        beeps.play(2, 10);
        assert!(beeps.is_on(10));
        assert!(!beeps.is_on(110));
        assert!(beeps.is_on(190));
        assert!(!beeps.is_on(290));
        assert!(!beeps.is_on(5_000));
    }

    #[test]
    fn independent_buzzer_test_ends_and_normal_feedback_restores_short_tones() {
        let mut beeps = BeepSequence::new();
        beeps.play_test(100);
        assert!(!beeps.is_on(599));
        assert!(beeps.is_on(600));
        assert!(!beeps.is_on(1_300));
        assert!(beeps.is_on(1_700));
        assert!(beeps.is_on(2_800));
        assert!(!beeps.is_on(3_900));
        beeps.play(1, 4_000);
        assert!(beeps.is_on(4_000));
        assert!(!beeps.is_on(4_100));
    }

    #[test]
    fn logger_state_selects_action_and_led_pattern() {
        assert_eq!(action_for(Status::Disabled), Some(Action::Start));
        assert_eq!(action_for(Status::Error), Some(Action::Start));
        assert_eq!(action_for(Status::Ready), Some(Action::Stop));
        assert_eq!(action_for(Status::Initializing), None);
        assert_eq!(action_for(Status::Stopping), None);
        assert_eq!(action_for(Status::Managing), None);

        assert_eq!(indicator_pattern(Status::Disabled), IndicatorPattern::Off);
        assert_eq!(indicator_pattern(Status::Ready), IndicatorPattern::On);
        let busy = indicator_pattern(Status::Initializing);
        assert!(busy.is_on(0));
        assert!(!busy.is_on(250));
        let error = indicator_pattern(Status::Error);
        assert!(error.is_on(0));
        assert!(!error.is_on(100));
        assert!(!IndicatorPattern::Blink { half_period_ms: 0 }.is_on(0));
    }
}
