//! Absolute-release accounting. All times are monotonic microseconds.

pub const PERIOD_US: u64 = 1_000;
pub const LATE_LIMITS_US: [u64; 8] = [1, 2, 5, 10, 20, 50, 100, 1_000];

#[derive(Clone, Copy, Debug)]
pub struct PeriodStats {
    pub samples: u64,
    pub skipped: u64,
    pub deadline_misses: u64,
    pub scheduling_overruns: u64,
    pub scheduling_skipped: u64,
    pub time_errors: u32,
    pub last_start_us: u64,
    pub interval_min_us: u64,
    pub interval_max_us: u64,
    pub late_max_us: u64,
    pub late_sum_us: u64,
    pub late_hist: [u64; 9],
    pub body_min_cycles: u32,
    pub body_max_cycles: u32,
    pub body_sum_cycles: u64,
    pub invalid_cycles: u32,
}

impl PeriodStats {
    pub const EMPTY: Self = Self {
        samples: 0,
        skipped: 0,
        deadline_misses: 0,
        scheduling_overruns: 0,
        scheduling_skipped: 0,
        time_errors: 0,
        last_start_us: 0,
        interval_min_us: u64::MAX,
        interval_max_us: 0,
        late_max_us: 0,
        late_sum_us: 0,
        late_hist: [0; 9],
        body_min_cycles: u32::MAX,
        body_max_cycles: 0,
        body_sum_cycles: 0,
        invalid_cycles: 0,
    };

    /// Records the intended release before skipping any overdue releases.
    /// Returns the next phase-aligned release strictly after completion.
    pub fn record(&mut self, release_us: u64, start_us: u64, end_us: u64, cycles: u32) -> u64 {
        if start_us < release_us || end_us < start_us {
            self.time_errors = self.time_errors.saturating_add(1);
        }
        if self.samples != 0 {
            if start_us < self.last_start_us {
                self.time_errors = self.time_errors.saturating_add(1);
            } else {
                let interval = start_us - self.last_start_us;
                self.interval_min_us = self.interval_min_us.min(interval);
                self.interval_max_us = self.interval_max_us.max(interval);
            }
        }
        self.last_start_us = start_us;
        self.samples = self.samples.saturating_add(1);
        let late = start_us.saturating_sub(release_us);
        self.late_max_us = self.late_max_us.max(late);
        self.late_sum_us = self.late_sum_us.saturating_add(late);
        let bucket = LATE_LIMITS_US
            .iter()
            .position(|&limit| late <= limit)
            .unwrap_or(8);
        self.late_hist[bucket] = self.late_hist[bucket].saturating_add(1);

        // CYCCNT wraps after ~8.95 s at 480 MHz. A long observation is invalid,
        // even if its wrapped cycle difference looks small.
        if end_us.saturating_sub(start_us) >= 8_000_000 || end_us < start_us {
            self.invalid_cycles = self.invalid_cycles.saturating_add(1);
        } else {
            self.body_min_cycles = self.body_min_cycles.min(cycles);
            self.body_max_cycles = self.body_max_cycles.max(cycles);
            self.body_sum_cycles = self.body_sum_cycles.saturating_add(u64::from(cycles));
        }

        let next = release_us.saturating_add(PERIOD_US);
        if end_us >= next {
            self.deadline_misses = self.deadline_misses.saturating_add(1);
            let skipped = (end_us - next) / PERIOD_US + 1;
            self.skipped = self.skipped.saturating_add(skipped);
            next.saturating_add(skipped.saturating_mul(PERIOD_US))
        } else {
            next
        }
    }

    /// Account for statistics/snapshot overhead before arming the next timer.
    /// Do not replay already expired releases, even when the body itself fit.
    pub fn finish_poll(&mut self, next_us: u64, now_us: u64) -> u64 {
        if now_us < next_us {
            return next_us;
        }
        let skipped = (now_us - next_us) / PERIOD_US + 1;
        self.scheduling_overruns = self.scheduling_overruns.saturating_add(1);
        self.scheduling_skipped = self.scheduling_skipped.saturating_add(skipped);
        self.skipped = self.skipped.saturating_add(skipped);
        next_us.saturating_add(skipped.saturating_mul(PERIOD_US))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CpuWindow {
    pub wall_us: u64,
    pub idle_us: u32,
    pub idle_calls: u32,
    pub busy_permille: u16,
    pub valid: bool,
}

impl CpuWindow {
    pub const EMPTY: Self = Self {
        wall_us: 0,
        idle_us: 0,
        idle_calls: 0,
        busy_permille: 0,
        valid: false,
    };

    /// The idle counters wrap at 2^32; windows must be shorter than that.
    /// Invalid intervals are reported, never silently clamped into a pass.
    pub fn between(start: (u64, u32, u32), end: (u64, u32, u32)) -> Self {
        let wall_us = end.0.saturating_sub(start.0);
        let idle_us = end.1.wrapping_sub(start.1);
        let idle_calls = end.2.wrapping_sub(start.2);
        let valid =
            end.0 > start.0 && wall_us < u64::from(u32::MAX) && u64::from(idle_us) <= wall_us;
        Self {
            wall_us,
            idle_us,
            idle_calls,
            busy_permille: if valid {
                ((wall_us - u64::from(idle_us)) * 1_000 / wall_us) as u16
            } else {
                0
            },
            valid,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn absolute_phase_does_not_drift_with_variable_runtime() {
        let mut stats = PeriodStats::EMPTY;
        let mut release = 1_000;
        for n in 1..=60_000 {
            assert_eq!(release, n * PERIOD_US);
            release = stats.record(release, release + n % 7, release + 20 + n % 13, 600);
        }
        assert_eq!(stats.samples, 60_000);
        assert_eq!(stats.late_hist.iter().sum::<u64>(), 60_000);
        assert_eq!(stats.skipped, 0);
        assert_eq!(stats.deadline_misses, 0);
        assert_eq!(stats.time_errors, 0);
    }

    #[test]
    fn stalls_are_counted_without_replaying_old_releases() {
        let mut stats = PeriodStats::EMPTY;
        assert_eq!(stats.record(1_000, 3_510, 3_540, 100), 4_000);
        assert_eq!(stats.late_max_us, 2_510);
        assert_eq!(stats.skipped, 2);
        assert_eq!(stats.deadline_misses, 1);
        // Finishing exactly at the next release counts a miss and skips it.
        assert_eq!(stats.record(4_000, 4_001, 5_000, 200), 6_000);
        assert_eq!(stats.skipped, 3);
        assert_eq!(stats.deadline_misses, 2);
    }

    #[test]
    fn bookkeeping_overrun_is_visible_and_never_replayed() {
        let mut stats = PeriodStats::EMPTY;
        let next = stats.record(1_000, 1_001, 1_002, 100);
        assert_eq!(stats.finish_poll(next, 3_010), 4_000);
        assert_eq!(stats.skipped, 2);
        assert_eq!(stats.scheduling_skipped, 2);
        assert_eq!(stats.scheduling_overruns, 1);
        assert_eq!(stats.deadline_misses, 0);
        assert_eq!(stats.finish_poll(4_000, 3_011), 4_000);
    }

    #[test]
    fn cycles_wrap_but_multiwrap_observations_are_invalid() {
        let mut stats = PeriodStats::EMPTY;
        let cycles = 50u32.wrapping_sub(u32::MAX - 49);
        stats.record(1_000, 1_000, 1_001, cycles);
        assert_eq!(stats.body_max_cycles, 100);
        stats.record(2_000, 2_000, 9_002_000, 17);
        assert_eq!(stats.invalid_cycles, 1);
        assert_eq!(stats.body_sum_cycles, 100);
        assert_eq!(stats.skipped, 9_000);
    }

    #[test]
    fn regressing_time_is_visible() {
        let mut stats = PeriodStats::EMPTY;
        stats.record(2_000, 1_000, 999, 1);
        assert_eq!(stats.time_errors, 1);
        assert_eq!(stats.invalid_cycles, 1);
    }

    #[test]
    fn cpu_windows_include_busy_time_and_handle_idle_counter_wrap() {
        let start = (1_000_000, u32::MAX - 399_999, u32::MAX - 10);
        let end = (2_000_000, 400_000, 989);
        let window = CpuWindow::between(start, end);
        assert!(window.valid);
        assert_eq!(window.idle_us, 800_000);
        assert_eq!(window.idle_calls, 1_000);
        assert_eq!(window.busy_permille, 200);
        assert_eq!(
            CpuWindow::between((0, 0, 0), (1_000, 0, 0)).busy_permille,
            1_000
        );
    }

    #[test]
    fn impossible_cpu_windows_cannot_appear_as_zero_load_success() {
        for (start, end) in [
            ((0, 0, 0), (0, 0, 0)),
            ((2, 0, 0), (1, 0, 0)),
            ((0, 0, 0), (100, 101, 1)),
            ((0, 0, 0), (u64::from(u32::MAX), 10, 1)),
        ] {
            assert!(!CpuWindow::between(start, end).valid);
        }
    }
}
