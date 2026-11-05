//! Bounded timing histograms; p99 is a reported upper bound, never an average.

#[derive(Clone, Copy, Debug, Default)]
pub struct TimingReport {
    pub samples: u32,
    pub max_us: u32,
    pub p99_upper_us: u32,
    pub total_us: u64,
}

#[derive(Clone, Copy, Debug)]
pub struct Timing {
    pub samples: u32,
    pub max_us: u32,
    pub total_us: u64,
    buckets: [u32; 64],
}

impl Timing {
    pub const EMPTY: Self = Self {
        samples: 0,
        max_us: 0,
        total_us: 0,
        buckets: [0; 64],
    };

    pub fn record(&mut self, us: u32) {
        self.samples = self.samples.saturating_add(1);
        self.max_us = self.max_us.max(us);
        self.total_us = self.total_us.saturating_add(u64::from(us));
        let bucket = ((us / 16) as usize).min(63);
        self.buckets[bucket] = self.buckets[bucket].saturating_add(1);
    }

    pub fn report(&self) -> TimingReport {
        TimingReport {
            samples: self.samples,
            max_us: self.max_us,
            p99_upper_us: self.p99_upper_us(),
            total_us: self.total_us,
        }
    }

    pub fn p99_upper_us(&self) -> u32 {
        if self.samples == 0 {
            return 0;
        }
        let threshold = (u64::from(self.samples) * 99).div_ceil(100);
        let mut count = 0_u64;
        for (index, samples) in self.buckets.iter().enumerate() {
            count += u64::from(*samples);
            if count >= threshold {
                return if index == 63 {
                    self.max_us
                } else {
                    ((index as u32 + 1) * 16).min(self.max_us)
                };
            }
        }
        self.max_us
    }
}
