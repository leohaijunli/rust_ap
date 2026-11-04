//! Fixed PX4 RingBuffer observation selection, shared by independently delayed aids.
//! Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.

#[derive(Clone)]
pub(super) struct ObservationBuffer<T: Copy + Default> {
    samples: [T; 64],
    times: [u64; 64],
    pub length: usize,
    head: usize,
    tail: usize,
    first: bool,
}

impl<T: Copy + Default> ObservationBuffer<T> {
    pub fn new(length: usize) -> Self {
        assert!((2..=64).contains(&length));
        Self {
            samples: [T::default(); 64],
            times: [0; 64],
            length,
            head: 0,
            tail: 0,
            first: true,
        }
    }

    pub fn newest_time(&self) -> u64 {
        self.times[self.head]
    }

    pub fn push(&mut self, sample: T, time: u64) {
        if !self.first {
            self.head = (self.head + 1) % self.length;
        }
        self.samples[self.head] = sample;
        self.times[self.head] = time;
        if self.head == self.tail && !self.first {
            self.tail = (self.tail + 1) % self.length;
        } else {
            self.first = false;
        }
    }

    pub fn pop(&mut self, time: u64) -> Option<(T, u64)> {
        for offset in 0..self.length {
            let index = (self.head + self.length - offset) % self.length;
            let timestamp = self.times[index];
            if time >= timestamp && time < timestamp.saturating_add(100_000) {
                if index == self.head {
                    self.tail = self.head;
                    self.first = true;
                } else {
                    self.tail = (index + 1) % self.length;
                }
                self.times[index] = 0;
                return Some((self.samples[index], timestamp));
            }
            if index == self.tail {
                return None;
            }
        }
        None
    }
}
