//! Fixed-memory MPSC queue for complete runtime ULog records.

use core::cell::{RefCell, UnsafeCell};
use core::mem::size_of;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use crate::ulog::{self, Record};

pub const QUEUE_BYTES: usize = 120 * 1024;
pub const EVENT_RESERVE_BYTES: usize = 8 * 1024;
pub const QUEUE_SLOTS: usize = 512;
// Complete estimator_states payload (213 B) plus the 5 B record header.
// Reduce slots instead of expanding static RAM; drops remain explicit.
pub const RUNTIME_RECORD_LEN: usize = 218;
const EVENT_RESERVE_SLOTS: u32 = 64;
const NORMAL_LIMIT: u32 = QUEUE_SLOTS as u32 - EVENT_RESERVE_SLOTS;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Class {
    Normal,
    Event,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PushError {
    NotInitialized,
    TooLong,
    Full,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    pub normal_dropped: u32,
    pub event_dropped: u32,
    pub high_water_bytes: u32,
}

#[derive(Clone, Copy)]
pub struct QueuedRecord {
    class: Class,
    bytes: [u8; RUNTIME_RECORD_LEN],
    len: u8,
}

impl QueuedRecord {
    const EMPTY: Self = Self {
        class: Class::Normal,
        bytes: [0; RUNTIME_RECORD_LEN],
        len: 0,
    };

    fn new(class: Class, record: Record) -> Result<Self, PushError> {
        let bytes = record.as_bytes();
        if bytes.len() > RUNTIME_RECORD_LEN {
            return Err(PushError::TooLong);
        }
        let mut queued = Self::EMPTY;
        queued.class = class;
        queued.bytes[..bytes.len()].copy_from_slice(bytes);
        queued.len = bytes.len() as u8;
        Ok(queued)
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..usize::from(self.len)]
    }
}

struct Slot {
    sequence: AtomicU32,
    value: UnsafeCell<QueuedRecord>,
}

impl Slot {
    const fn new() -> Self {
        Self {
            sequence: AtomicU32::new(0),
            value: UnsafeCell::new(QueuedRecord::EMPTY),
        }
    }
}

unsafe impl Sync for Slot {}

struct Queue {
    enqueue: AtomicU32,
    dequeue: AtomicU32,
    occupancy: AtomicU32,
    high_water: AtomicU32,
    slots: [Slot; QUEUE_SLOTS],
}

impl Queue {
    const fn new() -> Self {
        Self {
            enqueue: AtomicU32::new(0),
            dequeue: AtomicU32::new(0),
            occupancy: AtomicU32::new(0),
            high_water: AtomicU32::new(0),
            slots: [const { Slot::new() }; QUEUE_SLOTS],
        }
    }

    fn initialize(&self) {
        self.enqueue.store(0, Ordering::Relaxed);
        self.dequeue.store(0, Ordering::Relaxed);
        self.occupancy.store(0, Ordering::Relaxed);
        self.high_water.store(0, Ordering::Relaxed);
        for (index, slot) in self.slots.iter().enumerate() {
            slot.sequence.store(index as u32, Ordering::Relaxed);
        }
    }

    fn try_push(&self, value: QueuedRecord) -> Result<(), PushError> {
        self.try_push_values(value, None)
    }

    fn try_push_pair(&self, first: QueuedRecord, second: QueuedRecord) -> Result<(), PushError> {
        self.try_push_values(first, Some(second))
    }

    fn try_push_values(
        &self,
        first: QueuedRecord,
        second: Option<QueuedRecord>,
    ) -> Result<(), PushError> {
        let count = 1 + u32::from(second.is_some());
        let mut position = self.enqueue.load(Ordering::Relaxed);
        loop {
            let first_slot = &self.slots[position as usize & (QUEUE_SLOTS - 1)];
            let first_difference = first_slot
                .sequence
                .load(Ordering::Acquire)
                .wrapping_sub(position) as i32;
            let second_slot =
                second.map(|_| &self.slots[position.wrapping_add(1) as usize & (QUEUE_SLOTS - 1)]);
            let second_difference = second_slot.map_or(0, |slot| {
                slot.sequence
                    .load(Ordering::Acquire)
                    .wrapping_sub(position.wrapping_add(1)) as i32
            });
            if first_difference == 0 && second_difference == 0 {
                match self.enqueue.compare_exchange_weak(
                    position,
                    position.wrapping_add(count),
                    Ordering::Relaxed,
                    Ordering::Relaxed,
                ) {
                    Ok(_) => {
                        let used = self.occupancy.fetch_add(count, Ordering::AcqRel) + count;
                        self.high_water.fetch_max(used, Ordering::Relaxed);
                        unsafe { first_slot.value.get().write(first) };
                        if let (Some(slot), Some(value)) = (second_slot, second) {
                            unsafe { slot.value.get().write(value) };
                        }
                        first_slot
                            .sequence
                            .store(position.wrapping_add(1), Ordering::Release);
                        if let Some(slot) = second_slot {
                            slot.sequence
                                .store(position.wrapping_add(2), Ordering::Release);
                        }
                        return Ok(());
                    }
                    Err(actual) => position = actual,
                }
            } else if first_difference < 0 || second_difference < 0 {
                return Err(PushError::Full);
            } else {
                position = self.enqueue.load(Ordering::Relaxed);
            }
        }
    }

    fn pop(&self) -> Option<QueuedRecord> {
        let mut position = self.dequeue.load(Ordering::Relaxed);
        loop {
            let slot = &self.slots[position as usize & (QUEUE_SLOTS - 1)];
            let sequence = slot.sequence.load(Ordering::Acquire);
            let difference = sequence.wrapping_sub(position.wrapping_add(1)) as i32;
            if difference == 0 {
                match self.dequeue.compare_exchange_weak(
                    position,
                    position.wrapping_add(1),
                    Ordering::Relaxed,
                    Ordering::Relaxed,
                ) {
                    Ok(_) => {
                        let value = unsafe { slot.value.get().read() };
                        slot.sequence
                            .store(position.wrapping_add(QUEUE_SLOTS as u32), Ordering::Release);
                        self.occupancy.fetch_sub(1, Ordering::AcqRel);
                        return Some(value);
                    }
                    Err(actual) => position = actual,
                }
            } else if difference < 0 {
                return None;
            } else {
                position = self.dequeue.load(Ordering::Relaxed);
            }
        }
    }

    fn high_water_bytes(&self) -> u32 {
        self.high_water.load(Ordering::Relaxed) * size_of::<Slot>() as u32
    }
}

const _: () = assert!(QUEUE_SLOTS.is_power_of_two());
const _: () = assert!(size_of::<Queue>() <= QUEUE_BYTES);
const _: () = assert!(EVENT_RESERVE_SLOTS * size_of::<Slot>() as u32 >= EVENT_RESERVE_BYTES as u32);

pub struct Logger {
    initialized: AtomicU8,
    normal_push_lock: AtomicU8,
    normal_count: AtomicU32,
    normal_dropped: AtomicU32,
    event_dropped: AtomicU32,
    dropout: critical_section::Mutex<RefCell<Dropout>>,
    queue: Queue,
}

#[derive(Clone, Copy)]
struct Dropout {
    active: bool,
    closing: bool,
    start_ms: u32,
    last_ms: u32,
    pending: bool,
    pending_start_ms: u32,
    pending_last_ms: u32,
}

impl Dropout {
    const EMPTY: Self = Self {
        active: false,
        closing: false,
        start_ms: 0,
        last_ms: 0,
        pending: false,
        pending_start_ms: 0,
        pending_last_ms: 0,
    };
}

impl Logger {
    pub const fn new() -> Self {
        Self {
            initialized: AtomicU8::new(0),
            normal_push_lock: AtomicU8::new(0),
            normal_count: AtomicU32::new(0),
            normal_dropped: AtomicU32::new(0),
            event_dropped: AtomicU32::new(0),
            dropout: critical_section::Mutex::new(RefCell::new(Dropout::EMPTY)),
            queue: Queue::new(),
        }
    }

    pub fn initialize(&self) -> bool {
        if self
            .initialized
            .compare_exchange(0, 1, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return false;
        }
        self.normal_push_lock.store(0, Ordering::Relaxed);
        self.normal_count.store(0, Ordering::Relaxed);
        self.normal_dropped.store(0, Ordering::Relaxed);
        self.event_dropped.store(0, Ordering::Relaxed);
        critical_section::with(|section| {
            *self.dropout.borrow(section).borrow_mut() = Dropout::EMPTY;
        });
        self.queue.initialize();
        self.initialized.store(2, Ordering::Release);
        true
    }

    pub fn try_record(&self, class: Class, now_us: u64, record: Record) -> Result<(), PushError> {
        if self.initialized.load(Ordering::Acquire) != 2 {
            return Err(PushError::NotInitialized);
        }
        let queued = QueuedRecord::new(class, record)?;
        match class {
            Class::Normal => self.push_normal(now_us, queued),
            Class::Event => match self.queue.try_push(queued) {
                Ok(()) => Ok(()),
                Err(error) => {
                    self.record_drop(Class::Event, now_us);
                    Err(error)
                }
            },
        }
    }

    pub fn pop(&self) -> Option<QueuedRecord> {
        let record = self.queue.pop()?;
        if record.class == Class::Normal {
            self.normal_count.fetch_sub(1, Ordering::AcqRel);
        }
        Some(record)
    }

    pub fn stats(&self) -> Stats {
        Stats {
            normal_dropped: self.normal_dropped.load(Ordering::Relaxed),
            event_dropped: self.event_dropped.load(Ordering::Relaxed),
            high_water_bytes: self.queue.high_water_bytes(),
        }
    }

    fn push_normal(&self, now_us: u64, queued: QueuedRecord) -> Result<(), PushError> {
        if self
            .normal_push_lock
            .compare_exchange(0, 1, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            self.record_drop(Class::Normal, now_us);
            return Err(PushError::Full);
        }
        if !self.reserve_normal() {
            self.record_drop(Class::Normal, now_us);
            self.normal_push_lock.store(0, Ordering::Release);
            return Err(PushError::Full);
        }

        let dropout = critical_section::with(|section| {
            let mut dropout = self.dropout.borrow(section).borrow_mut();
            if dropout.active {
                dropout.closing = true;
                Some((dropout.start_ms, dropout.last_ms))
            } else {
                None
            }
        });
        let result = if let Some((start_ms, last_ms)) = dropout {
            let now = (now_us / 1_000) as u32;
            let duration = last_ms
                .wrapping_sub(start_ms)
                .max(now.wrapping_sub(start_ms))
                .min(u32::from(u16::MAX)) as u16;
            let marker = QueuedRecord::new(Class::Event, ulog::dropout(duration))
                .expect("dropout record fits runtime queue");
            self.queue.try_push_pair(marker, queued)
        } else {
            self.queue.try_push(queued)
        };
        if dropout.is_some() {
            critical_section::with(|section| {
                let mut dropout = self.dropout.borrow(section).borrow_mut();
                if result.is_ok() {
                    if dropout.pending {
                        dropout.active = true;
                        dropout.closing = false;
                        dropout.start_ms = dropout.pending_start_ms;
                        dropout.last_ms = dropout.pending_last_ms;
                        dropout.pending = false;
                    } else {
                        *dropout = Dropout::EMPTY;
                    }
                } else {
                    dropout.closing = false;
                    if dropout.pending {
                        dropout.last_ms = dropout.pending_last_ms;
                        dropout.pending = false;
                    }
                }
            });
        }
        if result.is_err() {
            self.normal_count.fetch_sub(1, Ordering::AcqRel);
            self.record_drop(Class::Normal, now_us);
        }
        self.normal_push_lock.store(0, Ordering::Release);
        result
    }

    fn reserve_normal(&self) -> bool {
        let mut count = self.normal_count.load(Ordering::Relaxed);
        loop {
            if count >= NORMAL_LIMIT {
                return false;
            }
            match self.normal_count.compare_exchange_weak(
                count,
                count + 1,
                Ordering::AcqRel,
                Ordering::Relaxed,
            ) {
                Ok(_) => return true,
                Err(actual) => count = actual,
            }
        }
    }

    fn record_drop(&self, class: Class, now_us: u64) {
        match class {
            Class::Normal => {
                self.normal_dropped.fetch_add(1, Ordering::Relaxed);
            }
            Class::Event => {
                self.event_dropped.fetch_add(1, Ordering::Relaxed);
            }
        }
        let now_ms = (now_us / 1_000) as u32;
        critical_section::with(|section| {
            let mut dropout = self.dropout.borrow(section).borrow_mut();
            if dropout.closing {
                if !dropout.pending {
                    dropout.pending = true;
                    dropout.pending_start_ms = now_ms;
                }
                dropout.pending_last_ms = now_ms;
            } else {
                if !dropout.active {
                    dropout.active = true;
                    dropout.start_ms = now_ms;
                }
                dropout.last_ms = now_ms;
            }
        });
    }
}

impl Default for Logger {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn queue_reserves_event_capacity_and_wraps() {
        let logger = Logger::new();
        assert_eq!(
            logger.try_record(Class::Normal, 0, ulog::dropout(0)),
            Err(PushError::NotInitialized)
        );
        assert!(logger.initialize());
        for value in 0..NORMAL_LIMIT {
            logger
                .try_record(
                    Class::Normal,
                    u64::from(value),
                    Record::from_bytes(&value.to_le_bytes()).unwrap(),
                )
                .unwrap();
        }
        assert_eq!(
            logger.try_record(Class::Normal, 1, ulog::dropout(0)),
            Err(PushError::Full)
        );
        for _ in 0..EVENT_RESERVE_SLOTS {
            logger
                .try_record(Class::Event, 1, ulog::dropout(0))
                .unwrap();
        }
        assert_eq!(
            logger.try_record(Class::Event, 1, ulog::dropout(0)),
            Err(PushError::Full)
        );
        for _ in 0..QUEUE_SLOTS {
            assert!(logger.pop().is_some());
        }
        assert!(logger.pop().is_none());
        for _ in 0..QUEUE_SLOTS * 2 {
            logger
                .try_record(Class::Event, 1, ulog::dropout(0))
                .unwrap();
            assert!(logger.pop().is_some());
        }
    }

    #[test]
    fn dropout_precedes_first_recovery_record() {
        let logger = Logger::new();
        logger.initialize();
        for _ in 0..NORMAL_LIMIT {
            logger
                .try_record(Class::Normal, 0, Record::from_bytes(&[1]).unwrap())
                .unwrap();
        }
        assert_eq!(
            logger.try_record(Class::Normal, 1_000, Record::from_bytes(&[2]).unwrap()),
            Err(PushError::Full)
        );
        assert_eq!(logger.pop().unwrap().as_bytes(), &[1]);
        logger
            .try_record(Class::Normal, 3_000, Record::from_bytes(&[4]).unwrap())
            .unwrap();
        for _ in 1..NORMAL_LIMIT {
            assert_eq!(logger.pop().unwrap().as_bytes(), &[1]);
        }
        assert_eq!(logger.pop().unwrap().as_bytes()[2], b'O');
        assert_eq!(logger.pop().unwrap().as_bytes(), &[4]);
        assert_eq!(logger.stats().normal_dropped, 1);
    }

    #[test]
    fn full_event_reserve_keeps_dropout_pending() {
        let logger = Logger::new();
        logger.initialize();
        for _ in 0..NORMAL_LIMIT {
            logger
                .try_record(Class::Normal, 0, Record::from_bytes(&[1]).unwrap())
                .unwrap();
        }
        assert!(logger
            .try_record(Class::Normal, 1_000, Record::from_bytes(&[2]).unwrap())
            .is_err());
        for _ in 0..EVENT_RESERVE_SLOTS {
            logger
                .try_record(Class::Event, 1_000, ulog::dropout(1))
                .unwrap();
        }
        assert!(logger.pop().is_some());
        assert!(logger
            .try_record(Class::Normal, 2_000, Record::from_bytes(&[3]).unwrap())
            .is_err());
        assert!(logger.pop().is_some());
        logger
            .try_record(Class::Normal, 3_000, Record::from_bytes(&[4]).unwrap())
            .unwrap();
        while let Some(record) = logger.pop() {
            if record.as_bytes() == [4] {
                return;
            }
        }
        panic!("recovery record missing");
    }

    #[test]
    fn concurrent_producers_publish_complete_records() {
        let logger = Logger::new();
        logger.initialize();
        std::thread::scope(|scope| {
            for producer in 0..4_u8 {
                let logger = &logger;
                scope.spawn(move || {
                    for sequence in 0..50_u8 {
                        loop {
                            if logger
                                .try_record(
                                    Class::Normal,
                                    u64::from(sequence),
                                    Record::from_bytes(&[producer, sequence]).unwrap(),
                                )
                                .is_ok()
                            {
                                break;
                            }
                            core::hint::spin_loop();
                        }
                    }
                });
            }
        });
        let mut seen = [[false; 50]; 4];
        let mut records = 0;
        while let Some(record) = logger.pop() {
            let bytes = record.as_bytes();
            if bytes.len() != 2 {
                continue;
            }
            let slot = &mut seen[usize::from(bytes[0])][usize::from(bytes[1])];
            assert!(!*slot);
            *slot = true;
            records += 1;
        }
        assert_eq!(records, 200);
        assert!(seen.into_iter().flatten().all(|value| value));
        assert!(logger.stats().high_water_bytes <= QUEUE_BYTES as u32);
    }

    #[test]
    fn queue_positions_cross_u32_wrap() {
        let logger = Logger::new();
        logger.initialize();
        let base = u32::MAX - (QUEUE_SLOTS as u32 - 1);
        logger.queue.enqueue.store(base, Ordering::Relaxed);
        logger.queue.dequeue.store(base, Ordering::Relaxed);
        for (index, slot) in logger.queue.slots.iter().enumerate() {
            slot.sequence
                .store(base.wrapping_add(index as u32), Ordering::Relaxed);
        }
        for value in 0..QUEUE_SLOTS * 2 {
            logger
                .try_record(
                    Class::Event,
                    value as u64,
                    Record::from_bytes(&(value as u32).to_le_bytes()).unwrap(),
                )
                .unwrap();
            assert_eq!(
                logger.pop().unwrap().as_bytes(),
                &(value as u32).to_le_bytes()
            );
        }
    }
}
