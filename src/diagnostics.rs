//! P1 periodic substitute, masked-WFE CPU accounting, shared-stack watermark.

use core::cell::Cell;
use core::fmt;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use cortex_m::interrupt::{free, Mutex};
use cortex_m::peripheral::DWT;
use embassy_executor::{InterruptExecutor, SendSpawner};
use embassy_stm32::{interrupt, peripherals, Peri};
use embassy_time::{Instant, Timer};
use v6x_rust_led::metrics::{CpuWindow, PeriodStats, PERIOD_US};

static FAST_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static IDLE_US: AtomicU32 = AtomicU32::new(0);
static IDLE_CALLS: AtomicU32 = AtomicU32::new(0);
static IDLE_EXIT_MAX_CYCLES: AtomicU32 = AtomicU32::new(0);
static MIN_MSP: AtomicU32 = AtomicU32::new(u32::MAX);
static RESET_REQUEST: AtomicBool = AtomicBool::new(false);

#[derive(Clone, Copy)]
struct IrqStats {
    count: u64,
    sum_cycles: u64,
    max_cycles: u32,
}

impl IrqStats {
    const EMPTY: Self = Self {
        count: 0,
        sum_cycles: 0,
        max_cycles: 0,
    };
}

#[derive(Clone, Copy)]
struct Snapshot {
    epoch: u32,
    at_us: u64,
    stats: PeriodStats,
    cpu: CpuWindow,
    irq: IrqStats,
}

impl Snapshot {
    const EMPTY: Self = Self {
        epoch: 0,
        at_us: 0,
        stats: PeriodStats::EMPTY,
        cpu: CpuWindow::EMPTY,
        irq: IrqStats::EMPTY,
    };
}

static IRQ: Mutex<Cell<IrqStats>> = Mutex::new(Cell::new(IrqStats::EMPTY));
static SNAPSHOT: Mutex<Cell<Snapshot>> = Mutex::new(Cell::new(Snapshot::EMPTY));

#[derive(Clone, Copy)]
struct StackSnapshot {
    at_us: u64,
    untouched_bytes: u32,
    guard_ok: bool,
}

static STACK: Mutex<Cell<StackSnapshot>> = Mutex::new(Cell::new(StackSnapshot {
    at_us: 0,
    untouched_bytes: 65_536,
    guard_ok: false,
}));

unsafe extern "C" {
    static _stack_start: u32;
    static _stack_end: u32;
    static __euninit: u32;
    static __fault_stack_bottom: u32;
}

pub fn start(timer: Peri<'static, peripherals::TIM7>) -> SendSpawner {
    // TIM7 is a reserved software-pend vector; the timer peripheral is not run.
    let spawner = FAST_EXECUTOR.start(interrupt::TIM7);
    spawner.spawn(periodic(timer)).unwrap();
    spawner
}

#[interrupt]
unsafe fn TIM7() {
    let start = DWT::cycle_count();
    sample_stack_pointer();
    // Called only from TIM7, after start(), and never reentrantly.
    unsafe { FAST_EXECUTOR.on_interrupt() };
    let cycles = DWT::cycle_count().wrapping_sub(start);
    free(|cs| {
        let mut irq = IRQ.borrow(cs).get();
        irq.count = irq.count.saturating_add(1);
        irq.sum_cycles = irq.sum_cycles.saturating_add(u64::from(cycles));
        irq.max_cycles = irq.max_cycles.max(cycles);
        IRQ.borrow(cs).set(irq);
    });
}

fn cpu_counters(now: u64) -> (u64, u32, u32) {
    (
        now,
        IDLE_US.load(Ordering::Relaxed),
        IDLE_CALLS.load(Ordering::Relaxed),
    )
}

#[embassy_executor::task]
async fn periodic(_timer: Peri<'static, peripherals::TIM7>) {
    let mut stats = PeriodStats::EMPTY;
    let mut epoch = 0u32;
    let mut cpu_start = cpu_counters(Instant::now().as_micros());
    let mut next_snapshot = cpu_start.0 + 1_000_000;
    let mut release = cpu_start.0 + PERIOD_US;
    let mut state = 0x3141_5926u32;
    loop {
        Timer::at(Instant::from_micros(release)).await;
        if RESET_REQUEST.swap(false, Ordering::Relaxed) {
            epoch = epoch.wrapping_add(1);
            stats = PeriodStats::EMPTY;
            cpu_start = cpu_counters(Instant::now().as_micros());
            next_snapshot = cpu_start.0 + 1_000_000;
            free(|cs| IRQ.borrow(cs).set(IrqStats::EMPTY));
        }
        let start_cycles = DWT::cycle_count();
        let start_us = Instant::now().as_micros();
        // Fixed, bounded integer work only. This is neither an IMU sample nor a
        // control calculation; black_box prevents its removal from release builds.
        for _ in 0..32 {
            state = core::hint::black_box(state.rotate_left(5).wrapping_add(0x9e37_79b9));
        }
        let end_us = Instant::now().as_micros();
        let cycles = DWT::cycle_count().wrapping_sub(start_cycles);
        release = stats.record(release, start_us, end_us, cycles);

        let now = Instant::now().as_micros();
        if now >= next_snapshot {
            let cpu_end = cpu_counters(now);
            let cpu = CpuWindow::between(cpu_start, cpu_end);
            // Only the final snapshot copy is inside a global critical section.
            let irq = free(|cs| IRQ.borrow(cs).get());
            let snapshot = Snapshot {
                epoch,
                at_us: now,
                stats,
                cpu,
                irq,
            };
            free(|cs| SNAPSHOT.borrow(cs).set(snapshot));
            cpu_start = cpu_end;
            next_snapshot = now + 1_000_000;
        }
        release = stats.finish_poll(release, Instant::now().as_micros());
    }
}

/// Called only from the thread executor, with interrupts initially enabled.
/// SEVONPEND is set by platform::init. Thread wakeups use Embassy's SEV.
/// No ISR can execute inside the timed WFE bracket; its time stays in busy time.
pub fn idle() {
    free(|_| {
        let start = embassy_stm32::pac::TIM2.cnt().read() as u16;
        cortex_m::asm::dsb();
        cortex_m::asm::wfe();
        let end = embassy_stm32::pac::TIM2.cnt().read() as u16;
        let exit_start = DWT::cycle_count();
        // TIM2 has a half-overflow interrupt at most 32768 us away. It wakes
        // this shallow sleep, so the u16 wrapping difference is unambiguous.
        IDLE_US.fetch_add(u32::from(end.wrapping_sub(start)), Ordering::Relaxed);
        IDLE_CALLS.fetch_add(1, Ordering::Relaxed);
        IDLE_EXIT_MAX_CYCLES.fetch_max(
            DWT::cycle_count().wrapping_sub(exit_start),
            Ordering::Relaxed,
        );
    });
}

pub fn request_reset() {
    RESET_REQUEST.store(true, Ordering::Relaxed);
}

pub fn cpu_load() -> Option<u16> {
    let snapshot = free(|cs| SNAPSHOT.borrow(cs).get());
    (snapshot.cpu.valid && Instant::now().as_micros().saturating_sub(snapshot.at_us) < 2_000_000)
        .then_some(snapshot.cpu.busy_permille)
}

fn sample_stack_pointer() -> u32 {
    let sp = cortex_m::register::msp::read();
    MIN_MSP.fetch_min(sp, Ordering::Relaxed);
    sp
}

pub async fn stack_monitor() -> ! {
    let bottom = core::ptr::addr_of!(_stack_end) as u32;
    let top = core::ptr::addr_of!(_stack_start) as u32;
    let mut minimum = top - bottom;
    loop {
        let sp = sample_stack_pointer().min(MIN_MSP.load(Ordering::Relaxed));
        if sp < bottom || sp > top {
            crate::fault::stack_guard_failed(sp);
        }
        let mut cursor = bottom;
        // Read only painted memory below the lowest sampled live stack. This
        // bounded scan is preemptible; never mask interrupts across 64 KiB.
        while cursor < sp && unsafe { (cursor as *const u32).read_volatile() } == 0xcccc_cccc {
            cursor += 4;
        }
        minimum = minimum.min(cursor - bottom);
        let guard_ok = minimum >= 256;
        let stack = StackSnapshot {
            at_us: Instant::now().as_micros(),
            untouched_bytes: minimum,
            guard_ok,
        };
        free(|cs| STACK.borrow(cs).set(stack));
        if !guard_ok {
            crate::fault::stack_guard_failed(sp);
        }
        Timer::after_secs(1).await;
    }
}

pub fn write_diagnostics(out: &mut impl fmt::Write) -> fmt::Result {
    let snapshot = free(|cs| SNAPSHOT.borrow(cs).get());
    let s = snapshot.stats;
    let c = snapshot.cpu;
    let irq = snapshot.irq;
    let stack = free(|cs| STACK.borrow(cs).get());
    writeln!(
        out,
        "P1_METRICS schema=1 epoch={} snapshot_us={} now_us={}",
        snapshot.epoch,
        snapshot.at_us,
        Instant::now().as_micros()
    )?;
    writeln!(out, "PERIOD target_us={PERIOD_US} samples={} skipped={} deadline_misses={} scheduling_overruns={} scheduling_skipped={} time_errors={} last_start_us={} interval_min_us={} interval_max_us={} late_max_us={} late_sum_us={}",
        s.samples, s.skipped, s.deadline_misses, s.scheduling_overruns, s.scheduling_skipped, s.time_errors, s.last_start_us,
        if s.samples < 2 { 0 } else { s.interval_min_us }, s.interval_max_us, s.late_max_us, s.late_sum_us)?;
    writeln!(
        out,
        "LATE_HIST le1={} le2={} le5={} le10={} le20={} le50={} le100={} le1000={} gt1000={}",
        s.late_hist[0],
        s.late_hist[1],
        s.late_hist[2],
        s.late_hist[3],
        s.late_hist[4],
        s.late_hist[5],
        s.late_hist[6],
        s.late_hist[7],
        s.late_hist[8]
    )?;
    writeln!(out, "EXEC body_min_cycles={} body_max_cycles={} body_sum_cycles={} invalid_cycles={} irq_count={} irq_max_cycles={} irq_sum_cycles={}",
        if s.samples == 0 { 0 } else { s.body_min_cycles }, s.body_max_cycles, s.body_sum_cycles,
        s.invalid_cycles, irq.count, irq.max_cycles, irq.sum_cycles)?;
    writeln!(out, "CPU mode=masked-wfe valid={} window_us={} idle_us={} idle_calls={} busy_permille={} exit_bookkeeping_max_cycles={}",
        c.valid, c.wall_us, c.idle_us, c.idle_calls, c.busy_permille, IDLE_EXIT_MAX_CYCLES.load(Ordering::Relaxed))?;
    writeln!(
        out,
        "STACK at_us={} bytes=65536 untouched_bytes={} touched_bytes={} min_msp={:08x} guard_ok={}",
        stack.at_us,
        stack.untouched_bytes,
        65_536 - stack.untouched_bytes,
        MIN_MSP.load(Ordering::Relaxed),
        stack.guard_ok
    )?;
    let static_end = core::ptr::addr_of!(__euninit) as u32;
    let fault_bottom = core::ptr::addr_of!(__fault_stack_bottom) as u32;
    writeln!(
        out,
        "RAM static_end={static_end:08x} unallocated_bytes={} fault_stack_bytes=1024",
        fault_bottom - static_end
    )
}
