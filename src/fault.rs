//! Bounded, allocation-free fault recording and PX4 bootloader recovery.

use core::cell::Cell;
use core::fmt;
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::interrupt::{free, Mutex};
use cortex_m::peripheral::SCB;
use embassy_stm32::pac::{PWR, RCC, RTC};
use v6x_rust_led::build_info::SOURCE_ID;
use v6x_rust_led::fault_record::{self, FaultRecord, WORDS};

const FIRST_BKP: usize = 16;
static LAST: Mutex<Cell<Option<FaultRecord>>> = Mutex::new(Cell::new(None));
static RESET_FLAGS: AtomicU32 = AtomicU32::new(0);

fn enable_backup_access() -> bool {
    RCC.apb4enr().modify(|w| w.set_rtcapben(true));
    let _ = RCC.apb4enr().read();
    PWR.cr1().modify(|w| w.set_dbp(true));
    for _ in 0..10_000 {
        if PWR.cr1().read().dbp() {
            return true;
        }
    }
    false
}

fn read_record() -> Option<FaultRecord> {
    let mut record = FaultRecord([0; WORDS]);
    for (index, word) in record.0.iter_mut().enumerate() {
        *word = RTC.bkpr(FIRST_BKP + index).read().bkp();
    }
    record.valid().then_some(record)
}

fn write_record(record: FaultRecord) {
    if !enable_backup_access() {
        return;
    }
    RTC.bkpr(FIRST_BKP).write(|w| w.set_bkp(0));
    for index in 1..WORDS {
        RTC.bkpr(FIRST_BKP + index)
            .write(|w| w.set_bkp(record.0[index]));
    }
    cortex_m::asm::dsb();
    RTC.bkpr(FIRST_BKP).write(|w| w.set_bkp(record.0[0]));
    cortex_m::asm::dsb();
}

pub fn capture_before_hal() {
    RCC.apb4enr().modify(|w| w.set_rtcapben(true));
    let _ = RCC.apb4enr().read();
    free(|cs| LAST.borrow(cs).set(read_record()));
    RESET_FLAGS.store(RCC.rsr().read().0, Ordering::Relaxed);
    RCC.rsr().modify(|w| w.set_rmvf(true));
}

pub fn reset_flags() -> u32 {
    RESET_FLAGS.load(Ordering::Relaxed)
}

pub fn present() -> bool {
    free(|cs| LAST.borrow(cs).get().is_some())
}

pub fn clear() -> bool {
    if !enable_backup_access() {
        return false;
    }
    RTC.bkpr(FIRST_BKP).write(|w| w.set_bkp(0));
    cortex_m::asm::dsb();
    if RTC.bkpr(FIRST_BKP).read().bkp() != 0 {
        return false;
    }
    free(|cs| LAST.borrow(cs).set(None));
    true
}

pub fn write_diagnostics(out: &mut impl fmt::Write) -> fmt::Result {
    let last = free(|cs| LAST.borrow(cs).get());
    if let Some(FaultRecord(w)) = last {
        writeln!(
            out,
            "P1_FAULT schema=1 valid=true kind={} source={:08x}{:08x} detail={} file_hash={:08x}",
            w[1], w[3], w[2], w[4], w[5]
        )?;
        writeln!(
            out,
            "FRAME pc={:08x} lr={:08x} sp={:08x} xpsr={:08x} exc_return={:08x}",
            w[6], w[7], w[8], w[9], w[14]
        )?;
        writeln!(
            out,
            "FAULT_REG cfsr={:08x} hfsr={:08x} mmfar={:08x} bfar={:08x} crc={:08x}",
            w[10], w[11], w[12], w[13], w[15]
        )
    } else {
        writeln!(out, "P1_FAULT schema=1 valid=false")
    }
}

pub fn reboot_to_bootloader() -> ! {
    cortex_m::interrupt::disable();
    if enable_backup_access() {
        RTC.bkpr(0).write(|w| w.set_bkp(0xb007_b007));
    }
    cortex_m::asm::dsb();
    SCB::sys_reset()
}

fn recover(mut record: FaultRecord) -> ! {
    cortex_m::interrupt::disable();
    let scb = unsafe { &*SCB::PTR };
    record.0[10] = scb.cfsr.read();
    record.0[11] = scb.hfsr.read();
    record.0[12] = scb.mmfar.read();
    record.0[13] = scb.bfar.read();
    record.seal();
    write_record(record);
    reboot_to_bootloader()
}

#[panic_handler]
fn panic(info: &PanicInfo<'_>) -> ! {
    cortex_m::interrupt::disable();
    let mut record = FaultRecord::new(fault_record::PANIC, SOURCE_ID);
    record.0[8] = cortex_m::register::msp::read();
    if let Some(location) = info.location() {
        record.0[4] = location.line();
        record.0[5] = fault_record::file_hash(location.file());
    }
    // No fmt, USB, mutex, heap, or timer service in the panic path.
    recover(record)
}

pub fn stack_guard_failed(sp: u32) -> ! {
    let mut record = FaultRecord::new(fault_record::STACK_GUARD, SOURCE_ID);
    record.0[8] = sp;
    recover(record)
}

#[cortex_m_rt::exception]
unsafe fn DefaultHandler(irqn: i16) -> ! {
    let mut record = FaultRecord::new(fault_record::UNEXPECTED_EXCEPTION, SOURCE_ID);
    record.0[4] = irqn as i32 as u32;
    record.0[8] = cortex_m::register::msp::read();
    recover(record)
}

unsafe extern "C" {
    static __fault_stack_top: u32;
}

// Architecture glue only: switch MSP before Rust uses the potentially damaged
// application stack. No C/C++ code or external fault handler is linked.
#[unsafe(naked)]
#[unsafe(export_name = "HardFault")]
unsafe extern "C" fn hard_fault_entry() -> ! {
    core::arch::naked_asm!(
        "cpsid i",
        "mov r1, lr",
        "tst lr, #4",
        "ite eq",
        "mrseq r0, msp",
        "mrsne r0, psp",
        "ldr r2, ={stack}",
        "msr msp, r2",
        "isb",
        "b {handler}",
        stack = sym __fault_stack_top,
        handler = sym hard_fault,
    );
}

unsafe extern "C" fn hard_fault(sp: u32, exc_return: u32) -> ! {
    let cfsr = unsafe { (*SCB::PTR).cfsr.read() };
    let mut record = FaultRecord::new(fault_record::HARD_FAULT, SOURCE_ID);
    record.0[8] = sp;
    record.0[14] = exc_return;
    if fault_record::frame_is_readable(sp, cfsr, exc_return) {
        let frame = sp as *const u32;
        // The preempted stack is no longer executing. Validate all bounds and
        // stacking-error bits above before reading the hardware core frame.
        unsafe {
            record.0[7] = frame.add(5).read_volatile();
            record.0[6] = frame.add(6).read_volatile();
            record.0[9] = frame.add(7).read_volatile();
        }
        record.0[4] = 1; // core frame available
    }
    recover(record)
}
