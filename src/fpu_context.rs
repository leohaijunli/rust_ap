//! Shared H753 floating-point context policy for every firmware entry point.

use cortex_m::peripheral::FPU;

// ARM CMSIS 5.9.0 core_cm7.h: architectural FPCCR control/status bits.
pub const FPCCR_ASPEN: u32 = 1 << 31;
pub const FPCCR_LSPEN: u32 = 1 << 30;
pub const FPCCR_LSPACT: u32 = 1;

pub fn init(fpu: &mut FPU) {
    // CP10/11 access alone does not preserve registers across IRQs. Use eager
    // hardware saving of S0-S15/FPSCR; callee-saved FP registers follow the ABI.
    cortex_m::interrupt::free(|_| {
        assert_eq!(fpu.fpccr.read() & FPCCR_LSPACT, 0);
        // SAFETY: the caller owns FPU and calls before the HAL/executors start.
        // IRQs are masked and no pending lazy FP frame is being changed.
        unsafe {
            fpu.fpccr
                .modify(|value| (value | FPCCR_ASPEN) & !FPCCR_LSPEN);
        }
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
        assert_eq!(fpu.fpccr.read() & (FPCCR_ASPEN | FPCCR_LSPEN), FPCCR_ASPEN);
    });
}
