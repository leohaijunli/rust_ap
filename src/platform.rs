//! Fixed FMUv6X Rev 3 platform. Source references: docs/fmuv6x-hardware.md.

use core::fmt;

use cortex_m::peripheral::SCB;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::pac::{DBGMCU, FLASH, PWR, RCC, SYSCFG, TIM2};
use embassy_stm32::rcc::{
    mux, AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllDiv, PllMul, PllPreDiv, PllSource,
    Sysclk, VoltageScale,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, Peripherals};

pub const CPU_HZ: u32 = 480_000_000;
pub const HCLK_HZ: u32 = 240_000_000;
pub const PCLK_HZ: u32 = 120_000_000;
pub const TICK_HZ: u32 = 1_000_000;

pub fn init() -> Peripherals {
    crate::fault::capture_before_hal();
    // The only supported board has revision V silicon (PX4 MCU_REV_V=0x2003).
    assert_eq!(DBGMCU.idc().read().rev_id(), 0x2003);
    let mut core = cortex_m::Peripherals::take().expect("core peripherals taken once");
    core.SCB.disable_dcache(&mut core.CPUID);
    core.SCB.enable_icache();
    core.DCB.enable_trace();
    cortex_m::peripheral::DWT::unlock();
    core.DWT.set_cycle_count(0);
    core.DWT.enable_cycle_counter();

    // All four implemented priority bits are preemption priority. Shallow WFE
    // wakes on an interrupt becoming pending even while PRIMASK is set, so idle
    // accounting can stop BEFORE any ISR runs. No SLEEPONEXIT/STOP mode.
    unsafe {
        core.SCB.aircr.write(0x05fa_0000);
        core.SCB
            .scr
            .modify(|value| (value | (1 << 4)) & !((1 << 2) | (1 << 1)));
    }
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    let mut config = Config::default();
    config.rcc.hsi48 = None;
    config.rcc.hse = Some(Hse {
        freq: Hertz(16_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL60,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV20),
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.d1c_pre = AHBPrescaler::DIV1;
    config.rcc.ahb_pre = AHBPrescaler::DIV2;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.apb3_pre = APBPrescaler::DIV2;
    config.rcc.apb4_pre = APBPrescaler::DIV2;
    // On H753, embassy-stm32 enables SCUEN/LDO and reaches Scale0 via
    // Scale1 + SYSCFG.ODEN. It selects LATENCY=4/WRHIGHFREQ=2 at HCLK=240 MHz.
    config.rcc.voltage_scale = VoltageScale::Scale0;
    config.rcc.pll3 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL48,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV4),
        divr: Some(PllDiv::DIV2),
    });
    config.rcc.mux.usbsel = mux::Usbsel::PLL3_Q;
    config.rcc.mux.fdcansel = mux::Fdcansel::HSE;
    config.rcc.mux.sdmmcsel = mux::Sdmmcsel::PLL1_Q;
    config.rcc.mux.spi45sel = mux::Spi45sel::PCLK2;
    let p = crate::backup_domain::init(config);
    assert!(crate::backup_domain::preserved());

    embassy_stm32::interrupt::TIM2.set_priority(Priority::P1);
    embassy_stm32::interrupt::EXTI15_10.set_priority(Priority::P0);
    embassy_stm32::interrupt::TIM7.set_priority(Priority::P3);
    embassy_stm32::interrupt::TIM6_DAC.set_priority(Priority::P4);
    embassy_stm32::interrupt::FDCAN1_IT0.set_priority(Priority::P2);
    embassy_stm32::interrupt::FDCAN1_IT1.set_priority(Priority::P2);
    embassy_stm32::interrupt::SDMMC2.set_priority(Priority::P7);
    embassy_stm32::interrupt::OTG_FS.set_priority(Priority::P6);
    embassy_stm32::interrupt::DMA2_STREAM0.set_priority(Priority::P7);
    embassy_stm32::interrupt::DMA2_STREAM1.set_priority(Priority::P7);
    embassy_stm32::interrupt::DMA1_STREAM2.set_priority(Priority::P1);
    embassy_stm32::interrupt::DMA1_STREAM3.set_priority(Priority::P1);
    embassy_stm32::interrupt::I2C3_EV.set_priority(Priority::P6);
    embassy_stm32::interrupt::I2C3_ER.set_priority(Priority::P6);
    embassy_stm32::interrupt::DMA1_STREAM4.set_priority(Priority::P6);
    embassy_stm32::interrupt::DMA1_STREAM5.set_priority(Priority::P6);
    assert!(clock_registers_ok());
    p
}

pub fn clock_registers_ok() -> bool {
    // SCUEN controls supply updates; it is not a regulator-ready flag. This
    // Rev V board reads CR3=0x05000042 before/after the working P0 HAL init.
    // CR3 supply bits are POR/write-once protected, so validate the actual
    // LDO/bypass state and voltage readiness, not whether SCUEN stayed set.
    let supply = PWR.cr3().read();
    let pll_source = RCC.pllckselr().read();
    let pll_config = RCC.pllcfgr().read();
    let pll1 = RCC.plldivr(0).read();
    let pll3 = RCC.plldivr(2).read();
    RCC.cfgr().read().sws() == Sysclk::PLL1_P
        && pll_source.pllsrc() == PllSource::HSE
        && pll_source.divm(0) == PllPreDiv::DIV1
        && pll_source.divm(2) == PllPreDiv::DIV4
        && pll1.plln() == PllMul::MUL60
        && pll1.pllp() == PllDiv::DIV2
        && pll1.pllq() == PllDiv::DIV20
        && pll3.plln() == PllMul::MUL48
        && pll3.pllq() == PllDiv::DIV4
        && pll_config.divpen(0)
        && pll_config.divqen(0)
        && pll_config.divqen(2)
        && !pll_config.pllfracen(0)
        && !pll_config.pllfracen(2)
        && RCC.d2ccip2r().read().usbsel() == mux::Usbsel::PLL3_Q
        && RCC.d1ccipr().read().sdmmcsel() == mux::Sdmmcsel::PLL1_Q
        && RCC.d1cfgr().read().d1cpre() == AHBPrescaler::DIV1
        && RCC.d1cfgr().read().hpre() == AHBPrescaler::DIV2
        && RCC.d1cfgr().read().d1ppre() == APBPrescaler::DIV2
        && RCC.d2cfgr().read().d2ppre1() == APBPrescaler::DIV2
        && RCC.d2cfgr().read().d2ppre2() == APBPrescaler::DIV2
        && RCC.d3cfgr().read().d3ppre() == APBPrescaler::DIV2
        && RCC.cr().read().hserdy()
        && !RCC.cr().read().hsebyp()
        && RCC.cr().read().pllrdy(0)
        && RCC.cr().read().pllrdy(2)
        && supply.ldoen()
        && !supply.bypass()
        && PWR.d3cr().read().vosrdy()
        && PWR.d3cr().read().vos() == embassy_stm32::pac::pwr::vals::Vos::SCALE1
        && PWR.csr1().read().actvosrdy()
        && SYSCFG.pwrcr().read().oden() == 1
        && FLASH.acr().read().latency() == 4
        && FLASH.acr().read().wrhighfreq() == 2
        && TIM2.psc().read() == 239
        && TIM2.arr().read() == 65_535
}

pub fn write_diagnostics(out: &mut impl fmt::Write) -> fmt::Result {
    let scb = unsafe { &*SCB::PTR };
    writeln!(
        out,
        "P1_PLATFORM schema=2 version={} source={} git={} fault_injection={}",
        env!("CARGO_PKG_VERSION"),
        env!("BUILD_SOURCE_ID"),
        env!("BUILD_GIT"),
        cfg!(feature = "fault-injection")
    )?;
    crate::backup_domain::write_diagnostics(out, "BACKUP")?;
    writeln!(out, "CLOCK configured_cpu_hz={CPU_HZ} hclk_hz={HCLK_HZ} pclk_hz={PCLK_HZ} usb_hz=48000000 tick_hz={TICK_HZ} registers_ok={}", clock_registers_ok())?;
    writeln!(out, "RCC cr={:08x} cfgr={:08x} d1={:08x} d2={:08x} d3={:08x} pllck={:08x} pllcfg={:08x} pll1={:08x} pll3={:08x} usbsel={:08x}",
        RCC.cr().read().0, RCC.cfgr().read().0, RCC.d1cfgr().read().0, RCC.d2cfgr().read().0, RCC.d3cfgr().read().0,
        RCC.pllckselr().read().0, RCC.pllcfgr().read().0, RCC.plldivr(0).read().0, RCC.plldivr(2).read().0, RCC.d2ccip2r().read().0)?;
    writeln!(
        out,
        "POWER cr3={:08x} d3cr={:08x} csr1={:08x} syscfg={:08x} flash_acr={:08x}",
        PWR.cr3().read().0,
        PWR.d3cr().read().0,
        PWR.csr1().read().0,
        SYSCFG.pwrcr().read().0,
        FLASH.acr().read().0
    )?;
    writeln!(out, "CORE id={:08x} vtor={:08x} ccr={:08x} scr={:08x} aircr={:08x} dwt_ctrl={:08x} reset_flags={:08x}",
        DBGMCU.idc().read().0, scb.vtor.read(), scb.ccr.read(), scb.scr.read(), scb.aircr.read(),
        unsafe { (*cortex_m::peripheral::DWT::PTR).ctrl.read() }, crate::fault::reset_flags())?;
    writeln!(out, "TIMER name=TIM2 psc={} arr={} cr1={:08x} irq_priority={} fast_irq_priority={} usb_irq_priority={}",
        TIM2.psc().read(), TIM2.arr().read(), TIM2.cr1().read().0,
        embassy_stm32::interrupt::TIM2.get_priority() as u8,
        embassy_stm32::interrupt::TIM7.get_priority() as u8,
        embassy_stm32::interrupt::OTG_FS.get_priority() as u8)
}
