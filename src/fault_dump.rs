//! P1 startup-regression diagnostics.
//!
//! Read retained state before the HAL can touch the backup domain, then bring
//! up the known-good P0 clock tree (HSI + PLL3Q/48 MHz USB) and stream the
//! captured data over USB CDC. Both images preserve the inherited RTC/LSE
//! configuration. This image checks all BKP words without writing them;
//! only an explicit !bootloader command writes the request in BKP0.

use core::fmt::{self, Write};

use cortex_m::peripheral::SCB;
use embassy_executor::raw::Executor;
use embassy_stm32::gpio::{Input, Level, Output, OutputOpenDrain, Pull, Speed};
use embassy_stm32::pac::{DBGMCU, FLASH, PWR, RCC, RTC, SYSCFG};
use embassy_stm32::rcc::{mux, Hse, HseMode, Pll, PllDiv, PllMul, PllPreDiv, PllSource};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::Driver;
use embassy_stm32::{Config as HalConfig, Peripherals};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::{Driver as UsbDriver, EndpointError};
use embassy_usb::{Builder, Config as UsbConfig};
use static_cell::StaticCell;
use v6x_rust_led::fault_record::{self, FaultRecord, WORDS};

const BOOTLOADER_REQUEST: u32 = 0xb007_b007;
const RCC_WORDS: usize = 11;

#[derive(Clone, Copy)]
struct Snapshot {
    bkp: [u32; 32],
    rsr: u32,
    rcc: [u32; RCC_WORDS],
    power: [u32; 3],
    pwrcr: u32,
    flash_acr: u32,
    idc: u32,
}

fn capture() -> Snapshot {
    // RTC backup registers are only visible after the APB4 interface is
    // clocked. Reading does not need the write-protection unlock.
    RCC.apb4enr().modify(|w| w.set_rtcapben(true));
    let _ = RCC.apb4enr().read();

    let mut bkp = [0; 32];
    for (index, word) in bkp.iter_mut().enumerate() {
        *word = RTC.bkpr(index).read().bkp();
    }

    Snapshot {
        bkp,
        rsr: RCC.rsr().read().0,
        rcc: [
            RCC.cr().read().0,
            RCC.cfgr().read().0,
            RCC.d1cfgr().read().0,
            RCC.d2cfgr().read().0,
            RCC.d3cfgr().read().0,
            RCC.pllckselr().read().0,
            RCC.pllcfgr().read().0,
            RCC.plldivr(0).read().0,
            RCC.plldivr(1).read().0,
            RCC.plldivr(2).read().0,
            RCC.d2ccip2r().read().0,
        ],
        power: [PWR.cr3().read().0, PWR.d3cr().read().0, PWR.csr1().read().0],
        pwrcr: SYSCFG.pwrcr().read().0,
        flash_acr: FLASH.acr().read().0,
        idc: DBGMCU.idc().read().0,
    }
}

fn init_p0() -> Peripherals {
    let mut core = cortex_m::Peripherals::take().expect("core peripherals taken once");
    crate::fpu_context::init(&mut core.FPU);
    // Exact known-good P0 tree: HSI 64 MHz core, PLL3Q 48 MHz USB.
    let mut config = HalConfig::default();
    config.rcc.hsi48 = None;
    config.rcc.hse = Some(Hse {
        freq: Hertz(16_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll3 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL48,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV4),
        divr: Some(PllDiv::DIV2),
    });
    config.rcc.mux.usbsel = mux::Usbsel::PLL3_Q;
    crate::backup_domain::init(config)
}

fn reboot_to_bootloader() -> ! {
    cortex_m::interrupt::disable();
    PWR.cr1().modify(|w| w.set_dbp(true));
    for _ in 0..10_000 {
        if PWR.cr1().read().dbp() {
            break;
        }
    }
    if PWR.cr1().read().dbp() {
        RTC.bkpr(0).write(|w| w.set_bkp(BOOTLOADER_REQUEST));
    }
    cortex_m::asm::dsb();
    SCB::sys_reset()
}

struct Report {
    bytes: [u8; 4096],
    len: usize,
}

impl Report {
    fn new() -> Self {
        Self {
            bytes: [0; 4096],
            len: 0,
        }
    }

    fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

impl Write for Report {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let end = self.len.checked_add(text.len()).ok_or(fmt::Error)?;
        if end > self.bytes.len() {
            return Err(fmt::Error);
        }
        self.bytes[self.len..end].copy_from_slice(text.as_bytes());
        self.len = end;
        Ok(())
    }
}

fn kind_name(kind: u32) -> &'static str {
    match kind {
        fault_record::PANIC => "PANIC",
        fault_record::HARD_FAULT => "HARD_FAULT",
        fault_record::UNEXPECTED_EXCEPTION => "UNEXPECTED_EXCEPTION",
        fault_record::STACK_GUARD => "STACK_GUARD",
        _ => "UNKNOWN",
    }
}

fn write_bkp(out: &mut impl Write, stage: &str, bkp: &[u32; 32]) -> fmt::Result {
    write!(
        out,
        "V6X_DUMP_BKP0 stage={stage} value={:08x} bootloader_request={}\r\n",
        bkp[0],
        bkp[0] == BOOTLOADER_REQUEST
    )?;
    write!(out, "V6X_DUMP_BKP16 stage={stage} values=")?;
    for word in &bkp[16..32] {
        write!(out, "{word:08x} ")?;
    }
    write!(out, "\r\n")
}

fn write_rcc(out: &mut impl Write, stage: &str, rcc: &[u32; RCC_WORDS]) -> fmt::Result {
    write!(
        out,
        "V6X_DUMP_RCC stage={stage} cr={:08x} cfgr={:08x} d1={:08x} d2={:08x} d3={:08x} pllckselr={:08x} pllcfgr={:08x} pll1={:08x} pll2={:08x} pll3={:08x} usbsel={:08x}\r\n",
        rcc[0], rcc[1], rcc[2], rcc[3], rcc[4], rcc[5], rcc[6], rcc[7], rcc[8], rcc[9], rcc[10]
    )
}

fn write_power(out: &mut impl Write, stage: &str, snapshot: &Snapshot) -> fmt::Result {
    write!(
        out,
        "V6X_DUMP_POWER stage={stage} cr3={:08x} d3cr={:08x} csr1={:08x} pwrcr={:08x} flash_acr={:08x}\r\n",
        snapshot.power[0], snapshot.power[1], snapshot.power[2], snapshot.pwrcr, snapshot.flash_acr
    )
}

fn write_fault(out: &mut impl Write, bkp: &[u32; 32]) -> fmt::Result {
    let mut words = [0u32; WORDS];
    words.copy_from_slice(&bkp[16..32]);
    let record = FaultRecord(words);
    let kind = record.0[1];
    let valid = record.valid();
    write!(
        out,
        "V6X_DUMP_FAULT valid={} kind={} name={} source={:08x}{:08x} detail={} file_hash={:08x}\r\n",
        valid,
        kind,
        kind_name(kind),
        record.0[3],
        record.0[2],
        record.0[4],
        record.0[5]
    )?;
    write!(
        out,
        "V6X_DUMP_FRAME pc={:08x} lr={:08x} sp={:08x} xpsr={:08x} exc_return={:08x}\r\n",
        record.0[6], record.0[7], record.0[8], record.0[9], record.0[14]
    )?;
    write!(
        out,
        "V6X_DUMP_FAULTREG cfsr={:08x} hfsr={:08x} mmfar={:08x} bfar={:08x} stored_crc={:08x}\r\n",
        record.0[10], record.0[11], record.0[12], record.0[13], record.0[15]
    )
}

fn make_report(before: &Snapshot, after: &Snapshot, preserved: bool) -> Report {
    let mut out = Report::new();
    let _ = write!(
        out,
        "V6X_DUMP schema=2 version={} source={} git={} feature=fault-dump\r\n",
        env!("CARGO_PKG_VERSION"),
        env!("BUILD_SOURCE_ID"),
        env!("BUILD_GIT")
    );
    let _ = crate::backup_domain::write_diagnostics(&mut out, "V6X_DUMP_BACKUP");
    let _ = write!(
        out,
        "V6X_DUMP_STATE stage=entry rsr={:08x} idc={:08x} backup_preserved={}\r\n",
        before.rsr, before.idc, preserved
    );
    let _ = write_rcc(&mut out, "entry", &before.rcc);
    let _ = write_power(&mut out, "entry", before);
    let _ = write_bkp(&mut out, "entry", &before.bkp);
    let _ = write_fault(&mut out, &before.bkp);
    let _ = write!(
        out,
        "V6X_DUMP_STATE stage=p0 rsr={:08x} idc={:08x} backup_preserved={}\r\n",
        after.rsr, after.idc, preserved
    );
    let _ = write_rcc(&mut out, "p0", &after.rcc);
    let _ = write_power(&mut out, "p0", after);
    let _ = write_bkp(&mut out, "p0", &after.bkp);
    let _ = write!(out, "V6X_DUMP_END\r\n");
    out
}

async fn write_transfer<'d, D: UsbDriver<'d>>(
    sender: &mut Sender<'d, D>,
    data: &[u8],
) -> Result<(), EndpointError> {
    let max_packet_size = usize::from(sender.max_packet_size());
    for packet in data.chunks(max_packet_size) {
        sender.write_packet(packet).await?;
    }
    if data.len().is_multiple_of(max_packet_size) {
        sender.write_packet(&[]).await?;
    }
    Ok(())
}

async fn send_report<'d, D: UsbDriver<'d>>(sender: &mut Sender<'d, D>, report: &Report) -> ! {
    loop {
        sender.wait_connection().await;
        let _ = write_transfer(sender, report.as_bytes()).await;
        Timer::after(Duration::from_millis(2000)).await;
    }
}

async fn bootloader_command<'d, D: UsbDriver<'d>>(receiver: &mut Receiver<'d, D>) -> ! {
    let mut line = [0u8; 32];
    let mut len = 0;
    loop {
        receiver.wait_connection().await;
        let mut packet = [0u8; 64];
        while let Ok(read) = receiver.read_packet(&mut packet).await {
            for &byte in &packet[..read] {
                if byte == b'\n' {
                    let command = &line[..len];
                    let command = command.strip_suffix(b"\r").unwrap_or(command);
                    if command == b"!bootloader" {
                        reboot_to_bootloader();
                    }
                    len = 0;
                } else if len < line.len() {
                    line[len] = byte;
                    len += 1;
                }
            }
        }
    }
}

async fn heartbeat(mut red: OutputOpenDrain<'static>) -> ! {
    loop {
        Timer::after(Duration::from_millis(500)).await;
        red.toggle();
    }
}

#[embassy_executor::task]
async fn report_task(p: Peripherals, report: Report) {
    // Red on: the P0 tree and the application code are alive even if USB does
    // not enumerate. Green and blue stay off.
    let red = OutputOpenDrain::new(p.PE3, Level::Low, Speed::Low);
    let _green = OutputOpenDrain::new(p.PE4, Level::High, Speed::Low);
    let _blue = OutputOpenDrain::new(p.PE5, Level::Low, Speed::Low);
    let _heater = Output::new(p.PB10, Level::Low, Speed::Low);
    let _not_armed = Input::new(p.PE6, Pull::Up);

    let mut ep_out_buffer = [0u8; 256];
    let mut usb_config = embassy_stm32::usb::Config::default();
    usb_config.vbus_detection = false;
    let driver = Driver::new_fs(
        p.USB_OTG_FS,
        crate::UsbIrqs,
        p.PA12,
        p.PA11,
        &mut ep_out_buffer,
        usb_config,
    );

    let mut config = UsbConfig::new(0x1209, 0x0001);
    config.manufacturer = Some("Rust FMUv6X");
    config.product = Some("V6X Rust Dump");
    config.serial_number = Some("V6X-RUST-DUMP");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut config_descriptor = [0u8; 256];
    let mut bos_descriptor = [0u8; 256];
    let mut control_buf = [0u8; 64];
    let mut state = State::new();
    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );
    let class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();
    let (mut sender, mut receiver) = class.split();

    embassy_futures::join::join(
        usb.run(),
        embassy_futures::join::join(
            embassy_futures::join::join(
                send_report(&mut sender, &report),
                bootloader_command(&mut receiver),
            ),
            heartbeat(red),
        ),
    )
    .await;
}

pub fn run() -> ! {
    let before = capture();
    let p = init_p0();
    let after = capture();
    let report = make_report(&before, &after, crate::backup_domain::preserved());

    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    let executor = EXECUTOR.init(Executor::new(usize::MAX as *mut ()));
    executor.spawner().spawn(report_task(p, report)).unwrap();
    loop {
        unsafe { executor.poll() };
        cortex_m::asm::wfe();
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    // Never touch BKP16..31 from the diagnosis image: those words hold the
    // evidence this image exists to read.
    loop {
        cortex_m::asm::wfi();
    }
}
