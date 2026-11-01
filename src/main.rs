#![no_std]
#![no_main]

#[cfg(all(feature = "fault-dump", feature = "fault-injection"))]
compile_error!("fault-dump and fault-injection are mutually exclusive");

mod backup_domain;
#[cfg(not(feature = "fault-dump"))]
mod diagnostics;
#[cfg(not(feature = "fault-dump"))]
mod dronecan_can;
#[cfg(not(feature = "fault-dump"))]
mod eth_led;
#[cfg(not(feature = "fault-dump"))]
mod fault;
#[cfg(feature = "fault-dump")]
mod fault_dump;
#[cfg(not(feature = "fault-dump"))]
mod fram;
#[cfg(not(feature = "fault-dump"))]
mod mavlink_usb;
#[cfg(not(feature = "fault-dump"))]
mod parameter_manager;
#[cfg(not(feature = "fault-dump"))]
mod platform;
#[cfg(not(feature = "fault-dump"))]
mod sd_logger;
#[cfg(not(feature = "fault-dump"))]
mod usb_console;

use embassy_stm32::bind_interrupts;
use embassy_stm32::sdmmc::InterruptHandler as SdmmcInterruptHandler;
use embassy_stm32::usb::InterruptHandler as UsbInterruptHandler;

#[cfg(not(feature = "fault-dump"))]
use core::cell::RefCell;
#[cfg(not(feature = "fault-dump"))]
use embassy_executor::{raw::Executor, InterruptExecutor};
#[cfg(not(feature = "fault-dump"))]
use embassy_futures::join::join;
#[cfg(not(feature = "fault-dump"))]
use embassy_stm32::gpio::{Input, Level, Output, OutputOpenDrain, Pull, Speed};
#[cfg(not(feature = "fault-dump"))]
use embassy_stm32::spi::{Config as SpiConfig, Spi, MODE_3};
#[cfg(not(feature = "fault-dump"))]
use embassy_stm32::time::Hertz;
#[cfg(not(feature = "fault-dump"))]
use embassy_stm32::usb::Driver;
#[cfg(not(feature = "fault-dump"))]
use embassy_stm32::Peripherals;
#[cfg(not(feature = "fault-dump"))]
use embassy_time::{Instant, Timer};
#[cfg(not(feature = "fault-dump"))]
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
#[cfg(not(feature = "fault-dump"))]
use embassy_usb::{Builder, Config as UsbConfig};
#[cfg(not(feature = "fault-dump"))]
use static_cell::StaticCell;

#[cfg(not(feature = "fault-dump"))]
use eth_led::Lan8742;

#[cfg(not(feature = "fault-dump"))]
use v6x_rust_led::parameter_store::OutputState;
#[cfg(not(feature = "fault-dump"))]
use v6x_rust_led::parameters::Id;

#[cfg(not(feature = "fault-dump"))]
pub(crate) const OUTPUT_STATE: OutputState = OutputState {
    armed: false,
    outputs_enabled: false,
};

bind_interrupts!(struct UsbIrqs {
    OTG_FS => UsbInterruptHandler<embassy_stm32::peripherals::USB_OTG_FS>;
});

bind_interrupts!(struct SdIrqs {
    SDMMC2 => SdmmcInterruptHandler<embassy_stm32::peripherals::SDMMC2>;
});

#[cfg(not(feature = "fault-dump"))]
static CAN_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[cfg(not(feature = "fault-dump"))]
#[allow(non_snake_case)]
#[unsafe(no_mangle)]
unsafe extern "C" fn TIM6_DAC() {
    unsafe { CAN_EXECUTOR.on_interrupt() }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    #[cfg(feature = "fault-dump")]
    {
        fault_dump::run()
    }
    #[cfg(not(feature = "fault-dump"))]
    {
        application_main()
    }
}

#[cfg(not(feature = "fault-dump"))]
fn application_main() -> ! {
    let p = platform::init();
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    // Embassy executor 0.9.1's Cortex-M thread pender uses usize::MAX -> SEV.
    // Keep its wake protocol while adding a measured idle loop.
    let executor = EXECUTOR.init(Executor::new(usize::MAX as *mut ()));
    executor.spawner().spawn(application(p)).unwrap();
    loop {
        // This is the only polling context for this static thread executor.
        unsafe { executor.poll() };
        diagnostics::idle();
    }
}

#[cfg(not(feature = "fault-dump"))]
#[embassy_executor::task]
async fn application(p: Peripherals) {
    let mut red = OutputOpenDrain::new(p.PE3, Level::High, Speed::Low);
    let mut green = OutputOpenDrain::new(p.PE4, Level::High, Speed::Low);
    let mut blue = OutputOpenDrain::new(p.PE5, Level::Low, Speed::Low);
    let _heater = Output::new(p.PB10, Level::Low, Speed::Low);
    let _not_armed = Input::new(p.PE6, Pull::Up);
    diagnostics::start(p.TIM7);

    let fram_cs = Output::new(p.PG7, Level::High, Speed::Medium);
    let mut fram_config = SpiConfig::default();
    fram_config.mode = MODE_3;
    fram_config.frequency = Hertz(1_000_000);
    fram_config.gpio_speed = Speed::Medium;
    let fram = fram::Fram::new(
        Spi::new(
            p.SPI5,
            p.PF7,
            p.PF11,
            p.PH7,
            p.DMA2_CH0,
            p.DMA2_CH1,
            fram_config,
        ),
        fram_cs,
    );

    // The ETH LEDs are driven by the LAN8742A PHY. Power it up and give it
    // time to boot before touching the MDIO bus.
    // Keep PHY power (and its PWR LED) on throughout the application.
    let eth_power = Output::new(p.PG15, Level::High, Speed::Low);
    Timer::after_millis(200).await;

    let mut phy = Lan8742::new(
        OutputOpenDrain::new_pull(p.PA2, Level::High, Speed::Low, Pull::Up),
        Output::new(p.PC1, Level::Low, Speed::Low),
    );
    for _ in 0..10 {
        if phy.init() {
            phy.configure_leds();
            break;
        }
        Timer::after_millis(100).await;
    }
    let phy = RefCell::new(phy);
    blue.set_high();

    let parameters = parameter_manager::ParameterManager::new(fram);
    // PHY startup has already exceeded the FRAM power-up access delay.
    // Load failures remain visible in !params while USB recovery stays available.
    let _ = parameters.load().await;
    assert!(sd_logger::initialize());
    let dronecan = dronecan_can::State::new();
    let can_spawner = CAN_EXECUTOR.start(embassy_stm32::interrupt::TIM6_DAC);
    can_spawner
        .spawn(dronecan_can::io_task(p.FDCAN1, p.PD0, p.PD1))
        .unwrap();

    let sd_power = Output::new(p.PC13, Level::Low, Speed::Low);
    let sdmmc = embassy_stm32::sdmmc::Sdmmc::new_4bit(
        p.SDMMC2,
        SdIrqs,
        p.PD6,
        p.PD7,
        p.PB14,
        p.PB15,
        p.PG11,
        p.PB4,
        embassy_stm32::sdmmc::Config::default(),
    );

    // USB CDC ACM on the FS OTG peripheral (PA11/PA12), clocked from PLL3Q.
    let mut ep_out_buffer = [0u8; 256];
    let mut usb_config = embassy_stm32::usb::Config::default();
    usb_config.vbus_detection = false;
    let driver = Driver::new_fs(
        p.USB_OTG_FS,
        UsbIrqs,
        p.PA12,
        p.PA11,
        &mut ep_out_buffer,
        usb_config,
    );

    let mut config = UsbConfig::new(0x1209, 0x0001);
    config.manufacturer = Some("Rust FMUv6X");
    config.product = Some("V6X Rust CDC");
    config.serial_number = Some("V6X-RUST-001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut config_descriptor = [0u8; 256];
    let mut bos_descriptor = [0u8; 256];
    let mut control_buf = [0u8; 64];
    let mut state = State::new();
    let session = mavlink_usb::UsbSession::new_with(0);
    let mut session_events = mavlink_usb::UsbSessionEvents::new(&session);

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );
    builder.handler(&mut session_events);
    let class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();

    join(
        usb.run(),
        join(
            mavlink_usb::run(class, &phy, &eth_power, &session, &parameters, &dronecan),
            join(
                dronecan_can::run(&parameters, &dronecan),
                join(
                    join(
                        sd_logger::run(sdmmc, sd_power, &parameters),
                        sd_logger::health_task(),
                    ),
                    join(
                        async {
                            loop {
                                let half_period_us =
                                    (500_000.0 / parameters.get(Id::LinkLedHz)) as u64;
                                // All MDIO transactions finish before yielding to the USB task.
                                {
                                    let mut phy = phy.borrow_mut();
                                    if !phy.found() {
                                        phy.init();
                                    }
                                    if phy.found() && !phy.led_control_ready() {
                                        phy.configure_leds();
                                    }
                                    let on = (Instant::now().as_micros() / half_period_us)
                                        .is_multiple_of(2);
                                    phy.set_link_led(on);
                                    green.set_level(if on { Level::Low } else { Level::High });
                                    red.set_level(if fault::present() {
                                        Level::Low
                                    } else {
                                        Level::High
                                    });
                                }

                                Timer::after_micros(half_period_us).await;
                            }
                        },
                        diagnostics::stack_monitor(),
                    ),
                ),
            ),
        ),
    )
    .await;
}
