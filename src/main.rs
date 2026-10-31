#![no_std]
#![no_main]

#[cfg(all(feature = "fault-dump", feature = "fault-injection"))]
compile_error!("fault-dump and fault-injection are mutually exclusive");

mod backup_domain;
#[cfg(not(feature = "fault-dump"))]
mod diagnostics;
#[cfg(not(feature = "fault-dump"))]
mod eth_led;
#[cfg(not(feature = "fault-dump"))]
mod fault;
#[cfg(feature = "fault-dump")]
mod fault_dump;
#[cfg(not(feature = "fault-dump"))]
mod mavlink_usb;
#[cfg(not(feature = "fault-dump"))]
mod platform;
#[cfg(not(feature = "fault-dump"))]
mod usb_console;

use embassy_stm32::bind_interrupts;
use embassy_stm32::usb::InterruptHandler;

#[cfg(not(feature = "fault-dump"))]
use core::cell::RefCell;
#[cfg(not(feature = "fault-dump"))]
use embassy_executor::raw::Executor;
#[cfg(not(feature = "fault-dump"))]
use embassy_futures::join::join;
#[cfg(not(feature = "fault-dump"))]
use embassy_stm32::gpio::{Input, Level, Output, OutputOpenDrain, Pull, Speed};
#[cfg(not(feature = "fault-dump"))]
use embassy_stm32::usb::Driver;
#[cfg(not(feature = "fault-dump"))]
use embassy_stm32::Peripherals;
#[cfg(not(feature = "fault-dump"))]
use embassy_time::{Duration, Instant, Ticker, Timer};
#[cfg(not(feature = "fault-dump"))]
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
#[cfg(not(feature = "fault-dump"))]
use embassy_usb::{Builder, Config as UsbConfig};
#[cfg(not(feature = "fault-dump"))]
use static_cell::StaticCell;

#[cfg(not(feature = "fault-dump"))]
use eth_led::Lan8742;

#[cfg(not(feature = "fault-dump"))]
const LINK_HALF_PERIOD_MS: u64 = 500;

bind_interrupts!(struct UsbIrqs {
    OTG_FS => InterruptHandler<embassy_stm32::peripherals::USB_OTG_FS>;
});

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
            mavlink_usb::run(class, &phy, &eth_power, &session),
            join(
                async {
                    let mut ticker = Ticker::every(Duration::from_millis(LINK_HALF_PERIOD_MS));
                    loop {
                        // All MDIO transactions finish before yielding to the USB task.
                        {
                            let mut phy = phy.borrow_mut();
                            if !phy.found() {
                                phy.init();
                            }
                            if phy.found() && !phy.led_control_ready() {
                                phy.configure_leds();
                            }
                            let now = Instant::now().as_millis();
                            phy.set_link_led((now / LINK_HALF_PERIOD_MS).is_multiple_of(2));
                            green.set_level(if (now / LINK_HALF_PERIOD_MS).is_multiple_of(2) {
                                Level::Low
                            } else {
                                Level::High
                            });
                            red.set_level(if fault::present() {
                                Level::Low
                            } else {
                                Level::High
                            });
                        }

                        ticker.next().await;
                    }
                },
                diagnostics::stack_monitor(),
            ),
        ),
    )
    .await;
}
