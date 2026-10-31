#![no_std]
#![no_main]

mod eth_led;
mod mavlink_usb;
mod usb_console;

use core::cell::RefCell;
use core::panic::PanicInfo;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::gpio::{Level, Output, OutputOpenDrain, Pull, Speed};
use embassy_stm32::rcc::{mux, Hse, HseMode, Pll, PllDiv, PllMul, PllPreDiv, PllSource};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::{Driver, InterruptHandler};
use embassy_stm32::{bind_interrupts, Config as HalConfig};
use embassy_time::{Duration, Instant, Ticker};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config as UsbConfig};

use eth_led::Lan8742;

const LINK_HALF_PERIOD_MS: u64 = 500;

bind_interrupts!(struct UsbIrqs {
    OTG_FS => InterruptHandler<embassy_stm32::peripherals::USB_OTG_FS>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // The FMUv6X derives the 48 MHz USB clock from PLL3Q:
    // 16 MHz HSE / 4 * 48 / 4 = 48 MHz. This is the same source the PX4
    // bootloader and application use, so it is known to work on this board.
    let mut hal_config = HalConfig::default();
    hal_config.rcc.hsi48 = None;
    hal_config.rcc.hse = Some(Hse {
        freq: Hertz(16_000_000),
        mode: HseMode::Oscillator,
    });
    hal_config.rcc.pll3 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL48,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV4),
        divr: Some(PllDiv::DIV2),
    });
    hal_config.rcc.mux.usbsel = mux::Usbsel::PLL3_Q;
    let p = embassy_stm32::init(hal_config);

    // The ETH LEDs are driven by the LAN8742A PHY. Power it up and give it
    // time to boot before touching the MDIO bus.
    // Keep PHY power (and its PWR LED) on throughout the application.
    let eth_power = Output::new(p.PG15, Level::High, Speed::Low);
    cortex_m::asm::delay(12_800_000); // ~200 ms at 64 MHz for PHY power-up

    let mut phy = Lan8742::new(
        OutputOpenDrain::new_pull(p.PA2, Level::High, Speed::Low, Pull::Up),
        Output::new(p.PC1, Level::Low, Speed::Low),
    );
    for _ in 0..10 {
        if phy.init() {
            phy.configure_leds();
            break;
        }
        cortex_m::asm::delay(6_400_000); // ~100 ms between probes
    }
    let phy = RefCell::new(phy);

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
        join(mavlink_usb::run(class, &phy, &eth_power, &session), async {
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
                }

                ticker.next().await;
            }
        }),
    )
    .await;
}

#[panic_handler]
fn panic(_info: &PanicInfo<'_>) -> ! {
    cortex_m::interrupt::disable();
    loop {
        cortex_m::asm::wfi();
    }
}
