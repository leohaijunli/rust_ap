#![no_std]
#![no_main]

#[cfg(all(feature = "fault-dump", feature = "fault-injection"))]
compile_error!("fault-dump and fault-injection are mutually exclusive");

mod backup_domain;
#[cfg(not(feature = "fault-dump"))]
#[path = "drivers/barometer.rs"]
mod barometer;
#[cfg(not(feature = "fault-dump"))]
#[path = "drivers/bmm150.rs"]
mod bmm150_driver;
#[cfg(not(feature = "fault-dump"))]
mod diagnostics;
#[cfg(not(feature = "fault-dump"))]
#[path = "drivers/dronecan_can.rs"]
mod dronecan_can;
#[cfg(not(feature = "fault-dump"))]
#[path = "drivers/estimator.rs"]
mod estimator;
#[cfg(not(feature = "fault-dump"))]
#[path = "drivers/eth_led.rs"]
mod eth_led;
#[cfg(not(feature = "fault-dump"))]
mod fault;
#[cfg(feature = "fault-dump")]
mod fault_dump;
#[cfg(not(feature = "fault-dump"))]
#[path = "storage/fram.rs"]
mod fram;
#[cfg(not(feature = "fault-dump"))]
#[path = "drivers/imu.rs"]
mod imu;
#[cfg(not(feature = "fault-dump"))]
#[path = "drivers/ina219.rs"]
mod ina219;
#[cfg(not(feature = "fault-dump"))]
#[path = "drivers/logger_switch.rs"]
mod logger_switch;
#[cfg(not(feature = "fault-dump"))]
#[path = "drivers/magnetometer.rs"]
mod magnetometer;
#[cfg(not(feature = "fault-dump"))]
#[path = "drivers/mavcan.rs"]
mod mavcan;
#[cfg(not(feature = "fault-dump"))]
#[path = "comms/mavlink_core.rs"]
mod mavlink_core;
#[cfg(not(feature = "fault-dump"))]
#[path = "comms/mavlink_uart7.rs"]
mod mavlink_uart7;
#[cfg(not(feature = "fault-dump"))]
#[path = "comms/mavlink_usb.rs"]
mod mavlink_usb;
#[cfg(not(feature = "fault-dump"))]
#[path = "storage/parameter_manager.rs"]
mod parameter_manager;
#[cfg(not(feature = "fault-dump"))]
mod platform;
#[cfg(not(feature = "fault-dump"))]
#[path = "drivers/px4io.rs"]
mod px4io;
#[cfg(not(feature = "fault-dump"))]
#[path = "comms/router.rs"]
mod router;
#[cfg(not(feature = "fault-dump"))]
#[path = "drivers/rtc.rs"]
mod rtc;
#[cfg(not(feature = "fault-dump"))]
#[path = "storage/sd_logger.rs"]
mod sd_logger;
#[cfg(not(feature = "fault-dump"))]
#[path = "comms/telemetry.rs"]
mod telemetry;
#[cfg(not(feature = "fault-dump"))]
#[path = "comms/usb_console.rs"]
mod usb_console;

use embassy_stm32::bind_interrupts;
use embassy_stm32::i2c::{
    ErrorInterruptHandler as I2cErrorInterruptHandler,
    EventInterruptHandler as I2cEventInterruptHandler,
};
use embassy_stm32::sdmmc::InterruptHandler as SdmmcInterruptHandler;
use embassy_stm32::usart::InterruptHandler as UsartInterruptHandler;
use embassy_stm32::usb::InterruptHandler as UsbInterruptHandler;

#[cfg(not(feature = "fault-dump"))]
use core::cell::RefCell;
#[cfg(not(feature = "fault-dump"))]
use embassy_executor::{raw::Executor, InterruptExecutor, Spawner};
#[cfg(not(feature = "fault-dump"))]
use embassy_stm32::gpio::{Input, Level, Output, OutputOpenDrain, Pull, Speed};
#[cfg(not(feature = "fault-dump"))]
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
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

bind_interrupts!(struct Uart7Irqs {
    UART7 => UsartInterruptHandler<embassy_stm32::peripherals::UART7>;
});

#[cfg(not(feature = "fault-dump"))]
bind_interrupts!(struct Px4ioIrqs {
    USART6 => UsartInterruptHandler<embassy_stm32::peripherals::USART6>;
});

bind_interrupts!(struct I2c3Irqs {
    I2C3_EV => I2cEventInterruptHandler<embassy_stm32::peripherals::I2C3>;
    I2C3_ER => I2cErrorInterruptHandler<embassy_stm32::peripherals::I2C3>;
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
    rtc::init();
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    // Embassy executor 0.9.1's Cortex-M thread pender uses usize::MAX -> SEV.
    // Keep its wake protocol while adding a measured idle loop.
    let executor = EXECUTOR.init(Executor::new(usize::MAX as *mut ()));
    executor
        .spawner()
        .spawn(application(p, executor.spawner()))
        .unwrap();
    executor.spawner().spawn(estimator::log_task()).unwrap();
    executor
        .spawner()
        .spawn(diagnostics::stack_monitor())
        .unwrap();
    loop {
        // This is the only polling context for this static thread executor.
        unsafe { executor.poll() };
        diagnostics::idle();
    }
}

#[cfg(not(feature = "fault-dump"))]
#[embassy_executor::task]
async fn application(p: Peripherals, spawner: Spawner) {
    let mut red = OutputOpenDrain::new(p.PE3, Level::High, Speed::Low);
    let mut green = OutputOpenDrain::new(p.PE4, Level::High, Speed::Low);
    let mut blue = OutputOpenDrain::new(p.PE5, Level::Low, Speed::Low);
    let _heater = Output::new(p.PB10, Level::Low, Speed::Low);
    // GPIO_VDD_5V_PERIPH_nEN: power the standalone I2C connector.
    let _periph_power = Output::new(p.PG4, Level::Low, Speed::Low);
    let _not_armed = Input::new(p.PE6, Pull::Up);
    // PX4IO owns the shared safety LED on this board. Never drive PD10 from
    // FMU while the existing IO firmware may be driving the same net.
    let _safety_led = Input::new(p.PD10, Pull::None);
    let fast_spawner = diagnostics::start(p.TIM7);

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
    static ETH_POWER: StaticCell<Output<'static>> = StaticCell::new();
    let eth_power: &'static Output<'static> =
        ETH_POWER.init(Output::new(p.PG15, Level::High, Speed::Low));
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
    static PHY: StaticCell<RefCell<Lan8742<'static>>> = StaticCell::new();
    let phy: &'static RefCell<Lan8742<'static>> = PHY.init(RefCell::new(phy));
    blue.set_high();

    // Shared only by sibling tasks on this thread executor. The manager keeps
    // its existing NoopRawMutex; no reference is sent to an interrupt executor.
    static PARAMETERS: StaticCell<parameter_manager::ParameterManager<'static>> = StaticCell::new();
    let parameters: &'static parameter_manager::ParameterManager<'static> =
        PARAMETERS.init(parameter_manager::ParameterManager::new(fram));
    // PHY startup has already exceeded the FRAM power-up access delay.
    // Load failures remain visible in !params while USB recovery stays available.
    let _ = parameters.load().await;
    spawner
        .spawn(parameter_manager::save_task(parameters))
        .unwrap();
    assert!(sd_logger::initialize());
    spawner.spawn(sd_logger::health_task()).unwrap();
    fast_spawner
        .spawn(imu::task(
            p.SPI2, p.PI1, p.PI3, p.PI2, p.DMA1_CH2, p.DMA1_CH3, p.PH5, p.PF4, p.PA10, p.EXTI10,
        ))
        .unwrap();
    static DRONECAN: StaticCell<dronecan_can::State> = StaticCell::new();
    let dronecan: &'static dronecan_can::State = DRONECAN.init(dronecan_can::State::new());
    estimator::start(p.TIM5, fast_spawner);
    let can_spawner = CAN_EXECUTOR.start(embassy_stm32::interrupt::TIM6_DAC);
    can_spawner
        .spawn(dronecan_can::io_task(p.FDCAN1, p.PD0, p.PD1))
        .unwrap();
    can_spawner
        .spawn(mavcan::io_task(p.FDCAN2, p.PB12, p.PB13))
        .unwrap();
    spawner
        .spawn(dronecan_can::run(parameters, dronecan))
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
    spawner
        .spawn(sd_logger::run(sdmmc, sd_power, parameters))
        .unwrap();

    // TELEM1 is UART7 (RX PF6, TX PE8). 115200 8N1 is the first-round link
    // candidate; hardware flow control (RTS PF8 / CTS PE10) is deferred until
    // the physical adapter is confirmed under H-03.
    let mut uart7_config = embassy_stm32::usart::Config::default();
    uart7_config.baudrate = 115_200;
    let uart7 = embassy_stm32::usart::Uart::new(
        p.UART7,
        p.PF6,
        p.PE8,
        Uart7Irqs,
        p.DMA1_CH0,
        p.DMA1_CH1,
        uart7_config,
    )
    .expect("UART7 configuration");
    spawner.spawn(mavlink_uart7::run(uart7)).unwrap();

    // FMUv6X IO link is USART6 (TX PC6/RX PC7), not the TELEM1 UART.
    let mut px4io_config = embassy_stm32::usart::Config::default();
    px4io_config.baudrate = 1_500_000;
    let px4io_uart = embassy_stm32::usart::Uart::new(
        p.USART6,
        p.PC7,
        p.PC6,
        Px4ioIrqs,
        p.DMA2_CH2,
        p.DMA2_CH3,
        px4io_config,
    )
    .expect("USART6 PX4IO configuration");
    spawner.spawn(px4io::run(px4io_uart)).unwrap();
    spawner
        .spawn(logger_switch::run(p.PF5, p.TIM14, p.PF9))
        .unwrap();

    // Standalone I2C connector: SCL3/SDA3 (PA8/PH8), not the GPS/POWER I2C buses.
    let mut i2c_config = I2cConfig::default();
    i2c_config.timeout = embassy_time::Duration::from_millis(20);
    // Weak internal pull-ups distinguish a floating bus from a slave holding
    // SCL/SDA low; external 10 kOhm pull-ups on the module dominate when powered.
    i2c_config.scl_pullup = true;
    i2c_config.sda_pullup = true;
    let i2c3 = I2c::new(
        p.I2C3, p.PA8, p.PH8, I2c3Irqs, p.DMA1_CH4, p.DMA1_CH5, i2c_config,
    );
    spawner.spawn(ina219::run(i2c3)).unwrap();

    // Fixed FMUv6X Rev 3 internal BMP388/BMM150 bus. I2C4's BDMA cannot
    // access AXI; use the HAL's 10-ms-bounded blocking path instead.
    let _sensor_power = Output::new(p.PG8, Level::High, Speed::Low);
    Timer::after_millis(10).await;
    let mut baro_config = I2cConfig::default();
    baro_config.frequency = Hertz(400_000);
    baro_config.timeout = embassy_time::Duration::from_millis(10);
    static INTERNAL_I2C: StaticCell<
        RefCell<I2c<'static, embassy_stm32::mode::Blocking, embassy_stm32::i2c::mode::Master>>,
    > = StaticCell::new();
    // Non-Sync RefCell is shared only by sibling thread tasks. Each blocking
    // transfer releases the borrow before yielding; interrupts never use it.
    let i2c4 = INTERNAL_I2C.init(RefCell::new(I2c::new_blocking(
        p.I2C4,
        p.PF14,
        p.PF15,
        baro_config,
    )));
    spawner.spawn(barometer::run(i2c4)).unwrap();
    spawner.spawn(magnetometer::run()).unwrap();
    spawner.spawn(bmm150_driver::run(i2c4)).unwrap();

    // USB CDC ACM on the FS OTG peripheral (PA11/PA12), clocked from PLL3Q.
    static EP_OUT_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
    let mut usb_config = embassy_stm32::usb::Config::default();
    usb_config.vbus_detection = false;
    let driver = Driver::new_fs(
        p.USB_OTG_FS,
        UsbIrqs,
        p.PA12,
        p.PA11,
        EP_OUT_BUFFER.init([0; 256]),
        usb_config,
    );

    let mut config = UsbConfig::new(0x1209, 0x0001);
    config.manufacturer = Some("Rust FMUv6X");
    config.product = Some("V6X Rust CDC");
    config.serial_number = Some("V6X-RUST-001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static USB_STATE: StaticCell<State<'static>> = StaticCell::new();
    static USB_SESSION: StaticCell<mavlink_usb::UsbSession> = StaticCell::new();
    static SESSION_EVENTS: StaticCell<mavlink_usb::UsbSessionEvents<'static>> = StaticCell::new();
    let session: &'static mavlink_usb::UsbSession =
        USB_SESSION.init(mavlink_usb::UsbSession::new_with(0));

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        &mut [],
        CONTROL_BUF.init([0; 64]),
    );
    builder.handler(SESSION_EVENTS.init(mavlink_usb::UsbSessionEvents::new(session)));
    let class = CdcAcmClass::new(&mut builder, USB_STATE.init(State::new()), 64);
    spawner
        .spawn(mavlink_usb::device_task(builder.build()))
        .unwrap();
    spawner.spawn(mavlink_usb::run(class, session)).unwrap();
    spawner
        .spawn(router::run(parameters, dronecan, phy, eth_power))
        .unwrap();

    // Slow services are sibling tasks: their poll frames are no longer retained
    // beneath SD/FAT or other unrelated task calls when the fast IRQ preempts.
    loop {
        let half_period_us = (500_000.0 / parameters.get(Id::LinkLedHz)) as u64;
        // All MDIO transactions finish before yielding to the USB task.
        {
            let mut phy = phy.borrow_mut();
            if !phy.found() {
                phy.init();
            }
            if phy.found() && !phy.led_control_ready() {
                phy.configure_leds();
            }
            let on = (Instant::now().as_micros() / half_period_us).is_multiple_of(2);
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
}
