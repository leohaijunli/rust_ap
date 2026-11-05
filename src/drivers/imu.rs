//! Embassy SPI2/EXTI owner and bounded IMU sample fan-out.
#![allow(clippy::too_many_arguments)] // Fixed board resources stay explicit at the task boundary.

use core::fmt;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use embassy_futures::select::{select, Either};
use embassy_stm32::bind_interrupts;
use embassy_stm32::exti::{ExtiInput, InterruptHandler as ExtiInterruptHandler};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::interrupt::typelevel::{Handler, EXTI15_10};
use embassy_stm32::mode::Async;
use embassy_stm32::pac::SPI2;
use embassy_stm32::peripherals;
use embassy_stm32::spi::{mode::Master, Config as SpiConfig, Spi, MODE_3};
use embassy_stm32::time::Hertz;
use embassy_stm32::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use v6x_rust_led::icm42688::{
    self, bits, register, Bank, Config, Sample, FIFO_BYTES, FIFO_PACKET_BYTES, SETTINGS,
    WHO_AM_I_VALUE,
};

const SPI_HZ: Hertz = Hertz(24_000_000);
const MAX_FIFO_SAMPLES: usize = 8;
const TRANSFER_TIMEOUT: Duration = Duration::from_millis(2);
const RESET_TIMEOUT: Duration = Duration::from_secs(1);

type AhrsQueue = Channel<CriticalSectionRawMutex, Sample, 16>;
type Latest = Watch<CriticalSectionRawMutex, Sample, 2>;

static AHRS: AhrsQueue = AhrsQueue::new();
static AHRS_ACTIVE: AtomicBool = AtomicBool::new(false);
static LATEST: Latest = Latest::new();
static DRDY_TIMESTAMP_LOW: AtomicU32 = AtomicU32::new(0);
static STATE: RuntimeState = RuntimeState::new();

struct DrdyTimestampHandler;

impl Handler<EXTI15_10> for DrdyTimestampHandler {
    unsafe fn on_interrupt() {
        if embassy_stm32::pac::EXTI.pr(0).read().0 & (1 << 10) != 0 {
            DRDY_TIMESTAMP_LOW.store(Instant::now().as_micros() as u32, Ordering::Relaxed);
        }
    }
}

bind_interrupts!(struct ExtiIrqs {
    EXTI15_10 => DrdyTimestampHandler, ExtiInterruptHandler<EXTI15_10>;
});

struct RuntimeState {
    initialized: AtomicBool,
    who_am_i: AtomicU8,
    spi_hz: AtomicU32,
    samples: AtomicU32,
    transfer_errors: AtomicU32,
    decode_errors: AtomicU32,
    fifo_resets: AtomicU32,
    fifo_rejected: AtomicU32,
    max_fifo_samples: AtomicU32,
    ahrs_dropped: AtomicU32,
    ahrs_cleared: AtomicU32,
    drdy_timeouts: AtomicU32,
}

impl RuntimeState {
    const fn new() -> Self {
        Self {
            initialized: AtomicBool::new(false),
            who_am_i: AtomicU8::new(0),
            spi_hz: AtomicU32::new(0),
            samples: AtomicU32::new(0),
            transfer_errors: AtomicU32::new(0),
            decode_errors: AtomicU32::new(0),
            fifo_resets: AtomicU32::new(0),
            fifo_rejected: AtomicU32::new(0),
            max_fifo_samples: AtomicU32::new(0),
            ahrs_dropped: AtomicU32::new(0),
            ahrs_cleared: AtomicU32::new(0),
            drdy_timeouts: AtomicU32::new(0),
        }
    }
}

pub fn latest() -> Option<Sample> {
    LATEST.try_get()
}

pub fn set_ahrs_active(enabled: bool) {
    // The IMU now preempts the estimator: clear and switch atomically, so no
    // sample can arrive between the count and clear. This is intentional reset
    // cleanup, not an overflow or lost sample during continuous operation.
    critical_section::with(|_| {
        AHRS_ACTIVE.store(false, Ordering::Release);
        let cleared = AHRS.len() as u32;
        AHRS.clear();
        STATE.ahrs_cleared.fetch_add(cleared, Ordering::Relaxed);
        AHRS_ACTIVE.store(enabled, Ordering::Release);
    });
}

pub async fn next_for_ahrs() -> Sample {
    AHRS.receive().await
}

pub fn write_diagnostics(output: &mut impl fmt::Write) -> fmt::Result {
    let latest = latest();
    writeln!(
        output,
        "IMU schema=2 initialized={} whoami={:02x} odr_hz={} log_rate_hz={} spi_hz={} samples={} transfer_errors={} decode_errors={} fifo_resets={} drdy_timeouts={} ahrs_dropped={} ahrs_cleared={} fifo_rejected={} max_fifo_samples={} latest_seq={} latest_age_us={}",
        STATE.initialized.load(Ordering::Acquire),
        STATE.who_am_i.load(Ordering::Relaxed),
        Config::DEFAULT.odr_hz,
        Config::DEFAULT.log_rate_hz,
        STATE.spi_hz.load(Ordering::Relaxed),
        STATE.samples.load(Ordering::Relaxed),
        STATE.transfer_errors.load(Ordering::Relaxed),
        STATE.decode_errors.load(Ordering::Relaxed),
        STATE.fifo_resets.load(Ordering::Relaxed),
        STATE.drdy_timeouts.load(Ordering::Relaxed),
        STATE.ahrs_dropped.load(Ordering::Relaxed),
        STATE.ahrs_cleared.load(Ordering::Relaxed),
        STATE.fifo_rejected.load(Ordering::Relaxed),
        STATE.max_fifo_samples.load(Ordering::Relaxed),
        latest.map_or(0, |sample| sample.sequence),
        latest.map_or(u32::MAX, |sample| sample.age_us(Instant::now().as_micros())),
    )?;
    writeln!(
        output,
        "IMU_CLOCK delta_ticks={} resolution_us=16 integration_us={}",
        latest.map_or(0, |sample| sample.sensor_timestamp),
        latest.map_or(0, |sample| u32::from(sample.sensor_timestamp) * 16)
    )
}

#[embassy_executor::task]
pub async fn task(
    spi: Peri<'static, peripherals::SPI2>,
    sck: Peri<'static, peripherals::PI1>,
    mosi: Peri<'static, peripherals::PI3>,
    miso: Peri<'static, peripherals::PI2>,
    tx_dma: Peri<'static, peripherals::DMA1_CH2>,
    rx_dma: Peri<'static, peripherals::DMA1_CH3>,
    cs: Peri<'static, peripherals::PH5>,
    power: Peri<'static, peripherals::PF4>,
    drdy: Peri<'static, peripherals::PA10>,
    exti: Peri<'static, peripherals::EXTI10>,
) {
    let mut power = Output::new(power, Level::Low, Speed::Low);
    let cs = Output::new(cs, Level::High, Speed::VeryHigh);
    let mut drdy = ExtiInput::new(drdy, exti, Pull::None, ExtiIrqs);
    let mut spi_config = SpiConfig::default();
    spi_config.mode = MODE_3;
    spi_config.frequency = SPI_HZ;
    spi_config.gpio_speed = Speed::VeryHigh;
    let spi = Spi::new(spi, sck, mosi, miso, tx_dma, rx_dma, spi_config);
    STATE
        .spi_hz
        .store(spi.get_current_config().frequency.0, Ordering::Relaxed);
    let mut sensor = Icm42688::new(spi, cs);

    Timer::after_millis(1).await;
    power.set_high();
    Timer::after_millis(10).await;

    let config = v6x_rust_led::config::Config::DEFAULT.imu();
    let mut sequence = 0u32;
    loop {
        STATE.initialized.store(false, Ordering::Release);
        if sensor.initialize(config).await.is_err() {
            STATE.transfer_errors.fetch_add(1, Ordering::Relaxed);
            Timer::after_millis(100).await;
            continue;
        }
        STATE.initialized.store(true, Ordering::Release);
        let mut consecutive_failures = 0u8;
        let mut checked_setting = 0usize;
        let mut next_config_check = Instant::now() + Duration::from_millis(100);

        loop {
            let timed_out = matches!(
                select(drdy.wait_for_falling_edge(), Timer::after_millis(100)).await,
                Either::Second(())
            );
            if timed_out {
                STATE.drdy_timeouts.fetch_add(1, Ordering::Relaxed);
            }
            let now = Instant::now().as_micros();
            let newest_timestamp = reconstruct_timestamp(now);
            match sensor.read_fifo(config).await {
                Ok(read) if read.samples > 0 => {
                    consecutive_failures = 0;
                    let newest_timestamp = newest_timestamp.saturating_sub(read.newest_offset_us);
                    for index in 0..read.samples {
                        let age = u64::try_from(read.samples - 1 - index).unwrap()
                            * config.sample_period_us();
                        let timestamp = newest_timestamp.saturating_sub(age);
                        match icm42688::decode_fifo_packet(
                            sensor.packet(index),
                            timestamp,
                            sequence,
                            config,
                        ) {
                            Ok(sample) => {
                                sequence = sequence.wrapping_add(1);
                                publish(sample);
                            }
                            Err(_) => {
                                STATE.decode_errors.fetch_add(1, Ordering::Relaxed);
                                consecutive_failures = consecutive_failures.saturating_add(1);
                            }
                        }
                    }
                }
                Ok(_) => {
                    consecutive_failures = consecutive_failures.saturating_add(1);
                }
                Err(_) => {
                    STATE.transfer_errors.fetch_add(1, Ordering::Relaxed);
                    consecutive_failures = consecutive_failures.saturating_add(1);
                }
            }
            if consecutive_failures > 10 {
                break;
            }
            if Instant::now() >= next_config_check {
                if sensor
                    .check_setting(SETTINGS[checked_setting], config)
                    .await
                    .is_err()
                {
                    STATE.transfer_errors.fetch_add(1, Ordering::Relaxed);
                    break;
                }
                checked_setting = (checked_setting + 1) % SETTINGS.len();
                next_config_check = Instant::now() + Duration::from_millis(100);
            }
        }
    }
}

fn publish(sample: Sample) {
    LATEST.sender().send(sample);
    if AHRS_ACTIVE.load(Ordering::Acquire) && AHRS.try_send(sample).is_err() {
        let dropped = AHRS.len() as u32;
        AHRS.clear();
        STATE.ahrs_dropped.fetch_add(dropped, Ordering::Relaxed);
        let _ = AHRS.try_send(sample);
    }
    STATE.samples.fetch_add(1, Ordering::Relaxed);
    let error_count = STATE
        .transfer_errors
        .load(Ordering::Relaxed)
        .saturating_add(STATE.decode_errors.load(Ordering::Relaxed))
        .saturating_add(STATE.drdy_timeouts.load(Ordering::Relaxed));
    crate::sd_logger::record_imu(sample, error_count);
}

fn reconstruct_timestamp(now: u64) -> u64 {
    icm42688::reconstruct_u32_timestamp(now, DRDY_TIMESTAMP_LOW.load(Ordering::Relaxed))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Error {
    Spi,
    Timeout,
    Identity,
    Reset,
    Configure,
    Fifo,
}

struct Icm42688<'d> {
    spi: Spi<'d, Async, Master>,
    cs: Output<'d>,
    bank: Bank,
    fifo: [u8; 1 + MAX_FIFO_SAMPLES * FIFO_PACKET_BYTES],
}

struct FifoRead {
    samples: usize,
    newest_offset_us: u64,
}

impl<'d> Icm42688<'d> {
    fn new(spi: Spi<'d, Async, Master>, cs: Output<'d>) -> Self {
        Self {
            spi,
            cs,
            bank: Bank::Zero,
            fifo: [0; 1 + MAX_FIFO_SAMPLES * FIFO_PACKET_BYTES],
        }
    }

    async fn initialize(&mut self, config: Config) -> Result<(), Error> {
        self.select_bank(Bank::Zero, true).await?;
        self.write_register(register::DEVICE_CONFIG, bits::SOFT_RESET)
            .await?;
        Timer::after_millis(2).await;

        let deadline = Instant::now() + RESET_TIMEOUT;
        loop {
            self.select_bank(Bank::Zero, true).await?;
            let who_am_i = self.read_register(register::WHO_AM_I).await?;
            STATE.who_am_i.store(who_am_i, Ordering::Relaxed);
            let device_config = self.read_register(register::DEVICE_CONFIG).await?;
            let status = self.read_register(register::INT_STATUS).await?;
            if who_am_i == WHO_AM_I_VALUE && device_config == 0 && status & bits::RESET_DONE != 0 {
                break;
            }
            if Instant::now() >= deadline {
                return Err(if who_am_i != WHO_AM_I_VALUE {
                    Error::Identity
                } else {
                    Error::Reset
                });
            }
            Timer::after_millis(10).await;
        }

        self.write_register(register::PWR_MGMT0, bits::LOW_NOISE)
            .await?;
        Timer::after_millis(30).await;
        for setting in SETTINGS {
            self.select_bank(setting.bank, false).await?;
            let current = self.read_register(setting.address).await?;
            let expected = setting.apply(current);
            if expected != current {
                self.write_register(setting.address, expected).await?;
            }
            let actual = self.read_register(setting.address).await?;
            if actual & setting.set != setting.set || actual & setting.clear != 0 {
                return Err(Error::Configure);
            }
        }
        self.configure_runtime(config).await?;
        self.select_bank(Bank::Zero, false).await?;
        self.flush_fifo().await
    }

    async fn configure_runtime(&mut self, config: Config) -> Result<(), Error> {
        self.select_bank(Bank::Zero, false).await?;
        let odr = match config.odr_hz {
            1_000 => 0x06,
            8_000 => 0x03,
            _ => return Err(Error::Configure),
        };
        self.write_register(register::GYRO_CONFIG0, odr).await?;
        self.write_register(register::ACCEL_CONFIG0, odr).await?;
        let watermark = config.fifo_batch_samples() * FIFO_PACKET_BYTES;
        self.write_register(register::FIFO_CONFIG2, watermark as u8)
            .await?;
        self.write_register(register::FIFO_CONFIG3, (watermark >> 8) as u8)
            .await?;
        let fifo_config1 = self.read_register(register::FIFO_CONFIG1).await?;
        let fifo_config1 = if config.odr_hz == 8_000 {
            fifo_config1 | 0x20
        } else {
            fifo_config1 & !0x20
        };
        self.write_register(register::FIFO_CONFIG1, fifo_config1)
            .await
    }

    async fn check_setting(
        &mut self,
        setting: v6x_rust_led::icm42688::RegisterSetting,
        config: Config,
    ) -> Result<(), Error> {
        self.select_bank(setting.bank, false).await?;
        let actual = self.read_register(setting.address).await?;
        if setting.bank == Bank::Zero && setting.address == register::FIFO_CONFIG1 {
            let (set, clear) = if config.odr_hz == 8_000 {
                (0x37, 0x08)
            } else {
                (0x17, 0x28)
            };
            if actual & set != set || actual & clear != 0 {
                return Err(Error::Configure);
            }
            return Ok(());
        }
        let runtime_expected = match (setting.bank, setting.address) {
            (Bank::Zero, register::GYRO_CONFIG0 | register::ACCEL_CONFIG0) => {
                Some(match config.odr_hz {
                    1_000 => 0x06,
                    8_000 => 0x03,
                    _ => return Err(Error::Configure),
                })
            }
            (Bank::Zero, register::FIFO_CONFIG2) => {
                Some((config.fifo_batch_samples() * FIFO_PACKET_BYTES) as u8)
            }
            (Bank::Zero, register::FIFO_CONFIG3) => {
                Some(((config.fifo_batch_samples() * FIFO_PACKET_BYTES) >> 8) as u8)
            }
            _ => None,
        };
        if runtime_expected.is_some_and(|expected| actual != expected)
            || (runtime_expected.is_none()
                && (actual & setting.set != setting.set || actual & setting.clear != 0))
        {
            return Err(Error::Configure);
        }
        Ok(())
    }

    async fn read_fifo(&mut self, config: Config) -> Result<FifoRead, Error> {
        self.select_bank(Bank::Zero, false).await?;
        let mut header = [register::INT_STATUS | icm42688::READ, 0, 0, 0];
        self.transaction(&mut header).await?;
        let count = usize::from(u16::from_be_bytes([header[2], header[3]]));
        STATE
            .max_fifo_samples
            .fetch_max((count / FIFO_PACKET_BYTES) as u32, Ordering::Relaxed);
        if header[1] & bits::FIFO_FULL != 0
            || count >= FIFO_BYTES
            || count / FIFO_PACKET_BYTES > config.fifo_batch_samples() + 1
        {
            STATE.fifo_rejected.fetch_add(1, Ordering::Relaxed);
            self.flush_fifo().await?;
            return Err(Error::Fifo);
        }
        let available = count / FIFO_PACKET_BYTES;
        let samples = available.min(config.fifo_batch_samples());
        if samples == 0 {
            return Ok(FifoRead {
                samples: 0,
                newest_offset_us: 0,
            });
        }
        let length = 1 + samples * FIFO_PACKET_BYTES;
        self.fifo[..length].fill(0);
        self.fifo[0] = register::FIFO_DATA | icm42688::READ;
        self.transaction_fifo(length).await?;
        Ok(FifoRead {
            samples,
            newest_offset_us: u64::try_from(available - samples).unwrap()
                * config.sample_period_us(),
        })
    }

    fn packet(&self, index: usize) -> &[u8; FIFO_PACKET_BYTES] {
        let start = 1 + index * FIFO_PACKET_BYTES;
        self.fifo[start..start + FIFO_PACKET_BYTES]
            .try_into()
            .unwrap()
    }

    async fn flush_fifo(&mut self) -> Result<(), Error> {
        let value = self.read_register(register::SIGNAL_PATH_RESET).await? | bits::FIFO_FLUSH;
        self.write_register(register::SIGNAL_PATH_RESET, value)
            .await?;
        STATE.fifo_resets.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    async fn select_bank(&mut self, bank: Bank, force: bool) -> Result<(), Error> {
        if force || self.bank != bank {
            self.write_register(register::REG_BANK_SEL, bank as u8)
                .await?;
            self.bank = bank;
        }
        Ok(())
    }

    async fn read_register(&mut self, address: u8) -> Result<u8, Error> {
        let mut bytes = [address | icm42688::READ, 0];
        self.transaction(&mut bytes).await?;
        Ok(bytes[1])
    }

    async fn write_register(&mut self, address: u8, value: u8) -> Result<(), Error> {
        self.transaction(&mut [address, value]).await
    }

    async fn transaction(&mut self, bytes: &mut [u8]) -> Result<(), Error> {
        self.cs.set_low();
        let selected = Selected(&mut self.cs);
        let result = with_timeout(TRANSFER_TIMEOUT, self.spi.transfer_in_place(bytes))
            .await
            .map_err(|_| Error::Timeout)?
            .map_err(|_| Error::Spi);
        drop(selected);
        result
    }

    async fn transaction_fifo(&mut self, length: usize) -> Result<(), Error> {
        self.cs.set_low();
        let selected = Selected(&mut self.cs);
        let result = with_timeout(
            TRANSFER_TIMEOUT,
            self.spi.transfer_in_place(&mut self.fifo[..length]),
        )
        .await
        .map_err(|_| Error::Timeout)?
        .map_err(|_| Error::Spi);
        drop(selected);
        result
    }
}

struct Selected<'a, 'd>(&'a mut Output<'d>);

impl Drop for Selected<'_, '_> {
    fn drop(&mut self) {
        self.0.set_high();
        SPI2.cr1().modify(|w| w.set_spe(false));
        SPI2.cfg1().modify(|w| {
            w.set_txdmaen(false);
            w.set_rxdmaen(false);
        });
        SPI2.ifcr().write(|w| {
            w.set_ovrc(true);
            w.set_udrc(true);
            w.set_modfc(true);
            w.set_suspc(true);
        });
    }
}
