//! Embassy I2C3 owner for the standalone FMUv6X I2C connector.

use core::fmt;
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicU8, Ordering};

use embassy_stm32::i2c::{mode::Master, Error as I2cError, I2c};
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Timer};
use v6x_rust_ap::ina219::{self, register, Measurement};

const CONVERSION_TIMEOUT: Duration = Duration::from_millis(10);
const SAMPLE_PERIOD: Duration = Duration::from_micros(ina219::SAMPLE_PERIOD_US);

static LATEST: Watch<CriticalSectionRawMutex, Measurement, 0> = Watch::new();
static INITIALIZED: AtomicBool = AtomicBool::new(false);
static VALID: AtomicBool = AtomicBool::new(false);
static SAMPLES: AtomicU32 = AtomicU32::new(0);
static ERRORS: AtomicU32 = AtomicU32::new(0);
static LAST_ERROR: AtomicU8 = AtomicU8::new(0);
static CONFIG_READBACK: AtomicU16 = AtomicU16::new(0);
static CALIBRATION_READBACK: AtomicU16 = AtomicU16::new(0);
static LAST_I2C_ERROR: AtomicU8 = AtomicU8::new(0);
static ACTIVE_ADDRESS: AtomicU8 = AtomicU8::new(ina219::ADDRESS);
static RECOVERIES: AtomicU32 = AtomicU32::new(0);
static IDLE_SCL_HIGH: AtomicBool = AtomicBool::new(false);
static IDLE_SDA_HIGH: AtomicBool = AtomicBool::new(false);
static PROBE_RESULTS: Watch<CriticalSectionRawMutex, [Probe; 4], 0> = Watch::new();

#[derive(Clone, Copy)]
struct Probe {
    config: Option<u16>,
    i2c_error: u8,
}

#[derive(Clone, Copy, Debug)]
#[repr(u8)]
enum Error {
    I2c = 1,
    Timeout,
    Configuration,
    ConversionTimeout,
    Measurement,
}

pub fn latest() -> Option<Measurement> {
    if !VALID.load(Ordering::Acquire) {
        return None;
    }
    LATEST
        .try_get()
        .filter(|measurement| measurement.is_fresh(Instant::now().as_micros()))
}

pub fn sample_count() -> u32 {
    SAMPLES.load(Ordering::Relaxed)
}

pub fn health() -> v6x_rust_ap::mavlink_vehicle::SensorState {
    let present = INITIALIZED.load(Ordering::Acquire) || sample_count() != 0;
    v6x_rust_ap::mavlink_vehicle::SensorState {
        present,
        enabled: present,
        healthy: latest().is_some(),
    }
}

pub fn write_diagnostics(output: &mut impl fmt::Write) -> fmt::Result {
    let measurement = latest();
    writeln!(
        output,
        "INA219 schema=2 bus=I2C3 scl=PA8 sda=PH8 address={:02x} bus_hz=100000 initialized={} valid={} shunt_ohm={} max_current_a={} config={:04x} calibration={} samples={} errors={} last_error={} i2c_error={} timestamp_us={} voltage_v={} current_a={} power_w={}",
        ACTIVE_ADDRESS.load(Ordering::Relaxed),
        INITIALIZED.load(Ordering::Acquire),
        measurement.is_some(),
        ina219::SHUNT_RESISTANCE_OHM,
        ina219::MAX_CURRENT_A,
        CONFIG_READBACK.load(Ordering::Relaxed),
        CALIBRATION_READBACK.load(Ordering::Relaxed),
        SAMPLES.load(Ordering::Relaxed),
        ERRORS.load(Ordering::Relaxed),
        LAST_ERROR.load(Ordering::Relaxed),
        LAST_I2C_ERROR.load(Ordering::Relaxed),
        measurement.map_or(0, |value| value.timestamp_us),
        measurement.map_or(f32::NAN, |value| value.bus_voltage_v),
        measurement.map_or(f32::NAN, |value| value.current_a),
        measurement.map_or(f32::NAN, |value| value.power_w),
    )?;
    if let Some(probes) = PROBE_RESULTS.try_get() {
        for (address, probe) in ina219::PROBE_ADDRESSES.into_iter().zip(probes) {
            writeln!(
                output,
                "INA219_PROBE address={address:02x} responded={} config={:04x} i2c_error={}",
                probe.config.is_some(),
                probe.config.unwrap_or(0),
                probe.i2c_error,
            )?;
        }
    }
    let gpio_a = embassy_stm32::pac::GPIOA;
    let gpio_h = embassy_stm32::pac::GPIOH;
    writeln!(
        output,
        "INA219_BUS scl_high={} sda_high={} isr={:08x} timingr={:08x} periph_power_enabled={} scl_mode={} scl_pull={} scl_af={} sda_mode={} sda_pull={} sda_af={} recoveries={} idle_scl_high={} idle_sda_high={}",
        gpio_a.idr().read().0 & (1 << 8) != 0,
        gpio_h.idr().read().0 & (1 << 8) != 0,
        embassy_stm32::pac::I2C3.isr().read().0,
        embassy_stm32::pac::I2C3.timingr().read().0,
        embassy_stm32::pac::GPIOG.odr().read().0 & (1 << 4) == 0,
        gpio_a.moder().read().0 >> 16 & 3,
        gpio_a.pupdr().read().0 >> 16 & 3,
        gpio_a.afr(1).read().0 & 0xf,
        gpio_h.moder().read().0 >> 16 & 3,
        gpio_h.pupdr().read().0 >> 16 & 3,
        gpio_h.afr(1).read().0 & 0xf,
        RECOVERIES.load(Ordering::Relaxed),
        IDLE_SCL_HIGH.load(Ordering::Relaxed),
        IDLE_SDA_HIGH.load(Ordering::Relaxed),
    )
}

#[embassy_executor::task]
pub async fn run(i2c: I2c<'static, Async, Master>) -> ! {
    let mut sensor = Ina219 {
        i2c,
        address: ina219::ADDRESS,
    };
    loop {
        INITIALIZED.store(false, Ordering::Release);
        VALID.store(false, Ordering::Release);
        // Re-probe before every attempt so a DIP-switch change or late power-up
        // is adopted without a reboot; no writes are made to non-matching addresses.
        if let Some(address) = sensor.probe_addresses().await {
            sensor.address = address;
        }
        if let Err(error) = sensor.initialize().await {
            record_error(error);
            recover_bus();
            Timer::after_millis(500).await;
            continue;
        }
        INITIALIZED.store(true, Ordering::Release);
        loop {
            let next_sample = Instant::now() + SAMPLE_PERIOD;
            match sensor.measure().await {
                Ok(measurement) => {
                    LATEST.sender().send(measurement);
                    crate::sd_logger::record_current_sensor(
                        Instant::now().as_micros(),
                        measurement,
                    );
                    VALID.store(true, Ordering::Release);
                    LAST_ERROR.store(0, Ordering::Relaxed);
                    LAST_I2C_ERROR.store(0, Ordering::Relaxed);
                    SAMPLES.fetch_add(1, Ordering::Relaxed);
                }
                Err(error) => {
                    record_error(error);
                    recover_bus();
                    break;
                }
            }
            Timer::at(next_sample).await;
        }
        Timer::after_millis(500).await;
    }
}

fn record_error(error: Error) {
    INITIALIZED.store(false, Ordering::Release);
    VALID.store(false, Ordering::Release);
    ERRORS.fetch_add(1, Ordering::Relaxed);
    LAST_ERROR.store(error as u8, Ordering::Relaxed);
}

struct Ina219 {
    i2c: I2c<'static, Async, Master>,
    address: u8,
}

impl Ina219 {
    async fn probe_addresses(&mut self) -> Option<u8> {
        let mut probes = [Probe {
            config: None,
            i2c_error: 0,
        }; 4];
        let mut found = None;
        for (address, probe) in ina219::PROBE_ADDRESSES.into_iter().zip(&mut probes) {
            LAST_I2C_ERROR.store(0, Ordering::Relaxed);
            // Only set the register pointer and read CONFIG: no configuration
            // or calibration writes to any alternative address.
            probe.config = self
                .read_register_at(address, register::CONFIGURATION)
                .await
                .ok();
            probe.i2c_error = LAST_I2C_ERROR.load(Ordering::Relaxed);
            if probe.config.is_some() && found.is_none() {
                found = Some(address);
            }
        }
        PROBE_RESULTS.sender().send(probes);
        if let Some(address) = found {
            ACTIVE_ADDRESS.store(address, Ordering::Relaxed);
        }
        found
    }

    async fn initialize(&mut self) -> Result<(), Error> {
        self.write_register(register::CONFIGURATION, ina219::RESET)
            .await?;
        Timer::after_millis(1).await;
        if self.read_register(register::CONFIGURATION).await? != ina219::RESET_CONFIGURATION {
            return Err(Error::Configuration);
        }
        self.write_register(register::CALIBRATION, ina219::CALIBRATION_VALUE)
            .await?;
        self.write_register(register::CONFIGURATION, ina219::CONFIGURATION)
            .await?;
        self.check_configuration().await
    }

    async fn check_configuration(&mut self) -> Result<(), Error> {
        let config = self.read_register(register::CONFIGURATION).await?;
        let calibration = self.read_register(register::CALIBRATION).await?;
        CONFIG_READBACK.store(config, Ordering::Relaxed);
        CALIBRATION_READBACK.store(calibration, Ordering::Relaxed);
        if config != ina219::CONFIGURATION || calibration != ina219::CALIBRATION_VALUE {
            return Err(Error::Configuration);
        }
        Ok(())
    }

    async fn measure(&mut self) -> Result<Measurement, Error> {
        // Triggering a new conversion clears CNVR; all registers then belong to
        // one conversion and remain stable while the ADC is powered down.
        self.check_configuration().await?;
        self.write_register(register::CONFIGURATION, ina219::CONFIGURATION)
            .await?;
        let bus = self.wait_for_conversion().await?;
        let timestamp_us = Instant::now().as_micros();
        let shunt = self.read_register(register::SHUNT_VOLTAGE).await? as i16;
        let current = self.read_register(register::CURRENT).await? as i16;
        // Reading POWER clears CNVR. Signed telemetry power is V*I.
        let _ = self.read_register(register::POWER).await?;
        self.check_configuration().await?;
        Measurement::decode(bus, shunt, current, timestamp_us).map_err(|_| Error::Measurement)
    }

    async fn wait_for_conversion(&mut self) -> Result<u16, Error> {
        let deadline = Instant::now() + CONVERSION_TIMEOUT;
        loop {
            Timer::after_millis(2).await;
            if Instant::now() >= deadline {
                return Err(Error::ConversionTimeout);
            }
            let bus = self.read_register(register::BUS_VOLTAGE).await?;
            if bus & ina219::CONVERSION_READY != 0 {
                return Ok(bus);
            }
        }
    }

    async fn read_register(&mut self, register: u8) -> Result<u16, Error> {
        self.read_register_at(self.address, register).await
    }

    async fn read_register_at(&mut self, address: u8, register: u8) -> Result<u16, Error> {
        let mut bytes = [0; 2];
        self.i2c
            .write_read(address, &[register], &mut bytes)
            .await
            .map_err(record_i2c_error)?;
        Ok(u16::from_be_bytes(bytes))
    }

    async fn write_register(&mut self, register: u8, value: u16) -> Result<(), Error> {
        let [high, low] = value.to_be_bytes();
        self.i2c
            .write(self.address, &[register, high, low])
            .await
            .map_err(record_i2c_error)
    }
}

/// Release a bus left mid-transfer: disable the peripheral, read the idle
/// levels, clock SCL until SDA is free, issue a STOP, then restore AF4.
fn recover_bus() {
    use embassy_stm32::pac::gpio::vals::{Moder, Ot, Pupdr};

    let regs = embassy_stm32::pac::I2C3;
    let scl = embassy_stm32::pac::GPIOA;
    let sda = embassy_stm32::pac::GPIOH;
    regs.cr1().modify(|w| w.set_pe(false));

    scl.otyper().modify(|w| w.set_ot(8, Ot::OPEN_DRAIN));
    sda.otyper().modify(|w| w.set_ot(8, Ot::OPEN_DRAIN));
    scl.moder().modify(|w| w.set_moder(8, Moder::OUTPUT));
    sda.moder().modify(|w| w.set_moder(8, Moder::OUTPUT));
    scl.bsrr().write(|w| w.set_bs(8, true));
    sda.bsrr().write(|w| w.set_bs(8, true));
    scl.pupdr().modify(|w| w.set_pupdr(8, Pupdr::PULL_UP));
    sda.pupdr().modify(|w| w.set_pupdr(8, Pupdr::PULL_UP));
    cortex_m::asm::delay(2_400);

    IDLE_SCL_HIGH.store(scl.idr().read().0 & (1 << 8) != 0, Ordering::Relaxed);
    IDLE_SDA_HIGH.store(sda.idr().read().0 & (1 << 8) != 0, Ordering::Relaxed);

    for _ in 0..9 {
        if sda.idr().read().0 & (1 << 8) != 0 {
            break;
        }
        scl.bsrr().write(|w| w.set_br(8, true));
        cortex_m::asm::delay(2_400);
        scl.bsrr().write(|w| w.set_bs(8, true));
        cortex_m::asm::delay(2_400);
    }
    // STOP: SDA low while SCL high, then release SDA.
    sda.bsrr().write(|w| w.set_br(8, true));
    cortex_m::asm::delay(2_400);
    scl.bsrr().write(|w| w.set_bs(8, true));
    cortex_m::asm::delay(2_400);
    sda.bsrr().write(|w| w.set_bs(8, true));
    cortex_m::asm::delay(2_400);

    scl.moder().modify(|w| w.set_moder(8, Moder::ALTERNATE));
    sda.moder().modify(|w| w.set_moder(8, Moder::ALTERNATE));
    scl.afr(1).modify(|w| w.set_afr(0, 4));
    sda.afr(1).modify(|w| w.set_afr(0, 4));
    regs.icr().write(|w| {
        w.set_nackcf(true);
        w.set_berrcf(true);
        w.set_arlocf(true);
        w.set_ovrcf(true);
        w.set_stopcf(true);
    });
    regs.cr1().modify(|w| w.set_pe(true));
    RECOVERIES.fetch_add(1, Ordering::Relaxed);
}

fn record_i2c_error(error: I2cError) -> Error {
    LAST_I2C_ERROR.store(
        match error {
            I2cError::Nack => 1,
            I2cError::Bus => 2,
            I2cError::Arbitration => 3,
            I2cError::Timeout => 4,
            I2cError::Crc => 5,
            I2cError::Overrun => 6,
            I2cError::ZeroLengthTransfer => 7,
        },
        Ordering::Relaxed,
    );
    if error == I2cError::Timeout {
        Error::Timeout
    } else {
        Error::I2c
    }
}
