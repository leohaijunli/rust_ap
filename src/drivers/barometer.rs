//! BMP388 device owner sharing internal I2C4 with BMM150 on the thread executor.
//! I2C4 uses BDMA, which cannot access the current AXI buffers. The HAL's bounded
//! blocking transfers avoid an invalid DMA placement; IMU remains on its higher
//! priority interrupt executor. This is not an async preemption guarantee.

use core::cell::RefCell;
use core::fmt;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use embassy_stm32::{
    i2c::{mode::Master, Error as I2cError, I2c},
    mode::Blocking,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Instant, Timer};
use v6x_rust_led::bmp388::{self, register, Calibration, Sample};
use v6x_rust_led::mavlink_vehicle::{self, SensorState};

const TRANSACTION_US: u64 = 10_000;
const RECOVERY_MS: u64 = 500;
static ERRORS: AtomicU32 = AtomicU32::new(0);
static SAMPLES: AtomicU32 = AtomicU32::new(0);
static CHIP_ID: AtomicU8 = AtomicU8::new(0);
static REVISION: AtomicU8 = AtomicU8::new(0);
static TRIM_CRC: AtomicU8 = AtomicU8::new(0);
static MAX_TRANSACTION_US: AtomicU32 = AtomicU32::new(0);
static LATEST: Watch<CriticalSectionRawMutex, State, 0> = Watch::new();

#[derive(Clone, Copy, Debug)]
enum Error {
    Bus(I2cError),
    Timeout,
    Identity,
    Configure,
    Decode(bmp388::Error),
}

#[derive(Clone, Copy)]
struct State {
    sample: Option<Sample>,
    error: Option<Error>,
}

pub fn latest() -> Option<Sample> {
    let state = LATEST.try_get()?;
    if state.error.is_some() {
        return None;
    }
    state
        .sample
        .filter(|sample| sample.fresh(Instant::now().as_micros()))
}

pub fn health() -> SensorState {
    let sample = LATEST
        .try_get()
        .filter(|state| state.error.is_none())
        .and_then(|state| state.sample);
    mavlink_vehicle::barometer_health(
        CHIP_ID.load(Ordering::Acquire),
        sample,
        Instant::now().as_micros(),
    )
}

struct Sensor {
    i2c: &'static RefCell<I2c<'static, Blocking, Master>>,
}

impl Sensor {
    fn read(&mut self, register: u8, bytes: &mut [u8]) -> Result<(), Error> {
        let started = Instant::now();
        let result = self
            .i2c
            .borrow_mut()
            .blocking_write_read(bmp388::ADDRESS, &[register], bytes);
        self.finish(result, started)
    }

    fn write(&mut self, register: u8, value: u8) -> Result<(), Error> {
        let started = Instant::now();
        let result = self
            .i2c
            .borrow_mut()
            .blocking_write(bmp388::ADDRESS, &[register, value]);
        self.finish(result, started)
    }

    fn finish(&self, result: Result<(), I2cError>, started: Instant) -> Result<(), Error> {
        let elapsed = started.elapsed().as_micros();
        MAX_TRANSACTION_US.fetch_max(elapsed.min(u64::from(u32::MAX)) as u32, Ordering::Relaxed);
        if elapsed > TRANSACTION_US {
            return Err(Error::Timeout);
        }
        result.map_err(Error::Bus)
    }

    fn byte(&mut self, register: u8) -> Result<u8, Error> {
        let mut data = [0];
        self.read(register, &mut data)?;
        Ok(data[0])
    }

    async fn initialize(&mut self) -> Result<Calibration, Error> {
        let id = self.byte(register::CHIP_ID)?;
        CHIP_ID.store(id, Ordering::Release);
        if id != bmp388::CHIP_ID {
            return Err(Error::Identity);
        }
        // Never issue reset to a merely ACKing or mismatched I2C device.
        if self.byte(register::STATUS)? & bmp388::COMMAND_READY == 0 {
            return Err(Error::Configure);
        }
        self.write(register::COMMAND, bmp388::RESET)?;
        Timer::after_millis(2).await;
        if self.byte(register::ERROR)? & 7 != 0 {
            return Err(Error::Configure);
        }
        if self.byte(register::CHIP_ID)? != bmp388::CHIP_ID {
            return Err(Error::Identity);
        }
        REVISION.store(self.byte(register::REVISION)?, Ordering::Release);
        let mut bytes = [0; bmp388::TRIM_LEN];
        self.read(register::TRIM, &mut bytes)?;
        let crc = self.byte(register::TRIM_CRC)?;
        TRIM_CRC.store(crc, Ordering::Release);
        let calibration = Calibration::decode(&bytes, crc).map_err(Error::Decode)?;
        for (register, value, mask) in bmp388::SETTINGS {
            self.write(register, value)?;
            if self.byte(register)? & mask != value & mask {
                return Err(Error::Configure);
            }
        }
        Ok(calibration)
    }

    fn collect(&mut self, calibration: &Calibration, sequence: u32) -> Result<Sample, Error> {
        let error = self.byte(register::ERROR)?;
        let timestamp_sample = Instant::now().as_micros();
        let mut data = [0; 7];
        self.read(register::STATUS, &mut data)?;
        let (pressure_pa, temperature_c) = calibration
            .compensate(&data, error)
            .map_err(Error::Decode)?;
        Ok(Sample {
            timestamp_sample,
            sequence,
            pressure_pa,
            temperature_c,
            error_count: ERRORS.load(Ordering::Relaxed),
        })
    }
}

#[embassy_executor::task]
pub async fn run(i2c: &'static RefCell<I2c<'static, Blocking, Master>>) -> ! {
    let mut sensor = Sensor { i2c };
    let mut sequence = 0_u32;
    loop {
        LATEST.sender().send(State {
            sample: None,
            error: None,
        });
        let result: Result<(), Error> = async {
            let calibration = sensor.initialize().await?;
            loop {
                sensor.write(register::POWER, bmp388::FORCE)?;
                Timer::after_micros(bmp388::CONVERSION_US).await;
                let deadline = Instant::now() + Duration::from_millis(10);
                let sample = loop {
                    match sensor.collect(&calibration, sequence) {
                        Ok(sample) => break sample,
                        Err(Error::Decode(bmp388::Error::NotReady))
                            if Instant::now() < deadline =>
                        {
                            Timer::after_millis(1).await
                        }
                        Err(error) => return Err(error),
                    }
                };
                sequence = sequence.wrapping_add(1);
                SAMPLES.fetch_add(1, Ordering::Relaxed);
                LATEST.sender().send(State {
                    sample: Some(sample),
                    error: None,
                });
                crate::sd_logger::record_baro(sample);
                crate::estimator::submit_measurement(
                    v6x_rust_led::estimator::Measurement::Baro(
                        v6x_rust_led::estimator::imu_ekf::baro_mag::BaroAltitude {
                            timestamp: sample.timestamp_sample,
                            altitude_m: sample.pressure_altitude_m(),
                        },
                    ),
                    bmp388::DEVICE_ID,
                );
            }
        }
        .await;
        if let Err(error) = result {
            ERRORS.fetch_add(1, Ordering::Relaxed);
            LATEST.sender().send(State {
                sample: None,
                error: Some(error),
            });
        }
        // Do not toggle PG8: the rail also feeds other internal devices. Retry
        // this identified device's soft reset/configuration after backoff only.
        Timer::after_millis(RECOVERY_MS).await;
    }
}

pub fn write_diagnostics(output: &mut impl fmt::Write) -> fmt::Result {
    let state = LATEST.try_get();
    let sample = latest();
    writeln!(output, "BARO schema=1 sensor=BMP388 bus=I2C4 scl=PF14 sda=PF15 address=77 power=PG8 chip_id={:02x} revision={:02x} trim_crc={:02x} valid={} samples={} errors={} conversion_us={} max_transaction_us={} last_error={:?}",
        CHIP_ID.load(Ordering::Acquire), REVISION.load(Ordering::Acquire), TRIM_CRC.load(Ordering::Acquire), sample.is_some(),
        SAMPLES.load(Ordering::Relaxed), ERRORS.load(Ordering::Relaxed), bmp388::CONVERSION_US, MAX_TRANSACTION_US.load(Ordering::Relaxed), state.and_then(|s| s.error))?;
    if let Some(sample) = sample {
        writeln!(output, "BARO_DATA timestamp_sample={} sequence={} pressure_pa={} temperature_c={} pressure_altitude_m={} age_us={}",
            sample.timestamp_sample, sample.sequence, sample.pressure_pa, sample.temperature_c, sample.pressure_altitude_m(),
            Instant::now().as_micros().saturating_sub(sample.timestamp_sample))?;
    }
    if let Some(Error::Bus(error)) = state.and_then(|s| s.error) {
        writeln!(output, "BARO_I2C_ERROR error={error:?}")?;
    }
    if let Some(Error::Decode(error)) = state.and_then(|s| s.error) {
        writeln!(output, "BARO_DATA_ERROR error={error:?}")?;
    }
    Ok(())
}
