//! Fixed FMUv6X Rev 3 BMM150. Shares I2C4 with BMP388 on the thread executor.
//! No RefCell borrow spans an await; transactions use the HAL's bounded blocking
//! path because I2C4 BDMA cannot access the current AXI buffers.

use core::cell::RefCell;
use core::fmt;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use embassy_stm32::{
    i2c::{mode::Master, Error as I2cError, I2c},
    mode::Blocking,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Instant, Timer};
use v6x_rust_led::bmm150::{self, register, Sample, Trim};
use v6x_rust_led::mavlink_vehicle::{self, SensorState};

const RECOVERY_MS: u64 = 500;
static ERRORS: AtomicU32 = AtomicU32::new(0);
static SAMPLES: AtomicU32 = AtomicU32::new(0);
static RESETS: AtomicU32 = AtomicU32::new(0);
static CHIP_ID: AtomicU8 = AtomicU8::new(0);
static MAX_TRANSACTION_US: AtomicU32 = AtomicU32::new(0);
static LATEST: Watch<CriticalSectionRawMutex, State, 0> = Watch::new();

#[derive(Clone, Copy, Debug)]
enum Error {
    Bus(I2cError),
    Timeout,
    Configure,
    SelfTest,
    Decode(bmm150::Error),
}

#[derive(Clone, Copy)]
struct State {
    sample: Option<Sample>,
    error: Option<Error>,
}

pub fn latest() -> Option<Sample> {
    LATEST
        .try_get()
        .filter(|state| state.error.is_none())?
        .sample
        .filter(|sample| sample.fresh(Instant::now().as_micros()))
}

pub fn health() -> SensorState {
    mavlink_vehicle::magnetometer_health(
        CHIP_ID.load(Ordering::Acquire),
        latest(),
        Instant::now().as_micros(),
    )
}

struct Sensor {
    // Only sibling tasks on the thread executor receive this non-Sync reference.
    i2c: &'static RefCell<I2c<'static, Blocking, Master>>,
}

impl Sensor {
    fn read(&self, register: u8, bytes: &mut [u8]) -> Result<(), Error> {
        let started = Instant::now();
        let result = self
            .i2c
            .borrow_mut()
            .blocking_write_read(bmm150::ADDRESS, &[register], bytes);
        self.finish(result, started)
    }

    fn write(&self, register: u8, value: u8) -> Result<(), Error> {
        let started = Instant::now();
        let result = self
            .i2c
            .borrow_mut()
            .blocking_write(bmm150::ADDRESS, &[register, value]);
        self.finish(result, started)
    }

    fn finish(&self, result: Result<(), I2cError>, started: Instant) -> Result<(), Error> {
        let elapsed = started.elapsed().as_micros();
        MAX_TRANSACTION_US.fetch_max(elapsed.min(u64::from(u32::MAX)) as u32, Ordering::Relaxed);
        if elapsed > 10_000 {
            return Err(Error::Timeout);
        }
        result.map_err(Error::Bus)
    }

    fn byte(&self, register: u8) -> Result<u8, Error> {
        let mut bytes = [0];
        self.read(register, &mut bytes)?;
        Ok(bytes[0])
    }

    fn check(&self, setting: (u8, u8, u8)) -> Result<(), Error> {
        let (register, value, mask) = setting;
        if self.byte(register)? & mask != value & mask {
            return Err(Error::Configure);
        }
        Ok(())
    }

    async fn initialize(&self) -> Result<Trim, Error> {
        let id = self.byte(register::CHIP_ID)?;
        let power = self.byte(register::POWER)?;
        if bmm150::needs_wake(id, power).map_err(Error::Decode)? {
            // The only pre-identification write: wake the documented suspended
            // device at this board's fixed address. Never scan external buses.
            self.write(register::POWER, bmm150::POWER_ON)?;
            Timer::after_millis(3).await;
        }
        let id = self.byte(register::CHIP_ID)?;
        CHIP_ID.store(id, Ordering::Release);
        if id != bmm150::CHIP_ID {
            return Err(Error::Decode(bmm150::Error::Identity));
        }
        self.write(register::POWER, bmm150::SOFT_RESET)?;
        RESETS.fetch_add(1, Ordering::Relaxed);
        Timer::after_millis(10).await;
        if self.byte(register::CHIP_ID)? != bmm150::CHIP_ID
            || self.byte(register::POWER)? & 0x83 != bmm150::POWER_ON
            || self.byte(register::OP_MODE)? != bmm150::SLEEP_MODE
        {
            return Err(Error::Configure);
        }
        self.write(register::OP_MODE, bmm150::SLEEP_MODE | bmm150::SELF_TEST)?;
        let mut self_test_done = false;
        for _ in 0..10 {
            Timer::after_millis(10).await;
            if self.byte(register::OP_MODE)? & bmm150::SELF_TEST == 0 {
                self_test_done = true;
                break;
            }
        }
        if !self_test_done {
            return Err(Error::SelfTest);
        }
        let mut self_test = [0; 6];
        self.read(register::DATA, &mut self_test)?;
        if [self_test[0], self_test[2], self_test[4]]
            .iter()
            .any(|v| v & 1 == 0)
        {
            // Unlike PX4's eventual "reluctantly proceed", fail closed here.
            return Err(Error::SelfTest);
        }
        let mut xy1 = [0; 2];
        let mut z4_xy2 = [0; 4];
        let mut z2_xyz1 = [0; 10];
        self.read(register::TRIM_XY1, &mut xy1)?;
        self.read(register::TRIM_Z4_XY2, &mut z4_xy2)?;
        self.read(register::TRIM_Z2_XYZ1, &mut z2_xyz1)?;
        let trim = Trim::decode(&xy1, &z4_xy2, &z2_xyz1).map_err(Error::Decode)?;
        for setting in bmm150::SETTINGS {
            self.write(setting.0, setting.1)?;
            self.check(setting)?;
        }
        Ok(trim)
    }

    fn collect(&self, trim: &Trim, sequence: u32) -> Result<Sample, Error> {
        let timestamp_sample = Instant::now().as_micros();
        let mut bytes = [0; bmm150::DATA_LEN];
        self.read(register::DATA, &mut bytes)?;
        let field_gauss = trim.compensate(&bytes).map_err(Error::Decode)?;
        Ok(Sample {
            timestamp_sample,
            sequence,
            field_gauss,
            error_count: ERRORS.load(Ordering::Relaxed),
        })
    }
}

#[embassy_executor::task]
pub async fn run(i2c: &'static RefCell<I2c<'static, Blocking, Master>>) -> ! {
    let sensor = Sensor { i2c };
    let mut sequence = 0_u32;
    loop {
        LATEST.sender().send(State {
            sample: None,
            error: None,
        });
        let result: Result<(), Error> = async {
            let trim = sensor.initialize().await?;
            let mut last_good = Instant::now();
            let mut next_read = last_good + Duration::from_micros(bmm150::PERIOD_US);
            let mut next_check = last_good;
            let mut checked_register = 0;
            loop {
                Timer::at(next_read).await;
                next_read = Instant::now() + Duration::from_micros(bmm150::PERIOD_US);
                if Instant::now() >= next_check {
                    sensor.check(bmm150::SETTINGS[checked_register])?;
                    checked_register = (checked_register + 1) % bmm150::SETTINGS.len();
                    next_check = Instant::now() + Duration::from_millis(100);
                }
                let sample = match sensor.collect(&trim, sequence) {
                    Ok(sample) => sample,
                    Err(Error::Decode(bmm150::Error::NotReady))
                        if last_good.elapsed().as_micros() <= bmm150::FRESH_US =>
                    {
                        continue
                    }
                    Err(error) => return Err(error),
                };
                last_good = Instant::now();
                sequence = sequence.wrapping_add(1);
                SAMPLES.fetch_add(1, Ordering::Relaxed);
                LATEST.sender().send(State {
                    sample: Some(sample),
                    error: None,
                });
                crate::sd_logger::record_mag(sample);
                if let Some(measurement) = sample.measurement(Instant::now().as_micros()) {
                    crate::estimator::submit_measurement(measurement, bmm150::DEVICE_ID);
                }
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
        // PG8 is shared with BMP388: never power-cycle the rail for recovery.
        Timer::after_millis(RECOVERY_MS).await;
    }
}

pub fn write_diagnostics(output: &mut impl fmt::Write) -> fmt::Result {
    let error = LATEST.try_get().and_then(|state| state.error);
    let sample = latest();
    writeln!(output, "MAG schema=1 sensor=BMM150 bus=I2C4 scl=PF14 sda=PF15 address=10 power=PG8 chip_id={:02x} device_id={} external=false rotation=0 calibration=factory_trim_only valid={} samples={} errors={} resets={} period_us={} max_transaction_us={} last_error={:?}",
        CHIP_ID.load(Ordering::Acquire), bmm150::DEVICE_ID, sample.is_some(), SAMPLES.load(Ordering::Relaxed), ERRORS.load(Ordering::Relaxed), RESETS.load(Ordering::Relaxed), bmm150::PERIOD_US, MAX_TRANSACTION_US.load(Ordering::Relaxed), error)?;
    if let Some(sample) = sample {
        writeln!(
            output,
            "MAG_DATA timestamp_sample={} sequence={} x_gauss={} y_gauss={} z_gauss={} age_us={}",
            sample.timestamp_sample,
            sample.sequence,
            sample.field_gauss[0],
            sample.field_gauss[1],
            sample.field_gauss[2],
            Instant::now()
                .as_micros()
                .saturating_sub(sample.timestamp_sample)
        )?;
    }
    if let Some(Error::Bus(error)) = error {
        writeln!(output, "MAG_I2C_ERROR error={error:?}")?;
    }
    if let Some(Error::Decode(error)) = error {
        writeln!(output, "MAG_DATA_ERROR error={error:?}")?;
    }
    Ok(())
}
