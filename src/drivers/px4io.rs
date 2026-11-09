//! Single USART6 owner for the existing PX4IO v5 firmware. No IO updater.

use core::fmt;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_futures::join::join;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{Error as UartError, Uart, UartRx, UartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use v6x_rust_ap::px4io::{
    self, flags, page, setup, Configuration, PacketError, Request, SafeWrite, Status,
};

const TRANSACTION_TIMEOUT: Duration = Duration::from_millis(10);
const RETRY_DELAY: Duration = Duration::from_millis(1);
const RECOVERY_DELAY: Duration = Duration::from_millis(500);
const MAX_ATTEMPTS: u8 = 3;

static LATEST: Watch<CriticalSectionRawMutex, Snapshot, 0> = Watch::new();
static TRANSACTIONS: AtomicU32 = AtomicU32::new(0);
static RETRIES: AtomicU32 = AtomicU32::new(0);
static ERRORS: AtomicU32 = AtomicU32::new(0);
static TIMEOUTS: AtomicU32 = AtomicU32::new(0);
static CRC_ERRORS: AtomicU32 = AtomicU32::new(0);
static BUTTON_EVENTS: AtomicU32 = AtomicU32::new(0);
static BUTTON_ACKS: AtomicU32 = AtomicU32::new(0);
static MAX_TRANSACTION_US: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy, Debug)]
enum Error {
    Uart(UartError),
    Timeout,
    Packet(PacketError),
    Configuration,
    NotInitialized,
    OutputsNotInhibited,
}

impl Error {
    fn retryable(self) -> bool {
        matches!(
            self,
            Self::Uart(_)
                | Self::Timeout
                | Self::Packet(
                    PacketError::Crc | PacketError::CorruptRequest | PacketError::Length
                )
        )
    }
}

#[derive(Clone, Copy)]
struct Snapshot {
    configuration: Option<Configuration>,
    firmware_crc: Option<u32>,
    status: Option<Status>,
    configured: bool,
    error: Option<Error>,
}

impl Snapshot {
    const fn new() -> Self {
        Self {
            configuration: None,
            firmware_crc: None,
            status: None,
            configured: false,
            error: None,
        }
    }

    fn ready(self, now_us: u64) -> bool {
        self.configured
            && self.error.is_none()
            && self.status.is_some_and(|status| {
                status.is_fresh(now_us)
                    && status.flags & flags::INIT_OK != 0
                    && status.outputs_inhibited()
            })
    }
}

struct Serial {
    tx: UartTx<'static, Async>,
    rx: UartRx<'static, Async>,
}

// Both buffers stay alive until cancellation/completion stops their DMA.
// Static task storage and the shared stack are AXI SRAM; D-cache is disabled.
#[repr(align(32))]
struct ReceiveBuffer([u8; px4io::MAX_PACKET_LEN]);

impl Serial {
    async fn exchange(&mut self, request: &Request, output: &mut [u16]) -> Result<(), Error> {
        let mut result = Err(Error::Timeout);
        for attempt in 0..MAX_ATTEMPTS {
            TRANSACTIONS.fetch_add(1, Ordering::Relaxed);
            let started_at = Instant::now();
            result = self.exchange_once(request, output).await;
            let elapsed_us = started_at.elapsed().as_micros();
            MAX_TRANSACTION_US.fetch_max(
                elapsed_us.min(u64::from(u32::MAX)) as u32,
                Ordering::Relaxed,
            );
            match result {
                Ok(()) => return Ok(()),
                Err(error) => {
                    ERRORS.fetch_add(1, Ordering::Relaxed);
                    if matches!(error, Error::Timeout) {
                        TIMEOUTS.fetch_add(1, Ordering::Relaxed);
                    }
                    if matches!(
                        error,
                        Error::Packet(PacketError::Crc | PacketError::CorruptRequest)
                    ) {
                        CRC_ERRORS.fetch_add(1, Ordering::Relaxed);
                    }
                    if !error.retryable() || attempt + 1 == MAX_ATTEMPTS {
                        return Err(error);
                    }
                    RETRIES.fetch_add(1, Ordering::Relaxed);
                    // Abort a partial IO receive packet using the HAL's serial
                    // break, then allow error/IDLE handling before the retry.
                    self.tx.send_break();
                    Timer::after(RETRY_DELAY).await;
                }
            }
        }
        result
    }

    async fn exchange_once(&mut self, request: &Request, output: &mut [u16]) -> Result<(), Error> {
        let mut response = ReceiveBuffer([0; px4io::MAX_PACKET_LEN]);
        let started_at = Instant::now();
        let exchange = async {
            // RX must be armed before TX: IO can reply immediately after its
            // IDLE interrupt. join polls its first future first.
            let (received, sent) = join(self.rx.read_until_idle(&mut response.0), async {
                self.tx.write(request.as_bytes()).await?;
                self.tx.flush().await
            })
            .await;
            sent.map_err(Error::Uart)?;
            received.map_err(Error::Uart)
        };
        let length = with_timeout(TRANSACTION_TIMEOUT, exchange)
            .await
            .map_err(|_| Error::Timeout)??;
        // The thread executor can be delayed by synchronous SD/HAL work. Do not
        // accept a late reply just because its future is polled before Timer.
        if started_at.elapsed() > TRANSACTION_TIMEOUT {
            return Err(Error::Timeout);
        }
        request
            .decode_reply(&response.0[..length])
            .and_then(|reply| reply.copy_registers(output))
            .map_err(Error::Packet)
    }

    async fn read(&mut self, page: u8, offset: u8, output: &mut [u16]) -> Result<(), Error> {
        let request = Request::read(page, offset, output.len()).map_err(Error::Packet)?;
        self.exchange(&request, output).await
    }

    async fn write(&mut self, operation: SafeWrite) -> Result<(), Error> {
        let request = Request::safe_write(operation).map_err(Error::Packet)?;
        self.exchange(&request, &mut []).await
    }

    async fn configure(&mut self, snapshot: &mut Snapshot) -> Result<usize, Error> {
        let mut registers = [0; 8];
        self.read(page::CONFIG, 0, &mut registers).await?;
        let config = Configuration::from_registers(registers);
        snapshot.configuration = Some(config);
        if !config.is_supported() {
            // Do not reproduce PX4's automatic reboot/update fallback.
            return Err(Error::Configuration);
        }
        let actuators = usize::from(config.actuators);
        self.write(SafeWrite::Disarm).await?;
        self.write(SafeWrite::SafetyOn).await?;
        self.write(SafeWrite::DisableSbus).await?;
        self.write(SafeWrite::DisableDisarmedPwm(actuators)).await?;
        let mut crc = [0; 2];
        self.read(page::SETUP, setup::CRC, &mut crc).await?;
        snapshot.firmware_crc = Some(u32::from(crc[0]) | (u32::from(crc[1]) << 16));
        // Allow IO's mixer to apply the disabled pulse widths before readback.
        Timer::after_millis(px4io::STATUS_PERIOD_MS).await;
        Ok(actuators)
    }

    async fn status(&mut self, actuators: usize) -> Result<Status, Error> {
        let mut registers = [0; 8];
        let mut control = [0; 2];
        let mut pwm = [0; px4io::MAX_ACTUATORS];
        let mut disarmed_pwm = [0; px4io::MAX_ACTUATORS];
        let timestamp_us = Instant::now().as_micros();
        self.read(page::STATUS, 0, &mut registers).await?;
        self.read(page::SETUP, setup::FEATURES, &mut control)
            .await?;
        self.read(page::SERVOS, 0, &mut pwm[..actuators]).await?;
        self.read(page::DISARMED_PWM, 0, &mut disarmed_pwm[..actuators])
            .await?;
        Ok(Status {
            timestamp_us,
            flags: registers[2],
            alarms: registers[3],
            features: control[0],
            arming: control[1],
            servo_mv: registers[6],
            rssi_mv: registers[7],
            pwm,
            disarmed_pwm,
        })
    }
}

#[embassy_executor::task]
pub async fn run(uart: Uart<'static, Async>) -> ! {
    let (tx, rx) = uart.split();
    let mut serial = Serial { tx, rx };
    let mut snapshot = Snapshot::new();
    let mut event_latched = false;
    loop {
        snapshot.configured = false;
        snapshot.error = None;
        LATEST.sender().send(snapshot);
        let result: Result<(), Error> = async {
            let actuators = serial.configure(&mut snapshot).await?;
            loop {
                let status = serial.status(actuators).await?;
                snapshot.status = Some(status);
                if status.flags & flags::INIT_OK == 0 {
                    return Err(Error::NotInitialized);
                }
                if !status.outputs_inhibited() {
                    return Err(Error::OutputsNotInhibited);
                }
                let button_event = status.flags & flags::SAFETY_BUTTON_EVENT != 0;
                if button_event && !event_latched {
                    BUTTON_EVENTS.fetch_add(1, Ordering::Relaxed);
                }
                event_latched = button_event;
                if button_event {
                    // Official IO reports only a >=1 s latch, not raw press or
                    // release. Keep this separate from PF5 gesture recognition
                    // so one physical press cannot toggle the logger twice.
                    serial.write(SafeWrite::AcknowledgeButton).await?;
                    BUTTON_ACKS.fetch_add(1, Ordering::Relaxed);
                }
                snapshot.configured = true;
                snapshot.error = None;
                LATEST.sender().send(snapshot);
                Timer::after_millis(px4io::STATUS_PERIOD_MS).await;
            }
        }
        .await;
        if let Err(error) = result {
            snapshot.configured = false;
            snapshot.error = Some(error);
            LATEST.sender().send(snapshot);
        }
        // No automatic re-arm, nonzero output, IO reset, or firmware update.
        Timer::after(RECOVERY_DELAY).await;
    }
}

pub fn ready() -> bool {
    LATEST
        .try_get()
        .is_some_and(|snapshot| snapshot.ready(Instant::now().as_micros()))
}

pub fn write_diagnostics(output: &mut impl fmt::Write) -> fmt::Result {
    let snapshot = LATEST.try_get().unwrap_or_else(Snapshot::new);
    let now_us = Instant::now().as_micros();
    let config = snapshot.configuration;
    let status = snapshot.status;
    writeln!(
        output,
        "PX4IO schema=1 uart=USART6 tx=PC6 rx=PC7 baud=1500000 ready={} protocol={} hardware={} bootloader={} actuators={} rc_inputs={} firmware_crc={:08x} configured={} stale={} age_us={} last_error={:?}",
        snapshot.ready(now_us),
        config.map_or(0, |value| value.protocol),
        config.map_or(0, |value| value.hardware),
        config.map_or(0, |value| value.bootloader),
        config.map_or(0, |value| value.actuators),
        config.map_or(0, |value| value.rc_inputs),
        snapshot.firmware_crc.unwrap_or(0),
        snapshot.configured,
        !status.is_some_and(|value| value.is_fresh(now_us)),
        status.map_or(u64::MAX, |value| now_us.saturating_sub(value.timestamp_us)),
        snapshot.error,
    )?;
    if let Some(status) = status {
        writeln!(
            output,
            "PX4IO_STATUS flags={:04x} alarms={:04x} features={:04x} arming={:04x} safety_off={} timer_outputs_enabled={} init_ok={} rc_ok={} fmu_ok={} arm_sync={} failsafe={} servo_mv={} rssi_mv={} pwm={:?} disarmed_pwm={:?}",
            status.flags, status.alarms, status.features, status.arming,
            status.flags & flags::SAFETY_OFF != 0,
            status.flags & flags::OUTPUTS_ARMED != 0,
            status.flags & flags::INIT_OK != 0,
            status.flags & flags::RC_OK != 0,
            status.flags & flags::FMU_OK != 0,
            status.flags & flags::ARM_SYNC != 0,
            status.flags & flags::FAILSAFE != 0,
            status.servo_mv, status.rssi_mv, status.pwm, status.disarmed_pwm,
        )?;
    }
    if let Some(Error::Uart(error)) = snapshot.error {
        writeln!(output, "PX4IO_UART_ERROR error={error:?}")?;
    }
    writeln!(
        output,
        "PX4IO_COUNTERS transactions={} retries={} errors={} timeouts={} crc_errors={} button_events={} button_acks={} max_transaction_us={} safety_led_owner=PX4IO safety_led_override=false io_update=false",
        TRANSACTIONS.load(Ordering::Relaxed), RETRIES.load(Ordering::Relaxed),
        ERRORS.load(Ordering::Relaxed), TIMEOUTS.load(Ordering::Relaxed),
        CRC_ERRORS.load(Ordering::Relaxed), BUTTON_EVENTS.load(Ordering::Relaxed),
        BUTTON_ACKS.load(Ordering::Relaxed), MAX_TRANSACTION_US.load(Ordering::Relaxed),
    )
}
