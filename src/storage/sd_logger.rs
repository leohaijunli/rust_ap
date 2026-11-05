//! Single-owner SDMMC2/FAT task consuming the bounded ULog queues.

use core::fmt;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use block_device_adapters::BufStream;
use embassy_futures::select::{select, Either};
use embassy_stm32::gpio::Output;
use embassy_stm32::sdmmc::sd::{Card, CmdBlock, StorageDevice};
use embassy_stm32::sdmmc::Sdmmc;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use embedded_fatfs::{
    Date, DateTime, Error as FatError, FileSystem, FsOptions, Time, TimeProvider,
};
use embedded_io_async::{Read, Seek, SeekFrom, Write};
use embedded_partitions::mbr::Scheme;
use v6x_rust_led::log_config::Group;
use v6x_rust_led::log_queue::{Class, Logger};
use v6x_rust_led::logger::{
    accepts, download_count, is_ulog_name, list_bounds, Command, Config as LoggerConfig,
    ControlError, Link, LogError, LogRequest, LogResponse, RoutedLogRequest, Snapshot, Status,
};
use v6x_rust_led::parameters::{Kind, REGISTRY};
use v6x_rust_led::px4_messages::{
    ActuatorArmed, SensorAccel, SensorBaro, SensorGps, SensorGyro, PX4_REVISION,
};
use v6x_rust_led::ulog::{self, DataEncoder, TopicId, TOPICS};
use v6x_rust_led::wall_clock::Utc;

use crate::parameter_manager::ParameterManager;

const STAGING_BYTES: usize = 32 * 1024;
const CARD_HZ: Hertz = Hertz(25_000_000);
const START_DEFER: Duration = Duration::from_millis(100);
const INIT_TIMEOUT: Duration = Duration::from_secs(5);
const MANAGEMENT_IDLE_TIMEOUT: Duration = Duration::from_secs(10);
const RESPONSE_TIMEOUT: Duration = Duration::from_secs(2);

pub static LOGGER: Logger = Logger::new();

struct RuntimeState {
    status: AtomicU8,
    stage: AtomicU8,
    file_index: AtomicU32,
    bytes_written: AtomicU32,
    flushes: AtomicU32,
    errors: AtomicU32,
    sync_period_ms: AtomicU32,
}

impl RuntimeState {
    const fn new() -> Self {
        Self {
            status: AtomicU8::new(Status::Disabled as u8),
            stage: AtomicU8::new(0),
            file_index: AtomicU32::new(0),
            bytes_written: AtomicU32::new(0),
            flushes: AtomicU32::new(0),
            errors: AtomicU32::new(0),
            sync_period_ms: AtomicU32::new(LoggerConfig::DEFAULT.sync_period_ms),
        }
    }

    fn set_status(&self, status: Status, stage: u8) {
        self.stage.store(stage, Ordering::Relaxed);
        self.status.store(status as u8, Ordering::Release);
    }
}

static STATE: RuntimeState = RuntimeState::new();
static CONTROL: Channel<CriticalSectionRawMutex, Command, 4> = Channel::new();
static LOG_REQUESTS: Channel<CriticalSectionRawMutex, RoutedLogRequest, 4> = Channel::new();
static USB_LOG_RESPONSES: Channel<CriticalSectionRawMutex, LogResponse, 8> = Channel::new();
static UART_LOG_RESPONSES: Channel<CriticalSectionRawMutex, LogResponse, 8> = Channel::new();
static MANAGEMENT_LINK: AtomicU8 = AtomicU8::new(0);
static MOTOR_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static ESC_SEQUENCE: AtomicU32 = AtomicU32::new(0);

#[repr(align(32))]
struct Staging([u8; STAGING_BYTES]);

pub fn initialize() -> bool {
    LOGGER.initialize()
}

pub fn ready() -> bool {
    current_status() == Status::Ready
}

pub fn record_baro(sample: v6x_rust_led::bmp388::Sample) {
    if !ready() || !v6x_rust_led::log_config::rate(Group::Barometer, sample.timestamp_sample) {
        return;
    }
    let now = Instant::now().as_micros();
    if !sample.fresh(now) {
        return;
    }
    let report = SensorBaro::from_sample(sample, now);
    let payload = ulog::sensor_baro_payload(report);
    let record = ulog::data_record(
        TopicId::SensorBaro as u16,
        report.timestamp,
        payload.as_bytes(),
    )
    .expect("fixed SensorBaro fits the runtime record");
    let _ = LOGGER.try_record(Class::Normal, now, record);
}

pub fn request_start() -> Result<(), ControlError> {
    let current = current_status();
    if !accepts(current, Command::Start)
        || STATE
            .status
            .compare_exchange(
                current as u8,
                Status::Initializing as u8,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_err()
    {
        return Err(ControlError::InvalidState);
    }
    if CONTROL.try_send(Command::Start).is_err() {
        STATE.set_status(current, 0);
        return Err(ControlError::QueueFull);
    }
    Ok(())
}

pub fn request_stop() -> Result<(), ControlError> {
    if !accepts(current_status(), Command::Stop) {
        return Err(ControlError::InvalidState);
    }
    if STATE
        .status
        .compare_exchange(
            Status::Ready as u8,
            Status::Stopping as u8,
            Ordering::AcqRel,
            Ordering::Acquire,
        )
        .is_err()
    {
        return Err(ControlError::InvalidState);
    }
    if CONTROL.try_send(Command::Stop).is_err() {
        STATE.set_status(Status::Ready, 11);
        return Err(ControlError::QueueFull);
    }
    Ok(())
}

pub fn request_sync() -> Result<(), ControlError> {
    if !accepts(current_status(), Command::Sync) {
        return Err(ControlError::InvalidState);
    }
    CONTROL
        .try_send(Command::Sync)
        .map_err(|_| ControlError::QueueFull)
}

pub fn apply_config(config: LoggerConfig) -> Result<(), ControlError> {
    let config = config.validate().map_err(ControlError::InvalidConfig)?;
    STATE
        .sync_period_ms
        .store(config.sync_period_ms, Ordering::Release);
    Ok(())
}

pub fn snapshot() -> Snapshot {
    Snapshot {
        status: current_status(),
        stage: STATE.stage.load(Ordering::Relaxed),
        file_index: STATE.file_index.load(Ordering::Relaxed),
        bytes_written: STATE.bytes_written.load(Ordering::Relaxed),
        flushes: STATE.flushes.load(Ordering::Relaxed),
        errors: STATE.errors.load(Ordering::Relaxed),
        config: LoggerConfig {
            sync_period_ms: STATE.sync_period_ms.load(Ordering::Acquire),
        },
    }
}

pub fn request_logs(link: Link, request: LogRequest) -> Result<(), ControlError> {
    let current = current_status();
    let owner = MANAGEMENT_LINK.load(Ordering::Acquire);
    let starts_session = matches!(request, LogRequest::List { .. } | LogRequest::Erase);
    if current == Status::Managing {
        if owner != link as u8 {
            return Err(ControlError::InvalidState);
        }
    } else if starts_session && matches!(current, Status::Disabled | Status::Error) {
        if STATE
            .status
            .compare_exchange(
                current as u8,
                Status::Managing as u8,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_err()
        {
            return Err(ControlError::InvalidState);
        }
        clear_log_responses(link);
        MANAGEMENT_LINK.store(link as u8, Ordering::Release);
    } else {
        return Err(ControlError::InvalidState);
    }
    if LOG_REQUESTS
        .try_send(RoutedLogRequest { link, request })
        .is_err()
    {
        if starts_session && current != Status::Managing {
            MANAGEMENT_LINK.store(0, Ordering::Release);
            STATE.set_status(current, 0);
        }
        return Err(ControlError::QueueFull);
    }
    Ok(())
}

fn clear_log_responses(link: Link) {
    match link {
        Link::Usb => USB_LOG_RESPONSES.clear(),
        Link::Uart => UART_LOG_RESPONSES.clear(),
    }
}

pub async fn next_log_response(link: Link) -> LogResponse {
    match link {
        Link::Usb => USB_LOG_RESPONSES.receive().await,
        Link::Uart => UART_LOG_RESPONSES.receive().await,
    }
}

async fn send_log_response(link: Link, response: LogResponse) -> Result<(), ()> {
    let send = async {
        match link {
            Link::Usb => USB_LOG_RESPONSES.send(response).await,
            Link::Uart => UART_LOG_RESPONSES.send(response).await,
        }
    };
    with_timeout(RESPONSE_TIMEOUT, send).await.map_err(|_| ())
}

fn current_status() -> Status {
    match STATE.status.load(Ordering::Acquire) {
        value if value == Status::Initializing as u8 => Status::Initializing,
        value if value == Status::Ready as u8 => Status::Ready,
        value if value == Status::Stopping as u8 => Status::Stopping,
        value if value == Status::Error as u8 => Status::Error,
        value if value == Status::Managing as u8 => Status::Managing,
        _ => Status::Disabled,
    }
}

pub fn write_diagnostics(output: &mut impl fmt::Write) -> fmt::Result {
    let stats = LOGGER.stats();
    let snapshot = snapshot();
    writeln!(
        output,
        "LOGGER schema=2 status={} stage={} file_index={} bytes_written={} flushes={} errors={} sync_period_ms={} normal_dropped={} event_dropped={} high_water_bytes={}",
        snapshot.status as u8,
        snapshot.stage,
        snapshot.file_index,
        snapshot.bytes_written,
        snapshot.flushes,
        snapshot.errors,
        snapshot.config.sync_period_ms,
        stats.normal_dropped,
        stats.event_dropped,
        stats.high_water_bytes,
    )
}

pub fn record_motor_command(timestamp_us: u64) {
    if !ready() || !v6x_rust_led::log_config::rate(Group::Can, timestamp_us) {
        return;
    }
    let mut payload = ulog::Payload::new();
    payload
        .push_u32(MOTOR_SEQUENCE.fetch_add(1, Ordering::Relaxed))
        .unwrap();
    payload.push_u8(1).unwrap();
    payload.push_u8(0).unwrap();
    payload.push_u8(0).unwrap();
    for _ in 0..4 {
        payload.push_i16(0).unwrap();
    }
    payload.push_u32(0).unwrap();
    record(
        TopicId::DronecanRawCommand,
        Class::Normal,
        timestamp_us,
        &payload,
    );
}

pub fn record_arming_status(timestamp_us: u64) {
    if !ready() || !v6x_rust_led::log_config::enabled(Group::Event) {
        return;
    }
    let report = ActuatorArmed {
        timestamp: timestamp_us,
        armed: crate::OUTPUT_STATE.armed,
        prearmed: false,
        ready_to_arm: false,
        lockdown: false,
        kill: false,
        termination: false,
        in_esc_calibration_mode: false,
    };
    record(
        TopicId::ActuatorArmed,
        Class::Event,
        report.timestamp,
        &ulog::actuator_armed_payload(report),
    );
}

/// Rate-limit on the reconstructed sample timestamp so the 8-sample FIFO
/// batches cannot alias the limiter to batch boundaries; the record keeps the
/// publish time as `timestamp` and the sample time as `timestamp_sample`.
pub fn record_imu(sample: v6x_rust_led::icm42688::Sample, error_count: u32) {
    if !ready() || !v6x_rust_led::log_config::rate(Group::Imu, sample.sample_timestamp_us) {
        return;
    }
    let timestamp_us = Instant::now().as_micros();
    let accel = SensorAccel::from_sample(timestamp_us, sample, error_count);
    let gyro = SensorGyro::from_sample(timestamp_us, sample, error_count);
    record(
        TopicId::SensorAccel,
        Class::Normal,
        accel.timestamp,
        &ulog::sensor_accel_payload(accel),
    );
    record(
        TopicId::SensorGyro,
        Class::Normal,
        gyro.timestamp,
        &ulog::sensor_gyro_payload(gyro),
    );
}

pub fn record_current_sensor(timestamp_us: u64, measurement: v6x_rust_led::ina219::Measurement) {
    if !ready() || !v6x_rust_led::log_config::rate(Group::Current, timestamp_us) {
        return;
    }
    record(
        TopicId::CurrentSensor,
        Class::Normal,
        timestamp_us,
        &ulog::current_sensor_payload(measurement),
    );
}

pub fn record_sensor_gps(snapshot: SensorGps) {
    if !ready() || !v6x_rust_led::log_config::rate(Group::Can, snapshot.timestamp) {
        return;
    }
    record(
        TopicId::SensorGps,
        Class::Normal,
        snapshot.timestamp,
        &ulog::sensor_gps_payload(snapshot),
    );
}

pub fn record_esc_status(
    timestamp_us: u64,
    source_node_id: u8,
    status: v6x_rust_led::dronecan::dsdl::EscStatus,
    trusted: bool,
    stale: bool,
) {
    if !ready() || !v6x_rust_led::log_config::rate(Group::Can, timestamp_us) {
        return;
    }
    let mut payload = ulog::Payload::new();
    payload
        .push_u32(ESC_SEQUENCE.fetch_add(1, Ordering::Relaxed))
        .unwrap();
    payload.push_u8(source_node_id).unwrap();
    payload.push_u8(status.esc_index).unwrap();
    payload.push_u8(u8::from(trusted)).unwrap();
    payload.push_u8(u8::from(stale)).unwrap();
    payload.push_u32(status.error_count).unwrap();
    payload.push_f32(status.voltage).unwrap();
    payload.push_f32(status.current).unwrap();
    payload.push_f32(status.temperature).unwrap();
    payload.push_i32(status.rpm).unwrap();
    payload.push_u8(status.power_rating_pct).unwrap();
    record(
        TopicId::DronecanEscStatus,
        Class::Normal,
        timestamp_us,
        &payload,
    );
}

pub fn record_node_status(
    timestamp_us: u64,
    source_node_id: u8,
    status: v6x_rust_led::dronecan::dsdl::NodeStatus,
    online: bool,
) {
    if !ready() || !v6x_rust_led::log_config::enabled(Group::Event) {
        return;
    }
    let mut payload = ulog::Payload::new();
    payload.push_u8(source_node_id).unwrap();
    payload.push_u8(status.health).unwrap();
    payload.push_u8(status.mode).unwrap();
    payload.push_u8(u8::from(online)).unwrap();
    payload.push_u32(status.uptime_sec).unwrap();
    payload.push_u16(status.vendor_status).unwrap();
    record(
        TopicId::DronecanNodeStatus,
        Class::Normal,
        timestamp_us,
        &payload,
    );
}

pub fn record_can_stats(timestamp_us: u64, counters: [u32; 10]) {
    if !ready() || !v6x_rust_led::log_config::rate(Group::Health, timestamp_us) {
        return;
    }
    let mut payload = ulog::Payload::new();
    for counter in counters {
        payload.push_u32(counter).unwrap();
    }
    record(TopicId::CanStats, Class::Normal, timestamp_us, &payload);
}

pub fn record_parameter(timestamp_us: u64, index: u16, kind: Kind, value: f32, dirty: bool) {
    let status = STATE.status.load(Ordering::Acquire);
    let definitions_started = STATE.stage.load(Ordering::Acquire) >= 10;
    if status != Status::Ready as u8
        && !(status == Status::Initializing as u8 && definitions_started)
    {
        return;
    }
    if !v6x_rust_led::log_config::enabled(Group::Event) {
        return;
    }
    let payload = parameter_payload(index, kind, value, dirty);
    enqueue(
        TopicId::ParameterChange,
        Class::Event,
        timestamp_us,
        &payload,
    );
}

#[embassy_executor::task]
pub async fn run(
    mut sdmmc: Sdmmc<'static>,
    mut power: Output<'static>,
    parameters: &'static ParameterManager<'static>,
) -> ! {
    loop {
        match select(CONTROL.receive(), LOG_REQUESTS.receive()).await {
            Either::First(Command::Start) => {
                while LOGGER.pop().is_some() {}
                STATE.file_index.store(0, Ordering::Relaxed);
                STATE.bytes_written.store(0, Ordering::Relaxed);
                STATE.flushes.store(0, Ordering::Relaxed);
                STATE.set_status(Status::Initializing, 1);
                // Let the control response transmit before vendor HAL polling.
                Timer::after(START_DEFER).await;
                power.set_high();
                Timer::after_millis(20).await;
                let result = run_session(&mut sdmmc, parameters).await;
                power.set_low();
                match result {
                    Ok(()) => STATE.set_status(Status::Disabled, 0),
                    Err(stage) => {
                        STATE.errors.fetch_add(1, Ordering::Relaxed);
                        STATE.set_status(Status::Error, stage);
                    }
                }
            }
            Either::First(_) => {}
            Either::Second(request) => {
                STATE.set_status(Status::Managing, 20);
                power.set_high();
                Timer::after_millis(20).await;
                let result = run_management_session(&mut sdmmc, request).await;
                power.set_low();
                MANAGEMENT_LINK.store(0, Ordering::Release);
                match result {
                    Ok(()) => STATE.set_status(Status::Disabled, 0),
                    Err((link, error)) => {
                        let _ = send_log_response(link, LogResponse::Error(error)).await;
                        let _ = send_log_response(link, LogResponse::Done).await;
                        STATE.errors.fetch_add(1, Ordering::Relaxed);
                        STATE.set_status(Status::Error, 21);
                    }
                }
            }
        }
    }
}

async fn run_session(
    sdmmc: &mut Sdmmc<'static>,
    parameters: &ParameterManager<'_>,
) -> Result<(), u8> {
    let mut command = CmdBlock::new();
    let storage = with_timeout(
        INIT_TIMEOUT,
        StorageDevice::<Card>::new_sd_card(sdmmc, &mut command, CARD_HZ),
    )
    .await
    .map_err(|_| 2)?
    .map_err(|_| 2)?;
    STATE.set_status(Status::Initializing, 3);
    let stream = BufStream::<_, 512>::new(storage);
    let scheme = with_timeout(INIT_TIMEOUT, Scheme::open(stream))
        .await
        .map_err(|_| 3)?
        .map_err(|_| 3)?;
    match scheme {
        Scheme::Mbr(mut mbr) => {
            let partition_index = mbr
                .iter_used()
                .find(|(_, partition)| partition.is_fat())
                .map(|(index, _)| index);
            if let Some(partition_index) = partition_index {
                let mut partition = with_timeout(INIT_TIMEOUT, mbr.open_partition(partition_index))
                    .await
                    .map_err(|_| 5)?
                    .map_err(|_| 5)?;
                run_filesystem(&mut partition, parameters).await
            } else {
                let mut stream = mbr.into_inner();
                run_filesystem(&mut stream, parameters).await
            }
        }
        Scheme::Superfloppy(mut stream) => run_filesystem(&mut stream, parameters).await,
        Scheme::Unknown(_) => Err(4),
    }
}

async fn run_management_session(
    sdmmc: &mut Sdmmc<'static>,
    first_request: RoutedLogRequest,
) -> Result<(), (Link, LogError)> {
    let link = first_request.link;
    let mut command = CmdBlock::new();
    let storage = with_timeout(
        INIT_TIMEOUT,
        StorageDevice::<Card>::new_sd_card(sdmmc, &mut command, CARD_HZ),
    )
    .await
    .map_err(|_| (link, LogError::Mount))?
    .map_err(|_| (link, LogError::Mount))?;
    let stream = BufStream::<_, 512>::new(storage);
    let scheme = with_timeout(INIT_TIMEOUT, Scheme::open(stream))
        .await
        .map_err(|_| (link, LogError::Mount))?
        .map_err(|_| (link, LogError::Mount))?;
    match scheme {
        Scheme::Mbr(mut mbr) => {
            let partition_index = mbr
                .iter_used()
                .find(|(_, partition)| partition.is_fat())
                .map(|(index, _)| index);
            if let Some(partition_index) = partition_index {
                let mut partition = with_timeout(INIT_TIMEOUT, mbr.open_partition(partition_index))
                    .await
                    .map_err(|_| (link, LogError::Mount))?
                    .map_err(|_| (link, LogError::Mount))?;
                run_management_filesystem(&mut partition, first_request).await
            } else {
                let mut stream = mbr.into_inner();
                run_management_filesystem(&mut stream, first_request).await
            }
        }
        Scheme::Superfloppy(mut stream) => {
            run_management_filesystem(&mut stream, first_request).await
        }
        Scheme::Unknown(_) => Err((link, LogError::Mount)),
    }
}

async fn run_management_filesystem<IO>(
    storage: &mut IO,
    first_request: RoutedLogRequest,
) -> Result<(), (Link, LogError)>
where
    IO: Read + Write + Seek,
{
    let link = first_request.link;
    let filesystem = with_timeout(
        INIT_TIMEOUT,
        FileSystem::new(storage, FsOptions::new().time_provider(WallClock)),
    )
    .await
    .map_err(|_| (link, LogError::Mount))?
    .map_err(|_| (link, LogError::Mount))?;
    let root = filesystem.root_dir();
    let mut next_request = Some(first_request);
    loop {
        let request = if let Some(request) = next_request.take() {
            request
        } else {
            match select(
                LOG_REQUESTS.receive(),
                Timer::after(MANAGEMENT_IDLE_TIMEOUT),
            )
            .await
            {
                Either::First(request) => request,
                Either::Second(()) => return Ok(()),
            }
        };
        if request.link != link {
            let _ = send_log_response(request.link, LogResponse::Error(LogError::Busy)).await;
            let _ = send_log_response(request.link, LogResponse::Done).await;
            continue;
        }
        match request.request {
            LogRequest::List { start, end } => {
                let mut num_logs = 0u16;
                let mut entries = root.iter();
                while let Some(entry) = entries.next().await {
                    let entry = entry.map_err(|_| (link, LogError::Directory))?;
                    if is_ulog_entry(&entry) {
                        num_logs = num_logs.saturating_add(1);
                    }
                }
                if num_logs == 0 {
                    send_log_response(
                        link,
                        LogResponse::Entry {
                            id: 0,
                            num_logs: 0,
                            last_log_num: 0,
                            size: 0,
                            time_utc: 0,
                        },
                    )
                    .await
                    .map_err(|_| (link, LogError::Busy))?;
                } else {
                    let last_log_num = num_logs - 1;
                    let Some((first, last)) = list_bounds(num_logs, start, end) else {
                        send_log_response(link, LogResponse::Done)
                            .await
                            .map_err(|_| (link, LogError::Busy))?;
                        continue;
                    };
                    let mut id = 0u16;
                    let mut entries = root.iter();
                    while let Some(entry) = entries.next().await {
                        let entry = entry.map_err(|_| (link, LogError::Directory))?;
                        if !is_ulog_entry(&entry) {
                            continue;
                        }
                        if id >= first && id <= last {
                            send_log_response(
                                link,
                                LogResponse::Entry {
                                    id,
                                    num_logs,
                                    last_log_num,
                                    size: entry.len().min(u64::from(u32::MAX)) as u32,
                                    time_utc: entry_time_utc(&entry),
                                },
                            )
                            .await
                            .map_err(|_| (link, LogError::Busy))?;
                        }
                        id = id.saturating_add(1);
                    }
                }
                send_log_response(link, LogResponse::Done)
                    .await
                    .map_err(|_| (link, LogError::Busy))?;
            }
            LogRequest::Data { id, offset, count } => {
                let mut current_id = 0u16;
                let mut selected = None;
                let mut entries = root.iter();
                while let Some(entry) = entries.next().await {
                    let entry = entry.map_err(|_| (link, LogError::Directory))?;
                    if !is_ulog_entry(&entry) {
                        continue;
                    }
                    if current_id == id {
                        selected = Some(entry);
                        break;
                    }
                    current_id = current_id.saturating_add(1);
                }
                let entry = selected.ok_or((link, LogError::NotFound))?;
                let size = entry.len().min(u64::from(u32::MAX)) as u32;
                let requested =
                    download_count(size, offset, count).map_err(|error| (link, error))?;
                let mut file = entry.to_file();
                file.seek(SeekFrom::Start(u64::from(offset)))
                    .await
                    .map_err(|_| (link, LogError::Read))?;
                let mut sent = 0u32;
                while sent < requested {
                    let mut data = [0u8; 90];
                    let length = usize::try_from((requested - sent).min(90)).unwrap();
                    let read = file
                        .read(&mut data[..length])
                        .await
                        .map_err(|_| (link, LogError::Read))?;
                    if read == 0 {
                        return Err((link, LogError::Read));
                    }
                    send_log_response(
                        link,
                        LogResponse::Data {
                            id,
                            offset: offset + sent,
                            count: read as u8,
                            data,
                        },
                    )
                    .await
                    .map_err(|_| (link, LogError::Busy))?;
                    sent += read as u32;
                }
                send_log_response(link, LogResponse::Done)
                    .await
                    .map_err(|_| (link, LogError::Busy))?;
            }
            LogRequest::End => {
                send_log_response(link, LogResponse::Done)
                    .await
                    .map_err(|_| (link, LogError::Busy))?;
                return Ok(());
            }
            LogRequest::Erase => {
                send_log_response(link, LogResponse::Error(LogError::EraseDisabled))
                    .await
                    .map_err(|_| (link, LogError::Busy))?;
                send_log_response(link, LogResponse::Done)
                    .await
                    .map_err(|_| (link, LogError::Busy))?;
            }
        }
    }
}

fn is_ulog_entry<IO, TP, OCC>(entry: &embedded_fatfs::DirEntry<'_, IO, TP, OCC>) -> bool
where
    IO: embedded_fatfs::ReadWriteSeek,
    OCC: embedded_fatfs::OemCpConverter,
{
    entry.is_file() && is_ulog_name(entry.short_file_name_as_bytes())
}

/// FAT directory timestamps come from the RTC. A calendar that is not set (or
/// outside the DOS 1980..=2107 range) falls back to 1980-01-01 so recording
/// still works; `LOG_ENTRY.time_utc` then reports 0 for that file.
#[derive(Clone, Copy, Debug)]
struct WallClock;

impl TimeProvider for WallClock {
    fn get_current_date(&self) -> Date {
        fat_date_time().date
    }

    fn get_current_date_time(&self) -> DateTime {
        fat_date_time()
    }
}

fn fat_date_time() -> DateTime {
    crate::rtc::now()
        .filter(|utc| utc.is_fat_range())
        .map(|utc| {
            DateTime::new(
                Date::new(utc.year, u16::from(utc.month), u16::from(utc.day)),
                Time::new(
                    u16::from(utc.hour),
                    u16::from(utc.minute),
                    u16::from(utc.second),
                    0,
                ),
            )
        })
        .unwrap_or_else(|| DateTime::new(Date::new(1980, 1, 1), Time::new(0, 0, 0, 0)))
}

fn entry_time_utc<IO, TP, OCC>(entry: &embedded_fatfs::DirEntry<'_, IO, TP, OCC>) -> u32
where
    IO: embedded_fatfs::ReadWriteSeek,
    OCC: embedded_fatfs::OemCpConverter,
{
    let modified = entry.modified();
    Utc::new(
        modified.date.year,
        u8::try_from(modified.date.month).unwrap_or(0),
        u8::try_from(modified.date.day).unwrap_or(0),
        u8::try_from(modified.time.hour).unwrap_or(0),
        u8::try_from(modified.time.min).unwrap_or(0),
        u8::try_from(modified.time.sec).unwrap_or(0),
    )
    .and_then(Utc::plausible_unix)
    .and_then(|seconds| u32::try_from(seconds).ok())
    .unwrap_or(0)
}

async fn run_filesystem<IO>(storage: &mut IO, parameters: &ParameterManager<'_>) -> Result<(), u8>
where
    IO: Read + Write + Seek,
{
    let filesystem = with_timeout(
        INIT_TIMEOUT,
        FileSystem::new(storage, FsOptions::new().time_provider(WallClock)),
    )
    .await
    .map_err(|_| 6)?
    .map_err(|_| 6)?;
    let root = filesystem.root_dir();
    let mut file_name = [0; 12];
    let mut selected = None;
    for index in 0..=99_999_999 {
        format_file_name(index, &mut file_name);
        let name = core::str::from_utf8(&file_name).expect("ASCII filename");
        match root.open_file(name).await {
            Ok(_) => {}
            Err(FatError::NotFound) => {
                selected = Some(index);
                break;
            }
            Err(_) => return Err(7),
        }
    }
    let file_index = selected.ok_or(8)?;
    let name = core::str::from_utf8(&file_name).expect("ASCII filename");
    let mut file = match root.create_file(name).await {
        Ok(file) => file,
        Err(_) => return Err(9),
    };
    STATE.file_index.store(file_index, Ordering::Relaxed);
    let mut definition_bytes = match write_definitions(&mut file, parameters).await {
        Ok(bytes) => bytes,
        Err(_) => return Err(10),
    };
    if write_alignment(&mut file, &mut definition_bytes)
        .await
        .is_err()
    {
        return Err(10);
    }
    if file.flush().await.is_err() || filesystem.flush().await.is_err() {
        return Err(10);
    }
    STATE
        .bytes_written
        .store(definition_bytes, Ordering::Relaxed);
    STATE.flushes.store(1, Ordering::Relaxed);
    STATE.set_status(Status::Ready, 11);

    let mut staging = Staging([0; STAGING_BYTES]);
    let mut used = 0;
    let mut pending = None;
    let mut next_sync = Instant::now() + sync_period();
    let mut stopping = false;
    loop {
        while let Ok(command) = CONTROL.try_receive() {
            match command {
                Command::Stop => stopping = true,
                Command::Sync => next_sync = Instant::now(),
                Command::Start => {}
            }
        }
        let mut progressed = false;
        loop {
            if pending.is_none() {
                pending = LOGGER.pop();
            }
            let Some(record) = pending else { break };
            let bytes = record.as_bytes();
            if bytes.len() > staging.0.len() - used {
                break;
            }
            staging.0[used..used + bytes.len()].copy_from_slice(bytes);
            used += bytes.len();
            pending = None;
            progressed = true;
        }

        let blocked = pending.is_some() && used != 0;
        if used == staging.0.len() || blocked {
            let write_len = used / 512 * 512;
            if write_len != 0
                && flush_staging(&mut file, &mut staging, &mut used, write_len)
                    .await
                    .is_err()
            {
                return Err(12);
            }
            progressed = true;
        }
        if stopping || Instant::now() >= next_sync {
            if used != 0 {
                let (lengths, count) = alignment_lengths(used);
                let padding_bytes = lengths[..count].iter().sum::<usize>();
                if used + padding_bytes > staging.0.len() {
                    let write_len = used / 512 * 512;
                    if flush_staging(&mut file, &mut staging, &mut used, write_len)
                        .await
                        .is_err()
                    {
                        return Err(12);
                    }
                }
                append_alignment(&mut staging, &mut used);
                let write_len = used;
                if flush_staging(&mut file, &mut staging, &mut used, write_len)
                    .await
                    .is_err()
                {
                    return Err(12);
                }
            }
            if file.flush().await.is_err() || filesystem.flush().await.is_err() {
                return Err(13);
            }
            STATE.flushes.fetch_add(1, Ordering::Relaxed);
            if stopping && pending.is_none() {
                return Ok(());
            }
            next_sync = Instant::now() + sync_period();
        }
        if !progressed {
            Timer::after_millis(1).await;
        }
    }
}

fn sync_period() -> Duration {
    Duration::from_millis(u64::from(STATE.sync_period_ms.load(Ordering::Acquire)))
}

#[embassy_executor::task]
pub async fn health_task() -> ! {
    let mut encoder = DataEncoder::new();
    loop {
        if ready() {
            let timestamp = Instant::now().as_micros();
            if v6x_rust_led::log_config::rate(Group::Health, timestamp) {
                let stats = LOGGER.stats();
                let mut payload = ulog::Payload::new();
                let cpu = crate::diagnostics::cpu_load();
                payload.push_u8(u8::from(cpu.is_some())).unwrap();
                payload.push_u16(cpu.unwrap_or(0)).unwrap();
                payload.push_u8(0).unwrap();
                payload.push_u32(0).unwrap();
                payload.push_u8(0).unwrap();
                payload.push_u32(0).unwrap();
                payload.push_u32(0).unwrap();
                payload.push_u32(stats.normal_dropped).unwrap();
                payload.push_u32(stats.event_dropped).unwrap();
                payload.push_u32(stats.high_water_bytes).unwrap();
                if let Ok(record) =
                    encoder.encode(TopicId::RuntimeHealth, timestamp, payload.as_bytes())
                {
                    let _ = LOGGER.try_record(Class::Normal, timestamp, record);
                }
            }
        }
        Timer::after_millis(100).await;
    }
}

fn record(topic: TopicId, class: Class, timestamp_us: u64, payload: &ulog::Payload) {
    if !ready() {
        return;
    }
    enqueue(topic, class, timestamp_us, payload);
}

fn enqueue(topic: TopicId, class: Class, timestamp_us: u64, payload: &ulog::Payload) {
    let mut encoder = DataEncoder::new();
    if let Ok(record) = encoder.encode(topic, timestamp_us, payload.as_bytes()) {
        let _ = LOGGER.try_record(class, timestamp_us, record);
    }
}

fn parameter_payload(index: u16, kind: Kind, value: f32, dirty: bool) -> ulog::Payload {
    let mut payload = ulog::Payload::new();
    payload.push_u16(index).unwrap();
    payload.push_u8(kind as u8).unwrap();
    payload.push_u8(u8::from(dirty)).unwrap();
    payload.push_u8(u8::from(!dirty)).unwrap();
    payload.push_f32(value).unwrap();
    payload
}

async fn write_definitions<W: Write>(
    writer: &mut W,
    parameters: &ParameterManager<'_>,
) -> Result<u32, W::Error> {
    let mut bytes_written = ulog::FILE_HEADER_LEN as u32;
    let log_start = Instant::now().as_micros();
    writer.write_all(&ulog::file_header(log_start)).await?;
    STATE.stage.store(10, Ordering::Release);
    let flags = ulog::flag_bits();
    writer.write_all(flags.as_bytes()).await?;
    bytes_written += flags.as_bytes().len() as u32;
    for topic in TOPICS {
        let record = ulog::format(topic.id).unwrap();
        writer.write_all(record.as_bytes()).await?;
        bytes_written += record.as_bytes().len() as u32;
    }
    let mut format = [0; v6x_rust_led::estimator::standard_ulog::FORMAT_CAPACITY];
    for topic in &v6x_rust_led::estimator::standard_ulog::TOPICS {
        let length = v6x_rust_led::estimator::standard_ulog::format_record(topic.id, &mut format)
            .expect("fixed estimator format fits metadata storage");
        writer.write_all(&format[..length]).await?;
        bytes_written += length as u32;
    }
    let source = hex_u64(v6x_rust_led::build_info::SOURCE_ID);
    let version = v6x_rust_led::build_info::FLIGHT_SW_VERSION.to_le_bytes();
    // Encode one record at a time. An array of eight 800-byte records creates
    // multiple large temporaries in the encompassing async poll frame.
    for (key, value) in [
        ("char[7] sys_name", b"rust_ap".as_slice()),
        ("char[9] ver_hw", b"FMUv6X_R3".as_slice()),
        ("char[9] sys_mcu", b"STM32H753".as_slice()),
        ("char[16] ver_sw", source.as_slice()),
        ("uint32_t ver_sw_release", version.as_slice()),
        ("char[40] px4_msg_revision", PX4_REVISION.as_bytes()),
        ("char[3] body_frame", b"FRD".as_slice()),
        ("char[3] world_frame", b"NED".as_slice()),
    ] {
        let record = ulog::information(key, value).unwrap();
        writer.write_all(record.as_bytes()).await?;
        bytes_written += record.as_bytes().len() as u32;
    }
    for definition in REGISTRY {
        let record = match definition.kind {
            Kind::Integer => {
                ulog::parameter_i32(definition.name, parameters.get(definition.id) as i32)
            }
            Kind::Real => ulog::parameter_f32(definition.name, parameters.get(definition.id)),
        }
        .expect("validated parameter registry");
        writer.write_all(record.as_bytes()).await?;
        bytes_written += record.as_bytes().len() as u32;
    }
    for topic in TOPICS {
        let record = ulog::subscription(topic.id).unwrap();
        writer.write_all(record.as_bytes()).await?;
        bytes_written += record.as_bytes().len() as u32;
    }
    let record = ulog::raw_subscription(
        ulog::INDEPENDENT_EULER_ID,
        1,
        TopicId::AttitudeEuler.definition().name,
    )
    .expect("fixed independent Euler subscription");
    writer.write_all(record.as_bytes()).await?;
    bytes_written += record.as_bytes().len() as u32;
    for instance in [0, 1] {
        for topic in &v6x_rust_led::estimator::standard_ulog::TOPICS {
            let record = v6x_rust_led::estimator::standard_ulog::subscription(topic.id, instance)
                .expect("fixed estimator subscription");
            writer.write_all(record.as_bytes()).await?;
            bytes_written += record.as_bytes().len() as u32;
        }
    }
    let dirty = parameters.is_dirty();
    let mut encoder = DataEncoder::new();
    for definition in REGISTRY {
        let payload = parameter_payload(
            definition.id as u16,
            definition.kind,
            parameters.get(definition.id),
            dirty,
        );
        let record = encoder
            .encode(TopicId::ParameterChange, log_start, payload.as_bytes())
            .expect("rust_parameter_change schema");
        writer.write_all(record.as_bytes()).await?;
        bytes_written += record.as_bytes().len() as u32;
    }
    Ok(bytes_written)
}

async fn write_alignment<W: Write>(writer: &mut W, offset: &mut u32) -> Result<(), W::Error> {
    let (lengths, count) = alignment_lengths(*offset as usize);
    for &length in &lengths[..count] {
        let record = ulog::padding(length).expect("alignment record fits");
        writer.write_all(record.as_bytes()).await?;
        *offset += record.as_bytes().len() as u32;
    }
    Ok(())
}

fn append_alignment(staging: &mut Staging, used: &mut usize) {
    let (lengths, count) = alignment_lengths(*used);
    for &length in &lengths[..count] {
        let record = ulog::padding(length).expect("alignment record fits");
        let end = *used + record.as_bytes().len();
        staging.0[*used..end].copy_from_slice(record.as_bytes());
        *used = end;
    }
    assert!(used.is_multiple_of(512));
}

fn alignment_lengths(offset: usize) -> ([usize; 2], usize) {
    let remainder = offset % 512;
    if remainder == 0 {
        return ([0; 2], 0);
    }
    let needed = 512 - remainder;
    if needed >= 4 {
        ([needed, 0], 1)
    } else {
        ([5, 512 - ((remainder + 5) % 512)], 2)
    }
}

async fn flush_staging<W: Write>(
    writer: &mut W,
    staging: &mut Staging,
    used: &mut usize,
    write_len: usize,
) -> Result<(), W::Error> {
    if write_len == 0 {
        return Ok(());
    }
    writer.write_all(&staging.0[..write_len]).await?;
    STATE
        .bytes_written
        .fetch_add(write_len as u32, Ordering::Relaxed);
    staging.0.copy_within(write_len..*used, 0);
    *used -= write_len;
    Ok(())
}

fn format_file_name(mut index: u32, output: &mut [u8; 12]) {
    const DIGITS: &[u8; 16] = b"0123456789ABCDEF";
    for byte in output[..8].iter_mut().rev() {
        *byte = DIGITS[(index & 0x0f) as usize];
        index >>= 4;
    }
    output[8..].copy_from_slice(b".ULG");
}

fn hex_u64(mut value: u64) -> [u8; 16] {
    const DIGITS: &[u8; 16] = b"0123456789abcdef";
    let mut output = [0; 16];
    for byte in output.iter_mut().rev() {
        *byte = DIGITS[(value & 0x0f) as usize];
        value >>= 4;
    }
    output
}
