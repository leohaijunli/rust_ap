//! Single-owner SDMMC2/FAT task consuming the bounded ULog queues.

use core::fmt;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use block_device_adapters::BufStream;
use embassy_stm32::gpio::Output;
use embassy_stm32::sdmmc::sd::{Card, CmdBlock, StorageDevice};
use embassy_stm32::sdmmc::Sdmmc;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use embedded_fatfs::{Error as FatError, FileSystem, FsOptions};
use embedded_io_async::{Read, Seek, Write};
use embedded_partitions::mbr::Scheme;
use v6x_rust_led::log_queue::{Class, Logger};
use v6x_rust_led::parameters::{Kind, REGISTRY};
use v6x_rust_led::ulog::{self, DataEncoder, TopicId, TOPICS};

use crate::parameter_manager::ParameterManager;

const STAGING_BYTES: usize = 32 * 1024;
const SYNC_PERIOD: Duration = Duration::from_secs(1);
const CARD_HZ: Hertz = Hertz(25_000_000);

pub static LOGGER: Logger = Logger::new();

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
enum Status {
    Disabled,
    Initializing,
    Ready,
    Error,
}

struct RuntimeState {
    status: AtomicU8,
    stage: AtomicU8,
    file_index: AtomicU32,
    bytes_written: AtomicU32,
    flushes: AtomicU32,
    errors: AtomicU32,
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
        }
    }

    fn set_status(&self, status: Status, stage: u8) {
        self.stage.store(stage, Ordering::Relaxed);
        self.status.store(status as u8, Ordering::Release);
    }
}

static STATE: RuntimeState = RuntimeState::new();
static START: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static MOTOR_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static ARMING_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static ESC_SEQUENCE: AtomicU32 = AtomicU32::new(0);

#[repr(align(32))]
struct Staging([u8; STAGING_BYTES]);

pub fn initialize() -> bool {
    LOGGER.initialize()
}

pub fn ready() -> bool {
    STATE.status.load(Ordering::Acquire) == Status::Ready as u8
}

pub fn request_start() -> bool {
    if STATE
        .status
        .compare_exchange(
            Status::Disabled as u8,
            Status::Initializing as u8,
            Ordering::AcqRel,
            Ordering::Acquire,
        )
        .is_err()
    {
        return false;
    }
    START.signal(());
    true
}

pub fn write_diagnostics(output: &mut impl fmt::Write) -> fmt::Result {
    let stats = LOGGER.stats();
    writeln!(
        output,
        "LOGGER schema=1 status={} stage={} file_index={} bytes_written={} flushes={} errors={} normal_dropped={} event_dropped={} high_water_bytes={}",
        STATE.status.load(Ordering::Acquire),
        STATE.stage.load(Ordering::Relaxed),
        STATE.file_index.load(Ordering::Relaxed),
        STATE.bytes_written.load(Ordering::Relaxed),
        STATE.flushes.load(Ordering::Relaxed),
        STATE.errors.load(Ordering::Relaxed),
        stats.normal_dropped,
        stats.event_dropped,
        stats.high_water_bytes,
    )
}

pub fn record_motor_command(timestamp_us: u64) {
    if !ready() {
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
    record(TopicId::MotorCommand, Class::Normal, timestamp_us, &payload);
}

pub fn record_arming_status(timestamp_us: u64) {
    if !ready() {
        return;
    }
    let mut payload = ulog::Payload::new();
    payload
        .push_u32(ARMING_SEQUENCE.fetch_add(1, Ordering::Relaxed))
        .unwrap();
    payload.push_u8(0).unwrap();
    record(TopicId::ArmingStatus, Class::Event, timestamp_us, &payload);
}

pub fn record_esc_status(
    timestamp_us: u64,
    source_node_id: u8,
    status: v6x_rust_led::dronecan::dsdl::EscStatus,
    trusted: bool,
    stale: bool,
) {
    if !ready() {
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
    record(TopicId::EscStatus, Class::Normal, timestamp_us, &payload);
}

pub fn record_node_status(
    timestamp_us: u64,
    source_node_id: u8,
    status: v6x_rust_led::dronecan::dsdl::NodeStatus,
    online: bool,
) {
    if !ready() {
        return;
    }
    let mut payload = ulog::Payload::new();
    payload.push_u8(source_node_id).unwrap();
    payload.push_u8(status.health).unwrap();
    payload.push_u8(status.mode).unwrap();
    payload.push_u8(u8::from(online)).unwrap();
    payload.push_u32(status.uptime_sec).unwrap();
    payload.push_u16(status.vendor_status).unwrap();
    record(TopicId::NodeStatus, Class::Normal, timestamp_us, &payload);
}

pub fn record_can_stats(timestamp_us: u64, counters: [u32; 10]) {
    if !ready() {
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
    let payload = parameter_payload(index, kind, value, dirty);
    enqueue(
        TopicId::ParameterChange,
        Class::Event,
        timestamp_us,
        &payload,
    );
}

pub async fn run(
    mut sdmmc: Sdmmc<'static>,
    mut power: Output<'static>,
    parameters: &ParameterManager<'_>,
) -> ! {
    START.wait().await;
    STATE.set_status(Status::Initializing, 1);
    power.set_high();
    Timer::after_millis(20).await;
    let mut command = CmdBlock::new();
    let storage = match StorageDevice::<Card>::new_sd_card(&mut sdmmc, &mut command, CARD_HZ).await
    {
        Ok(storage) => storage,
        Err(_) => failed(2).await,
    };
    STATE.set_status(Status::Initializing, 3);
    let stream = BufStream::<_, 512>::new(storage);
    let scheme = match Scheme::open(stream).await {
        Ok(scheme) => scheme,
        Err(_) => failed(3).await,
    };
    match scheme {
        Scheme::Mbr(mut mbr) => {
            let partition_index = mbr
                .iter_used()
                .find(|(_, partition)| partition.is_fat())
                .map(|(index, _)| index);
            if let Some(partition_index) = partition_index {
                let mut partition = match mbr.open_partition(partition_index).await {
                    Ok(partition) => partition,
                    Err(_) => failed(5).await,
                };
                run_filesystem(&mut partition, parameters).await;
            } else {
                let mut stream = mbr.into_inner();
                run_filesystem(&mut stream, parameters).await;
            }
        }
        Scheme::Superfloppy(mut stream) => run_filesystem(&mut stream, parameters).await,
        Scheme::Unknown(_) => failed(4).await,
    }
}

async fn run_filesystem<IO>(storage: &mut IO, parameters: &ParameterManager<'_>) -> !
where
    IO: Read + Write + Seek,
{
    let filesystem = match FileSystem::new(storage, FsOptions::new()).await {
        Ok(filesystem) => filesystem,
        Err(_) => failed(6).await,
    };
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
            Err(_) => failed(7).await,
        }
    }
    let Some(file_index) = selected else {
        failed(8).await;
    };
    let name = core::str::from_utf8(&file_name).expect("ASCII filename");
    let mut file = match root.create_file(name).await {
        Ok(file) => file,
        Err(_) => failed(9).await,
    };
    STATE.file_index.store(file_index, Ordering::Relaxed);
    let mut definition_bytes = match write_definitions(&mut file, parameters).await {
        Ok(bytes) => bytes,
        Err(_) => failed(10).await,
    };
    if write_alignment(&mut file, &mut definition_bytes)
        .await
        .is_err()
    {
        failed(10).await;
    }
    if file.flush().await.is_err() || filesystem.flush().await.is_err() {
        failed(10).await;
    }
    STATE
        .bytes_written
        .store(definition_bytes, Ordering::Relaxed);
    STATE.flushes.store(1, Ordering::Relaxed);
    STATE.set_status(Status::Ready, 11);

    let mut staging = Staging([0; STAGING_BYTES]);
    let mut used = 0;
    let mut pending = None;
    let mut next_sync = Instant::now() + SYNC_PERIOD;
    loop {
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
                failed(12).await;
            }
            progressed = true;
        }
        if Instant::now() >= next_sync {
            if used != 0 {
                let (lengths, count) = alignment_lengths(used);
                let padding_bytes = lengths[..count].iter().sum::<usize>();
                if used + padding_bytes > staging.0.len() {
                    let write_len = used / 512 * 512;
                    if flush_staging(&mut file, &mut staging, &mut used, write_len)
                        .await
                        .is_err()
                    {
                        failed(12).await;
                    }
                }
                append_alignment(&mut staging, &mut used);
                let write_len = used;
                if flush_staging(&mut file, &mut staging, &mut used, write_len)
                    .await
                    .is_err()
                {
                    failed(12).await;
                }
            }
            if file.flush().await.is_err() || filesystem.flush().await.is_err() {
                failed(13).await;
            }
            STATE.flushes.fetch_add(1, Ordering::Relaxed);
            next_sync = Instant::now() + SYNC_PERIOD;
        }
        if !progressed {
            Timer::after_millis(1).await;
        }
    }
}

pub async fn health_task() -> ! {
    let mut encoder = DataEncoder::new();
    loop {
        if ready() {
            let timestamp = Instant::now().as_micros();
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
    let source = hex_u64(v6x_rust_led::build_info::SOURCE_ID);
    for record in [
        ulog::information("char[7] sys_name", b"rust_ap").unwrap(),
        ulog::information("char[9] ver_hw", b"FMUv6X_R3").unwrap(),
        ulog::information("char[9] sys_mcu", b"STM32H753").unwrap(),
        ulog::information("char[16] ver_sw", &source).unwrap(),
        ulog::information(
            "uint32_t ver_sw_release",
            &v6x_rust_led::build_info::FLIGHT_SW_VERSION.to_le_bytes(),
        )
        .unwrap(),
        ulog::information("char[3] body_frame", b"FLU").unwrap(),
        ulog::information("char[3] world_frame", b"ENU").unwrap(),
    ] {
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
            .expect("parameter_change schema");
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

async fn failed(stage: u8) -> ! {
    STATE.errors.fetch_add(1, Ordering::Relaxed);
    STATE.set_status(Status::Error, stage);
    loop {
        Timer::after_secs(1).await;
    }
}
