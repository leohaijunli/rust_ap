//! FMUv6X FDCAN1 adapter and the P4b-a zero-output DroneCAN node.

use core::cell::{Cell, RefCell};
use core::fmt;
use core::num::{NonZeroU16, NonZeroU8};
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_stm32::bind_interrupts;
use embassy_stm32::can::config::{
    FrameTransmissionConfig, GlobalFilter, NominalBitTiming, TxBufferMode,
};
use embassy_stm32::can::enums::BusErrorMode;
use embassy_stm32::can::filter::{ExtendedFilter, ExtendedFilterSlot};
use embassy_stm32::can::{
    CanConfigurator, CanRx, CanTx, Frame as HalFrame, IT0InterruptHandler, IT1InterruptHandler,
    Properties,
};
use embassy_stm32::pac::{FDCAN1, RCC};
use embassy_stm32::{peripherals, Peri};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use embedded_can::Id as CanId;
use v6x_rust_led::dronecan::dsdl::{
    self, GetSetRequest, GetSetResponse, NodeInfoResponse, NodeStatus, NumericValue,
    SoftwareVersion, Value,
};
use v6x_rust_led::dronecan::gnss::{self as gnss_dsdl, GnssFix2};
use v6x_rust_led::dronecan::node::{EscMapping, EscMonitor, NodeDirectory, ESC_COUNT};
use v6x_rust_led::dronecan::transport::{
    Frame, Metadata, Payload, RxStats, RxTransport, TransferKind, TxFrames,
};
use v6x_rust_led::gps::{process_fix2, AuxiliaryCache};
use v6x_rust_led::parameters::{Id, Kind};
use v6x_rust_led::px4_messages::SensorGps;

use crate::parameter_manager::ParameterManager;

pub const LOCAL_NODE_ID: u8 = 42;
const NODE_NAME: &[u8] = b"org.rust-ap.fmuv6x";
const ESC_SOURCE_NODE_IDS: [Option<u8>; ESC_COUNT] = [None; ESC_COUNT];

const RAW_COMMAND_PERIOD: Duration = Duration::from_micros(2_500);
const ARMING_STATUS_PERIOD: Duration = Duration::from_millis(100);
const NODE_STATUS_PERIOD: Duration = Duration::from_secs(1);
const PERIODIC_DEADLINE: Duration = Duration::from_millis(5);
const SERVICE_DEADLINE: Duration = Duration::from_secs(1);
const MAX_TX_WAIT_US: u64 = 5_000;

bind_interrupts!(struct CanIrqs {
    FDCAN1_IT0 => IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => IT1InterruptHandler<peripherals::FDCAN1>;
});

#[derive(Clone, Copy, Debug, Default)]
pub struct Counters {
    pub rx_codec_errors: u32,
    pub response_drops: u32,
    pub service_errors: u32,
}

struct IoState {
    rx_frames: AtomicU32,
    rx_bus_errors: AtomicU32,
    rx_queue_drops: AtomicU32,
    tx_frames: AtomicU32,
    tx_transfers: AtomicU32,
    tx_timeouts: AtomicU32,
    rx_errors: AtomicU8,
    tx_errors: AtomicU8,
    mode: AtomicU8,
}

impl IoState {
    const fn new() -> Self {
        Self {
            rx_frames: AtomicU32::new(0),
            rx_bus_errors: AtomicU32::new(0),
            rx_queue_drops: AtomicU32::new(0),
            tx_frames: AtomicU32::new(0),
            tx_transfers: AtomicU32::new(0),
            tx_timeouts: AtomicU32::new(0),
            rx_errors: AtomicU8::new(0),
            tx_errors: AtomicU8::new(0),
            mode: AtomicU8::new(0),
        }
    }
}

static IO_STATE: IoState = IoState::new();

#[derive(Clone, Copy)]
struct PendingInfo {
    node_id: u8,
    transfer_id: u8,
    deadline_us: u64,
}

pub struct State {
    counters: Cell<Counters>,
    transport: Cell<RxStats>,
    esc: RefCell<EscMonitor>,
    nodes: RefCell<NodeDirectory>,
    pending_info: RefCell<[Option<PendingInfo>; 8]>,
    next_info_transfer_id: Cell<u8>,
}

impl State {
    pub fn new() -> Self {
        let mapping = EscMapping::new(ESC_SOURCE_NODE_IDS).expect("static ESC mapping");
        Self {
            counters: Cell::new(Counters::default()),
            transport: Cell::new(RxStats::default()),
            esc: RefCell::new(EscMonitor::new(mapping)),
            nodes: RefCell::new(NodeDirectory::new()),
            pending_info: RefCell::new([None; 8]),
            next_info_transfer_id: Cell::new(0),
        }
    }

    pub fn write_diagnostics(&self, output: &mut impl fmt::Write) -> fmt::Result {
        let counters = self.counters.get();
        let transport = self.transport.get();
        writeln!(
            output,
            "DRONECAN schema=1 node_id={} bitrate=1000000 zero_only=true registers_ok={} rx_frames={} rx_bus_errors={} rx_queue_drops={} rx_codec_errors={} tx_frames={} tx_transfers={} tx_timeouts={} response_drops={} service_errors={}",
            LOCAL_NODE_ID,
            registers_ok(),
            IO_STATE.rx_frames.load(Ordering::Relaxed),
            IO_STATE.rx_bus_errors.load(Ordering::Relaxed),
            IO_STATE.rx_queue_drops.load(Ordering::Relaxed),
            counters.rx_codec_errors,
            IO_STATE.tx_frames.load(Ordering::Relaxed),
            IO_STATE.tx_transfers.load(Ordering::Relaxed),
            IO_STATE.tx_timeouts.load(Ordering::Relaxed),
            counters.response_drops,
            counters.service_errors,
        )?;
        writeln!(
            output,
            "CAN_BUS rx_error_count={} tx_error_count={} mode={} transfers={} duplicates={} tid_gaps={} crc_errors={} timeouts={} rejected={} capacity_errors={}",
            IO_STATE.rx_errors.load(Ordering::Relaxed),
            IO_STATE.tx_errors.load(Ordering::Relaxed),
            IO_STATE.mode.load(Ordering::Relaxed),
            transport.transfers,
            transport.duplicates,
            transport.transfer_id_gaps,
            transport.crc_errors,
            transport.timeouts,
            transport.rejected,
            transport.capacity_errors,
        )?;

        let now_us = Instant::now().as_micros();
        let esc = self.esc.borrow();
        let esc_stats = esc.stats();
        writeln!(
            output,
            "ESC_STATUS trusted={} candidates={} source_mismatch={} index_out_of_range={} invalid_value={}",
            esc_stats.trusted,
            esc_stats.candidates,
            esc_stats.source_mismatch,
            esc_stats.index_out_of_range,
            esc_stats.invalid_value,
        )?;
        for index in 0..ESC_COUNT {
            if let Some(slot) = esc.slot(index) {
                writeln!(
                    output,
                    "ESC index={} expected_source={:?} source={} fresh={} age_us={} rpm={} voltage={} current={} temperature={} errors={} sequence={}",
                    index,
                    esc.mapping().source(index),
                    slot.source,
                    slot.fresh(now_us),
                    now_us.saturating_sub(slot.received_at_us),
                    slot.status.rpm,
                    slot.status.voltage,
                    slot.status.current,
                    slot.status.temperature,
                    slot.status.error_count,
                    slot.sequence,
                )?;
            } else {
                writeln!(
                    output,
                    "ESC index={} expected_source={:?} candidate_source={:?} fresh=false",
                    index,
                    esc.mapping().source(index),
                    esc.candidate(index),
                )?;
            }
        }
        drop(esc);

        let nodes = self.nodes.borrow();
        for node in nodes.entries() {
            let name = node
                .info
                .as_ref()
                .and_then(|info| core::str::from_utf8(info.name()).ok())
                .unwrap_or("?");
            writeln!(
                output,
                "CAN_NODE id={} online={} age_us={} uptime={} health={} mode={} name={}",
                node.node_id,
                node.online(now_us),
                now_us.saturating_sub(node.status_received_at_us),
                node.status.uptime_sec,
                node.status.health,
                node.status.mode,
                name,
            )?;
        }
        if let Some(gps) = GNSS.try_get() {
            writeln!(
                output,
                "GNSS node_id_from_device={} fresh={} age_us={} fix_type={} sats_used={} lat_1e7={} lon_1e7={} alt_msl_mm={} vel_m_s={} eph={} epv={} time_utc={}",
                gps.device_id,
                (now_us.saturating_sub(gps.timestamp)) <= v6x_rust_led::gps::GPS_FRESH_US,
                now_us.saturating_sub(gps.timestamp),
                gps.fix_type,
                gps.satellites_used,
                (gps.latitude_deg * 1e7) as i64,
                (gps.longitude_deg * 1e7) as i64,
                (gps.altitude_msl_m * 1e3) as i64,
                gps.vel_m_s,
                gps.eph,
                gps.epv,
                gps.time_utc_usec,
            )?;
        } else {
            writeln!(output, "GNSS none")?;
        }
        Ok(())
    }

    fn count(&self, update: impl FnOnce(&mut Counters)) {
        let mut counters = self.counters.get();
        update(&mut counters);
        self.counters.set(counters);
    }

    fn queue_node_info(&self, node_id: u8, outgoing: &OutgoingChannel) {
        let now_us = Instant::now().as_micros();
        let mut pending = self.pending_info.borrow_mut();
        for entry in pending.iter_mut() {
            if entry.is_some_and(|entry| entry.deadline_us <= now_us) {
                *entry = None;
            }
        }
        if pending
            .iter()
            .flatten()
            .any(|entry| entry.node_id == node_id)
        {
            return;
        }
        let Some(slot) = pending.iter_mut().find(|entry| entry.is_none()) else {
            self.count(|counters| {
                counters.response_drops = counters.response_drops.saturating_add(1)
            });
            return;
        };
        let transfer_id = self.next_info_transfer_id.get();
        let metadata = Metadata::service(
            16,
            dsdl::GET_NODE_INFO_ID,
            true,
            LOCAL_NODE_ID,
            node_id,
            transfer_id,
        )
        .expect("fixed GetNodeInfo metadata");
        let request = Outgoing::new(
            metadata,
            Payload::new(),
            now_us + SERVICE_DEADLINE.as_micros(),
        );
        if outgoing.try_send(request).is_ok() {
            *slot = Some(PendingInfo {
                node_id,
                transfer_id,
                deadline_us: now_us + SERVICE_DEADLINE.as_micros(),
            });
            self.next_info_transfer_id.set((transfer_id + 1) & 0x1f);
        } else {
            self.count(|counters| {
                counters.response_drops = counters.response_drops.saturating_add(1)
            });
        }
    }

    fn take_pending_info(&self, node_id: u8, transfer_id: u8, now_us: u64) -> bool {
        let mut pending = self.pending_info.borrow_mut();
        let Some(entry) = pending.iter_mut().find(|entry| {
            entry.is_some_and(|entry| {
                entry.node_id == node_id
                    && entry.transfer_id == transfer_id
                    && entry.deadline_us >= now_us
            })
        }) else {
            return false;
        };
        *entry = None;
        true
    }
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}

struct Outgoing {
    metadata: Metadata,
    payload: Payload,
    deadline_us: u64,
}

impl Outgoing {
    fn new(metadata: Metadata, payload: Payload, deadline_us: u64) -> Self {
        Self {
            metadata,
            payload,
            deadline_us,
        }
    }
}

#[derive(Clone, Copy)]
struct ReceivedFrame {
    frame: Frame,
    received_at_us: u64,
}

type ReceivedChannel = Channel<CriticalSectionRawMutex, ReceivedFrame, 64>;
type OutgoingChannel = Channel<CriticalSectionRawMutex, Outgoing, 8>;
type ServiceChannel = Channel<NoopRawMutex, ServiceAction, 4>;

static RECEIVED: ReceivedChannel = ReceivedChannel::new();
static OUTGOING: OutgoingChannel = OutgoingChannel::new();
/// Latest sensor_gps snapshot for MAVLink and diagnostics consumers.
pub static GNSS: Watch<CriticalSectionRawMutex, SensorGps, 0> = Watch::new();
/// Auxiliary DOP cache; only touched by the receive task.
static AUXILIARY: Mutex<CriticalSectionRawMutex, RefCell<AuxiliaryCache>> =
    Mutex::new(RefCell::new(AuxiliaryCache::new()));

#[derive(Clone, Copy)]
struct ResponseContext {
    priority: u8,
    requester: u8,
    service_id: u8,
    transfer_id: u8,
}

impl ResponseContext {
    fn metadata(self) -> Metadata {
        Metadata::service(
            self.priority,
            self.service_id,
            false,
            LOCAL_NODE_ID,
            self.requester,
            self.transfer_id,
        )
        .expect("validated response context")
    }
}

#[derive(Clone, Copy)]
enum ServiceAction {
    Execute {
        context: ResponseContext,
        opcode: u8,
        argument: i64,
    },
    Restart {
        context: ResponseContext,
        magic: u64,
    },
}

#[embassy_executor::task]
pub async fn io_task(
    fdcan: Peri<'static, peripherals::FDCAN1>,
    rx_pin: Peri<'static, peripherals::PD0>,
    tx_pin: Peri<'static, peripherals::PD1>,
) {
    let mut configurator = CanConfigurator::new(fdcan, rx_pin, tx_pin, CanIrqs);
    let timing = NominalBitTiming {
        prescaler: NonZeroU16::new(1).unwrap(),
        seg1: NonZeroU8::new(13).unwrap(),
        seg2: NonZeroU8::new(2).unwrap(),
        sync_jump_width: NonZeroU8::new(2).unwrap(),
    };
    let config = configurator
        .config()
        .set_nominal_bit_timing(timing)
        .set_frame_transmit(FrameTransmissionConfig::ClassicCanOnly)
        .set_tx_buffer_mode(TxBufferMode::Fifo)
        .set_global_filter(GlobalFilter::reject_all())
        // HAL 0.5.0 only offers an unbounded synchronous cancel path.
        .set_automatic_retransmit(false)
        .set_automatic_bus_off_recovery(true);
    configurator.set_config(config);
    configurator.properties().set_extended_filter(
        ExtendedFilterSlot::_0,
        ExtendedFilter::accept_all_into_fifo0(),
    );
    let (tx, rx, properties) = configurator.into_normal_mode().split();
    assert!(registers_ok());

    join(
        io_receive_loop(rx),
        join(io_transmit_loop(tx), io_bus_monitor_loop(properties)),
    )
    .await;
}

#[embassy_executor::task]
pub async fn run(parameters: &'static ParameterManager<'static>, state: &'static State) -> ! {
    let services = ServiceChannel::new();
    join(
        receive_loop(parameters, state, &OUTGOING, &services),
        join(
            service_loop(parameters, state, &OUTGOING, &services),
            freshness_loop(state),
        ),
    )
    .await;
    unreachable!()
}

async fn receive_loop(
    parameters: &ParameterManager<'_>,
    state: &State,
    outgoing: &OutgoingChannel,
    services: &ServiceChannel,
) -> ! {
    let mut transport = RxTransport::new();
    loop {
        let received = RECEIVED.receive().await;
        match transport.accept(
            &received.frame,
            received.received_at_us,
            LOCAL_NODE_ID,
            dsdl::type_info,
        ) {
            Ok(Some(transfer)) => handle_transfer(
                transfer.metadata,
                transfer.payload,
                received.received_at_us,
                parameters,
                state,
                outgoing,
                services,
            ),
            Ok(None) => {}
            Err(_) => state.count(|counters| {
                counters.rx_codec_errors = counters.rx_codec_errors.saturating_add(1)
            }),
        }
        state.transport.set(transport.stats());
    }
}

fn adapt_frame(frame: &HalFrame) -> Option<Frame> {
    if frame.header().rtr() || frame.header().fdcan() {
        return None;
    }
    let CanId::Extended(id) = frame.id() else {
        return None;
    };
    Frame::new(id.as_raw(), frame.data()).ok()
}

fn handle_transfer(
    metadata: Metadata,
    payload: Payload,
    now_us: u64,
    parameters: &ParameterManager<'_>,
    state: &State,
    outgoing: &OutgoingChannel,
    services: &ServiceChannel,
) {
    match metadata.kind {
        TransferKind::Message {
            data_type_id: gnss_dsdl::GNSS_FIX2_ID,
        } => match gnss_dsdl::decode_gnss_fix2(payload.as_slice()) {
            Ok(fix) => publish_gnss(fix, metadata.source, now_us),
            Err(_) => codec_error(state),
        },
        TransferKind::Message {
            data_type_id: gnss_dsdl::GNSS_AUXILIARY_ID,
        } => match gnss_dsdl::decode_gnss_auxiliary(payload.as_slice()) {
            Ok(auxiliary) => {
                AUXILIARY.lock(|cache| cache.borrow_mut().update(auxiliary, now_us));
            }
            Err(_) => codec_error(state),
        },
        TransferKind::Message {
            data_type_id: dsdl::NODE_STATUS_ID,
        } => match dsdl::decode_node_status(payload.as_slice()) {
            Ok(status) => {
                state
                    .nodes
                    .borrow_mut()
                    .update_status(metadata.source, status, now_us);
                let needs_info = state
                    .nodes
                    .borrow()
                    .entry(metadata.source)
                    .is_some_and(|entry| entry.info.is_none());
                if needs_info {
                    state.queue_node_info(metadata.source, outgoing);
                }
                crate::sd_logger::record_node_status(
                    now_us,
                    metadata.source,
                    status,
                    status.mode != 7,
                );
            }
            Err(_) => codec_error(state),
        },
        TransferKind::Message {
            data_type_id: dsdl::ESC_STATUS_ID,
        } => match dsdl::decode_esc_status(payload.as_slice()) {
            Ok(status) => {
                let observation = state
                    .esc
                    .borrow_mut()
                    .observe(metadata.source, status, now_us);
                crate::sd_logger::record_esc_status(
                    now_us,
                    metadata.source,
                    status,
                    matches!(
                        observation,
                        v6x_rust_led::dronecan::node::EscObservation::Trusted
                    ),
                    false,
                );
            }
            Err(_) => codec_error(state),
        },
        TransferKind::Service {
            service_id,
            request: true,
            destination: _,
        } => {
            let context = ResponseContext {
                priority: metadata.priority,
                requester: metadata.source,
                service_id,
                transfer_id: metadata.transfer_id,
            };
            handle_service_request(
                context,
                payload.as_slice(),
                parameters,
                state,
                outgoing,
                services,
            );
        }
        TransferKind::Service {
            service_id: dsdl::GET_NODE_INFO_ID,
            request: false,
            destination: _,
        } => {
            if !state.take_pending_info(metadata.source, metadata.transfer_id, now_us) {
                codec_error(state);
                return;
            }
            match dsdl::decode_node_info_response(payload.as_slice()) {
                Ok(info) => {
                    state.nodes.borrow_mut().update_info(metadata.source, info);
                }
                Err(_) => codec_error(state),
            }
        }
        _ => {}
    }
}

fn handle_service_request(
    context: ResponseContext,
    payload: &[u8],
    parameters: &ParameterManager<'_>,
    state: &State,
    outgoing: &OutgoingChannel,
    services: &ServiceChannel,
) {
    match context.service_id {
        dsdl::GET_NODE_INFO_ID if payload.is_empty() => {
            let mut response = Payload::new();
            let uid = unique_id();
            let info = NodeInfoResponse {
                status: local_node_status(),
                software: software_version(),
                hardware_major: 6,
                hardware_minor: 3,
                unique_id: uid,
                certificate: &[],
                name: NODE_NAME,
            };
            match dsdl::encode_node_info_response(&info, response.capacity_mut()) {
                Ok(len) => {
                    response.set_len(len).expect("bounded node info");
                    queue_response(context, response, state, outgoing);
                }
                Err(_) => codec_error(state),
            }
        }
        dsdl::GET_SET_ID => match dsdl::decode_get_set_request(payload) {
            Ok(request) => handle_get_set(context, request, parameters, state, outgoing),
            Err(_) => codec_error(state),
        },
        dsdl::EXECUTE_OPCODE_ID => match dsdl::decode_execute_opcode_request(payload) {
            Ok(request) => queue_service(
                ServiceAction::Execute {
                    context,
                    opcode: request.opcode,
                    argument: request.argument,
                },
                state,
                services,
            ),
            Err(_) => codec_error(state),
        },
        dsdl::RESTART_NODE_ID => match dsdl::decode_restart_node_request(payload) {
            Ok(magic) => queue_service(ServiceAction::Restart { context, magic }, state, services),
            Err(_) => codec_error(state),
        },
        _ => codec_error(state),
    }
}

fn handle_get_set(
    context: ResponseContext,
    request: GetSetRequest<'_>,
    parameters: &ParameterManager<'_>,
    state: &State,
    outgoing: &OutgoingChannel,
) {
    let id = if request.name.is_empty() {
        Id::from_index(usize::from(request.index))
    } else {
        core::str::from_utf8(request.name)
            .ok()
            .and_then(Id::from_name)
    };
    let Some(id) = id else {
        let response = GetSetResponse {
            value: Value::Empty,
            default_value: Value::Empty,
            max_value: NumericValue::Empty,
            min_value: NumericValue::Empty,
            name: &[],
        };
        encode_get_set_response(context, response, state, outgoing);
        return;
    };

    match request.value {
        Value::Empty => {}
        Value::Integer(value) => {
            let real = value as f32;
            if real as i64 == value {
                let _ = parameters.set(id, real);
            }
        }
        Value::Real(value) => {
            let _ = parameters.set(id, value);
        }
        Value::Boolean(_) | Value::String(_) => {}
    }

    let definition = id.definition();
    let current = parameters.get(id);
    let (value, default_value, min_value, max_value) = match definition.kind {
        Kind::Integer => (
            Value::Integer(current as i64),
            Value::Integer(definition.default as i64),
            NumericValue::Integer(definition.min as i64),
            NumericValue::Integer(definition.max as i64),
        ),
        Kind::Real => (
            Value::Real(current),
            Value::Real(definition.default),
            NumericValue::Real(definition.min),
            NumericValue::Real(definition.max),
        ),
    };
    let response = GetSetResponse {
        value,
        default_value,
        max_value,
        min_value,
        name: definition.name.as_bytes(),
    };
    encode_get_set_response(context, response, state, outgoing);
}

fn encode_get_set_response(
    context: ResponseContext,
    response_data: GetSetResponse<'_>,
    state: &State,
    outgoing: &OutgoingChannel,
) {
    let mut payload = Payload::new();
    match dsdl::encode_get_set_response(&response_data, payload.capacity_mut()) {
        Ok(len) => {
            payload.set_len(len).expect("bounded GetSet response");
            queue_response(context, payload, state, outgoing);
        }
        Err(_) => codec_error(state),
    }
}

fn queue_service(action: ServiceAction, state: &State, services: &ServiceChannel) {
    if services.try_send(action).is_err() {
        state.count(|counters| counters.response_drops = counters.response_drops.saturating_add(1));
    }
}

async fn service_loop(
    parameters: &ParameterManager<'_>,
    state: &State,
    outgoing: &OutgoingChannel,
    services: &ServiceChannel,
) -> ! {
    loop {
        match services.receive().await {
            ServiceAction::Execute {
                context,
                opcode,
                argument,
            } => {
                let ok = opcode == dsdl::OPCODE_SAVE
                    && argument == 0
                    && parameters.save(crate::OUTPUT_STATE).await.is_ok();
                let mut payload = Payload::new();
                match dsdl::encode_execute_opcode_response(0, ok, payload.capacity_mut()) {
                    Ok(len) => {
                        payload
                            .set_len(len)
                            .expect("bounded ExecuteOpcode response");
                        queue_response(context, payload, state, outgoing);
                    }
                    Err(_) => codec_error(state),
                }
            }
            ServiceAction::Restart { context, magic } => {
                let valid = magic == dsdl::RESTART_MAGIC
                    && !crate::OUTPUT_STATE.armed
                    && !crate::OUTPUT_STATE.outputs_enabled;
                let mut payload = Payload::new();
                // HAL 0.5.0 cannot cancel an automatically retransmitted frame with
                // a bounded operation. Never acknowledge a restart that may not run.
                match dsdl::encode_restart_node_response(false, payload.capacity_mut()) {
                    Ok(len) => {
                        payload.set_len(len).expect("bounded RestartNode response");
                        queue_response(context, payload, state, outgoing);
                        if valid {
                            state.count(|counters| {
                                counters.service_errors = counters.service_errors.saturating_add(1)
                            });
                        }
                    }
                    Err(_) => codec_error(state),
                }
            }
        }
    }
}

async fn freshness_loop(state: &State) -> ! {
    let mut ticks = 0_u8;
    let mut stale = [None; ESC_COUNT];
    loop {
        Timer::after_millis(100).await;
        let now_us = Instant::now().as_micros();
        {
            let esc = state.esc.borrow();
            for (index, previous) in stale.iter_mut().enumerate() {
                let Some(slot) = esc.slot(index) else {
                    continue;
                };
                let current = !slot.fresh(now_us);
                if *previous != Some(current) || ticks == 0 {
                    crate::sd_logger::record_esc_status(
                        now_us,
                        slot.source,
                        slot.status,
                        true,
                        current,
                    );
                    *previous = Some(current);
                }
            }
        }
        if ticks == 0 {
            for node in state.nodes.borrow().entries() {
                crate::sd_logger::record_node_status(
                    now_us,
                    node.node_id,
                    node.status,
                    node.online(now_us),
                );
            }
        }
        ticks = (ticks + 1) % 10;
    }
}

fn queue_response(
    context: ResponseContext,
    payload: Payload,
    state: &State,
    outgoing: &OutgoingChannel,
) {
    let response = Outgoing::new(
        context.metadata(),
        payload,
        Instant::now().as_micros() + SERVICE_DEADLINE.as_micros(),
    );
    if outgoing.try_send(response).is_err() {
        state.count(|counters| counters.response_drops = counters.response_drops.saturating_add(1));
    }
}

async fn io_transmit_loop(mut tx: CanTx<'_>) -> ! {
    let mut raw_transfer_id = 0;
    let mut arming_transfer_id = 0;
    let mut status_transfer_id = 0;
    let mut pending_tx = None;
    let mut next_raw = Instant::now();
    let mut next_arming = Instant::now();
    let mut next_status = Instant::now();

    loop {
        let now = Instant::now();
        let (request, periodic) = if now >= next_arming {
            next_arming = now + ARMING_STATUS_PERIOD;
            (periodic_arming_status(arming_transfer_id, now), true)
        } else if now >= next_status {
            next_status = now + NODE_STATUS_PERIOD;
            (periodic_node_status(status_transfer_id, now), true)
        } else if now >= next_raw {
            next_raw = now + RAW_COMMAND_PERIOD;
            (periodic_raw_command(raw_transfer_id, now), true)
        } else {
            let wake = next_raw.min(next_arming).min(next_status);
            match select(Timer::at(wake), OUTGOING.receive()).await {
                Either::First(()) => continue,
                Either::Second(request) => (request, false),
            }
        };

        let kind = request.metadata.kind;
        let transfer_id = request.metadata.transfer_id;
        let stop_at_us = if periodic {
            request.deadline_us
        } else {
            request
                .deadline_us
                .min(next_raw.min(next_arming).min(next_status).as_micros())
        };
        if send_transfer(&mut tx, request, stop_at_us, &mut pending_tx).await {
            match kind {
                TransferKind::Message {
                    data_type_id: dsdl::ESC_RAW_COMMAND_ID,
                } => raw_transfer_id = (transfer_id + 1) & 0x1f,
                TransferKind::Message {
                    data_type_id: dsdl::ARMING_STATUS_ID,
                } => arming_transfer_id = (transfer_id + 1) & 0x1f,
                TransferKind::Message {
                    data_type_id: dsdl::NODE_STATUS_ID,
                } => status_transfer_id = (transfer_id + 1) & 0x1f,
                _ => {}
            }
        }
    }
}

fn periodic_raw_command(transfer_id: u8, now: Instant) -> Outgoing {
    crate::sd_logger::record_motor_command(now.as_micros());
    let mut payload = Payload::new();
    let len =
        dsdl::encode_zero_raw_command(payload.capacity_mut()).expect("four zero commands fit");
    payload.set_len(len).expect("four zero commands fit");
    let metadata = Metadata::message(8, dsdl::ESC_RAW_COMMAND_ID, LOCAL_NODE_ID, transfer_id)
        .expect("fixed RawCommand metadata");
    Outgoing::new(
        metadata,
        payload,
        now.as_micros() + PERIODIC_DEADLINE.as_micros(),
    )
}

fn periodic_arming_status(transfer_id: u8, now: Instant) -> Outgoing {
    crate::sd_logger::record_arming_status(now.as_micros());
    let mut payload = Payload::new();
    let len = dsdl::encode_disarmed(payload.capacity_mut()).expect("DISARMED fits");
    payload.set_len(len).expect("DISARMED fits");
    let metadata = Metadata::message(8, dsdl::ARMING_STATUS_ID, LOCAL_NODE_ID, transfer_id)
        .expect("fixed ArmingStatus metadata");
    Outgoing::new(
        metadata,
        payload,
        now.as_micros() + PERIODIC_DEADLINE.as_micros(),
    )
}

fn periodic_node_status(transfer_id: u8, now: Instant) -> Outgoing {
    let mut payload = Payload::new();
    let len = dsdl::encode_node_status(local_node_status(), payload.capacity_mut())
        .expect("NodeStatus fits");
    payload.set_len(len).expect("NodeStatus fits");
    let metadata = Metadata::message(16, dsdl::NODE_STATUS_ID, LOCAL_NODE_ID, transfer_id)
        .expect("fixed NodeStatus metadata");
    Outgoing::new(
        metadata,
        payload,
        now.as_micros() + PERIODIC_DEADLINE.as_micros(),
    )
}

async fn io_receive_loop(mut rx: CanRx<'_>) -> ! {
    loop {
        match rx.read().await {
            Ok(envelope) => {
                IO_STATE.rx_frames.fetch_add(1, Ordering::Relaxed);
                if let Some(frame) = adapt_frame(&envelope.frame) {
                    if RECEIVED
                        .try_send(ReceivedFrame {
                            frame,
                            received_at_us: Instant::now().as_micros(),
                        })
                        .is_err()
                    {
                        IO_STATE.rx_queue_drops.fetch_add(1, Ordering::Relaxed);
                    }
                } else {
                    IO_STATE.rx_queue_drops.fetch_add(1, Ordering::Relaxed);
                }
            }
            Err(_) => {
                IO_STATE.rx_bus_errors.fetch_add(1, Ordering::Relaxed);
                Timer::after_millis(1).await;
            }
        }
    }
}

async fn send_transfer(
    tx: &mut CanTx<'_>,
    request: Outgoing,
    stop_at_us: u64,
    pending_tx: &mut Option<usize>,
) -> bool {
    let Some(type_info) = dsdl::type_info(&request.metadata) else {
        IO_STATE.tx_timeouts.fetch_add(1, Ordering::Relaxed);
        return false;
    };
    let Ok(frames) = TxFrames::new(request.metadata, type_info, request.payload.as_slice()) else {
        IO_STATE.tx_timeouts.fetch_add(1, Ordering::Relaxed);
        return false;
    };
    let expected = frames.frame_count();
    let mut queued = 0;
    let mut transmitted = 0;
    for frame in frames {
        if !finish_pending(pending_tx, stop_at_us).await {
            break;
        }
        let now_us = Instant::now().as_micros();
        if now_us >= stop_at_us || FDCAN1.txfqs().read().tfqf() {
            break;
        }
        let Ok(frame) = HalFrame::new_extended(frame.id(), frame.data()) else {
            break;
        };
        let buffer_index = usize::from(FDCAN1.txfqs().read().tfqpi());
        let remaining = Duration::from_micros((stop_at_us - now_us).min(MAX_TX_WAIT_US));
        match with_timeout(remaining, tx.write(&frame)).await {
            Ok(None) => {
                queued += 1;
                *pending_tx = Some(buffer_index);
                if finish_pending(pending_tx, stop_at_us).await {
                    transmitted += 1;
                } else {
                    break;
                }
            }
            Ok(Some(_)) => break,
            Err(_) => break,
        }
    }
    if transmitted == expected {
        IO_STATE.tx_transfers.fetch_add(1, Ordering::Relaxed);
    } else {
        IO_STATE.tx_timeouts.fetch_add(1, Ordering::Relaxed);
    }
    queued != 0
}

async fn finish_pending(pending_tx: &mut Option<usize>, stop_at_us: u64) -> bool {
    let Some(index) = *pending_tx else {
        return true;
    };
    loop {
        if !FDCAN1.txbrp().read().trp(index) {
            let transmitted = FDCAN1.txbto().read().to(index);
            *pending_tx = None;
            if transmitted {
                IO_STATE.tx_frames.fetch_add(1, Ordering::Relaxed);
            }
            return transmitted;
        }
        let now_us = Instant::now().as_micros();
        if now_us >= stop_at_us {
            return false;
        }
        Timer::after_micros((stop_at_us - now_us).min(50)).await;
    }
}

async fn io_bus_monitor_loop(properties: Properties) -> ! {
    loop {
        let mode = match properties.bus_error_mode() {
            BusErrorMode::ErrorActive => 0,
            BusErrorMode::ErrorPassive => 1,
            BusErrorMode::BusOff => 2,
        };
        IO_STATE
            .rx_errors
            .store(properties.rx_error_count(), Ordering::Relaxed);
        IO_STATE
            .tx_errors
            .store(properties.tx_error_count(), Ordering::Relaxed);
        IO_STATE.mode.store(mode, Ordering::Relaxed);
        crate::sd_logger::record_can_stats(
            Instant::now().as_micros(),
            [
                IO_STATE.rx_frames.load(Ordering::Relaxed),
                IO_STATE.rx_bus_errors.load(Ordering::Relaxed),
                IO_STATE.rx_queue_drops.load(Ordering::Relaxed),
                IO_STATE.tx_frames.load(Ordering::Relaxed),
                IO_STATE.tx_transfers.load(Ordering::Relaxed),
                IO_STATE.tx_timeouts.load(Ordering::Relaxed),
                u32::from(properties.rx_error_count()),
                u32::from(properties.tx_error_count()),
                u32::from(mode),
                0,
            ],
        );
        Timer::after_millis(100).await;
    }
}

fn codec_error(state: &State) {
    state.count(|counters| counters.service_errors = counters.service_errors.saturating_add(1));
}

/// PX4 flow: build the sensor_gps snapshot, publish it, and record it.
fn publish_gnss(fix: GnssFix2, source_node_id: u8, now_us: u64) {
    let auxiliary = AUXILIARY.lock(|cache| *cache.borrow());
    let snapshot = process_fix2(&fix, source_node_id, now_us, &auxiliary);
    crate::sd_logger::record_sensor_gps(snapshot);
    GNSS.sender().send(snapshot);
    if let Some(observation) =
        v6x_rust_led::estimator::gnss::GnssObservation::from_sensor_gps(&snapshot)
    {
        crate::estimator::submit_measurement(
            v6x_rust_led::estimator::Measurement::Gnss(observation),
            snapshot.device_id,
        );
    }
}

/// Latest sensor_gps snapshot for MAVLink; freshness is the caller's rule.
pub fn latest_gps() -> Option<SensorGps> {
    GNSS.try_get()
}

fn local_node_status() -> NodeStatus {
    NodeStatus::operational(Instant::now().as_secs().min(u64::from(u32::MAX)) as u32)
}

fn software_version() -> SoftwareVersion {
    SoftwareVersion::basic(
        env!("CARGO_PKG_VERSION_MAJOR").parse().unwrap_or(0),
        env!("CARGO_PKG_VERSION_MINOR").parse().unwrap_or(0),
    )
}

fn unique_id() -> [u8; 16] {
    let device = embassy_stm32::uid::uid();
    let mut unique = [0; 16];
    unique[..device.len()].copy_from_slice(&device);
    unique[12..].copy_from_slice(&[53, 0, 6, 3]);
    unique
}

pub fn registers_ok() -> bool {
    let nbtp = FDCAN1.nbtp().read();
    let control = FDCAN1.cccr().read();
    RCC.d2ccip1r().read().fdcansel() == embassy_stm32::rcc::mux::Fdcansel::HSE
        && nbtp.nbrp() == 0
        && nbtp.ntseg1() == 12
        && nbtp.ntseg2() == 1
        && nbtp.nsjw() == 1
        && !control.fdoe()
        && !control.bse()
        && control.dar()
        && FDCAN1.txbc().read().tfqm() == embassy_stm32::pac::can::vals::Tfqm::FIFO
}
