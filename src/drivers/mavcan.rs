//! USB MAVCAN passthrough on the standalone FMUv6X CAN2 connector.

use core::cell::RefCell;
use core::fmt;
use core::num::{NonZeroU16, NonZeroU8};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_futures::join::join3;
use embassy_stm32::bind_interrupts;
use embassy_stm32::can::config::{
    FrameTransmissionConfig, GlobalFilter, NominalBitTiming, TxBufferMode,
};
use embassy_stm32::can::{
    CanConfigurator, CanRx, CanTx, Frame as HalFrame, IT0InterruptHandler, IT1InterruptHandler,
};
use embassy_stm32::pac::{FDCAN2, FDCANRAM, RCC};
use embassy_stm32::{peripherals, Peri};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use embedded_can::Id as CanId;
use mavlink::common::*;
use mavlink::MavHeader;
use v6x_rust_led::mavcan::{
    FilterOperation, Frame, Peer, Session, SessionError, BITRATE, BUS_NUMBER, EXTENDED_FLAG,
    FRAME_BUS, RTR_FLAG,
};

use crate::mavlink_core;

const TX_TIMEOUT: Duration = Duration::from_millis(5);
const RX_QUEUE_LEN: usize = 64;
const TX_QUEUE_LEN: usize = 32;
// Embassy 0.5.0 reserves two header words and 64 data bytes per H753 TX element.
const TX_ELEMENT_WORDS: usize = 18;

bind_interrupts!(struct MavcanIrqs {
    FDCAN2_IT0 => IT0InterruptHandler<peripherals::FDCAN2>;
    FDCAN2_IT1 => IT1InterruptHandler<peripherals::FDCAN2>;
});

/// Compact queue entries avoid reserving a full MAVLink union for every frame.
#[derive(Clone, Copy)]
pub struct QueuedFrame {
    frame: Frame,
    epoch: u32,
    timestamp_us: u64,
}

pub static OUTBOUND: Channel<CriticalSectionRawMutex, QueuedFrame, RX_QUEUE_LEN> = Channel::new();
static INBOUND: Channel<CriticalSectionRawMutex, QueuedFrame, TX_QUEUE_LEN> = Channel::new();
static SESSION: Mutex<CriticalSectionRawMutex, RefCell<Session>> =
    Mutex::new(RefCell::new(Session::new()));
static READY: AtomicBool = AtomicBool::new(false);
static RX_FRAMES: AtomicU32 = AtomicU32::new(0);
static USB_FRAMES: AtomicU32 = AtomicU32::new(0);
static RX_DROPPED: AtomicU32 = AtomicU32::new(0);
static RX_FILTERED: AtomicU32 = AtomicU32::new(0);
static RX_ERRORS: AtomicU32 = AtomicU32::new(0);
static TX_RECEIVED: AtomicU32 = AtomicU32::new(0);
static TX_FRAMES: AtomicU32 = AtomicU32::new(0);
static TX_DROPPED: AtomicU32 = AtomicU32::new(0);
static TX_ERRORS: AtomicU32 = AtomicU32::new(0);
static TX_TIMEOUTS: AtomicU32 = AtomicU32::new(0);
static REJECTED: AtomicU32 = AtomicU32::new(0);
static FD_REJECTED: AtomicU32 = AtomicU32::new(0);
static FILTER_ERRORS: AtomicU32 = AtomicU32::new(0);

pub enum Outcome {
    Unhandled,
    Handled,
    ForwardAck(MavResult),
}

fn permitted() -> bool {
    !crate::OUTPUT_STATE.armed && !crate::OUTPUT_STATE.outputs_enabled
}

fn targeted(system: u8, component: u8) -> bool {
    matches!(system, 0 | mavlink_core::SYSTEM_ID)
        && matches!(component, 0 | mavlink_core::COMPONENT_ID)
}

fn cancel_pending_tx() {
    if READY.load(Ordering::Acquire) {
        let pending = FDCAN2.txbrp().read().0;
        // Only this module owns FDCAN2 TX. Request cancellation without HAL's
        // synchronous/unbounded abort loop; the TX task checks completion.
        FDCAN2.txbcr().write(|w| w.0 = pending);
    }
}

fn clear_queues() {
    INBOUND.clear();
    OUTBOUND.clear();
    cancel_pending_tx();
}

/// Called on USB reset/deconfiguration and endpoint loss as well as explicit close.
pub fn stop() {
    SESSION.lock(|session| session.borrow_mut().stop());
    clear_queues();
}

/// Consume bridge messages on USB only; ordinary MAVLink continues on its own path.
pub fn handle_message(header: MavHeader, message: &MavMessage) -> Outcome {
    let peer = Peer {
        system: header.system_id,
        component: header.component_id,
    };
    let now_us = Instant::now().as_micros();
    match message {
        MavMessage::COMMAND_LONG(data) if data.command == MavCmd::MAV_CMD_CAN_FORWARD => {
            if !targeted(data.target_system, data.target_component) {
                return Outcome::Handled;
            }
            if !READY.load(Ordering::Acquire) {
                return Outcome::ForwardAck(MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
            }
            let (result, changed) = SESSION.lock(|session| {
                let mut session = session.borrow_mut();
                let before = session.epoch();
                let result = session.request(peer, data.param1, now_us, permitted());
                (result, session.epoch() != before)
            });
            if changed {
                clear_queues();
            }
            Outcome::ForwardAck(match result {
                Ok(()) => MavResult::MAV_RESULT_ACCEPTED,
                Err(SessionError::Bus) => MavResult::MAV_RESULT_UNSUPPORTED,
                Err(SessionError::Busy) => MavResult::MAV_RESULT_TEMPORARILY_REJECTED,
                Err(_) => MavResult::MAV_RESULT_DENIED,
            })
        }
        MavMessage::CAN_FRAME(data) => {
            if !targeted(data.target_system, data.target_component) {
                return Outcome::Handled;
            }
            if !permitted() {
                stop();
                REJECTED.fetch_add(1, Ordering::Relaxed);
                return Outcome::Handled;
            }
            let frame = Frame::from_wire(data.id, data.len, data.data);
            let epoch = SESSION.lock(|session| {
                let session = session.borrow();
                session
                    .accepts_sender(peer, data.bus, now_us)
                    .then(|| session.epoch())
            });
            if let (Ok(frame), Some(epoch)) = (frame, epoch) {
                TX_RECEIVED.fetch_add(1, Ordering::Relaxed);
                if INBOUND
                    .try_send(QueuedFrame {
                        frame,
                        epoch,
                        timestamp_us: now_us,
                    })
                    .is_err()
                {
                    TX_DROPPED.fetch_add(1, Ordering::Relaxed);
                }
            } else {
                REJECTED.fetch_add(1, Ordering::Relaxed);
            }
            Outcome::Handled
        }
        MavMessage::CANFD_FRAME(data) => {
            // CANFD has different wire/TAO semantics. Never truncate it to Classic CAN.
            if targeted(data.target_system, data.target_component) {
                FD_REJECTED.fetch_add(1, Ordering::Relaxed);
            }
            Outcome::Handled
        }
        MavMessage::CAN_FILTER_MODIFY(data) => {
            if !targeted(data.target_system, data.target_component) {
                return Outcome::Handled;
            }
            let Some(ids) = data.ids.get(..usize::from(data.num_ids)) else {
                FILTER_ERRORS.fetch_add(1, Ordering::Relaxed);
                return Outcome::Handled;
            };
            let operation = match data.operation {
                CanFilterOp::CAN_FILTER_REPLACE => FilterOperation::Replace,
                CanFilterOp::CAN_FILTER_ADD => FilterOperation::Add,
                CanFilterOp::CAN_FILTER_REMOVE => FilterOperation::Remove,
            };
            let result = SESSION.lock(|session| {
                session
                    .borrow_mut()
                    .update_filters(peer, data.bus, now_us, operation, ids)
            });
            if result.is_err() {
                FILTER_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
            Outcome::Handled
        }
        _ => Outcome::Unhandled,
    }
}

fn queued_is_current(queued: QueuedFrame) -> bool {
    permitted()
        && SESSION.lock(|session| {
            session.borrow().accepts_queued(
                queued.epoch,
                queued.timestamp_us,
                Instant::now().as_micros(),
            )
        })
}

/// Validate again at USB dequeue so no frame crosses sessions after a reconnect.
pub fn outbound_message(queued: QueuedFrame) -> Option<MavMessage> {
    let peer = SESSION.lock(|session| {
        let session = session.borrow();
        let now_us = Instant::now().as_micros();
        if !permitted()
            || !session.accepts_queued(queued.epoch, queued.timestamp_us, now_us)
            || !session.accepts_filter(queued.frame)
        {
            None
        } else {
            session.active_peer(now_us)
        }
    });
    let Some(peer) = peer else {
        RX_DROPPED.fetch_add(1, Ordering::Relaxed);
        return None;
    };
    Some(MavMessage::CAN_FRAME(CAN_FRAME_DATA {
        id: queued.frame.wire_id(),
        target_system: peer.system,
        target_component: peer.component,
        bus: FRAME_BUS,
        len: queued.frame.len(),
        data: queued.frame.data(),
    }))
}

pub fn record_usb_send() {
    USB_FRAMES.fetch_add(1, Ordering::Relaxed);
}

/// Pure-software virtual frame: enqueue a synthetic DroneCAN NodeStatus frame
/// through the same gates as a frame read off the CAN2 bus. This exercises the
/// bridge's permitted/session/filter checks and its USB forwarding with no CAN
/// hardware attached, so a disconnected bus can still be validated end to end.
///
/// The frame matches the NodeStatus shape `check_mavcan.py` recognizes (node 0,
/// `health_ok`), so the injected frame is also visible to the Python check.
pub fn inject_virtual_frame() -> Result<(), InjectError> {
    const NODE_STATUS_ID: u32 = EXTENDED_FLAG | (341u32 << 8); // uavcan.protocol.NodeStatus
    let frame = Frame::from_wire(
        NODE_STATUS_ID,
        8,
        [0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0],
    )
    .map_err(|_| InjectError::Frame)?;
    let now_us = Instant::now().as_micros();
    let epoch = SESSION.lock(|session| {
        let session = session.borrow();
        if permitted() && session.active_peer(now_us).is_some() {
            if session.accepts_filter(frame) {
                Some(session.epoch())
            } else {
                None
            }
        } else {
            None
        }
    });
    match epoch {
        Some(epoch) => OUTBOUND
            .try_send(QueuedFrame {
                frame,
                epoch,
                timestamp_us: now_us,
            })
            .map_err(|_| InjectError::QueueFull),
        None => Err(InjectError::Inactive),
    }
}

/// Result of injecting a virtual frame; used by the console to report why a
/// frame was not queued.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InjectError {
    Frame,
    Inactive,
    QueueFull,
}

fn from_hal(frame: HalFrame) -> Option<Frame> {
    let id = match frame.id() {
        CanId::Extended(id) => EXTENDED_FLAG | id.as_raw(),
        CanId::Standard(id) => u32::from(id.as_raw()),
    };
    let header = frame.header();
    let wire_id = id | if header.rtr() { RTR_FLAG } else { 0 };
    if header.fdcan() || header.len() > 8 {
        return None;
    }
    let mut data = [0; 8];
    if !header.rtr() {
        data[..usize::from(header.len())].copy_from_slice(frame.data());
    }
    Frame::from_wire(wire_id, header.len(), data).ok()
}

async fn io_receive_loop(mut rx: CanRx<'_>) -> ! {
    loop {
        match rx.read().await {
            Ok(envelope) => {
                RX_FRAMES.fetch_add(1, Ordering::Relaxed);
                let Some(frame) = from_hal(envelope.frame) else {
                    RX_ERRORS.fetch_add(1, Ordering::Relaxed);
                    continue;
                };
                let timestamp_us = Instant::now().as_micros();
                let epoch = SESSION.lock(|session| {
                    let session = session.borrow();
                    if permitted() && session.active_peer(timestamp_us).is_some() {
                        if session.accepts_filter(frame) {
                            Some(session.epoch())
                        } else {
                            RX_FILTERED.fetch_add(1, Ordering::Relaxed);
                            None
                        }
                    } else {
                        None
                    }
                });
                if let Some(epoch) = epoch {
                    if OUTBOUND
                        .try_send(QueuedFrame {
                            frame,
                            epoch,
                            timestamp_us,
                        })
                        .is_err()
                    {
                        RX_DROPPED.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
            Err(_) => {
                RX_ERRORS.fetch_add(1, Ordering::Relaxed);
                Timer::after_millis(1).await;
            }
        }
    }
}

fn queue_frame(queued: QueuedFrame) -> Option<usize> {
    SESSION.lock(|session| {
        let session = session.borrow();
        if !permitted()
            || !session.accepts_queued(
                queued.epoch,
                queued.timestamp_us,
                Instant::now().as_micros(),
            )
            || FDCAN2.txbrp().read().0 != 0
            || FDCAN2.txfqs().read().tfqf()
        {
            return None;
        }
        let index = usize::from(FDCAN2.txfqs().read().tfqpi());
        let base = usize::from(FDCAN2.txbc().read().tbsa()) + TX_ELEMENT_WORDS * index;
        // HAL 0.5.0 clears RTR when DLC != 0. Write the complete Classic element
        // before TXBAR; the session lock also prevents close/reset racing this enqueue.
        for (offset, word) in queued.frame.fdcan_tx_header().into_iter().enumerate() {
            FDCANRAM.ram(base + offset).write_value(word);
        }
        let data = queued.frame.data();
        for (offset, bytes) in data.as_chunks::<4>().0.iter().enumerate() {
            FDCANRAM
                .ram(base + 2 + offset)
                .write_value(u32::from_le_bytes(*bytes));
        }
        cortex_m::asm::dmb();
        FDCAN2.txbar().write(|w| w.set_ar(index, true));
        Some(index)
    })
}

async fn io_transmit_loop(_tx: CanTx<'_>) -> ! {
    // Retain the HAL TX owner so its drop cannot disconnect PB13 or the CAN IRQ.
    loop {
        let queued = INBOUND.receive().await;
        let Some(index) = queue_frame(queued) else {
            cancel_pending_tx();
            TX_DROPPED.fetch_add(1, Ordering::Relaxed);
            Timer::after_micros(50).await;
            continue;
        };
        let deadline = Instant::now() + TX_TIMEOUT;
        loop {
            if !FDCAN2.txbrp().read().trp(index) {
                if FDCAN2.txbto().read().to(index) {
                    TX_FRAMES.fetch_add(1, Ordering::Relaxed);
                } else {
                    TX_ERRORS.fetch_add(1, Ordering::Relaxed);
                }
                break;
            }
            if !queued_is_current(queued) || Instant::now() >= deadline {
                cancel_pending_tx();
                TX_TIMEOUTS.fetch_add(1, Ordering::Relaxed);
                break;
            }
            Timer::after_micros(50).await;
        }
    }
}

async fn maintain_session() -> ! {
    loop {
        let now_us = Instant::now().as_micros();
        let expired = SESSION.lock(|session| {
            let mut session = session.borrow_mut();
            if !permitted() && session.active_peer(now_us).is_some() {
                session.stop();
                true
            } else {
                session.expire(now_us)
            }
        });
        if expired {
            clear_queues();
        }
        Timer::after_millis(10).await;
    }
}

#[embassy_executor::task]
pub async fn io_task(
    fdcan: Peri<'static, peripherals::FDCAN2>,
    rx_pin: Peri<'static, peripherals::PB12>,
    tx_pin: Peri<'static, peripherals::PB13>,
) {
    let mut configurator = CanConfigurator::new(fdcan, rx_pin, tx_pin, MavcanIrqs);
    let config = configurator
        .config()
        .set_nominal_bit_timing(NominalBitTiming {
            prescaler: NonZeroU16::new(1).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(2).unwrap(),
        })
        .set_frame_transmit(FrameTransmissionConfig::ClassicCanOnly)
        .set_tx_buffer_mode(TxBufferMode::Fifo)
        .set_global_filter(GlobalFilter::default())
        .set_automatic_retransmit(false)
        .set_automatic_bus_off_recovery(true);
    configurator.set_config(config);
    let (tx, rx, _) = configurator.into_normal_mode().split();
    assert!(registers_ok());
    READY.store(true, Ordering::Release);
    join3(
        io_receive_loop(rx),
        io_transmit_loop(tx),
        maintain_session(),
    )
    .await;
}

fn registers_ok() -> bool {
    let nbtp = FDCAN2.nbtp().read();
    let control = FDCAN2.cccr().read();
    let transmit = FDCAN2.txbc().read();
    RCC.d2ccip1r().read().fdcansel() == embassy_stm32::rcc::mux::Fdcansel::HSE
        && nbtp.nbrp() == 0
        && nbtp.ntseg1() == 12
        && nbtp.ntseg2() == 1
        && nbtp.nsjw() == 1
        && !control.init()
        && !control.mon()
        && !control.fdoe()
        && !control.bse()
        && control.dar()
        && transmit.tfqm() == embassy_stm32::pac::can::vals::Tfqm::FIFO
        && transmit.ndtb() == 0
        && transmit.tfqs() == 3
        && FDCAN2.txesc().read().tbds() == 7
        && FDCAN2.sidfc().read().flssa() == 0x300
        && transmit.tbsa() == 0x39e
        && FDCAN2.gfc().read().0 == 0
}

pub fn write_diagnostics(output: &mut impl fmt::Write) -> fmt::Result {
    let now_us = Instant::now().as_micros();
    let (peer, lease_us, filters) = SESSION.lock(|session| {
        let session = session.borrow();
        (
            session.active_peer(now_us),
            session.remaining_us(now_us),
            session.filter_count(),
        )
    });
    writeln!(output,
        "MAVCAN schema=1 ready={} bus={} frame_bus={} bitrate={} active={} owner_sys={} owner_comp={} lease_ms={} filters={} rx_frames={} usb_frames={} rx_dropped={} rx_filtered={} rx_errors={} tx_received={} tx_frames={} tx_dropped={} tx_errors={} tx_timeouts={} rejected={} fd_rejected={} filter_errors={} rx_queue={} tx_queue={}",
        READY.load(Ordering::Acquire), BUS_NUMBER, FRAME_BUS, BITRATE, peer.is_some(),
        peer.map_or(0, |peer| peer.system), peer.map_or(0, |peer| peer.component), lease_us / 1000, filters,
        RX_FRAMES.load(Ordering::Relaxed), USB_FRAMES.load(Ordering::Relaxed),
        RX_DROPPED.load(Ordering::Relaxed), RX_FILTERED.load(Ordering::Relaxed), RX_ERRORS.load(Ordering::Relaxed),
        TX_RECEIVED.load(Ordering::Relaxed), TX_FRAMES.load(Ordering::Relaxed), TX_DROPPED.load(Ordering::Relaxed),
        TX_ERRORS.load(Ordering::Relaxed), TX_TIMEOUTS.load(Ordering::Relaxed), REJECTED.load(Ordering::Relaxed),
        FD_REJECTED.load(Ordering::Relaxed), FILTER_ERRORS.load(Ordering::Relaxed), OUTBOUND.len(), INBOUND.len(),
    )?;
    writeln!(
        output,
        "MAVCAN_HW registers_ok={} can1_registers_ok={} cccr={:08x} can1_cccr={:08x} nbtp={:08x} gfc={:08x} rxf0s={:08x} txbrp={:08x} psr={:08x} ecr={:08x}",
        registers_ok(),
        crate::dronecan_can::registers_ok(),
        FDCAN2.cccr().read().0,
        embassy_stm32::pac::FDCAN1.cccr().read().0,
        FDCAN2.nbtp().read().0,
        FDCAN2.gfc().read().0,
        FDCAN2.rxfs(0).read().0,
        FDCAN2.txbrp().read().0,
        FDCAN2.psr().read().0,
        FDCAN2.ecr().read().0
    )
}
