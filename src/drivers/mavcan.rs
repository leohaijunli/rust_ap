//! MAVCAN bridge: FDCAN2 raw frames <-> USB MAVLink CANFD_FRAME messages.
//!
//! The flight controller acts as a USB-CAN adapter for DroneCAN tools that
//! support the `mavcan:` transport (pydronecan bridge, DroneCAN Web/GUI), the
//! same role as the VimDrones MAVCAN_Bridge hardware. FDCAN2 (PB12/PB13) is
//! independent of FDCAN1 (DroneCAN node). Reference: VimDrones/MAVCAN_Bridge.

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_futures::join::join;
use embassy_stm32::bind_interrupts;
use embassy_stm32::can::config::{
    FrameTransmissionConfig, GlobalFilter, NominalBitTiming, TxBufferMode,
};
use embassy_stm32::can::filter::{ExtendedFilter, ExtendedFilterSlot};
use embassy_stm32::can::frame::Header;
use embassy_stm32::can::{
    CanConfigurator, CanRx, CanTx, Frame as HalFrame, IT0InterruptHandler, IT1InterruptHandler,
};
use embassy_stm32::pac::FDCAN2;
use embassy_stm32::{peripherals, Peri};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embedded_can::Id as CanId;
use mavlink::common::*;

use crate::mavlink_core;

const RX_QUEUE: usize = 64;

/// A raw CAN frame carried between the USB and FDCAN2 directions.
#[derive(Clone, Copy, Debug)]
pub struct CanFrame {
    pub id: u32,
    pub data: [u8; 8],
    pub len: u8,
}

bind_interrupts!(struct MavcanIrqs {
    FDCAN2_IT0 => IT0InterruptHandler<peripherals::FDCAN2>;
    FDCAN2_IT1 => IT1InterruptHandler<peripherals::FDCAN2>;
});

/// CAN frames from FDCAN2 to send to the USB host as CANFD_FRAME messages.
pub static OUTBOUND: Channel<CriticalSectionRawMutex, MavMessage, RX_QUEUE> = Channel::new();
/// CAN frames from the USB host (CANFD_FRAME) to transmit on FDCAN2.
pub static INBOUND: Channel<CriticalSectionRawMutex, CanFrame, RX_QUEUE> = Channel::new();

static RX_DROPPED: AtomicU32 = AtomicU32::new(0);
static TX_ERRORS: AtomicU32 = AtomicU32::new(0);

/// Convert an inbound MAVLink CANFD_FRAME into a raw CAN frame for FDCAN2.
/// `target_system`/`target_component` must be ours or broadcast.
pub fn frame_from_mavlink(data: &CANFD_FRAME_DATA) -> Option<CanFrame> {
    let system_ok = matches!(data.target_system, 0 | mavlink_core::SYSTEM_ID);
    let component_ok = matches!(data.target_component, 0 | mavlink_core::COMPONENT_ID);
    if !system_ok || !component_ok {
        return None;
    }
    let len = usize::from(data.len).min(8);
    let mut frame = CanFrame {
        id: data.id,
        data: [0; 8],
        len: len as u8,
    };
    frame.data[..len].copy_from_slice(&data.data[..len]);
    Some(frame)
}

/// Route an inbound USB MAVLink message into the FDCAN2 TX path.
/// Returns true when the message was a CANFD_FRAME we accepted.
pub fn handle_message(message: &MavMessage) -> bool {
    match message {
        MavMessage::CANFD_FRAME(data) => match frame_from_mavlink(data) {
            Some(frame) => INBOUND.try_send(frame).is_ok(),
            None => false,
        },
        _ => false,
    }
}

/// Build a CANFD_FRAME message for the USB host from an FDCAN2 frame.
fn frame_to_mavlink(frame: HalFrame) -> MavMessage {
    let id = match frame.id() {
        CanId::Extended(id) => id.as_raw(),
        CanId::Standard(id) => u32::from(id.as_raw()),
    };
    let bytes = frame.data();
    let mut data = [0u8; 64];
    data[..bytes.len()].copy_from_slice(bytes);
    MavMessage::CANFD_FRAME(CANFD_FRAME_DATA {
        id,
        target_system: mavlink_core::SYSTEM_ID,
        target_component: mavlink_core::COMPONENT_ID,
        bus: 0,
        len: bytes.len() as u8,
        data,
    })
}

async fn io_receive_loop(mut rx: CanRx<'_>) -> ! {
    loop {
        match rx.read().await {
            Ok(envelope) => {
                if OUTBOUND.try_send(frame_to_mavlink(envelope.frame)).is_err() {
                    RX_DROPPED.fetch_add(1, Ordering::Relaxed);
                }
            }
            Err(_) => {
                embassy_time::Timer::after_millis(1).await;
            }
        }
    }
}

async fn io_transmit_loop(mut tx: CanTx<'_>) -> ! {
    loop {
        let frame = INBOUND.receive().await;
        let id = embedded_can::ExtendedId::new(frame.id).unwrap_or(embedded_can::ExtendedId::ZERO);
        let header = Header::new(CanId::Extended(id), frame.len, false);
        let hal = HalFrame::new(header, &frame.data[..usize::from(frame.len)]);
        match hal {
            Ok(hal) => {
                if tx.write(&hal).await.is_some() {
                    TX_ERRORS.fetch_add(1, Ordering::Relaxed);
                }
            }
            Err(_) => {
                TX_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

#[embassy_executor::task]
pub async fn io_task(
    fdcan: Peri<'static, peripherals::FDCAN2>,
    rx_pin: Peri<'static, peripherals::PB12>,
    tx_pin: Peri<'static, peripherals::PB13>,
) {
    let mut configurator = CanConfigurator::new(fdcan, rx_pin, tx_pin, MavcanIrqs);
    let timing = NominalBitTiming {
        prescaler: core::num::NonZeroU16::new(1).unwrap(),
        seg1: core::num::NonZeroU8::new(13).unwrap(),
        seg2: core::num::NonZeroU8::new(2).unwrap(),
        sync_jump_width: core::num::NonZeroU8::new(2).unwrap(),
    };
    let config = configurator
        .config()
        .set_nominal_bit_timing(timing)
        .set_frame_transmit(FrameTransmissionConfig::ClassicCanOnly)
        .set_tx_buffer_mode(TxBufferMode::Fifo)
        .set_global_filter(GlobalFilter::reject_all())
        .set_automatic_retransmit(false)
        .set_automatic_bus_off_recovery(true);
    configurator.set_config(config);
    configurator.properties().set_extended_filter(
        ExtendedFilterSlot::_0,
        ExtendedFilter::accept_all_into_fifo0(),
    );
    let (tx, rx, _) = configurator.into_normal_mode().split();
    assert!(registers_ok());

    join(io_receive_loop(rx), io_transmit_loop(tx)).await;
    unreachable!()
}

/// FDCAN2 clock and FIFO configuration sanity check.
pub fn registers_ok() -> bool {
    let nbtp = FDCAN2.nbtp().read();
    let control = FDCAN2.cccr().read();
    nbtp.nbrp() == 0
        && nbtp.ntseg1() == 12
        && nbtp.ntseg2() == 1
        && nbtp.nsjw() == 1
        && !control.fdoe()
        && !control.bse()
        && control.dar()
        && FDCAN2.txbc().read().tfqm() == embassy_stm32::pac::can::vals::Tfqm::FIFO
}
