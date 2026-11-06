//! USB CDC byte transport; protocol and telemetry state belong to the Router.

use embassy_futures::{join::join, select::select};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, watch::Watch};
use embassy_time::{with_timeout, Duration, Timer};
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, Receiver, Sender},
    driver::{Driver, EndpointError},
};
use v6x_rust_led::logger::Link;

use crate::router::{self, Packet};

pub type UsbSession = Watch<NoopRawMutex, u32, 2>;

pub struct UsbSessionEvents<'a> {
    session: &'a UsbSession,
    generation: u32,
}

impl<'a> UsbSessionEvents<'a> {
    pub fn new(session: &'a UsbSession) -> Self {
        Self {
            session,
            generation: 0,
        }
    }

    fn restart(&mut self) {
        crate::mavcan::stop();
        self.generation = self.generation.wrapping_add(1);
        router::reset_usb();
        self.session.sender().send(self.generation);
    }
}

impl embassy_usb::Handler for UsbSessionEvents<'_> {
    fn reset(&mut self) {
        self.restart();
    }
    fn configured(&mut self, _configured: bool) {
        self.restart();
    }
    fn enabled(&mut self, enabled: bool) {
        if !enabled {
            self.restart();
        }
    }
}

type BoardUsbDriver = embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>;

#[embassy_executor::task]
pub async fn device_task(mut usb: embassy_usb::UsbDevice<'static, BoardUsbDriver>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
pub async fn run(class: CdcAcmClass<'static, BoardUsbDriver>, session: &'static UsbSession) -> ! {
    let (mut sender, mut receiver) = class.split();
    let mut tx_reset = session.receiver().unwrap();
    let mut rx_reset = session.receiver().unwrap();
    tx_reset.get().await;
    rx_reset.get().await;
    join(
        async {
            loop {
                // Cancel old endpoint writes before using a reconfigured USB session.
                select(tx_reset.changed(), transmit(&mut sender)).await;
                router::connected(Link::Usb, false);
                crate::mavcan::stop();
            }
        },
        async {
            loop {
                select(rx_reset.changed(), receive(&mut receiver)).await;
            }
        },
    )
    .await;
    unreachable!()
}

async fn transmit<'d, D: Driver<'d>>(sender: &mut Sender<'d, D>) -> ! {
    loop {
        sender.wait_connection().await;
        router::connected(Link::Usb, true);
        loop {
            let transfer = router::USB_TX.receive().await;
            if transfer.generation != router::usb_generation() {
                continue;
            }
            let sent = transfer.len == 0
                || matches!(
                    with_timeout(
                        Duration::from_millis(100),
                        write_transfer(sender, &transfer.bytes[..transfer.len])
                    )
                    .await,
                    Ok(Ok(()))
                );
            if !sent {
                router::record_tx_error(Link::Usb);
                break;
            }
            if transfer.mavcan {
                crate::mavcan::record_usb_send();
            }
        }
        router::reset_usb();
        crate::mavcan::stop();
        // Let the Router observe disconnect before retrying an endpoint still configured by a stalled host.
        Timer::after_millis(20).await;
    }
}

async fn write_transfer<'d, D: Driver<'d>>(
    sender: &mut Sender<'d, D>,
    bytes: &[u8],
) -> Result<(), EndpointError> {
    let packet_size = usize::from(sender.max_packet_size());
    for packet in bytes.chunks(packet_size) {
        sender.write_packet(packet).await?;
    }
    if bytes.len().is_multiple_of(packet_size) {
        sender.write_packet(&[]).await?;
    }
    Ok(())
}

async fn receive<'d, D: Driver<'d>>(receiver: &mut Receiver<'d, D>) -> ! {
    let mut bytes = [0; 64];
    loop {
        receiver.wait_connection().await;
        while let Ok(len) = receiver.read_packet(&mut bytes).await {
            if len != 0 {
                router::USB_RX
                    .send(Packet::new(router::usb_generation(), &bytes[..len]))
                    .await;
            }
        }
        router::reset_usb();
        crate::mavcan::stop();
        router::record_rx_error(Link::Usb);
    }
}
