//! MAVLink telemetry over the TELEM1 UART7 link (115200 8N1).

use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{Uart, UartRx, UartTx};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use mavlink::common::*;

use crate::mavlink_core::{self, FrameParser};
use crate::parameter_manager::ParameterManager;

type Replies = Channel<NoopRawMutex, MavMessage, 4>;

pub async fn run(uart: Uart<'static, Async>, parameters: &ParameterManager<'_>) -> ! {
    let (mut tx, mut rx) = uart.split();
    let replies = Replies::new();
    join(
        transmit(&mut tx, &replies),
        receive(&mut rx, &replies, parameters),
    )
    .await;
    unreachable!()
}

async fn transmit(tx: &mut UartTx<'static, Async>, replies: &Replies) -> ! {
    let mut sequence = 0u8;
    let mut frame = [0u8; mavlink::MAX_FRAME_SIZE];
    let mut next_heartbeat = Instant::now() + Duration::from_secs(1);
    loop {
        let result = match select(Timer::at(next_heartbeat), replies.receive()).await {
            Either::First(()) => {
                send(tx, &mut sequence, &mavlink_core::heartbeat(), &mut frame).await
                    && send(
                        tx,
                        &mut sequence,
                        &mavlink_core::system_status(),
                        &mut frame,
                    )
                    .await
                    && send(tx, &mut sequence, &mavlink_core::system_time(), &mut frame).await
            }
            Either::Second(message) => send(tx, &mut sequence, &message, &mut frame).await,
        };
        // A slow or absent receiver must not stall the autopilot; resync the tick
        // so we never burst stale heartbeats once the link becomes usable again.
        if !result {
            let _ = tx.flush().await;
        }
        next_heartbeat = Instant::now() + Duration::from_secs(1);
    }
}

async fn send(
    tx: &mut UartTx<'static, Async>,
    sequence: &mut u8,
    message: &MavMessage,
    frame: &mut [u8; mavlink::MAX_FRAME_SIZE],
) -> bool {
    let len = mavlink_core::encode(sequence, message, frame);
    tx.write(&frame[..len]).await.is_ok()
}

async fn receive(
    rx: &mut UartRx<'static, Async>,
    replies: &Replies,
    parameters: &ParameterManager<'_>,
) -> ! {
    let mut buffer = [0u8; 256];
    let mut parser = FrameParser::new();
    loop {
        match rx.read_until_idle(&mut buffer).await {
            Ok(n) => {
                for &byte in &buffer[..n] {
                    if let Some(decoded) = parser.push(byte) {
                        let mut reply = mavlink_core::Reply::EMPTY;
                        mavlink_core::handle(
                            decoded.header,
                            decoded.message,
                            parameters,
                            &mut reply,
                        )
                        .await;
                        for message in reply.drain() {
                            replies.send(message).await;
                        }
                        if reply.reboot {
                            crate::fault::reboot_to_bootloader();
                        }
                    }
                }
            }
            Err(_) => {
                // A parity/frame/overrun error is cleared and ignored; the stream
                // continues at the next byte boundary.
                parser = FrameParser::new();
            }
        }
    }
}
