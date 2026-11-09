//! TELEM1 UART7 byte transport (115200 8N1), with bounded DMA writes.

use embassy_futures::join::join;
use embassy_stm32::{
    mode::Async,
    usart::{Uart, UartRx, UartTx},
};
use embassy_time::{with_timeout, Duration};
use v6x_rust_ap::logger::Link;

use crate::router::{self, Packet};

#[embassy_executor::task]
pub async fn run(uart: Uart<'static, Async>) -> ! {
    let (mut tx, mut rx) = uart.split();
    router::connected(Link::Uart, true);
    join(transmit(&mut tx), receive(&mut rx)).await;
    unreachable!()
}

async fn transmit(tx: &mut UartTx<'static, Async>) -> ! {
    loop {
        let transfer = router::UART_TX.receive().await;
        if transfer.len != 0
            && !matches!(
                with_timeout(
                    Duration::from_millis(100),
                    tx.write(&transfer.bytes[..transfer.len])
                )
                .await,
                Ok(Ok(()))
            )
        {
            // DMA cancellation is owned by the HAL; no catch-up burst follows a failed write.
            router::record_tx_error(Link::Uart);
            continue;
        }
    }
}

async fn receive(rx: &mut UartRx<'static, Async>) -> ! {
    let mut bytes = [0; 256];
    loop {
        match rx.read_until_idle(&mut bytes).await {
            Ok(len) => router::UART_RX.send(Packet::new(0, &bytes[..len])).await,
            // An empty packet tells the Router to discard a partial frame after an overrun/parity error.
            Err(_) => {
                router::record_rx_error(Link::Uart);
                router::UART_RX.send(Packet::new(0, &[])).await;
            }
        }
    }
}
