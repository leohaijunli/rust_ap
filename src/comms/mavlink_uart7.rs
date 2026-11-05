//! MAVLink telemetry over the TELEM1 UART7 link (115200 8N1).

use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{Uart, UartRx, UartTx};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use mavlink::common::*;

use crate::dronecan_can;
use crate::mavlink_core::{self, FrameParser};
use crate::parameter_manager::ParameterManager;
use crate::usb_console::{self, CommandLine};

type Replies = Channel<NoopRawMutex, MavMessage, 4>;

/// Per-link parameter-list download cursor; see `mavlink_core::begin_param_stream`.
static PARAM_CURSOR: mavlink_core::ParamCursor =
    mavlink_core::ParamCursor::new(v6x_rust_led::parameters::COUNT as u16);

#[embassy_executor::task]
pub async fn run(
    uart: Uart<'static, Async>,
    parameters: &'static ParameterManager<'static>,
    dronecan: &'static dronecan_can::State,
) -> ! {
    let (mut tx, mut rx) = uart.split();
    let replies = Replies::new();
    join(
        transmit(&mut tx, &replies, parameters),
        receive(&mut rx, &replies, parameters, dronecan),
    )
    .await;
    unreachable!()
}

async fn transmit(
    tx: &mut UartTx<'static, Async>,
    replies: &Replies,
    parameters: &ParameterManager<'static>,
) -> ! {
    const IMU_PERIOD: Duration = Duration::from_millis(50);
    const POWER_PERIOD: Duration = Duration::from_micros(v6x_rust_led::ina219::SAMPLE_PERIOD_US);
    // PX4 streams GPS_RAW_INT at the GNSS update rate; 5 Hz covers H-RTK.
    const GPS_PERIOD: Duration = Duration::from_millis(200);
    const GPS_NO_FIX_PERIOD: Duration = Duration::from_secs(1);
    let mut sequence = 0u8;
    let mut independent_sequence = 0u8;
    let mut estimator_tick = 0u8;
    let mut frame = [0u8; mavlink::MAX_FRAME_SIZE];
    let mut next_heartbeat = Instant::now() + Duration::from_secs(1);
    let mut next_imu = Instant::now() + IMU_PERIOD;
    let mut next_power = Instant::now() + POWER_PERIOD;
    let mut next_gps = Instant::now() + GPS_PERIOD;
    let mut next_gps_no_fix = Instant::now() + GPS_NO_FIX_PERIOD;
    loop {
        let next_wakeup = next_imu.min(next_heartbeat).min(next_power).min(next_gps);
        let result = match select(Timer::at(next_wakeup), replies.receive()).await {
            Either::First(()) => {
                let now = Instant::now();
                let mut success = true;
                if now >= next_imu {
                    if let Some(message) = mavlink_core::highres_imu() {
                        success = send(tx, &mut sequence, &message, &mut frame).await;
                    }
                    if success {
                        if let Some(message) = mavlink_core::scaled_pressure() {
                            success = send(tx, &mut sequence, &message, &mut frame).await;
                        }
                    }
                    next_imu = now + IMU_PERIOD;
                    estimator_tick = estimator_tick.wrapping_add(1);
                    if success && estimator_tick.is_multiple_of(2) {
                        for (component, message) in
                            mavlink_core::estimator_messages().into_iter().flatten()
                        {
                            let sequence = if component == mavlink_core::COMPONENT_ID {
                                &mut sequence
                            } else {
                                &mut independent_sequence
                            };
                            let length = mavlink_core::encode_from(
                                sequence, component, &message, &mut frame,
                            );
                            success = tx.write(&frame[..length]).await.is_ok();
                            if !success {
                                break;
                            }
                        }
                    }
                }
                if success && now >= next_power {
                    if let Some(message) = mavlink_core::ina219_status() {
                        success = send(tx, &mut sequence, &message, &mut frame).await;
                    }
                    next_power = now + POWER_PERIOD;
                }
                if success && now >= next_gps {
                    if let Some(message) = mavlink_core::gps_raw_int() {
                        success = send(tx, &mut sequence, &message, &mut frame).await;
                        next_gps_no_fix = now + GPS_NO_FIX_PERIOD;
                    } else if now >= next_gps_no_fix {
                        // PX4: one NO_GPS frame per second after data stops.
                        success = send(
                            tx,
                            &mut sequence,
                            &mavlink_core::gps_raw_int_no_fix(),
                            &mut frame,
                        )
                        .await;
                        next_gps_no_fix = now + GPS_NO_FIX_PERIOD;
                    }
                    next_gps = now + GPS_PERIOD;
                }
                if success && now >= next_heartbeat {
                    success = send(tx, &mut sequence, &mavlink_core::heartbeat(), &mut frame).await
                        && send(
                            tx,
                            &mut sequence,
                            &mavlink_core::system_status(),
                            &mut frame,
                        )
                        .await
                        && send(tx, &mut sequence, &mavlink_core::system_time(), &mut frame).await;
                    next_heartbeat = now + Duration::from_secs(1);
                }
                success
            }
            Either::Second(message) => send(tx, &mut sequence, &message, &mut frame).await,
        };
        // A slow or absent receiver must not stall the autopilot; resync the tick
        // so we never burst stale heartbeats once the link becomes usable again.
        if !result {
            let _ = tx.flush().await;
        }
        // Drain a pending full-parameter download, one value per loop, so a
        // large registry never bursts or stalls the heartbeat/stream pacing.
        if result {
            if let Some(message) = mavlink_core::next_param_value(parameters, &PARAM_CURSOR) {
                let _ = send(tx, &mut sequence, &message, &mut frame).await;
            }
        }
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
    dronecan: &dronecan_can::State,
) -> ! {
    let mut buffer = [0u8; 256];
    let mut parser = FrameParser::new();
    let mut command_line = CommandLine::new();
    loop {
        match rx.read_until_idle(&mut buffer).await {
            Ok(n) => {
                for &byte in &buffer[..n] {
                    if let Some(decoded) = parser.push(byte) {
                        if let Some(request) = mavlink_core::log_request(&decoded.message) {
                            handle_log_request(request, replies).await;
                            continue;
                        }
                        match decoded.message {
                            MavMessage::SERIAL_CONTROL(data)
                                if data.device == SerialControlDev::SERIAL_CONTROL_DEV_SHELL
                                    && matches!(
                                        data.target_system,
                                        0 | mavlink_core::SYSTEM_ID
                                    )
                                    && matches!(
                                        data.target_component,
                                        0 | mavlink_core::COMPONENT_ID
                                    ) =>
                            {
                                for &byte in &data.data[..usize::from(data.count.min(70))] {
                                    if let Some(command) = command_line.push(byte) {
                                        let response = usb_console::mavlink_report(
                                            command, parameters, dronecan,
                                        )
                                        .await;
                                        let chunks = response.as_bytes().chunks(70);
                                        let count = chunks.len();
                                        for (index, chunk) in chunks.enumerate() {
                                            replies
                                                .send(mavlink_core::serial_control_reply(
                                                    decoded.header,
                                                    chunk,
                                                    index + 1 < count,
                                                ))
                                                .await;
                                        }
                                    }
                                }
                            }
                            message => {
                                let mut reply = mavlink_core::Reply::EMPTY;
                                mavlink_core::handle(
                                    decoded.header,
                                    message,
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
                                if reply.param_stream {
                                    mavlink_core::begin_param_stream(&PARAM_CURSOR);
                                }
                            }
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

async fn handle_log_request(request: v6x_rust_led::logger::LogRequest, replies: &Replies) {
    use v6x_rust_led::logger::{Link, LogError, LogRequest, LogResponse};

    if crate::sd_logger::request_logs(Link::Uart, request).is_err() {
        if matches!(request, LogRequest::List { .. }) {
            let empty = LogResponse::Entry {
                id: 0,
                num_logs: 0,
                last_log_num: 0,
                size: 0,
                time_utc: 0,
            };
            replies
                .send(mavlink_core::log_response(empty).unwrap())
                .await;
        }
        replies
            .send(mavlink_core::log_response(LogResponse::Error(LogError::Busy)).unwrap())
            .await;
        return;
    }
    loop {
        let response = crate::sd_logger::next_log_response(Link::Uart).await;
        if response == LogResponse::Done {
            break;
        }
        if let Some(message) = mavlink_core::log_response(response) {
            replies.send(message).await;
        }
    }
}
