//! MAVLink telemetry over USB CDC, with a small diagnostic line console.

use core::cell::RefCell;

use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_stm32::gpio::Output;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender};
use embassy_usb::driver::{Driver, EndpointError};
use mavlink::common::*;

use crate::dronecan_can;
use crate::eth_led::Lan8742;
use crate::mavlink_core::{self, FrameParser};
use crate::parameter_manager::ParameterManager;
use crate::usb_console::{self, Command, CommandLine, Response};

// Fixed-size buffers are intentional: this no_std firmware has no allocator.
#[allow(clippy::large_enum_variant)]
enum Outbound {
    Message(MavMessage),
    Console(Response),
    Bootloader {
        console_ack: bool,
    },
    #[cfg(feature = "fault-injection")]
    InjectFault {
        hard: bool,
    },
}

type Replies = Channel<NoopRawMutex, Outbound, 4>;

pub type UsbSession = Watch<NoopRawMutex, u32, 2>;

/// Start a fresh transport session whenever the host resets or reconfigures USB.
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
        self.generation = self.generation.wrapping_add(1);
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

pub async fn run<'d, D: Driver<'d>>(
    class: CdcAcmClass<'d, D>,
    phy: &RefCell<Lan8742<'_>>,
    eth_power: &Output<'_>,
    session: &UsbSession,
    parameters: &ParameterManager<'_>,
    dronecan: &dronecan_can::State,
) -> ! {
    let (mut sender, mut receiver) = class.split();
    let replies = Replies::new();
    let mut tx_reset = session.receiver().unwrap();
    let mut rx_reset = session.receiver().unwrap();
    tx_reset.get().await;
    rx_reset.get().await;
    join(
        async {
            loop {
                // Poll reset first: never begin a fresh write for an old session.
                select(tx_reset.changed(), transmit(&mut sender, &replies)).await;
                replies.clear();
            }
        },
        async {
            loop {
                select(
                    rx_reset.changed(),
                    receive(
                        &mut receiver,
                        &replies,
                        phy,
                        eth_power,
                        parameters,
                        dronecan,
                    ),
                )
                .await;
            }
        },
    )
    .await;
    unreachable!()
}

async fn transmit<'d, D: Driver<'d>>(sender: &mut Sender<'d, D>, replies: &Replies) -> ! {
    const IMU_PERIOD: Duration = Duration::from_millis(50);
    const POWER_PERIOD: Duration = Duration::from_micros(v6x_rust_led::ina219::SAMPLE_PERIOD_US);
    let mut sequence = 0u8;
    'connection: loop {
        sender.wait_connection().await;
        let mut ticks = 0u32;
        for message in [
            mavlink_core::heartbeat(),
            mavlink_core::status_text(),
            mavlink_core::autopilot_version(),
        ] {
            if send_message(sender, &mut sequence, &message).await.is_err() {
                continue 'connection;
            }
        }
        let mut next_heartbeat = Instant::now() + Duration::from_secs(1);
        let mut next_imu = Instant::now() + IMU_PERIOD;
        let mut next_power = Instant::now() + POWER_PERIOD;

        loop {
            let next_wakeup = next_imu.min(next_heartbeat).min(next_power);
            let result = match select(Timer::at(next_wakeup), replies.receive()).await {
                Either::First(()) => {
                    let now = Instant::now();
                    let mut result = Ok(());
                    if now >= next_imu {
                        if let Some(message) = mavlink_core::highres_imu() {
                            result = send_message(sender, &mut sequence, &message).await;
                        }
                        next_imu = now + IMU_PERIOD;
                    }
                    if result.is_ok() && now >= next_power {
                        if let Some(message) = mavlink_core::ina219_status() {
                            result = send_message(sender, &mut sequence, &message).await;
                        }
                        next_power = now + POWER_PERIOD;
                    }
                    if result.is_ok() && now >= next_heartbeat {
                        ticks = ticks.wrapping_add(1);
                        result =
                            send_message(sender, &mut sequence, &mavlink_core::heartbeat()).await;
                        if result.is_ok() {
                            result =
                                send_message(sender, &mut sequence, &mavlink_core::system_status())
                                    .await;
                        }
                        if result.is_ok() {
                            // Generate boot time after earlier USB writes complete.
                            result =
                                send_message(sender, &mut sequence, &mavlink_core::system_time())
                                    .await;
                        }
                        if result.is_ok() && ticks.is_multiple_of(5) {
                            result =
                                send_message(sender, &mut sequence, &mavlink_core::status_text())
                                    .await;
                        }
                        next_heartbeat = now + Duration::from_secs(1);
                    }
                    result
                }
                Either::Second(Outbound::Message(message)) => {
                    send_message(sender, &mut sequence, &message).await
                }
                Either::Second(Outbound::Console(response)) => {
                    write_transfer(sender, response.as_bytes()).await
                }
                Either::Second(Outbound::Bootloader { console_ack }) => {
                    if console_ack {
                        let _ = with_timeout(
                            Duration::from_millis(100),
                            write_transfer(sender, b"OK bootloader\n"),
                        )
                        .await;
                    }
                    Timer::after_millis(100).await;
                    crate::fault::reboot_to_bootloader();
                }
                #[cfg(feature = "fault-injection")]
                Either::Second(Outbound::InjectFault { hard }) => {
                    let _ = with_timeout(
                        Duration::from_millis(100),
                        write_transfer(sender, b"OK fault-injection\n"),
                    )
                    .await;
                    Timer::after_millis(100).await;
                    if hard {
                        cortex_m::asm::udf();
                    }
                    panic!("P1 panic injection");
                }
            };
            if result.is_err() {
                break;
            }
        }
    }
}

async fn send_message<'d, D: Driver<'d>>(
    sender: &mut Sender<'d, D>,
    sequence: &mut u8,
    message: &MavMessage,
) -> Result<(), EndpointError> {
    let mut frame = [0u8; mavlink::MAX_FRAME_SIZE];
    let len = mavlink_core::encode(sequence, message, &mut frame);
    write_transfer(sender, &frame[..len]).await
}

async fn write_transfer<'d, D: Driver<'d>>(
    sender: &mut Sender<'d, D>,
    data: &[u8],
) -> Result<(), EndpointError> {
    let max_packet_size = usize::from(sender.max_packet_size());
    for packet in data.chunks(max_packet_size) {
        sender.write_packet(packet).await?;
    }
    if data.len().is_multiple_of(max_packet_size) {
        sender.write_packet(&[]).await?;
    }
    Ok(())
}

async fn receive<'d, D: Driver<'d>>(
    receiver: &mut Receiver<'d, D>,
    replies: &Replies,
    phy: &RefCell<Lan8742<'_>>,
    eth_power: &Output<'_>,
    parameters: &ParameterManager<'_>,
    dronecan: &dronecan_can::State,
) -> ! {
    let mut packet = [0u8; 64];
    loop {
        receiver.wait_connection().await;
        let mut parser = FrameParser::new();
        let mut command_line = CommandLine::new();
        let mut mavlink_command_line = CommandLine::new();
        let mut last_byte = Instant::now();
        while let Ok(n) = receiver.read_packet(&mut packet).await {
            let now = Instant::now();
            if parser.receiving() && (now - last_byte).as_millis() > 1000 {
                // A host that closed mid-frame must not block later requests.
                parser = FrameParser::new();
                command_line = CommandLine::new();
            }
            last_byte = now;
            for &byte in &packet[..n] {
                if parser.receiving() || matches!(byte, 0xfe | 0xfd) {
                    command_line = CommandLine::new();
                    if let Some(decoded) = parser.push(byte) {
                        handle_message(
                            decoded,
                            replies,
                            parameters,
                            dronecan,
                            &mut mavlink_command_line,
                        )
                        .await;
                    }
                } else {
                    match command_line.push(byte) {
                        Some(Command::Status) => {
                            replies
                                .send(Outbound::Console(usb_console::status(phy, eth_power)))
                                .await;
                        }
                        Some(Command::Bootloader) => {
                            replies
                                .send(Outbound::Bootloader { console_ack: true })
                                .await;
                        }
                        Some(command @ (Command::Fram | Command::FramRead(_))) => {
                            let response = usb_console::fram_report(command, parameters).await;
                            replies.send(Outbound::Console(response)).await;
                        }
                        Some(
                            command @ (Command::Parameters
                            | Command::ParamSave
                            | Command::ParamLoad),
                        ) => {
                            let response = usb_console::parameter_report(
                                command,
                                parameters,
                                crate::OUTPUT_STATE,
                            )
                            .await;
                            replies.send(Outbound::Console(response)).await;
                        }
                        #[cfg(feature = "fault-injection")]
                        Some(Command::Panic) => {
                            replies.send(Outbound::InjectFault { hard: false }).await
                        }
                        #[cfg(feature = "fault-injection")]
                        Some(Command::HardFault) => {
                            replies.send(Outbound::InjectFault { hard: true }).await
                        }
                        Some(command) => {
                            replies
                                .send(Outbound::Console(usb_console::report(command, dronecan)))
                                .await
                        }
                        None => {}
                    }
                }
            }
        }
    }
}

// Keep PING and command 520 for ground stations using the older discovery flow.
#[allow(deprecated)]
async fn handle_message(
    decoded: mavlink_core::Decoded,
    replies: &Replies,
    parameters: &ParameterManager<'_>,
    dronecan: &dronecan_can::State,
    command_line: &mut CommandLine,
) {
    if let Some(request) = mavlink_core::log_request(&decoded.message) {
        handle_log_request(request, replies).await;
        return;
    }
    match decoded.message {
        MavMessage::SERIAL_CONTROL(data) => {
            if data.device == SerialControlDev::SERIAL_CONTROL_DEV_SHELL
                && matches!(data.target_system, 0 | mavlink_core::SYSTEM_ID)
                && matches!(data.target_component, 0 | mavlink_core::COMPONENT_ID)
            {
                for &byte in &data.data[..usize::from(data.count.min(70))] {
                    if let Some(command) = command_line.push(byte) {
                        let response =
                            usb_console::mavlink_report(command, parameters, dronecan).await;
                        let chunks = response.as_bytes().chunks(70);
                        let count = chunks.len();
                        for (index, chunk) in chunks.enumerate() {
                            replies
                                .send(Outbound::Message(mavlink_core::serial_control_reply(
                                    decoded.header,
                                    chunk,
                                    index + 1 < count,
                                )))
                                .await;
                        }
                    }
                }
            }
        }
        message => {
            let mut reply = mavlink_core::Reply::EMPTY;
            mavlink_core::handle(decoded.header, message, parameters, &mut reply).await;
            for message in reply.drain() {
                replies.send(Outbound::Message(message)).await;
            }
            if reply.reboot {
                replies
                    .send(Outbound::Bootloader { console_ack: false })
                    .await;
            }
        }
    }
}

async fn handle_log_request(request: v6x_rust_led::logger::LogRequest, replies: &Replies) {
    use v6x_rust_led::logger::{Link, LogError, LogRequest, LogResponse};

    if crate::sd_logger::request_logs(Link::Usb, request).is_err() {
        if matches!(request, LogRequest::List { .. }) {
            let empty = LogResponse::Entry {
                id: 0,
                num_logs: 0,
                last_log_num: 0,
                size: 0,
                time_utc: 0,
            };
            replies
                .send(Outbound::Message(
                    mavlink_core::log_response(empty).unwrap(),
                ))
                .await;
        }
        replies
            .send(Outbound::Message(
                mavlink_core::log_response(LogResponse::Error(LogError::Busy)).unwrap(),
            ))
            .await;
        return;
    }
    loop {
        let response = crate::sd_logger::next_log_response(Link::Usb).await;
        if response == LogResponse::Done {
            break;
        }
        if let Some(message) = mavlink_core::log_response(response) {
            replies.send(Outbound::Message(message)).await;
        }
    }
}
