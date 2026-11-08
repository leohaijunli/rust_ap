//! One owner for RX parsing, commands, sessions and telemetry on both links.
//! Byte transports never own parameter cursors, FTP sessions or stream timers.

use core::{
    cell::RefCell,
    fmt,
    future::{pending, poll_fn},
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use embassy_futures::select::select3;
use embassy_stm32::gpio::Output;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};
use mavlink::{common::*, MavHeader};
use v6x_rust_led::{
    logger::{erase_allowed, Link, LogError, LogRequest, LogResponse},
    mavftp,
    mavlink_protocol::{addressed_to_vehicle, Decoded, FrameParser, ParamStream},
    telemetry::Scheduler,
};

use crate::{
    dronecan_can,
    eth_led::Lan8742,
    mavlink_core::{self, Reply},
    parameter_manager::ParameterManager,
    telemetry::{self, Outgoing},
    usb_console::{self, Command, CommandLine, Response},
};

pub struct Packet {
    pub generation: u32,
    pub received: u64,
    pub bytes: [u8; 256],
    pub len: usize,
}

impl Packet {
    pub fn new(generation: u32, bytes: &[u8]) -> Self {
        let mut packet = Self {
            generation,
            received: Instant::now().as_micros(),
            bytes: [0; 256],
            len: bytes.len(),
        };
        packet.bytes[..bytes.len()].copy_from_slice(bytes);
        packet
    }
}

#[derive(Clone, Copy, Default)]
pub enum Action {
    #[default]
    None,
    Bootloader,
    #[cfg(feature = "fault-injection")]
    InjectFault {
        hard: bool,
    },
}

pub struct Transfer {
    pub generation: u32,
    pub bytes: [u8; mavlink::MAX_FRAME_SIZE],
    pub len: usize,
    pub mavcan: bool,
}

impl Transfer {
    fn bytes(generation: u32, bytes: &[u8]) -> Self {
        let mut transfer = Self {
            generation,
            bytes: [0; mavlink::MAX_FRAME_SIZE],
            len: bytes.len(),
            mavcan: false,
        };
        transfer.bytes[..bytes.len()].copy_from_slice(bytes);
        transfer
    }
}

pub static USB_RX: Channel<CriticalSectionRawMutex, Packet, 4> = Channel::new();
pub static UART_RX: Channel<CriticalSectionRawMutex, Packet, 4> = Channel::new();
pub static USB_TX: Channel<CriticalSectionRawMutex, Transfer, 4> = Channel::new();
pub static UART_TX: Channel<CriticalSectionRawMutex, Transfer, 4> = Channel::new();

static USB_GENERATION: AtomicU32 = AtomicU32::new(0);
static USB_CONNECTED: AtomicBool = AtomicBool::new(false);
static UART_CONNECTED: AtomicBool = AtomicBool::new(false);
static TX_ERRORS: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];
static RX_ERRORS: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];
static PUBLISH_DROPS: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];

fn link_index(link: Link) -> usize {
    usize::from(link == Link::Uart)
}

pub fn record_tx_error(link: Link) {
    TX_ERRORS[link_index(link)].fetch_add(1, Ordering::Relaxed);
}
pub fn record_rx_error(link: Link) {
    RX_ERRORS[link_index(link)].fetch_add(1, Ordering::Relaxed);
}

pub fn write_diagnostics(output: &mut impl fmt::Write) -> fmt::Result {
    for (index, name) in ["USB", "UART7"].into_iter().enumerate() {
        writeln!(
            output,
            "MAVLINK_ROUTER link={name} tx_errors={} rx_errors={} publish_drops={}",
            TX_ERRORS[index].load(Ordering::Relaxed),
            RX_ERRORS[index].load(Ordering::Relaxed),
            PUBLISH_DROPS[index].load(Ordering::Relaxed)
        )?;
    }
    Ok(())
}

pub fn usb_generation() -> u32 {
    USB_GENERATION.load(Ordering::Acquire)
}

pub fn reset_usb() {
    USB_CONNECTED.store(false, Ordering::Release);
    USB_GENERATION.fetch_add(1, Ordering::AcqRel);
    USB_RX.clear();
    USB_TX.clear();
}

pub fn connected(link: Link, value: bool) {
    match link {
        Link::Usb => USB_CONNECTED.store(value, Ordering::Release),
        Link::Uart => UART_CONNECTED.store(value, Ordering::Release),
    }
}

fn rx(link: Link) -> &'static Channel<CriticalSectionRawMutex, Packet, 4> {
    match link {
        Link::Usb => &USB_RX,
        Link::Uart => &UART_RX,
    }
}

fn tx(link: Link) -> &'static Channel<CriticalSectionRawMutex, Transfer, 4> {
    match link {
        Link::Usb => &USB_TX,
        Link::Uart => &UART_TX,
    }
}

async fn log_work_ready(link: Link, connected: bool) {
    if !connected {
        pending::<()>().await;
        return;
    }
    crate::sd_logger::log_response_ready(link).await;
    // A queued response cannot make progress while TX is full. Wait for space
    // instead of repeatedly waking on the already nonempty response queue.
    if tx(link).is_full() {
        poll_fn(|context| tx(link).poll_ready_to_send(context)).await;
    }
}

/// Give ACKs a bounded chance to leave, then recover even if the host closed CDC.
async fn complete_action(action: Action) {
    match action {
        Action::None => {}
        Action::Bootloader => {
            Timer::after_millis(100).await;
            crate::fault::reboot_to_bootloader();
        }
        #[cfg(feature = "fault-injection")]
        Action::InjectFault { hard } => {
            Timer::after_millis(100).await;
            if hard {
                cortex_m::asm::udf();
            }
            panic!("P1 panic injection");
        }
    }
}

struct ConsoleReply {
    response: Response,
    offset: usize,
    shell: Option<MavHeader>,
}

struct ShellPacket {
    header: MavHeader,
    bytes: [u8; 70],
    offset: usize,
    len: usize,
}

struct State {
    link: Link,
    generation: u32,
    connected: bool,
    parser: FrameParser,
    raw_line: CommandLine,
    shell_line: CommandLine,
    shell_packet: Option<ShellPacket>,
    last_byte: u64,
    packet: Option<Packet>,
    offset: usize,
    ftp: mavftp::Server,
    scheduler: Scheduler,
    reply: Reply,
    console: Option<ConsoleReply>,
    raw_ack: Option<&'static [u8]>,
    action: Action,
    param_stream: ParamStream,
    param_due: u64,
    no_fix_due: u64,
    sequence: [u8; 2],
    publication: Option<Outgoing>,
}

impl State {
    fn new(link: Link, parameters: &ParameterManager<'_>, now: u64) -> Self {
        Self {
            link,
            generation: 0,
            connected: false,
            parser: FrameParser::new(),
            raw_line: CommandLine::new(),
            shell_line: CommandLine::new(),
            shell_packet: None,
            last_byte: now,
            packet: None,
            offset: 0,
            ftp: mavftp::Server::new(),
            scheduler: Scheduler::new(&parameters.snapshot(), now),
            reply: Reply::EMPTY,
            console: None,
            raw_ack: None,
            action: Action::None,
            param_stream: ParamStream::default(),
            param_due: now,
            no_fix_due: now,
            sequence: [0; 2],
            publication: None,
        }
    }

    fn sync_session(&mut self, parameters: &ParameterManager<'_>, now: u64) {
        let (generation, connected) = match self.link {
            Link::Usb => (usb_generation(), USB_CONNECTED.load(Ordering::Acquire)),
            Link::Uart => (0, UART_CONNECTED.load(Ordering::Acquire)),
        };
        if self.generation == generation && self.connected == connected {
            return;
        }
        // Stop an owned download on disconnect, but never cancel the other link.
        if self.connected
            && crate::sd_logger::snapshot().status == v6x_rust_led::logger::Status::Managing
        {
            let _ = crate::sd_logger::request_logs(self.link, LogRequest::End);
        }
        tx(self.link).clear();
        *self = Self::new(self.link, parameters, now);
        self.generation = generation;
        self.connected = connected;
        if connected {
            self.reply.push(mavlink_core::autopilot_version());
        }
    }

    fn busy(&self) -> bool {
        self.reply.pending()
            || self.console.is_some()
            || self.raw_ack.is_some()
            || self.shell_packet.is_some()
            || !matches!(self.action, Action::None)
    }

    fn queue(&mut self, outgoing: Outgoing, mavcan: bool) {
        // The Router is the only producer; the consumer can only free space.
        assert!(!tx(self.link).is_full());
        let mut transfer = Transfer::bytes(self.generation, &[]);
        let index = usize::from(outgoing.component != mavlink_core::COMPONENT_ID);
        transfer.len = mavlink_core::encode_from(
            &mut self.sequence[index],
            outgoing.component,
            &outgoing.message,
            &mut transfer.bytes,
        );
        transfer.mavcan = mavcan;
        assert!(tx(self.link).try_send(transfer).is_ok());
    }

    fn pump(&mut self, parameters: &ParameterManager<'_>, now: u64) {
        // At most one bounded queue's worth per pass, even at a requested high rate.
        for _ in 0..4 {
            if !self.connected || tx(self.link).is_full() {
                break;
            }
            if let Some(outgoing) = self.reply.pop() {
                self.queue(outgoing, false);
                continue;
            }
            if let Some(bytes) = self.raw_ack.take() {
                assert!(tx(self.link)
                    .try_send(Transfer::bytes(self.generation, bytes))
                    .is_ok());
                continue;
            }
            if let Some(console) = &mut self.console {
                let bytes = console.response.as_bytes();
                let size = if console.shell.is_some() {
                    70
                } else {
                    mavlink::MAX_FRAME_SIZE
                };
                let end = (console.offset + size).min(bytes.len());
                if let Some(header) = console.shell {
                    let message = mavlink_core::serial_control_reply(
                        header,
                        &bytes[console.offset..end],
                        end < bytes.len(),
                    );
                    console.offset = end;
                    if end == bytes.len() {
                        self.console = None;
                    }
                    self.queue(
                        Outgoing {
                            component: mavlink_core::COMPONENT_ID,
                            message,
                        },
                        false,
                    );
                } else {
                    let transfer = Transfer::bytes(self.generation, &bytes[console.offset..end]);
                    console.offset = end;
                    if end == bytes.len() {
                        self.console = None;
                    }
                    assert!(tx(self.link).try_send(transfer).is_ok());
                }
                continue;
            }
            if let Some(outgoing) = self.publication.take() {
                self.queue(outgoing, false);
                continue;
            }
            if let Some(id) = self.param_stream.peek().filter(|_| now >= self.param_due) {
                self.queue(
                    Outgoing {
                        component: mavlink_core::COMPONENT_ID,
                        message: mavlink_core::parameter(parameters, id),
                    },
                    false,
                );
                self.param_stream.advance();
                self.param_due = now.saturating_add(25_000);
                continue;
            }
            let mut produced = false;
            // Missing sensor data skips a beat, never causes a busy wait.
            for _ in 0..v6x_rust_led::telemetry::COUNT {
                let Some(kind) = self.scheduler.take_due(now) else {
                    break;
                };
                if let Some(outgoing) = telemetry::build(kind, now, &mut self.no_fix_due, false) {
                    self.queue(outgoing, false);
                    produced = true;
                    break;
                }
            }
            if produced {
                continue;
            }
            if let Some(response) = crate::sd_logger::try_next_log_response(self.link) {
                if let Some(message) = mavlink_core::log_response(response) {
                    self.queue(
                        Outgoing {
                            component: mavlink_core::COMPONENT_ID,
                            message,
                        },
                        false,
                    );
                }
                continue;
            }
            break;
        }
    }

    async fn console(
        &mut self,
        command: Command,
        shell: Option<MavHeader>,
        parameters: &ParameterManager<'_>,
        dronecan: &dronecan_can::State,
        phy: &RefCell<Lan8742<'_>>,
        eth_power: &Output<'_>,
    ) {
        match command {
            Command::Bootloader if shell.is_none() => {
                self.reply = Reply::EMPTY;
                self.action = Action::Bootloader;
                self.raw_ack = Some(b"OK bootloader\n");
            }
            #[cfg(feature = "fault-injection")]
            Command::Panic | Command::HardFault if shell.is_none() => {
                self.action = Action::InjectFault {
                    hard: matches!(command, Command::HardFault),
                };
                self.raw_ack = Some(b"OK fault-injection\n");
            }
            command => {
                let response = if shell.is_none() && matches!(command, Command::Status) {
                    usb_console::status(phy, eth_power)
                } else {
                    usb_console::mavlink_report(command, parameters, dronecan).await
                };
                self.console = Some(ConsoleReply {
                    response,
                    offset: 0,
                    shell,
                });
            }
        }
    }

    #[allow(deprecated)]
    async fn decoded(&mut self, decoded: Decoded, parameters: &ParameterManager<'_>) {
        if !addressed_to_vehicle(&decoded.message, mavlink_core::SYSTEM_ID) {
            return;
        }
        self.reply = Reply::EMPTY;
        if let Some(request) = mavlink_core::log_request(&decoded.message) {
            if crate::sd_logger::request_logs(self.link, request).is_err() {
                let error = if matches!(request, LogRequest::Erase)
                    && !erase_allowed(
                        crate::sd_logger::snapshot().status,
                        crate::OUTPUT_STATE.armed,
                        crate::OUTPUT_STATE.outputs_enabled,
                    ) {
                    LogError::EraseDenied
                } else {
                    LogError::Busy
                };
                // Busy/failed listing must not masquerade as an empty SD card.
                self.reply
                    .push(mavlink_core::log_response(LogResponse::Error(error)).unwrap());
            }
            return;
        }
        if self.link == Link::Usb {
            match crate::mavcan::handle_message(decoded.header, &decoded.message) {
                crate::mavcan::Outcome::ForwardAck(result) => {
                    self.reply.push(mavlink_core::command_ack(
                        &decoded.header,
                        MavCmd::MAV_CMD_CAN_FORWARD,
                        result,
                    ));
                    return;
                }
                crate::mavcan::Outcome::Handled => return,
                crate::mavcan::Outcome::Unhandled => {}
            }
        }
        if let MavMessage::SERIAL_CONTROL(data) = decoded.message {
            if data.device == SerialControlDev::SERIAL_CONTROL_DEV_SHELL
                && matches!(data.target_component, 0 | mavlink_core::COMPONENT_ID)
            {
                self.shell_packet = Some(ShellPacket {
                    header: decoded.header,
                    bytes: data.data,
                    offset: 0,
                    len: usize::from(data.count.min(70)),
                });
            }
            return;
        }
        mavlink_core::handle(
            decoded.header,
            decoded.message,
            parameters,
            &mut self.scheduler,
            &mut self.ftp,
            &mut self.reply,
        )
        .await;
        if self.reply.param_stream {
            self.param_stream.start();
            self.param_due = 0;
        }
        if self.reply.reboot {
            self.action = Action::Bootloader;
        }
    }

    async fn receive(
        &mut self,
        parameters: &ParameterManager<'_>,
        dronecan: &dronecan_can::State,
        phy: &RefCell<Lan8742<'_>>,
        eth_power: &Output<'_>,
    ) {
        if !self.connected
            || self.reply.pending()
            || self.console.is_some()
            || self.raw_ack.is_some()
            || !matches!(self.action, Action::None)
        {
            return;
        }
        // Preserve additional shell commands in a single SERIAL_CONTROL payload.
        if self.shell_packet.is_some() {
            loop {
                let shell = self.shell_packet.as_mut().unwrap();
                if shell.offset == shell.len {
                    self.shell_packet = None;
                    return;
                }
                let byte = shell.bytes[shell.offset];
                let header = shell.header;
                shell.offset += 1;
                if let Some(command) = self.shell_line.push(byte) {
                    self.console(command, Some(header), parameters, dronecan, phy, eth_power)
                        .await;
                    return;
                }
            }
        }
        if self.packet.is_none() {
            let Ok(packet) = rx(self.link).try_receive() else {
                return;
            };
            if packet.generation != self.generation {
                return;
            }
            if packet.len == 0 {
                self.parser = FrameParser::new();
                self.raw_line = CommandLine::new();
                return;
            }
            if self.parser.receiving() && packet.received.saturating_sub(self.last_byte) > 1_000_000
            {
                self.parser = FrameParser::new();
                self.raw_line = CommandLine::new();
            }
            self.last_byte = packet.received;
            self.packet = Some(packet);
            self.offset = 0;
        }
        for _ in 0..256 {
            let packet = self.packet.as_ref().unwrap();
            if self.offset >= packet.len {
                self.packet = None;
                break;
            }
            let byte = packet.bytes[self.offset];
            self.offset += 1;
            if self.parser.receiving() || matches!(byte, 0xfe | 0xfd) {
                self.raw_line = CommandLine::new();
                if let Some(decoded) = self.parser.push(byte) {
                    self.decoded(decoded, parameters).await;
                }
            } else if self.link == Link::Usb {
                if let Some(command) = self.raw_line.push(byte) {
                    self.console(command, None, parameters, dronecan, phy, eth_power)
                        .await;
                }
            }
            if self.busy() {
                break;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn run(
    parameters: &'static ParameterManager<'static>,
    dronecan: &'static dronecan_can::State,
    phy: &'static RefCell<Lan8742<'static>>,
    eth_power: &'static Output<'static>,
) -> ! {
    let now = Instant::now().as_micros();
    let mut states = [
        State::new(Link::Usb, parameters, now),
        State::new(Link::Uart, parameters, now),
    ];
    let mut last_report = crate::estimator::report();
    loop {
        let now = Instant::now();
        let mut recovery = None;
        let report = crate::estimator::report();
        if report != last_report && telemetry::publish(mavlink_core::status_text()).is_ok() {
            last_report = report;
        }
        for state in &mut states {
            state.sync_session(parameters, now.as_micros());
            state
                .scheduler
                .refresh(&parameters.snapshot(), now.as_micros());
            state.pump(parameters, now.as_micros());
            state.receive(parameters, dronecan, phy, eth_power).await;
            let action = core::mem::take(&mut state.action);
            if !matches!(action, Action::None) && recovery.is_none() {
                recovery = Some(action);
            }
            state.pump(parameters, Instant::now().as_micros());
        }
        if let Some(action) = recovery {
            complete_action(action).await;
        }
        if let Ok(outgoing) = telemetry::PUBLISHED.try_receive() {
            for state in &mut states {
                if state.connected {
                    // Best-effort live publication: an unresponsive link retains
                    // the newest value, without holding back the other link.
                    if state.publication.is_some() {
                        PUBLISH_DROPS[link_index(state.link)].fetch_add(1, Ordering::Relaxed);
                    }
                    state.publication = Some(Outgoing {
                        component: outgoing.component,
                        message: outgoing.message.clone(),
                    });
                }
            }
        }
        if states[0].connected && !USB_TX.is_full() {
            if let Ok(queued) = crate::mavcan::OUTBOUND.try_receive() {
                if let Some(message) = crate::mavcan::outbound_message(queued) {
                    states[0].queue(
                        Outgoing {
                            component: mavlink_core::COMPONENT_ID,
                            message,
                        },
                        true,
                    );
                }
            }
        }
        select3(
            Timer::at(now + Duration::from_millis(5)),
            log_work_ready(Link::Usb, states[0].connected),
            log_work_ready(Link::Uart, states[1].connected),
        )
        .await;
    }
}
