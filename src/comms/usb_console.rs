use core::cell::RefCell;
use core::fmt::{self, Write};

use crate::dronecan_can;
use crate::eth_led::Lan8742;
use crate::fram;
use crate::parameter_manager::ParameterManager;
use embassy_stm32::gpio::Output;
use v6x_rust_led::parameter_store::{self, OutputState};
use v6x_rust_led::parameters::{schema_tag, Group, REGISTRY};

pub(crate) fn status(phy: &RefCell<Lan8742<'_>>, eth_power: &Output<'_>) -> Response {
    let mut response = Response::new();
    let pwr = embassy_stm32::pac::PWR.cr3().read();
    let _ = writeln!(
        response,
        "Rust FMUv6X v{} USB=MAVLink2 clock=PLL3Q/48MHz ZLP=on",
        env!("CARGO_PKG_VERSION")
    );
    let _ = writeln!(
        response,
        "USB33RDY={} ETH_POWER={}",
        pwr.usb33rdy(),
        eth_power.is_set_high()
    );
    let _ = crate::rtc::write_diagnostics(&mut response, "RTC");
    let _ = phy.borrow_mut().write_diagnostics(&mut response);
    response.finish();
    response
}

pub(crate) fn report(command: Command, dronecan: &dronecan_can::State) -> Response {
    let mut response = Response::new();
    let _ = match command {
        Command::Help => writeln!(
            response,
            "COMMANDS help status platform metrics fault fram params dronecan mavcan imu ina219 px4io logger logger start logger stop logger sync logger config <100..10000> time time set <unix|YYYY-MM-DDTHH:MM:SSZ>"
        ),
        Command::Platform => crate::platform::write_diagnostics(&mut response),
        Command::Metrics => crate::diagnostics::write_diagnostics(&mut response)
            .and_then(|()| crate::router::write_diagnostics(&mut response)),
        Command::MetricsReset => {
            crate::diagnostics::request_reset();
            writeln!(response, "OK metrics-reset-requested")
        }
        Command::Fault => crate::fault::write_diagnostics(&mut response),
        Command::FaultClear => writeln!(response, "FAULT_CLEAR ok={}", crate::fault::clear()),
        Command::DroneCan => dronecan.write_diagnostics(&mut response),
        Command::MavCan => crate::mavcan::write_diagnostics(&mut response),
        Command::Imu => crate::imu::write_diagnostics(&mut response),
        Command::Ina219 => crate::ina219::write_diagnostics(&mut response),
        Command::Baro => crate::barometer::write_diagnostics(&mut response),
        Command::Estimator => crate::estimator::write_diagnostics(&mut response),
        Command::Px4io => crate::px4io::write_diagnostics(&mut response),
        Command::Logger => crate::sd_logger::write_diagnostics(&mut response)
            .and_then(|()| crate::logger_switch::write_diagnostics(&mut response)),
        Command::LoggerStart => {
            logger_result(&mut response, "start", crate::sd_logger::request_start())
        }
        Command::LoggerStop => {
            logger_result(&mut response, "stop", crate::sd_logger::request_stop())
        }
        Command::LoggerSync => {
            logger_result(&mut response, "sync", crate::sd_logger::request_sync())
        }
        Command::LoggerConfig(sync_period_ms) => logger_result(
            &mut response,
            "config",
            crate::sd_logger::apply_config(v6x_rust_led::logger::Config { sync_period_ms }),
        ),
        Command::Time => crate::rtc::write_diagnostics(&mut response, "RTC"),
        Command::TimeSet(seconds) => {
            let ok = crate::rtc::set_unix_seconds(seconds);
            let _ = writeln!(response, "TIME_SET ok={ok} unix={seconds}");
            crate::rtc::write_diagnostics(&mut response, "RTC")
        }
        Command::TimeSetInvalid => {
            writeln!(response, "ERR time expects unix seconds or YYYY-MM-DDTHH:MM:SSZ")
        }
        _ => writeln!(response, "ERR unsupported-report"),
    };
    response.finish();
    response
}

pub(crate) async fn mavlink_report(
    command: Command,
    parameters: &ParameterManager<'_>,
    dronecan: &dronecan_can::State,
) -> Response {
    match command {
        Command::EstimatorStart | Command::EstimatorStop => estimator_report(command, parameters),
        Command::Fram | Command::FramRead(_) => fram_report(command, parameters).await,
        Command::Parameters | Command::ParamSave | Command::ParamLoad => {
            parameter_report(command, parameters, crate::OUTPUT_STATE).await
        }
        Command::Status => {
            let mut response = Response::new();
            let _ = crate::platform::write_diagnostics(&mut response);
            let _ = crate::rtc::write_diagnostics(&mut response, "RTC");
            let _ = crate::imu::write_diagnostics(&mut response);
            let _ = crate::sd_logger::write_diagnostics(&mut response);
            let _ = crate::logger_switch::write_diagnostics(&mut response);
            response.finish();
            response
        }
        Command::Bootloader => {
            let mut response = Response::new();
            let _ = writeln!(response, "ERR use MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN");
            response.finish();
            response
        }
        #[cfg(feature = "fault-injection")]
        Command::Panic | Command::HardFault => {
            let mut response = Response::new();
            let _ = writeln!(
                response,
                "ERR fault injection unavailable via MAVLink console"
            );
            response.finish();
            response
        }
        _ => report(command, dronecan),
    }
}

pub(crate) fn estimator_report(command: Command, parameters: &ParameterManager<'_>) -> Response {
    use v6x_rust_led::parameters::Id;
    let mut response = Response::new();
    let starting = matches!(command, Command::EstimatorStart);
    let result = (|| {
        if starting && (crate::OUTPUT_STATE.armed || crate::OUTPUT_STATE.outputs_enabled) {
            return Err(v6x_rust_led::parameters::Error::Configuration);
        }
        let mut changed = false;
        if starting {
            changed |= parameters.set(Id::EstAlgorithm, 1.0)?;
            changed |= parameters.set(Id::Ekf2En, 1.0)?;
        }
        changed |= parameters.set(Id::EstEnable, if starting { 1.0 } else { 0.0 })?;
        // set() already publishes each changed mapped parameter. Only an
        // unchanged explicit command needs another epoch to reset a fault.
        if !changed {
            crate::estimator::configure(parameters.snapshot())?;
        }
        Ok(())
    })();
    let _ = writeln!(
        response,
        "EST_CONTROL start={} accepted={} data_only=true persisted=false error={:?}",
        starting,
        result.is_ok(),
        result.err()
    );
    let _ = crate::estimator::write_diagnostics(&mut response);
    response.finish();
    response
}

fn logger_result(
    response: &mut Response,
    operation: &str,
    result: Result<(), v6x_rust_led::logger::ControlError>,
) -> fmt::Result {
    writeln!(
        response,
        "LOGGER_CONTROL operation={operation} accepted={} error={:?}",
        result.is_ok(),
        result.err()
    )
}

pub(crate) async fn fram_report(command: Command, parameters: &ParameterManager<'_>) -> Response {
    let mut response = Response::new();
    let result = match command {
        Command::Fram => match parameters.fram_info().await {
            Ok(info) => {
                let _ = write!(response, "FRAM schema=1 id=");
                for byte in info.id {
                    let _ = write!(response, "{byte:02x}");
                }
                let _ = writeln!(
                    response,
                    " supported={} bytes={} sr={:02x} mode=3 spi_hz={}",
                    info.supported,
                    if info.supported { fram::CAPACITY } else { 0 },
                    info.status,
                    info.spi_hz,
                );
                Ok(())
            }
            Err(error) => Err(error),
        },
        Command::FramRead(address) => {
            let mut data = [0; fram::CHUNK];
            match parameters.read_fram(address, &mut data).await {
                Ok(()) => {
                    let _ = write!(
                        response,
                        "FRAM_DATA addr={address:04x} len={} hex=",
                        data.len()
                    );
                    for byte in data {
                        let _ = write!(response, "{byte:02x}");
                    }
                    let _ = writeln!(response);
                    Ok(())
                }
                Err(error) => Err(error),
            }
        }
        _ => unreachable!(),
    };
    if let Err(error) = result {
        let _ = writeln!(response, "ERR fram={error:?}");
    }
    response.finish();
    response
}

pub(crate) async fn parameter_report(
    command: Command,
    parameters: &ParameterManager<'_>,
    outputs: OutputState,
) -> Response {
    let mut response = Response::new();
    match command {
        Command::Parameters => {
            let status = parameters.status();
            let _ = writeln!(response, "PARAMS schema=1 storage={:?} valid={} dirty={} pending_save={} slot={} generation={} format={} registry={:08x} slot_a={:?} slot_b={:?} last_error={:?}",
                status.load, status.active.is_some(), parameters.is_dirty(), parameters.save_pending(),
                status.active.map_or(-1, |active| active.slot as i32),
                status.active.map_or(0, |active| active.generation),
                parameter_store::FORMAT_VERSION, schema_tag(), status.slots[0], status.slots[1], status.last_error);
            let _ = write!(response, "PARAM_VALUES");
            for entry in REGISTRY.iter().filter(|entry| entry.group != Group::Ekf2) {
                let _ = write!(response, " {}={}", entry.name, parameters.get(entry.id));
            }
            let _ = writeln!(response);
            let _ = writeln!(
                response,
                "PARAM_CATALOG count={} EKF2_count=142 use=PARAM_REQUEST_READ_or_MAVFTP",
                REGISTRY.len()
            );
        }
        Command::ParamSave => match parameters.save(outputs).await {
            Ok(saved) => {
                let _ = writeln!(
                    response,
                    "PARAM_SAVE ok=true written={} slot={} generation={} dirty={}",
                    saved.written,
                    saved.active.slot as usize,
                    saved.active.generation,
                    parameters.is_dirty()
                );
            }
            Err(error) => {
                let _ = writeln!(response, "ERR param={error:?}");
            }
        },
        Command::ParamLoad => match parameters.load().await {
            Ok(state) => {
                let _ = writeln!(response, "PARAM_LOAD storage={state:?}");
            }
            Err(error) => {
                let _ = writeln!(response, "ERR param={error:?}");
            }
        },
        _ => unreachable!(),
    }
    response.finish();
    response
}

pub(crate) enum Command {
    Help,
    Status,
    Bootloader,
    Platform,
    Metrics,
    MetricsReset,
    Fault,
    FaultClear,
    Fram,
    FramRead(u16),
    Parameters,
    ParamSave,
    ParamLoad,
    DroneCan,
    MavCan,
    Imu,
    Ina219,
    Baro,
    Estimator,
    EstimatorStart,
    EstimatorStop,
    Px4io,
    Logger,
    LoggerStart,
    LoggerStop,
    LoggerSync,
    LoggerConfig(u32),
    Time,
    TimeSet(u64),
    TimeSetInvalid,
    #[cfg(feature = "fault-injection")]
    Panic,
    #[cfg(feature = "fault-injection")]
    HardFault,
}

pub(crate) struct CommandLine {
    bytes: [u8; 32],
    len: usize,
    overflow: bool,
}

impl CommandLine {
    pub(crate) fn new() -> Self {
        Self {
            bytes: [0; 32],
            len: 0,
            overflow: false,
        }
    }

    pub(crate) fn push(&mut self, byte: u8) -> Option<Command> {
        if matches!(byte, 0x08 | 0x7f) {
            self.len = self.len.saturating_sub(1);
            self.overflow = false;
            return None;
        }
        if matches!(byte, b'\n' | b'\r') {
            let line = &self.bytes[..self.len];
            let command = if self.overflow {
                None
            } else {
                match line {
                    b"help" | b"?" => Some(Command::Help),
                    b"!status" | b"status" => Some(Command::Status),
                    b"!bootloader" => Some(Command::Bootloader),
                    b"!platform" | b"platform" => Some(Command::Platform),
                    b"!metrics" | b"metrics" => Some(Command::Metrics),
                    b"!metrics-reset" => Some(Command::MetricsReset),
                    b"!fault" | b"fault" => Some(Command::Fault),
                    b"!fault-clear" => Some(Command::FaultClear),
                    b"!fram" | b"fram" => Some(Command::Fram),
                    b"!params" | b"params" => Some(Command::Parameters),
                    b"!param-save" => Some(Command::ParamSave),
                    b"!param-load" => Some(Command::ParamLoad),
                    b"!dronecan" | b"dronecan" => Some(Command::DroneCan),
                    b"!mavcan" | b"mavcan" => Some(Command::MavCan),
                    b"!imu" | b"imu" => Some(Command::Imu),
                    b"!ina219" | b"ina219" => Some(Command::Ina219),
                    b"!baro" | b"baro" => Some(Command::Baro),
                    b"!ekf2" | b"ekf2" => Some(Command::Estimator),
                    b"!ekf2 start" | b"ekf2 start" => Some(Command::EstimatorStart),
                    b"!ekf2 stop" | b"ekf2 stop" => Some(Command::EstimatorStop),
                    b"!px4io" | b"px4io" => Some(Command::Px4io),
                    b"!time" | b"time" => Some(Command::Time),
                    #[cfg(feature = "fault-injection")]
                    b"!panic" => Some(Command::Panic),
                    #[cfg(feature = "fault-injection")]
                    b"!hardfault" => Some(Command::HardFault),
                    _ => line
                        .strip_prefix(b"!fram-read ")
                        .filter(|address| address.len() == 4)
                        .and_then(|address| core::str::from_utf8(address).ok())
                        .and_then(|address| u16::from_str_radix(address, 16).ok())
                        .map(Command::FramRead)
                        .or_else(|| {
                            line.strip_prefix(b"!time set ").map(|value| {
                                core::str::from_utf8(value)
                                    .ok()
                                    .and_then(v6x_rust_led::wall_clock::parse_set_value)
                                    .map_or(Command::TimeSetInvalid, Command::TimeSet)
                            })
                        })
                        .or_else(|| match v6x_rust_led::logger::parse_console(line)? {
                            v6x_rust_led::logger::ConsoleCommand::Status => Some(Command::Logger),
                            v6x_rust_led::logger::ConsoleCommand::Start => {
                                Some(Command::LoggerStart)
                            }
                            v6x_rust_led::logger::ConsoleCommand::Stop => Some(Command::LoggerStop),
                            v6x_rust_led::logger::ConsoleCommand::Sync => Some(Command::LoggerSync),
                            v6x_rust_led::logger::ConsoleCommand::Config(config) => {
                                Some(Command::LoggerConfig(config.sync_period_ms))
                            }
                        }),
                }
            };
            self.len = 0;
            self.overflow = false;
            return command;
        }

        if self.len < self.bytes.len() {
            self.bytes[self.len] = byte;
            self.len += 1;
        } else {
            // Ignore an overlong line until its newline; never match a suffix.
            self.overflow = true;
        }
        None
    }
}

pub(crate) struct Response {
    bytes: [u8; 2048],
    len: usize,
    overflow: bool,
}

impl Response {
    fn new() -> Self {
        Self {
            bytes: [0; 2048],
            len: 0,
            overflow: false,
        }
    }

    pub(crate) fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    fn finish(&mut self) {
        let _ = writeln!(self, "END");
        if self.overflow {
            self.len = 0;
            self.overflow = false;
            let _ = self.write_str("ERR response-overflow\nEND\n");
        }
    }
}

impl Write for Response {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let end = self.len + text.len();
        if end > self.bytes.len() {
            self.overflow = true;
            return Err(fmt::Error);
        }
        self.bytes[self.len..end].copy_from_slice(text.as_bytes());
        self.len = end;
        Ok(())
    }
}
