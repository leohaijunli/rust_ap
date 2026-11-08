//! Transport-agnostic MAVLink message logic shared by the USB CDC and TELEM1 links.

use mavlink::common::*;
use mavlink::MavHeader;

use v6x_rust_led::mavftp;
use v6x_rust_led::parameters::{Id, Kind, COUNT};
use v6x_rust_led::px4_messages::SensorGps;

use crate::parameter_manager::ParameterManager;
use crate::telemetry;
use v6x_rust_led::logger::{LogError, LogRequest, LogResponse};
use v6x_rust_led::{
    mavlink_protocol::addressed_to_vehicle,
    telemetry::{message_id, requested_kind, IntervalError, Scheduler},
};

pub const SYSTEM_ID: u8 = Id::MavSysId.definition().default as u8;
pub const COMPONENT_ID: u8 = v6x_rust_led::telemetry::MAIN_COMPONENT;

/// Replies produced for one inbound request, plus whether a reboot is pending
/// or a full parameter download was requested.
pub struct Reply {
    messages: [Option<telemetry::Outgoing>; Self::CAPACITY],
    count: usize,
    next: usize,
    pub reboot: bool,
    pub param_stream: bool,
}

impl Reply {
    /// One burst of FTP frames must fit without being dropped.
    pub const CAPACITY: usize = 4;

    pub const EMPTY: Self = Self {
        messages: [const { None }; Self::CAPACITY],
        count: 0,
        next: 0,
        reboot: false,
        param_stream: false,
    };

    pub fn push(&mut self, message: MavMessage) {
        self.push_from(COMPONENT_ID, message);
    }

    pub fn push_from(&mut self, component: u8, message: MavMessage) {
        assert!(self.count < Self::CAPACITY, "reply exceeds bounded burst");
        self.messages[self.count] = Some(telemetry::Outgoing { component, message });
        self.count += 1;
    }

    pub fn pending(&self) -> bool {
        self.next < self.count
    }

    pub fn pop(&mut self) -> Option<telemetry::Outgoing> {
        if !self.pending() {
            return None;
        }
        let outgoing = self.messages[self.next].take();
        self.next += 1;
        outgoing
    }
}

const _: () = assert!(mavftp::BURST_FRAMES <= Reply::CAPACITY);

/// Handle one inbound message, filling `reply` with the messages to send back.
// Keep PING and command 520 for ground stations using the older discovery flow.
#[allow(deprecated)]
pub async fn handle(
    header: MavHeader,
    message: MavMessage,
    parameters: &ParameterManager<'_>,
    scheduler: &mut Scheduler,
    ftp: &mut mavftp::Server,
    reply: &mut Reply,
) {
    if !addressed_to_vehicle(&message, SYSTEM_ID) {
        return;
    }

    match message {
        MavMessage::FILE_TRANSFER_PROTOCOL(data) => {
            if data.target_network == 0 {
                let mut responses = mavftp::Responses::new();
                ftp.handle(&data.payload, &mut responses);
                for frame in responses.frames() {
                    reply.push(file_transfer(header, frame));
                }
            }
        }
        MavMessage::PARAM_REQUEST_LIST(_) => {
            // The registry is too large for one Reply, so mark the request; the
            // calling link arms its own cursor and drains one value per tick.
            reply.param_stream = true;
        }
        MavMessage::PARAM_REQUEST_READ(data) => {
            let id = if data.param_index >= 0 {
                Id::from_index(data.param_index as usize)
            } else {
                data.param_id.to_str().ok().and_then(Id::from_name)
            };
            if let Some(id) = id {
                reply.push(parameter(parameters, id));
            }
        }
        MavMessage::PARAM_SET(data) => {
            if let Some(id) = data.param_id.to_str().ok().and_then(Id::from_name) {
                let value = if data.param_type == MavParamType::MAV_PARAM_TYPE_INT32 {
                    // Ground stations send the integer as its raw bit pattern.
                    (data.param_value.to_bits() as i32) as f32
                } else {
                    data.param_value
                };
                let outcome = if matches!(
                    data.param_type,
                    MavParamType::MAV_PARAM_TYPE_REAL32 | MavParamType::MAV_PARAM_TYPE_INT32
                ) {
                    parameters.set(id, value).map_err(|error| error.message())
                } else {
                    Err("PARAM_SET: expected REAL32 or INT32")
                };
                reply.push(parameter(parameters, id));
                match outcome {
                    Ok(true) => {
                        // PX4 accepts and persists any registered EKF2
                        // parameter. Defer the commit so a batch of sets costs
                        // one FRAM write instead of one per value; only
                        // disarmed vehicles with outputs closed may write.
                        if crate::OUTPUT_STATE.armed || crate::OUTPUT_STATE.outputs_enabled {
                            reply.push(parameter_warning("PARAM_SET: RAM only; outputs active"));
                        } else {
                            parameters.defer_save();
                        }
                    }
                    Ok(false) => {}
                    Err(error) => reply.push(parameter_warning(error)),
                }
            }
        }
        MavMessage::PING(data) if data.target_system == 0 => {
            reply.push(MavMessage::PING(PING_DATA {
                time_usec: data.time_usec,
                seq: data.seq,
                target_system: header.system_id,
                target_component: header.component_id,
            }));
        }
        MavMessage::TIMESYNC(data) if data.tc1 == 0 => {
            reply.push(MavMessage::TIMESYNC(TIMESYNC_DATA {
                tc1: (embassy_time::Instant::now().as_micros() * 1000) as i64,
                ts1: data.ts1,
                ..Default::default()
            }));
        }
        MavMessage::SYSTEM_TIME(data) => {
            // QGC broadcasts UTC once a second. Only a plausible UNIX time is
            // accepted, and equal values are skipped inside the RTC driver.
            crate::rtc::set_unix_usec(data.time_unix_usec);
        }
        MavMessage::COMMAND_LONG(data) if data.command == MavCmd::MAV_CMD_PREFLIGHT_STORAGE => {
            let result = if data.param1 == 1.0 {
                parameters.save(crate::OUTPUT_STATE).await.map(|_| ())
            } else if data.param1 == 0.0 {
                parameters.load().await.map(|_| ())
            } else {
                reply.push(command_ack(
                    &header,
                    data.command,
                    MavResult::MAV_RESULT_UNSUPPORTED,
                ));
                return;
            };
            let ack = if result.is_ok() {
                MavResult::MAV_RESULT_ACCEPTED
            } else {
                MavResult::MAV_RESULT_FAILED
            };
            reply.push(command_ack(&header, data.command, ack));
            if result.is_err() {
                reply.push(parameter_warning("Parameter storage failed; see !params"));
            }
        }
        MavMessage::COMMAND_LONG(data) if data.command == MavCmd::MAV_CMD_LOGGING_START => {
            reply.push(command_ack(
                &header,
                data.command,
                logger_result(crate::sd_logger::request_start()),
            ));
        }
        MavMessage::COMMAND_LONG(data) if data.command == MavCmd::MAV_CMD_LOGGING_STOP => {
            reply.push(command_ack(
                &header,
                data.command,
                logger_result(crate::sd_logger::request_stop()),
            ));
        }
        MavMessage::COMMAND_LONG(data) if data.command == MavCmd::MAV_CMD_REQUEST_MESSAGE => {
            let result = match message_id(data.param1) {
                None => MavResult::MAV_RESULT_DENIED,
                Some(id) => match requested_kind(id, data.target_component) {
                    None => MavResult::MAV_RESULT_UNSUPPORTED,
                    Some(kind) => {
                        let mut no_fix_due = 0;
                        match telemetry::build(
                            kind,
                            embassy_time::Instant::now().as_micros(),
                            &mut no_fix_due,
                            true,
                        ) {
                            Some(outgoing) => {
                                reply.push_from(outgoing.component, outgoing.message);
                                MavResult::MAV_RESULT_ACCEPTED
                            }
                            None => MavResult::MAV_RESULT_TEMPORARILY_REJECTED,
                        }
                    }
                },
            };
            reply.push(command_ack(&header, data.command, result));
        }
        MavMessage::COMMAND_LONG(data) if data.command == MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL => {
            let result = match message_id(data.param1)
                .ok_or(IntervalError::Invalid)
                .and_then(|id| {
                    scheduler.set_interval(
                        id,
                        data.target_component,
                        data.param2,
                        &parameters.snapshot(),
                        embassy_time::Instant::now().as_micros(),
                    )
                }) {
                Ok(()) => MavResult::MAV_RESULT_ACCEPTED,
                Err(IntervalError::Invalid) => MavResult::MAV_RESULT_DENIED,
                Err(IntervalError::Unsupported) => MavResult::MAV_RESULT_UNSUPPORTED,
            };
            reply.push(command_ack(&header, data.command, result));
        }
        MavMessage::COMMAND_LONG(data) if data.command == MavCmd::MAV_CMD_GET_MESSAGE_INTERVAL => {
            let result = match message_id(data.param1) {
                None => MavResult::MAV_RESULT_DENIED,
                Some(id) => match scheduler.interval(id, data.target_component) {
                    Some(interval) => {
                        reply.push(message_interval(id as u16, interval));
                        MavResult::MAV_RESULT_ACCEPTED
                    }
                    None => MavResult::MAV_RESULT_UNSUPPORTED,
                },
            };
            reply.push(command_ack(&header, data.command, result));
        }
        MavMessage::COMMAND_INT(data)
            if matches!(
                data.command,
                MavCmd::MAV_CMD_COMPONENT_ARM_DISARM | MavCmd::MAV_CMD_DO_SET_MODE
            ) =>
        {
            reply.push(command_ack(
                &header,
                data.command,
                MavResult::MAV_RESULT_UNSUPPORTED,
            ));
        }
        MavMessage::COMMAND_LONG(data) => {
            let mut reboot = false;
            let response = match data.command {
                // Arming and mode changes are flight-control commands; they
                // stay refused until the P5a control stages exist (plan §8b).
                MavCmd::MAV_CMD_COMPONENT_ARM_DISARM | MavCmd::MAV_CMD_DO_SET_MODE => None,
                MavCmd::MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES => Some(autopilot_version()),
                MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN if data.param1 == 3.0 => {
                    reboot = true;
                    None
                }
                _ => None,
            };
            let supported = response.is_some() || reboot;
            if let Some(response) = response {
                reply.push(response);
            }
            reply.push(command_ack(
                &header,
                data.command,
                if supported {
                    MavResult::MAV_RESULT_ACCEPTED
                } else {
                    MavResult::MAV_RESULT_UNSUPPORTED
                },
            ));
            if reboot {
                reply.reboot = true;
            }
        }
        _ => {}
    }
}

pub fn heartbeat() -> MavMessage {
    v6x_rust_led::mavlink_vehicle::heartbeat(crate::OUTPUT_STATE.armed)
}

/// Periodic status line. It carries the estimator's actual state so a ground
/// station can tell a live EKF2 from a Mahony fallback or an idle worker
/// without a vendor-specific message. A fallback raises the severity.
pub fn status_text() -> MavMessage {
    let (state, degraded) = crate::estimator::report().label();
    // A MAVLink STATUSTEXT is a 50-byte C string; this line stays far below it.
    let prefix = concat!("FMUv6X v", env!("CARGO_PKG_VERSION"), " ");
    assert!(prefix.len() + state.len() <= 50);
    let mut line = [0u8; 50];
    line[..prefix.len()].copy_from_slice(prefix.as_bytes());
    line[prefix.len()..prefix.len() + state.len()].copy_from_slice(state.as_bytes());
    MavMessage::STATUSTEXT(STATUSTEXT_DATA {
        severity: if degraded {
            MavSeverity::MAV_SEVERITY_WARNING
        } else {
            MavSeverity::MAV_SEVERITY_INFO
        },
        text: line.into(),
        ..Default::default()
    })
}

pub fn system_status() -> MavMessage {
    use v6x_rust_led::mavlink_vehicle::{self, SensorState, Sensors};

    let (gyro, accel) = crate::imu::health();
    let barometer = crate::barometer::health();
    let magnetometer = crate::magnetometer::health();
    let gps = crate::dronecan_can::latest_gps();
    let gps = mavlink_vehicle::gps_health(gps, embassy_time::Instant::now().as_micros());
    let output = crate::estimator::latest();
    // latest() rejects old epochs/stale output; AHRS health is not navigation qualification.
    let ahrs = SensorState {
        present: true,
        enabled: crate::estimator::report().requested,
        healthy: output.is_some_and(|output| {
            output.euler.is_some()
                && output
                    .snapshot
                    .angular_velocity
                    .is_some_and(|value| value.xyz.iter().all(|v| v.is_finite()))
        }),
    };
    mavlink_vehicle::system_status(
        Sensors {
            gyro,
            accel,
            barometer,
            magnetometer,
            gps,
            ahrs,
        },
        crate::diagnostics::cpu_load(),
    )
}

pub fn system_time() -> MavMessage {
    MavMessage::SYSTEM_TIME(SYSTEM_TIME_DATA {
        // 0 means "UTC unknown"; QGC then falls back to boot time.
        time_unix_usec: crate::rtc::valid_unix().map_or(0, |seconds| seconds * 1_000_000),
        time_boot_ms: embassy_time::Instant::now().as_millis() as u32,
    })
}

pub fn highres_imu() -> Option<MavMessage> {
    let sample = crate::imu::latest()?;
    let now = embassy_time::Instant::now().as_micros();
    if sample.is_stale(now, v6x_rust_led::icm42688::Config::DEFAULT) {
        return None;
    }
    let accel_frd = sample.accel_frd();
    let gyro_frd = sample.gyro_frd();
    let mut fields_updated = HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XACC
        | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_YACC
        | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_ZACC
        | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XGYRO
        | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_YGYRO
        | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_ZGYRO
        | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_TEMPERATURE;
    let baro = crate::barometer::latest();
    if baro.is_some() {
        fields_updated |= HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_ABS_PRESSURE;
        fields_updated |= HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_PRESSURE_ALT;
    }
    let mut data = HIGHRES_IMU_DATA {
        time_usec: sample.sample_timestamp_us,
        xacc: accel_frd[0],
        yacc: accel_frd[1],
        zacc: accel_frd[2],
        xgyro: gyro_frd[0],
        ygyro: gyro_frd[1],
        zgyro: gyro_frd[2],
        temperature: sample.temperature_c,
        abs_pressure: baro.map_or(f32::NAN, |value| value.pressure_pa / 100.0),
        diff_pressure: f32::NAN,
        pressure_alt: baro.map_or(f32::NAN, |value| value.pressure_altitude_m()),
        fields_updated,
        id: v6x_rust_led::icm42688::SOURCE_ID,
        ..Default::default()
    };
    v6x_rust_led::mavlink_magnetometer::update_highres_imu(
        &mut data,
        crate::magnetometer::latest(),
        now,
    );
    Some(MavMessage::HIGHRES_IMU(data))
}

/// Keep barometer temperature separate from the IMU temperature. This standard
/// stream carries absolute hPa and centi-degrees C; no differential sensor.
pub fn scaled_pressure() -> Option<MavMessage> {
    let sample = crate::barometer::latest()?;
    Some(MavMessage::SCALED_PRESSURE(SCALED_PRESSURE_DATA {
        time_boot_ms: (sample.timestamp_sample / 1_000) as u32,
        press_abs: sample.pressure_pa / 100.0,
        press_diff: f32::NAN,
        temperature: (sample.temperature_c * 100.0) as i16,
        temperature_press_diff: 0,
    }))
}

/// Which estimator message to build; see [`estimator_message`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EstimatorMessage {
    AttitudeEuler,
    AttitudeQuaternion,
    LocalPosition,
    GlobalPosition,
    Status,
}

/// Build one main-estimator stream message.
/// Messages whose data is invalid or stale are suppressed, never reported
/// with fabricated fields.
pub fn estimator_message(kind: EstimatorMessage) -> Option<MavMessage> {
    let output = crate::estimator::latest()?;
    let snapshot = output.snapshot;
    match kind {
        EstimatorMessage::AttitudeEuler => output
            .euler
            .zip(snapshot.angular_velocity)
            .map(|(angles, gyro)| v6x_rust_led::mavlink_estimator::attitude_euler(angles, gyro)),
        EstimatorMessage::AttitudeQuaternion => snapshot
            .attitude
            .zip(snapshot.angular_velocity)
            .map(|(attitude, gyro)| v6x_rust_led::mavlink_estimator::attitude(attitude, gyro)),
        EstimatorMessage::LocalPosition => {
            v6x_rust_led::mavlink_estimator::local_position(snapshot.local_position)
        }
        EstimatorMessage::GlobalPosition => snapshot.global_position.and_then(|global| {
            v6x_rust_led::mavlink_estimator::global_position(global, snapshot.local_position, None)
        }),
        EstimatorMessage::Status => Some(v6x_rust_led::mavlink_estimator::estimator_status(
            snapshot.status,
        )),
    }
}

/// BATTERY_STATUS mirrors the INA219 power monitor: voltage, current and power
/// are measured, but there is no coulomb counter, so consumption and remaining
/// capacity stay unknown instead of being fabricated.
pub fn battery_status() -> Option<MavMessage> {
    let measurement = crate::ina219::latest()?;
    v6x_rust_led::mavlink_power::battery_status(
        measurement,
        embassy_time::Instant::now().as_micros(),
    )
}

pub fn autopilot_version() -> MavMessage {
    let uid = embassy_stm32::uid::uid();
    let mut uid2 = [0u8; 18];
    uid2[..uid.len()].copy_from_slice(&uid);
    MavMessage::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
        capabilities: v6x_rust_led::mavlink_vehicle::CAPABILITIES,
        flight_sw_version: v6x_rust_led::build_info::FLIGHT_SW_VERSION,
        flight_custom_version: v6x_rust_led::build_info::SOURCE_ID.to_le_bytes(),
        vendor_id: 0x1209,
        product_id: 0x0001,
        uid: u64::from_le_bytes(uid[..8].try_into().unwrap()),
        uid2,
        ..Default::default()
    })
}

pub fn ina219_status() -> Option<MavMessage> {
    let measurement = crate::ina219::latest()?;
    // One standard debug message keeps all three SI values visible together
    // even in QGC versions which merge named scalar messages in Inspector.
    Some(MavMessage::DEBUG_VECT(DEBUG_VECT_DATA {
        time_usec: measurement.timestamp_us,
        name: "INA219".into(),
        x: measurement.bus_voltage_v,
        y: measurement.current_a,
        z: measurement.power_w,
    }))
}

/// PX4 GPS_RAW_INT.hpp conversion from the sensor_gps snapshot.
pub fn gps_raw_int() -> Option<MavMessage> {
    let gps = crate::dronecan_can::latest_gps()?;
    let now = embassy_time::Instant::now().as_micros();
    if now.saturating_sub(gps.timestamp) > v6x_rust_led::gps::GPS_FRESH_US {
        return None;
    }
    Some(gps_raw_int_from(&gps))
}

/// Fresh snapshot -> GPS_RAW_INT fields, exactly as the PX4 stream.
fn gps_raw_int_from(gps: &SensorGps) -> MavMessage {
    MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
        time_usec: gps.timestamp,
        fix_type: gps_fix_type(gps.fix_type),
        lat: rounded_degrees_1e7(gps.latitude_deg),
        lon: rounded_degrees_1e7(gps.longitude_deg),
        alt: rounded_millimeters(gps.altitude_msl_m),
        // PX4: eph/epv carry DOP * 100 (unitless); h/v_acc carry metres * 1e3.
        eph: dop_100(gps.hdop),
        epv: dop_100(gps.vdop),
        vel: centi(gps.vel_m_s).unwrap_or(u16::MAX),
        cog: centi(wrap_2pi_degrees(gps.cog_rad)).unwrap_or(u16::MAX),
        satellites_visible: gps.satellites_used,
        alt_ellipsoid: rounded_millimeters(gps.altitude_ellipsoid_m),
        // PX4 multiplies by 1e3 and lets the C cast saturate; 0 = unknown here.
        h_acc: mm_from_metres(gps.eph),
        v_acc: mm_from_metres(gps.epv),
        vel_acc: mm_from_metres(gps.s_variance_m_s),
        ..Default::default()
    })
}

/// PX4 sends this once per second after data stops arriving.
pub fn gps_raw_int_no_fix() -> MavMessage {
    MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
        fix_type: GpsFixType::GPS_FIX_TYPE_NO_GPS,
        eph: u16::MAX,
        epv: u16::MAX,
        vel: u16::MAX,
        cog: u16::MAX,
        satellites_visible: u8::MAX,
        ..Default::default()
    })
}

fn rounded_degrees_1e7(value: f64) -> i32 {
    let scaled = libm::round(value * 1e7);
    scaled.clamp(f64::from(i32::MIN), f64::from(i32::MAX)) as i32
}

fn rounded_millimeters(metres: f64) -> i32 {
    let scaled = libm::round(metres * 1e3);
    scaled.clamp(f64::from(i32::MIN), f64::from(i32::MAX)) as i32
}

/// PX4's C float->uint32 cast saturates negatives to 0 on ARM; 0 = unknown.
fn mm_from_metres(value: f32) -> u32 {
    if value.is_finite() && value > 0.0 {
        (value * 1e3) as u32
    } else {
        0
    }
}

/// common.xml GPS_FIX_TYPE; unknown values fall back to NO_GPS.
fn gps_fix_type(value: u8) -> GpsFixType {
    match value {
        1 => GpsFixType::GPS_FIX_TYPE_NO_FIX,
        2 => GpsFixType::GPS_FIX_TYPE_2D_FIX,
        3 => GpsFixType::GPS_FIX_TYPE_3D_FIX,
        4 => GpsFixType::GPS_FIX_TYPE_DGPS,
        5 => GpsFixType::GPS_FIX_TYPE_RTK_FLOAT,
        6 => GpsFixType::GPS_FIX_TYPE_RTK_FIXED,
        7 => GpsFixType::GPS_FIX_TYPE_STATIC,
        8 => GpsFixType::GPS_FIX_TYPE_PPP,
        _ => GpsFixType::GPS_FIX_TYPE_NO_GPS,
    }
}

/// C-like float->uint16 cast used by PX4; non-finite saturates to 0.
fn dop_100(dop: f32) -> u16 {
    if dop.is_finite() && dop > 0.0 {
        ((dop * 100.0).min(f32::from(u16::MAX))) as u16
    } else {
        0
    }
}

fn centi(value: f32) -> Option<u16> {
    if value.is_finite() {
        Some((value * 100.0).clamp(0.0, f32::from(u16::MAX)) as u16)
    } else {
        None
    }
}

fn wrap_2pi_degrees(cog_rad: f32) -> f32 {
    let two_pi = core::f32::consts::TAU;
    // rem_euclid equivalent without std: wrap to [0, 2pi).
    let mut wrapped = cog_rad;
    while wrapped < 0.0 {
        wrapped += two_pi;
    }
    while wrapped >= two_pi {
        wrapped -= two_pi;
    }
    wrapped / two_pi * 360.0
}

pub fn parameter(parameters: &ParameterManager<'_>, id: Id) -> MavMessage {
    let definition = id.definition();
    let value = parameters.get(id);
    // MAVLink carries the raw union: INT32 parameters travel as the bit pattern
    // of the integer, so a ground station reading `param_type == INT32` sees the
    // correct integer instead of a float bit-cast.
    let param_value = if definition.kind == Kind::Integer {
        f32::from_bits((value as i32) as u32)
    } else {
        value
    };
    MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
        param_id: definition.name.into(),
        param_value,
        param_type: if definition.kind == Kind::Integer {
            MavParamType::MAV_PARAM_TYPE_INT32
        } else {
            MavParamType::MAV_PARAM_TYPE_REAL32
        },
        param_count: COUNT as u16,
        param_index: id as u16,
    })
}

pub fn parameter_warning(text: &'static str) -> MavMessage {
    MavMessage::STATUSTEXT(STATUSTEXT_DATA {
        severity: MavSeverity::MAV_SEVERITY_WARNING,
        text: text.into(),
        ..Default::default()
    })
}

/// Echo of a successful `MAV_CMD_SET_MESSAGE_INTERVAL`, so the ground station
/// can read back the interval it just configured.
pub fn message_interval(message_id: u16, interval_us: i32) -> MavMessage {
    MavMessage::MESSAGE_INTERVAL(MESSAGE_INTERVAL_DATA {
        interval_us,
        message_id,
    })
}

/// One FTP frame addressed back to the requesting ground station.
pub fn file_transfer(header: MavHeader, frame: &mavftp::Frame) -> MavMessage {
    let mut payload = [0u8; mavftp::PAYLOAD_LEN];
    frame.encode(&mut payload);
    MavMessage::FILE_TRANSFER_PROTOCOL(FILE_TRANSFER_PROTOCOL_DATA {
        target_network: 0,
        target_system: header.system_id,
        target_component: header.component_id,
        payload,
    })
}

/// Advertise the static metadata file QGC fetches with MAVLink FTP.
pub fn component_metadata() -> MavMessage {
    MavMessage::COMPONENT_METADATA(COMPONENT_METADATA_DATA {
        time_boot_ms: embassy_time::Instant::now().as_millis() as u32,
        file_crc: v6x_rust_led::component_metadata::GENERAL_CRC,
        uri: v6x_rust_led::component_metadata::GENERAL_URI.into(),
    })
}

pub fn command_ack(header: &MavHeader, command: MavCmd, result: MavResult) -> MavMessage {
    MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
        command,
        result,
        target_system: header.system_id,
        target_component: header.component_id,
        ..Default::default()
    })
}

fn logger_result(result: Result<(), v6x_rust_led::logger::ControlError>) -> MavResult {
    match result {
        Ok(()) => MavResult::MAV_RESULT_ACCEPTED,
        Err(v6x_rust_led::logger::ControlError::InvalidState) => {
            MavResult::MAV_RESULT_TEMPORARILY_REJECTED
        }
        Err(_) => MavResult::MAV_RESULT_FAILED,
    }
}

pub fn serial_control_reply(header: MavHeader, bytes: &[u8], more: bool) -> MavMessage {
    let mut data = [0u8; 70];
    data[..bytes.len()].copy_from_slice(bytes);
    let mut flags = SerialControlFlag::SERIAL_CONTROL_FLAG_REPLY;
    if more {
        flags |= SerialControlFlag::SERIAL_CONTROL_FLAG_MULTI;
    }
    MavMessage::SERIAL_CONTROL(SERIAL_CONTROL_DATA {
        device: SerialControlDev::SERIAL_CONTROL_DEV_SHELL,
        flags,
        count: bytes.len() as u8,
        data,
        target_system: header.system_id,
        target_component: header.component_id,
        ..Default::default()
    })
}

pub fn log_request(message: &MavMessage) -> Option<LogRequest> {
    match message {
        MavMessage::LOG_REQUEST_LIST(data)
            if targeted(data.target_system, data.target_component) =>
        {
            Some(LogRequest::List {
                start: data.start,
                end: data.end,
            })
        }
        MavMessage::LOG_REQUEST_DATA(data)
            if targeted(data.target_system, data.target_component) =>
        {
            Some(LogRequest::Data {
                id: data.id,
                offset: data.ofs,
                count: data.count,
            })
        }
        MavMessage::LOG_REQUEST_END(data)
            if targeted(data.target_system, data.target_component) =>
        {
            Some(LogRequest::End)
        }
        MavMessage::LOG_ERASE(data) if targeted(data.target_system, data.target_component) => {
            Some(LogRequest::Erase)
        }
        _ => None,
    }
}

fn targeted(system: u8, component: u8) -> bool {
    matches!(system, 0 | SYSTEM_ID) && matches!(component, 0 | COMPONENT_ID)
}

pub fn log_response(response: LogResponse) -> Option<MavMessage> {
    match response {
        LogResponse::Entry {
            id,
            num_logs,
            last_log_num,
            size,
            time_utc,
        } => Some(MavMessage::LOG_ENTRY(LOG_ENTRY_DATA {
            time_utc,
            size,
            id,
            num_logs,
            last_log_num,
        })),
        LogResponse::Data {
            id,
            offset,
            count,
            data,
        } => Some(MavMessage::LOG_DATA(LOG_DATA_DATA {
            ofs: offset,
            id,
            count,
            data,
        })),
        LogResponse::Error(error) => Some(parameter_warning(log_error_text(error))),
        LogResponse::Erased { deleted } => Some(MavMessage::STATUSTEXT(STATUSTEXT_DATA {
            severity: MavSeverity::MAV_SEVERITY_INFO,
            text: v6x_rust_led::logger::erase_confirmation(deleted).into(),
            ..Default::default()
        })),
        LogResponse::Done => None,
    }
}

fn log_error_text(error: LogError) -> &'static str {
    match error {
        LogError::Busy => "Log service busy",
        LogError::Mount => "Log SD mount failed",
        LogError::Directory => "Log directory read failed",
        LogError::NotFound => "Log id not found",
        LogError::Range => "Log offset out of range",
        LogError::Read => "Log data read failed",
        LogError::EraseDenied => "Log erase denied: disarm and stop logging",
        LogError::Erase => "Log erase failed; refresh SD log list",
    }
}

pub fn encode_from(
    sequence: &mut u8,
    component_id: u8,
    message: &MavMessage,
    out: &mut [u8; mavlink::MAX_FRAME_SIZE],
) -> usize {
    let header = MavHeader {
        system_id: SYSTEM_ID,
        component_id,
        sequence: *sequence,
    };
    *sequence = sequence.wrapping_add(1);
    v6x_rust_led::mavlink_estimator::encode(header, message, out)
}
