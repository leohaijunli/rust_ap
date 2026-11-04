//! Bounded GNSS state from DroneCAN Fix2/Auxiliary, translated from the fixed
//! PX4 v1.17.0 UavcanGnssBridge (gnss.cpp) process_fixx flow. The snapshot
//! follows the sensor_gps contract (tools/px4_msgs/SensorGps.msg); GPS_RAW_INT
//! derives from it exactly as PX4's mavlink GPS_RAW_INT.hpp stream does.

use crate::dronecan::gnss::{GnssAuxiliary, GnssFix2};
use crate::px4_messages::SensorGps;

/// PX4 GPS_RAW_INT / sensor_gps fix_type values (GPS_FIX_TYPE).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum FixType {
    NoGps = 0,
    NoFix = 1,
    TwoD = 2,
    ThreeD = 3,
    Dgps = 4,
    RtkFloat = 5,
    RtkFixed = 6,
    Static = 7,
    Ppp = 8,
}

/// PX4 UavcanGnssBridge keeps auxiliary DOP for 2 s.
pub const AUXILIARY_FRESH_US: u64 = 2_000_000;

/// PX4 GPS_RAW_INT stream sends NO_GPS after 1 s without data.
pub const GPS_FRESH_US: u64 = 1_000_000;

/// Cached Auxiliary DOP, mirroring PX4's _last_gnss_auxiliary_* members.
#[derive(Clone, Copy, Debug, Default)]
pub struct AuxiliaryCache {
    seen: bool,
    hdop: f32,
    vdop: f32,
    received_at_us: u64,
}

impl AuxiliaryCache {
    pub const fn new() -> Self {
        Self {
            seen: false,
            hdop: 0.0,
            vdop: 0.0,
            received_at_us: 0,
        }
    }

    pub fn update(&mut self, aux: GnssAuxiliary, now_us: u64) {
        self.seen = true;
        self.hdop = aux.hdop;
        self.vdop = aux.vdop;
        self.received_at_us = now_us;
    }

    /// DOP from auxiliary when fresh, else Fix2 pdop (PX4 fallback rule).
    pub fn hdop_vdop(&self, fix_pdop: f32, now_us: u64) -> (f32, f32) {
        if self.seen && now_us.saturating_sub(self.received_at_us) < AUXILIARY_FRESH_US {
            (self.hdop, self.vdop)
        } else {
            (fix_pdop, fix_pdop)
        }
    }
}

/// Build the sensor_gps snapshot from a Fix2, following PX4 process_fixx.
pub fn process_fix2(
    fix: &GnssFix2,
    node_id: u8,
    now_us: u64,
    auxiliary: &AuxiliaryCache,
) -> SensorGps {
    let fix_type = fix_type_from_dronecan(fix.status, fix.mode, fix.sub_mode);

    // PX4: covariance cases 1 (scalar) and 6 (diagonal) are valid; other sizes
    // are treated as invalid rather than guessed.
    let (eph, epv, s_variance, c_variance) = match usize::from(fix.covariance_len) {
        1 => {
            let x = sqrt_or_invalid(fix.covariance[0]);
            (x, x, x, f32::NAN)
        }
        6 => {
            let eph = sqrt_or_invalid(f32::max(fix.covariance[0], fix.covariance[1]));
            let epv = sqrt_or_invalid(fix.covariance[2]);
            let s_variance = f32::max(
                fix.covariance[3],
                f32::max(fix.covariance[4], fix.covariance[5]),
            );
            // PX4 Jacobian: (ve^2*P00 - 2*vn*ve*P01 + vn^2*P11) / (vn^2+ve^2)^2;
            // a diagonal matrix has no cross terms, so P01 = 0.
            let (vn, ve) = (fix.ned_velocity[0], fix.ned_velocity[1]);
            let (vn_sq, ve_sq) = (vn * vn, ve * ve);
            let denom = (vn_sq + ve_sq) * (vn_sq + ve_sq);
            let c_variance = if denom > 0.0 {
                (ve_sq * fix.covariance[3] + vn_sq * fix.covariance[4]) / denom
            } else {
                0.0
            };
            (eph, epv, s_variance, c_variance)
        }
        _ => (INVALID, INVALID, INVALID, INVALID),
    };

    let vel_m_s = libm::sqrtf(
        fix.ned_velocity[0] * fix.ned_velocity[0]
            + fix.ned_velocity[1] * fix.ned_velocity[1]
            + fix.ned_velocity[2] * fix.ned_velocity[2],
    );
    let cog_rad = libm::atan2f(fix.ned_velocity[1], fix.ned_velocity[0]);

    let (hdop, vdop) = auxiliary.hdop_vdop(fix.pdop, now_us);

    SensorGps {
        timestamp: now_us,
        timestamp_sample: now_us,
        device_id: device_id_from_node(node_id),
        latitude_deg: fix.latitude_deg_1e8 as f64 / 1e8,
        longitude_deg: fix.longitude_deg_1e8 as f64 / 1e8,
        altitude_msl_m: f64::from(fix.height_msl_mm) / 1e3,
        altitude_ellipsoid_m: f64::from(fix.height_ellipsoid_mm) / 1e3,
        s_variance_m_s: s_variance,
        c_variance_rad: c_variance,
        fix_type,
        eph,
        epv,
        hdop,
        vdop,
        noise_per_ms: -1,
        automatic_gain_control: 0,
        jamming_state: 0,
        jamming_indicator: -1,
        spoofing_state: 0,
        authentication_state: 0,
        vel_m_s,
        vel_n_m_s: fix.ned_velocity[0],
        vel_e_m_s: fix.ned_velocity[1],
        vel_d_m_s: fix.ned_velocity[2],
        cog_rad,
        vel_ned_valid: true,
        timestamp_time_relative: 0,
        time_utc_usec: time_utc_usec(
            fix.gnss_timestamp_usec,
            fix.gnss_time_standard,
            fix.num_leap_seconds,
        ),
        satellites_used: fix.sats_used,
        system_error: 0,
        heading: f32::NAN,
        heading_offset: f32::NAN,
        heading_accuracy: f32::NAN,
        rtcm_injection_rate: 0.0,
        selected_rtcm_instance: 0,
        rtcm_crc_failed: false,
        rtcm_msg_used: 0,
    }
}

/// PX4 device id: UavcanSensorBridgeBase derives it from the source node.
pub fn device_id_from_node(node_id: u8) -> u32 {
    u32::from(node_id)
}

/// PX4: sqrt of a positive variance, else -1 (unknown).
const INVALID: f32 = -1.0;

fn sqrt_or_invalid(value: f32) -> f32 {
    if value.is_finite() && value > 0.0 {
        libm::sqrtf(value)
    } else {
        INVALID
    }
}

pub fn fix_type_from_dronecan(status: u8, mode: u8, sub_mode: u8) -> u8 {
    if status < 2 {
        return FixType::NoFix as u8;
    }
    if status == 2 {
        return FixType::TwoD as u8;
    }
    match mode {
        2 => {
            if sub_mode == 1 {
                FixType::RtkFixed as u8
            } else {
                FixType::RtkFloat as u8
            }
        }
        1 => FixType::Dgps as u8,
        3 => FixType::Ppp as u8,
        _ => FixType::ThreeD as u8,
    }
}

/// PX4: UTC = GPS time - leap + 9; UTC = TAI - leap - 10.
pub fn time_utc_usec(gnss_timestamp_usec: u64, standard: u8, leap_seconds: u8) -> u64 {
    match standard {
        2 => gnss_timestamp_usec,
        3 if leap_seconds > 0 => gnss_timestamp_usec - u64::from(leap_seconds) + 9,
        1 if leap_seconds > 0 => gnss_timestamp_usec - u64::from(leap_seconds) - 10,
        _ => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dronecan::gnss;

    fn reference_fix() -> gnss::GnssFix2 {
        // decode_gnss_fix2 on the fix2_covariance_0 reference vector
        let payload: std::vec::Vec<u8> = "0000000000000040222018240a0640001bb2641628ea758700c0503880450430080000803f000000c00000003f530000003d"
            .as_bytes()
            .chunks_exact(2)
            .map(|pair| u8::from_str_radix(core::str::from_utf8(pair).unwrap(), 16).unwrap())
            .collect();
        gnss::decode_gnss_fix2(&payload).unwrap()
    }

    #[test]
    fn process_fix2_matches_px4_semantics_on_reference_vector() {
        let fix = reference_fix();
        let auxiliary = AuxiliaryCache::default();
        let snapshot = process_fix2(&fix, 25, 1_000_000, &auxiliary);
        // PX4: degrees = 1e8 field / 1e8; heights mm -> m.
        assert!((snapshot.latitude_deg - 47.12345678).abs() < 1e-9);
        assert!((snapshot.longitude_deg + 122.12345678).abs() < 1e-9);
        assert!((snapshot.altitude_msl_m - 100.0).abs() < 1e-9);
        assert!((snapshot.altitude_ellipsoid_m - 123.456).abs() < 1e-9);
        assert_eq!(snapshot.fix_type, FixType::ThreeD as u8);
        assert_eq!(snapshot.satellites_used, 20);
        // Diagonal covariance [empty]: eph/epv/s_variance are PX4's -1 (unknown).
        assert_eq!(snapshot.eph, INVALID);
        assert_eq!(snapshot.epv, INVALID);
        assert_eq!(snapshot.s_variance_m_s, INVALID);
        assert_eq!(snapshot.c_variance_rad, INVALID);
        assert!(snapshot.vel_ned_valid);
        assert!((snapshot.vel_m_s - (1.0f32 * 1.0 + 4.0 + 0.25).sqrt()).abs() < 1e-6);
        assert!((snapshot.vel_n_m_s - 1.0).abs() < 1e-7);
        assert!((snapshot.vel_e_m_s + 2.0).abs() < 1e-7);
        // Empty auxiliary cache: hdop/vdop fall back to pdop (PX4 rule).
        assert!((snapshot.hdop - 1.25).abs() < 1e-6);
        assert!((snapshot.vdop - 1.25).abs() < 1e-6);
        // PX4 defaults for fields Fix2 does not provide.
        assert_eq!(snapshot.noise_per_ms, -1);
        assert_eq!(snapshot.jamming_indicator, -1);
        assert!(snapshot.heading.is_nan());
        // UTC: gnss_time_standard=2 (UTC) passes the GNSS timestamp through.
        assert_eq!(snapshot.time_utc_usec, 1_700_000_000_123_456);
        assert_eq!(snapshot.device_id, 25);
    }

    #[test]
    fn diagonal_covariance_produces_px4_accuracy_values() {
        let mut fix = reference_fix();
        fix.covariance = [0.25, 0.16, 1.44, 0.09, 0.04, 0.25];
        fix.covariance_len = 6;
        let snapshot = process_fix2(&fix, 25, 0, &AuxiliaryCache::default());
        // PX4: eph = sqrt(max(pos_cov[0], pos_cov[4])) = sqrt(0.25).
        assert!((snapshot.eph - 0.5).abs() < 1e-6);
        // epv = sqrt(pos_cov[8]) = sqrt(1.44).
        assert!((snapshot.epv - 1.2).abs() < 1e-6);
        // s_variance = max(vel diag).
        assert!((snapshot.s_variance_m_s - 0.25).abs() < 1e-6);
        // Diagonal c-variance with ve=-2, vn=1: (ve^2*P00 + vn^2*P11) / (vn^2+ve^2)^2.
        let expected = (4.0 * 0.09 + 1.0 * 0.04) / 25.0;
        assert!((snapshot.c_variance_rad - expected).abs() < 1e-6);
    }

    #[test]
    fn auxiliary_cache_prefers_fresh_dop_then_falls_back_to_pdop() {
        let fix = reference_fix();
        let mut auxiliary = AuxiliaryCache::default();
        auxiliary.update(
            gnss::GnssAuxiliary {
                gdop: 1.0,
                pdop: 1.25,
                hdop: 0.75,
                vdop: 1.5,
                tdop: f32::NAN,
                ndop: f32::NAN,
                edop: f32::NAN,
                sats_visible: 28,
                sats_used: 20,
            },
            1_000_000,
        );
        let fresh = process_fix2(&fix, 25, 1_000_000 + AUXILIARY_FRESH_US - 1, &auxiliary);
        assert!((fresh.hdop - 0.75).abs() < 1e-6);
        assert!((fresh.vdop - 1.5).abs() < 1e-6);
        let stale = process_fix2(&fix, 25, 1_000_000 + AUXILIARY_FRESH_US, &auxiliary);
        assert!((stale.hdop - 1.25).abs() < 1e-6);
        assert!((stale.vdop - 1.25).abs() < 1e-6);
    }

    #[test]
    fn fix_type_mapping_matches_px4_semantics() {
        assert_eq!(fix_type_from_dronecan(0, 0, 0), FixType::NoFix as u8);
        assert_eq!(fix_type_from_dronecan(1, 0, 0), FixType::NoFix as u8);
        assert_eq!(fix_type_from_dronecan(2, 0, 0), FixType::TwoD as u8);
        assert_eq!(fix_type_from_dronecan(3, 0, 0), FixType::ThreeD as u8);
        assert_eq!(fix_type_from_dronecan(3, 1, 0), FixType::Dgps as u8);
        assert_eq!(fix_type_from_dronecan(3, 2, 0), FixType::RtkFloat as u8);
        assert_eq!(fix_type_from_dronecan(3, 2, 1), FixType::RtkFixed as u8);
        assert_eq!(fix_type_from_dronecan(3, 3, 0), FixType::Ppp as u8);
    }

    #[test]
    fn time_utc_conversion_follows_px4_rules() {
        assert_eq!(time_utc_usec(1_000, 2, 0), 1_000);
        assert_eq!(time_utc_usec(1_000, 3, 27), 1_000 - 27 + 9);
        assert_eq!(time_utc_usec(1_000, 1, 27), 1_000 - 27 - 10);
        assert_eq!(time_utc_usec(1_000, 0, 0), 0);
    }
}
