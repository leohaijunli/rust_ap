//! DroneCAN GNSS message decode: `uavcan.equipment.gnss.Fix2` (1063) and
//! `uavcan.equipment.gnss.Auxiliary` (1061).
//!
//! Bit order follows the UAVCAN v0 DSDL rule: byte-internal MSB-first, with
//! multi-byte values interpreted little-endian (low byte first). Reference
//! vectors in docs/acceptance/p0/dronecan-vectors.json bind this implementation.

pub const GNSS_FIX2_ID: u16 = 1063;
pub const GNSS_AUXILIARY_ID: u16 = 1061;
pub const GNSS_FIX2_SIGNATURE: u64 = 0xca41_e700_0f37_435f;
pub const GNSS_AUXILIARY_SIGNATURE: u64 = 0x9be8_bdc4_c3db_bfd2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CodecError {
    Length,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssFix2 {
    pub timestamp_usec: u64,
    pub gnss_timestamp_usec: u64,
    pub gnss_time_standard: u8,
    pub num_leap_seconds: u8,
    pub longitude_deg_1e8: i64,
    pub latitude_deg_1e8: i64,
    pub height_ellipsoid_mm: i32,
    pub height_msl_mm: i32,
    pub ned_velocity: [f32; 3],
    pub sats_used: u8,
    pub status: u8,
    pub mode: u8,
    pub sub_mode: u8,
    pub covariance: [f32; 6],
    pub covariance_len: u8,
    pub pdop: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssAuxiliary {
    pub gdop: f32,
    pub pdop: f32,
    pub hdop: f32,
    pub vdop: f32,
    pub tdop: f32,
    pub ndop: f32,
    pub edop: f32,
    pub sats_visible: u8,
    pub sats_used: u8,
}

/// Reads `n` bits from a byte-internal MSB-first stream, accumulating into
/// byte-aligned slots, then interprets the multi-byte value little-endian.
fn read_bits(bytes: &[u8], pos: &mut usize, n: usize) -> Result<u64, CodecError> {
    if *pos + n > bytes.len() * 8 {
        return Err(CodecError::Length);
    }
    let mut slots = [0u8; 8];
    for i in 0..n {
        let p = *pos + i;
        let byte = bytes[p / 8];
        let bit = 7 - (p % 8);
        slots[i / 8] = (slots[i / 8] << 1) | ((byte >> bit) & 1);
    }
    *pos += n;
    let nbytes = n.div_ceil(8);
    let mut val: u64 = 0;
    for (i, slot) in slots.iter().enumerate().take(nbytes) {
        val |= u64::from(*slot) << (i * 8);
    }
    Ok(val)
}

fn read_signed(bytes: &[u8], pos: &mut usize, n: usize) -> Result<i64, CodecError> {
    let raw = read_bits(bytes, pos, n)?;
    if n < 64 && raw >= 1_u64 << (n - 1) {
        Ok(raw as i64 - (1_i64 << n))
    } else {
        Ok(raw as i64)
    }
}

fn read_float32(bytes: &[u8], pos: &mut usize) -> Result<f32, CodecError> {
    let raw = read_bits(bytes, pos, 32)? as u32;
    Ok(f32::from_le_bytes(raw.to_le_bytes()))
}

fn read_float16(bytes: &[u8], pos: &mut usize) -> Result<f32, CodecError> {
    let raw = read_bits(bytes, pos, 16)? as u16;
    Ok(half_to_float(raw))
}

pub fn decode_gnss_fix2(payload: &[u8]) -> Result<GnssFix2, CodecError> {
    let mut pos = 0;
    let timestamp_usec = read_bits(payload, &mut pos, 56)?;
    let gnss_timestamp_usec = read_bits(payload, &mut pos, 56)?;
    let gnss_time_standard = read_bits(payload, &mut pos, 3)? as u8;
    pos += 13;
    let num_leap_seconds = read_bits(payload, &mut pos, 8)? as u8;
    let longitude_deg_1e8 = read_signed(payload, &mut pos, 37)?;
    let latitude_deg_1e8 = read_signed(payload, &mut pos, 37)?;
    let height_ellipsoid_mm = read_signed(payload, &mut pos, 27)? as i32;
    let height_msl_mm = read_signed(payload, &mut pos, 27)? as i32;
    let ned_velocity = [
        read_float32(payload, &mut pos)?,
        read_float32(payload, &mut pos)?,
        read_float32(payload, &mut pos)?,
    ];
    let sats_used = read_bits(payload, &mut pos, 6)? as u8;
    let status = read_bits(payload, &mut pos, 2)? as u8;
    let mode = read_bits(payload, &mut pos, 4)? as u8;
    let sub_mode = read_bits(payload, &mut pos, 6)? as u8;
    let mut covariance = [f32::NAN; 6];
    let elements = read_bits(payload, &mut pos, 6)? as usize;
    let covariance_len = elements.min(u8::MAX as usize) as u8;
    let stored = elements.min(6);
    for slot in covariance.iter_mut().take(stored) {
        *slot = read_float16(payload, &mut pos)?;
    }
    for _ in stored..elements {
        read_float16(payload, &mut pos)?;
    }
    let pdop = read_float16(payload, &mut pos)?;
    Ok(GnssFix2 {
        timestamp_usec,
        gnss_timestamp_usec,
        gnss_time_standard,
        num_leap_seconds,
        longitude_deg_1e8,
        latitude_deg_1e8,
        height_ellipsoid_mm,
        height_msl_mm,
        ned_velocity,
        sats_used,
        status,
        mode,
        sub_mode,
        covariance,
        covariance_len,
        pdop,
    })
}

pub fn decode_gnss_auxiliary(payload: &[u8]) -> Result<GnssAuxiliary, CodecError> {
    let mut pos = 0;
    let aux = GnssAuxiliary {
        gdop: read_float16(payload, &mut pos)?,
        pdop: read_float16(payload, &mut pos)?,
        hdop: read_float16(payload, &mut pos)?,
        vdop: read_float16(payload, &mut pos)?,
        tdop: read_float16(payload, &mut pos)?,
        ndop: read_float16(payload, &mut pos)?,
        edop: read_float16(payload, &mut pos)?,
        sats_visible: read_bits(payload, &mut pos, 7)? as u8,
        sats_used: read_bits(payload, &mut pos, 6)? as u8,
    };
    Ok(aux)
}

fn half_to_float(h: u16) -> f32 {
    let sign = u32::from(h & 0x8000) << 16;
    let exponent = (h >> 10) & 0x1f;
    let fraction = u32::from(h & 0x03ff);
    let bits = match exponent {
        0 if fraction == 0 => sign,
        0 => {
            let mut fraction = fraction;
            let mut exponent = 113_u32;
            while fraction & 0x0400 == 0 {
                fraction <<= 1;
                exponent -= 1;
            }
            sign | exponent << 23 | (fraction & 0x03ff) << 13
        }
        31 => sign | 0x7f80_0000 | fraction << 13,
        _ => sign | (u32::from(exponent) + 112) << 23 | fraction << 13,
    };
    f32::from_bits(bits)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bytes(hex: &str) -> std::vec::Vec<u8> {
        hex.as_bytes()
            .as_chunks::<2>()
            .0
            .iter()
            .map(|pair| u8::from_str_radix(core::str::from_utf8(pair).unwrap(), 16).unwrap())
            .collect()
    }

    #[test]
    fn fix2_covariance_0_matches_reference_vector() {
        let payload = bytes("0000000000000040222018240a0640001bb2641628ea758700c0503880450430080000803f000000c00000003f530000003d");
        let fix = decode_gnss_fix2(&payload).unwrap();
        assert_eq!(fix.gnss_timestamp_usec, 1_700_000_000_123_456);
        assert_eq!(fix.gnss_time_standard, 2);
        assert_eq!(fix.num_leap_seconds, 27);
        assert_eq!(fix.longitude_deg_1e8, -12_212_345_678);
        assert_eq!(fix.latitude_deg_1e8, 4_712_345_678);
        assert_eq!(fix.height_ellipsoid_mm, 123_456);
        assert_eq!(fix.height_msl_mm, 100_000);
        assert_eq!(fix.ned_velocity, [1.0, -2.0, 0.5]);
        assert_eq!(fix.sats_used, 20);
        assert_eq!(fix.status, 3);
        assert_eq!(fix.mode, 0);
        assert_eq!(fix.sub_mode, 0);
        assert_eq!(fix.covariance_len, 0);
        assert_eq!(fix.pdop, 1.25);
    }

    #[test]
    fn fix2_covariance_1_matches_reference_vector() {
        let payload = bytes("0000000000000040222018240a0640001bb2641628ea758700c0503880450430080000803f000000c00000003f5300010034003d");
        let fix = decode_gnss_fix2(&payload).unwrap();
        assert_eq!(fix.sats_used, 20);
        assert_eq!(fix.status, 3);
        assert_eq!(fix.covariance_len, 1);
        assert_eq!(fix.covariance[0], 0.25);
    }

    #[test]
    fn fix2_rtk_float_matches_reference_vector() {
        let payload = bytes("0000000000000040222018240a0640001bb2641628ea758700c0503880450430080000803f000000c00000003f532024003400340034003400340034003400340034003400340034003400340034003400340034003400340034003400340034003400340034003400340034003400340034003400340034003d");
        let fix = decode_gnss_fix2(&payload).unwrap();
        assert_eq!(fix.sats_used, 20);
        assert_eq!(fix.status, 3);
        assert_eq!(fix.mode, 2);
        assert_eq!(fix.sub_mode, 0);
        assert_eq!(fix.covariance_len, 36);
        assert_eq!(fix.covariance[0], 0.25);
        assert_eq!(fix.pdop, 1.25);
    }

    #[test]
    fn auxiliary_matches_reference_vector() {
        let payload = bytes("003c003d003a003eff7fff7fff7f38a0");
        let aux = decode_gnss_auxiliary(&payload).unwrap();
        assert_eq!(aux.gdop, 1.0);
        assert_eq!(aux.pdop, 1.25);
        assert_eq!(aux.hdop, 0.75);
        assert_eq!(aux.vdop, 1.5);
        assert!(aux.tdop.is_nan());
        assert!(aux.ndop.is_nan());
        assert!(aux.edop.is_nan());
        assert_eq!(aux.sats_visible, 28);
        assert_eq!(aux.sats_used, 20);
    }
}
