//! UTC calendar arithmetic shared by the RTC driver and FAT timestamps.
//!
//! Everything here is integer-only and host testable; the RTC hardware lives in
//! `drivers/rtc.rs`. Years outside 1970..=9999 are rejected so that the
//! proleptic Gregorian conversion below cannot underflow.

/// Earliest UNIX second treated as a real date. A calendar that still reads
/// 2000-01-01 (power-on reset) or anything older is "not set".
pub const MIN_PLAUSIBLE_UNIX: u64 = 1_577_836_800; // 2020-01-01T00:00:00Z

/// Latest second representable by the RTC's two-digit BCD year field.
pub const MAX_RTC_UNIX: u64 = 4_102_444_799; // 2099-12-31T23:59:59Z

/// FAT directory entries store years 1980..=2107 in seven bits.
pub const MIN_FAT_YEAR: u16 = 1980;
pub const MAX_FAT_YEAR: u16 = 2107;

const SECONDS_PER_DAY: u64 = 86_400;

/// A validated UTC calendar value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Utc {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

impl Utc {
    /// Build a value after checking the month/day/clock ranges.
    pub fn new(year: u16, month: u8, day: u8, hour: u8, minute: u8, second: u8) -> Option<Self> {
        if !(1970..=9999).contains(&year)
            || !(1..=12).contains(&month)
            || hour > 23
            || minute > 59
            || second > 59
        {
            return None;
        }
        let max_day = match month {
            2 if is_leap_year(year) => 29,
            2 => 28,
            4 | 6 | 9 | 11 => 30,
            _ => 31,
        };
        if !(1..=max_day).contains(&day) {
            return None;
        }
        Some(Self {
            year,
            month,
            day,
            hour,
            minute,
            second,
        })
    }

    /// Seconds since the UNIX epoch.
    pub fn unix(self) -> u64 {
        let days = days_from_civil(
            i64::from(self.year),
            i64::from(self.month),
            i64::from(self.day),
        );
        // `new` only accepts 1970..=9999, so `days` is never negative.
        days as u64 * SECONDS_PER_DAY
            + u64::from(self.hour) * 3600
            + u64::from(self.minute) * 60
            + u64::from(self.second)
    }

    /// Inverse of [`Self::unix`], limited to the years `new` accepts.
    pub fn from_unix(seconds: u64) -> Option<Self> {
        let days = i64::try_from(seconds / SECONDS_PER_DAY).ok()?;
        let remainder = (seconds % SECONDS_PER_DAY) as u32;
        let (year, month, day) = civil_from_days(days);
        if !(1970..=9999).contains(&year) {
            return None;
        }
        Some(Self {
            year: u16::try_from(year).ok()?,
            month: u8::try_from(month).ok()?,
            day: u8::try_from(day).ok()?,
            hour: u8::try_from(remainder / 3600).ok()?,
            minute: u8::try_from((remainder % 3600) / 60).ok()?,
            second: u8::try_from(remainder % 60).ok()?,
        })
    }

    /// UNIX seconds, but only when the date is at or after 2020-01-01.
    pub fn plausible_unix(self) -> Option<u64> {
        let seconds = self.unix();
        (seconds >= MIN_PLAUSIBLE_UNIX).then_some(seconds)
    }

    /// Two-digit BCD year field range of the STM32 RTC.
    pub fn is_rtc_range(self) -> bool {
        (2000..=2099).contains(&self.year)
    }

    /// DOS/FAT directory entry range.
    pub fn is_fat_range(self) -> bool {
        (MIN_FAT_YEAR..=MAX_FAT_YEAR).contains(&self.year)
    }

    /// STM32 `WDU` encoding: 1 = Monday ... 7 = Sunday.
    pub fn weekday(self) -> u8 {
        let days = days_from_civil(
            i64::from(self.year),
            i64::from(self.month),
            i64::from(self.day),
        );
        // 1970-01-01 was a Thursday (WDU 4).
        (days + 3).rem_euclid(7) as u8 + 1
    }

    /// Parse `YYYY-MM-DDTHH:MM:SS` with an optional trailing `Z`.
    pub fn parse_iso(text: &str) -> Option<Self> {
        let bytes = text.as_bytes();
        let body = match bytes.len() {
            19 => bytes,
            20 if bytes[19] == b'Z' => &bytes[..19],
            _ => return None,
        };
        if body[4] != b'-'
            || body[7] != b'-'
            || body[10] != b'T'
            || body[13] != b':'
            || body[16] != b':'
        {
            return None;
        }
        Self::new(
            number(&body[0..4])?,
            u8::try_from(number(&body[5..7])?).ok()?,
            u8::try_from(number(&body[8..10])?).ok()?,
            u8::try_from(number(&body[11..13])?).ok()?,
            u8::try_from(number(&body[14..16])?).ok()?,
            u8::try_from(number(&body[17..19])?).ok()?,
        )
    }
}

/// Parse the argument of `!time set`: UNIX seconds or an ISO timestamp.
///
/// Values outside 2020-01-01..2099-12-31 are rejected so a typo cannot leave
/// the RTC in a state that FAT or the log listing has to special-case.
pub fn parse_set_value(text: &str) -> Option<u64> {
    let seconds = match Utc::parse_iso(text) {
        Some(utc) => utc.unix(),
        None => text.parse::<u64>().ok()?,
    };
    (MIN_PLAUSIBLE_UNIX..=MAX_RTC_UNIX)
        .contains(&seconds)
        .then_some(seconds)
}

impl core::fmt::Display for Utc {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            formatter,
            "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}Z",
            self.year, self.month, self.day, self.hour, self.minute, self.second
        )
    }
}

fn is_leap_year(year: u16) -> bool {
    year.is_multiple_of(4) && (!year.is_multiple_of(100) || year.is_multiple_of(400))
}

fn number(bytes: &[u8]) -> Option<u16> {
    let mut value = 0u16;
    for &byte in bytes {
        if !byte.is_ascii_digit() {
            return None;
        }
        value = value * 10 + u16::from(byte - b'0');
    }
    Some(value)
}

// Howard Hinnant's civil-from-days algorithms, kept in `i64` so the same code
// runs on the device and on the host.
fn days_from_civil(year: i64, month: i64, day: i64) -> i64 {
    let year = if month <= 2 { year - 1 } else { year };
    let era = if year >= 0 { year } else { year - 399 } / 400;
    let year_of_era = year - era * 400;
    let month_prime = (month + 9) % 12;
    let day_of_year = (153 * month_prime + 2) / 5 + day - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    era * 146_097 + day_of_era - 719_468
}

fn civil_from_days(days: i64) -> (i64, i64, i64) {
    let days = days + 719_468;
    let era = if days >= 0 { days } else { days - 146_096 } / 146_097;
    let day_of_era = days - era * 146_097;
    let year_of_era =
        (day_of_era - day_of_era / 1460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
    let year = year_of_era + era * 400;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    let month_prime = (5 * day_of_year + 2) / 153;
    let day = day_of_year - (153 * month_prime + 2) / 5 + 1;
    let month = if month_prime < 10 {
        month_prime + 3
    } else {
        month_prime - 9
    };
    (if month <= 2 { year + 1 } else { year }, month, day)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn epoch_and_known_dates_round_trip() {
        assert_eq!(
            Utc::from_unix(0),
            Some(Utc::new(1970, 1, 1, 0, 0, 0).unwrap())
        );
        let start_2020 = Utc::new(2020, 1, 1, 0, 0, 0).unwrap();
        assert_eq!(start_2020.unix(), MIN_PLAUSIBLE_UNIX);
        assert_eq!(Utc::from_unix(MIN_PLAUSIBLE_UNIX), Some(start_2020));
        let leap = Utc::new(2024, 2, 29, 23, 59, 59).unwrap();
        assert_eq!(Utc::from_unix(leap.unix()), Some(leap));
        let end = Utc::new(2099, 12, 31, 23, 59, 59).unwrap();
        assert_eq!(end.unix(), MAX_RTC_UNIX);
    }

    #[test]
    fn invalid_calendar_values_are_rejected() {
        assert!(Utc::new(2023, 2, 29, 0, 0, 0).is_none());
        assert!(Utc::new(2024, 2, 30, 0, 0, 0).is_none());
        assert!(Utc::new(2024, 4, 31, 0, 0, 0).is_none());
        assert!(Utc::new(2024, 13, 1, 0, 0, 0).is_none());
        assert!(Utc::new(2024, 0, 1, 0, 0, 0).is_none());
        assert!(Utc::new(2024, 1, 1, 24, 0, 0).is_none());
        assert!(Utc::new(1969, 12, 31, 23, 59, 59).is_none());
    }

    #[test]
    fn plausibility_starts_in_2020() {
        assert_eq!(
            Utc::new(2000, 1, 1, 0, 0, 0).unwrap().plausible_unix(),
            None
        );
        assert_eq!(
            Utc::new(2019, 12, 31, 23, 59, 59).unwrap().plausible_unix(),
            None
        );
        assert_eq!(
            Utc::new(2020, 1, 1, 0, 0, 0).unwrap().plausible_unix(),
            Some(MIN_PLAUSIBLE_UNIX)
        );
    }

    #[test]
    fn weekday_uses_stm32_encoding() {
        assert_eq!(Utc::new(2026, 11, 2, 0, 0, 0).unwrap().weekday(), 1);
        assert_eq!(Utc::new(1970, 1, 1, 0, 0, 0).unwrap().weekday(), 4);
        assert_eq!(Utc::new(2000, 1, 1, 0, 0, 0).unwrap().weekday(), 6);
    }

    #[test]
    fn iso_parser_accepts_optional_z_only() {
        let expected = Utc::new(2026, 11, 2, 7, 39, 27).unwrap();
        assert_eq!(Utc::parse_iso("2026-11-02T07:39:27Z"), Some(expected));
        assert_eq!(Utc::parse_iso("2026-11-02T07:39:27"), Some(expected));
        assert!(Utc::parse_iso("2026-11-02 07:39:27").is_none());
        assert!(Utc::parse_iso("2026-11-02T07:39:27+01:00").is_none());
        assert!(Utc::parse_iso("2026-13-02T07:39:27Z").is_none());
        assert!(Utc::parse_iso("2026-11-02T24:00:00Z").is_none());
        assert!(Utc::parse_iso("2026-11-02T07:39:2aZ").is_none());
    }

    #[test]
    fn set_value_accepts_unix_or_iso_and_rejects_typos() {
        let seconds = Utc::new(2026, 11, 2, 7, 39, 27).unwrap().unix();
        assert_eq!(parse_set_value("2026-11-02T07:39:27Z"), Some(seconds));
        assert_eq!(parse_set_value("1793605167"), Some(1_793_605_167));
        assert_eq!(parse_set_value("0"), None);
        assert_eq!(parse_set_value("2026-11-02T07:39:27Z "), None);
        assert_eq!(parse_set_value("not-a-time"), None);
    }

    #[test]
    fn display_is_iso_8601() {
        use core::fmt::Write;

        struct Buffer {
            bytes: [u8; 32],
            len: usize,
        }

        impl Write for Buffer {
            fn write_str(&mut self, text: &str) -> core::fmt::Result {
                let end = self.len + text.len();
                if end > self.bytes.len() {
                    return Err(core::fmt::Error);
                }
                self.bytes[self.len..end].copy_from_slice(text.as_bytes());
                self.len = end;
                Ok(())
            }
        }

        let mut buffer = Buffer {
            bytes: [0; 32],
            len: 0,
        };
        write!(&mut buffer, "{}", Utc::new(2026, 1, 2, 3, 4, 5).unwrap()).unwrap();
        assert_eq!(&buffer.bytes[..buffer.len], b"2026-01-02T03:04:05Z");
    }
}
