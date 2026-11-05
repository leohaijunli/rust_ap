//! Static component metadata served to ground stations over MAVLink FTP.
//!
//! QGroundControl only shows parameter groups when the vehicle exposes
//! `COMPONENT_METADATA` and the referenced `parameters.json`. Both files are
//! compile-time constants so the FTP server can answer without an allocator.

/// MAVLink FTP URI carried by `COMPONENT_METADATA`.
pub const GENERAL_URI: &str = "mftp:///component_metadata/general.json";

/// General metadata: points at the parameter metadata file and its CRC.
///
/// `fileCrc` must equal [`PARAMETERS_CRC`]; the unit test below enforces that.
pub const GENERAL_JSON: &str = include_str!("metadata/general.json");

/// Parameter metadata: one entry per entry in [`crate::parameters::REGISTRY`].
pub const PARAMETERS_JSON: &str = include_str!("metadata/parameters.json");

pub const GENERAL_CRC: u32 = crc32(GENERAL_JSON.as_bytes());
pub const PARAMETERS_CRC: u32 = crc32(PARAMETERS_JSON.as_bytes());

/// Resolve a virtual path, tolerating the extra leading slash QGC produces.
pub fn file(path: &str) -> Option<&'static [u8]> {
    match path.trim_start_matches('/') {
        "component_metadata/general.json" | "general.json" => Some(GENERAL_JSON.as_bytes()),
        "component_metadata/parameters.json" | "parameters.json" => {
            Some(PARAMETERS_JSON.as_bytes())
        }
        _ => None,
    }
}

/// CRC-32/IEEE (reflected), computed at compile time for the metadata files.
pub const fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xffff_ffffu32;
    let mut index = 0;
    while index < bytes.len() {
        crc ^= bytes[index] as u32;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xedb8_8320
            } else {
                crc >> 1
            };
            bit += 1;
        }
        index += 1;
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parameters::{Kind, COUNT, REGISTRY};
    use std::format;

    #[test]
    fn crc32_matches_the_standard_check_value() {
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
        assert_eq!(crc32(b""), 0);
    }

    #[test]
    fn general_metadata_references_the_parameter_file_with_its_crc() {
        let expected = format!(
            "\"uri\": \"{}\",\n      \"fileCrc\": {}",
            "mftp:///component_metadata/parameters.json", PARAMETERS_CRC
        );
        assert!(
            GENERAL_JSON.contains(&expected),
            "general.json fileCrc is stale; expected {expected}"
        );
        assert_eq!(
            file(GENERAL_URI.trim_start_matches("mftp:")),
            Some(GENERAL_JSON.as_bytes())
        );
    }

    #[test]
    fn parameter_metadata_covers_the_whole_registry() {
        assert_eq!(
            PARAMETERS_JSON.matches("\"name\":").count(),
            COUNT,
            "one metadata entry per registered parameter"
        );
        for entry in REGISTRY {
            let mut lines = PARAMETERS_JSON
                .lines()
                .filter(|line| line.contains(&format!("\"name\": \"{}\"", entry.name)));
            let line = lines
                .next()
                .unwrap_or_else(|| panic!("missing {}", entry.name));
            assert!(lines.next().is_none(), "duplicate {}", entry.name);

            let kind = match entry.kind {
                Kind::Integer => "\"type\": \"Int32\"",
                Kind::Real => "\"type\": \"Float\"",
            };
            assert!(line.contains(kind), "type mismatch for {}", entry.name);
            for (key, value) in [
                ("min", entry.min),
                ("max", entry.max),
                ("default", entry.default),
            ] {
                let prefix = format!("\"{key}\":");
                let text = line.split_once(&prefix).unwrap().1.trim_start();
                let number = text
                    .split([',', '}'])
                    .next()
                    .unwrap()
                    .trim()
                    .parse::<f32>()
                    .unwrap();
                assert_eq!(
                    number.to_bits(),
                    value.to_bits(),
                    "{} {key} value differs",
                    entry.name
                );
            }
            if entry.name.starts_with("LOG_") {
                assert!(
                    line.contains("\"group\": \"SDLog\""),
                    "{} must be grouped as SDLog",
                    entry.name
                );
            }
        }
    }

    #[test]
    fn lookup_accepts_the_paths_qgc_sends() {
        assert_eq!(
            file("//component_metadata/general.json"),
            Some(GENERAL_JSON.as_bytes())
        );
        assert_eq!(
            file("/component_metadata/parameters.json"),
            Some(PARAMETERS_JSON.as_bytes())
        );
        assert_eq!(file("parameters.json"), Some(PARAMETERS_JSON.as_bytes()));
        assert_eq!(file("/component_metadata/missing.json"), None);
    }
}
