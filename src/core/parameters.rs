//! Single parameter registry and validated RAM state, independent of transport/storage.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Integer,
    Real,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Access {
    ReadOnly,
    NextCycle,
}

/// QGC parameter-tree grouping. It lives with the definition so the generated
/// metadata reads this one table instead of re-deriving groups from name
/// prefixes, which would drift as soon as a parameter is renamed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Group {
    System,
    Status,
    SdLog,
    Estimator,
    Ekf2,
}

impl Group {
    pub const fn name(self) -> &'static str {
        match self {
            Self::System => "System",
            Self::Status => "Status",
            Self::SdLog => "SDLog",
            Self::Estimator => "Estimator",
            Self::Ekf2 => "EKF2",
        }
    }
}

#[derive(Clone, Copy)]
pub struct Definition {
    pub id: Id,
    pub name: &'static str,
    pub group: Group,
    pub kind: Kind,
    pub default: f32,
    pub min: f32,
    pub max: f32,
    pub access: Access,
}

// Generate typed keys and the wire-order table together: one entry per parameter.
macro_rules! parameter_table {
    (base { $($id:ident { name: $name:literal, group: $group:ident, kind: $kind:ident,
        default: $default:expr, min: $min:expr, max: $max:expr, access: $access:ident }),+ $(,)? }
        ekf { $($ekf_id:ident: $index:literal),+ $(,)? }) => {
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #[repr(usize)]
        pub enum Id { $($id),+, $($ekf_id),+ }

        pub const REGISTRY: &[Definition] = &[
            $(Definition { id: Id::$id, name: $name, group: Group::$group, kind: Kind::$kind,
                default: $default, min: $min, max: $max, access: Access::$access }),+,
            $(ekf_definition(Id::$ekf_id, $index)),+
        ];
    };
}

include!("estimator/runtime_parameter_table.rs");

with_ekf2_parameters! {
    MavSysId {
        name: "MAV_SYS_ID", group: System, kind: Integer, default: 1.0,
        min: 1.0, max: 255.0, access: ReadOnly
    },
    LinkLedHz {
        name: "LINK_LED_HZ", group: Status, kind: Real, default: 1.0,
        min: 0.1, max: 10.0, access: NextCycle
    },
    LogEnImu {
        name: "LOG_EN_IMU", group: SdLog, kind: Integer, default: 1.0,
        min: 0.0, max: 1.0, access: NextCycle
    },
    LogImuHz {
        name: "LOG_IMU_HZ", group: SdLog, kind: Real, default: 200.0,
        min: 1.0, max: 1000.0, access: NextCycle
    },
    LogEnIna {
        name: "LOG_EN_INA", group: SdLog, kind: Integer, default: 1.0,
        min: 0.0, max: 1.0, access: NextCycle
    },
    LogInaHz {
        name: "LOG_INA_HZ", group: SdLog, kind: Real, default: 10.0,
        min: 1.0, max: 100.0, access: NextCycle
    },
    LogEnCan {
        name: "LOG_EN_CAN", group: SdLog, kind: Integer, default: 1.0,
        min: 0.0, max: 1.0, access: NextCycle
    },
    LogCanHz {
        name: "LOG_CAN_HZ", group: SdLog, kind: Real, default: 400.0,
        min: 1.0, max: 1000.0, access: NextCycle
    },
    LogEnHealth {
        name: "LOG_EN_HEALTH", group: SdLog, kind: Integer, default: 1.0,
        min: 0.0, max: 1.0, access: NextCycle
    },
    LogHealthHz {
        name: "LOG_HEALTH_HZ", group: SdLog, kind: Real, default: 10.0,
        min: 1.0, max: 100.0, access: NextCycle
    },
    LogEnEvent {
        name: "LOG_EN_EVENT", group: SdLog, kind: Integer, default: 1.0,
        min: 0.0, max: 1.0, access: NextCycle
    },
    EstEnable {
        name: "EST_EN", group: Estimator, kind: Integer, default: 0.0,
        min: 0.0, max: 1.0, access: NextCycle
    },
    EstAlgorithm {
        name: "EST_ALGO", group: Estimator, kind: Integer, default: 1.0,
        min: 0.0, max: 1.0, access: NextCycle
    },
    LogEnEst {
        name: "LOG_EN_EST", group: SdLog, kind: Integer, default: 1.0,
        min: 0.0, max: 1.0, access: NextCycle
    },
    LogEstHz {
        name: "LOG_EST_HZ", group: SdLog, kind: Real, default: 5.0,
        min: 1.0, max: 10.0, access: NextCycle
    },
    LogEnBaro {
        name: "LOG_EN_BARO", group: SdLog, kind: Integer, default: 1.0,
        min: 0.0, max: 1.0, access: NextCycle
    },
    LogBaroHz {
        name: "LOG_BARO_HZ", group: SdLog, kind: Real, default: 10.0,
        min: 1.0, max: 20.0, access: NextCycle
    },
}

/// Parameters whose change reprograms the estimator: the generated EKF2
/// catalog plus the two selectors that choose and enable it. Every other
/// parameter belongs to its own consumer, such as the logging policy or the
/// link rate. The whole catalog is included, not only the entries this port
/// consumes yet, because PX4 also re-applies its configuration for any
/// `EKF2_*` change.
pub const fn is_estimator_parameter(id: Id) -> bool {
    matches!(id, Id::EstEnable | Id::EstAlgorithm) || is_ekf2_parameter(id)
}

const fn ekf_definition(id: Id, index: usize) -> Definition {
    use crate::estimator::{ParamKind, ParamValue, EKF2_PARAMS};
    let entry = &EKF2_PARAMS[index];
    let integer = !matches!(entry.kind, ParamKind::Float);
    Definition {
        id,
        name: entry.name,
        group: Group::Ekf2,
        kind: if integer { Kind::Integer } else { Kind::Real },
        default: match entry.default {
            ParamValue::Int(v) => v as f32,
            ParamValue::Float(v) => v,
        },
        min: match entry.min {
            Some(v) => v,
            None if matches!(entry.kind, ParamKind::Boolean | ParamKind::Bitmask) => 0.0,
            None if integer => i32::MIN as f32,
            None => f32::MIN,
        },
        max: match entry.max {
            Some(v) => v,
            None if matches!(entry.kind, ParamKind::Boolean) => 1.0,
            None if integer => i32::MAX as f32,
            None => f32::MAX,
        },
        access: Access::NextCycle,
    }
}

pub const COUNT: usize = REGISTRY.len();

impl Id {
    pub const fn definition(self) -> &'static Definition {
        &REGISTRY[self as usize]
    }

    pub fn from_name(name: &str) -> Option<Self> {
        REGISTRY
            .iter()
            .find(|entry| entry.name == name)
            .map(|entry| entry.id)
    }

    pub fn from_index(index: usize) -> Option<Self> {
        REGISTRY.get(index).map(|entry| entry.id)
    }

    /// Inverse of [`name_hash`]; `None` for keys written by another build.
    pub fn from_hash(hash: u16) -> Option<Self> {
        REGISTRY
            .iter()
            .find(|entry| name_hash(entry.name) == hash)
            .map(|entry| entry.id)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    ReadOnly,
    NonFinite,
    OutOfRange,
    NotInteger,
    Configuration,
}

impl Error {
    pub fn message(self) -> &'static str {
        match self {
            Self::ReadOnly => "PARAM_SET: read-only",
            Self::NonFinite => "PARAM_SET: non-finite value",
            Self::OutOfRange => "PARAM_SET: out of range",
            Self::NotInteger => "PARAM_SET: expected integer",
            Self::Configuration => "PARAM_SET: unsupported estimator configuration",
        }
    }
}

impl Definition {
    pub(crate) fn validate(&self, value: f32) -> Result<(), Error> {
        if !value.is_finite() {
            return Err(Error::NonFinite);
        }
        if value < self.min || value > self.max {
            return Err(Error::OutOfRange);
        }
        if self.kind == Kind::Integer && (value as i32) as f64 != value as f64 {
            return Err(Error::NotInteger);
        }
        Ok(())
    }
}

/// Values can only enter a snapshot after validation, including on FRAM load.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Snapshot([f32; COUNT]);

impl Default for Snapshot {
    fn default() -> Self {
        let mut values = [0.0; COUNT];
        for entry in REGISTRY {
            values[entry.id as usize] = entry.default;
        }
        Self(values)
    }
}

impl Snapshot {
    pub fn get(&self, id: Id) -> f32 {
        self.0[id as usize]
    }

    pub fn from_values(values: [f32; COUNT]) -> Result<Self, Error> {
        for entry in REGISTRY {
            let value = values[entry.id as usize];
            entry.validate(value)?;
            if entry.access == Access::ReadOnly && value != entry.default {
                return Err(Error::ReadOnly);
            }
        }
        Ok(Self(values))
    }

    pub fn with(self, id: Id, value: f32) -> Result<Self, Error> {
        let entry = id.definition();
        if entry.access == Access::ReadOnly {
            return Err(Error::ReadOnly);
        }
        entry.validate(value)?;
        let mut values = self.0;
        values[id as usize] = value;
        Ok(Self(values))
    }

    pub fn values(&self) -> &[f32; COUNT] {
        &self.0
    }
}

#[derive(Default)]
pub struct Parameters {
    current: Snapshot,
    saved: Option<Snapshot>,
}

impl Parameters {
    pub fn get(&self, id: Id) -> f32 {
        self.current.get(id)
    }

    /// Returns whether the actual value changed; errors leave it untouched.
    pub fn set(&mut self, id: Id, value: f32) -> Result<bool, Error> {
        let entry = id.definition();
        if entry.access == Access::ReadOnly {
            return Err(Error::ReadOnly);
        }
        entry.validate(value)?;
        let current = &mut self.current.0[id as usize];
        let changed = *current != value;
        *current = value;
        Ok(changed)
    }

    pub fn snapshot(&self) -> Snapshot {
        self.current
    }

    pub fn is_dirty(&self) -> bool {
        self.saved != Some(self.current)
    }

    pub fn restore(&mut self, snapshot: Snapshot) {
        self.current = snapshot;
        self.saved = Some(snapshot);
    }

    /// Adopt a snapshot that is not in the current storage format. The value is
    /// valid RAM state, but nothing on FRAM matches it yet, so keep it dirty and
    /// let the next save rewrite it in the current format.
    pub fn adopt(&mut self, snapshot: Snapshot) {
        self.current = snapshot;
        self.saved = None;
    }

    /// Mark the exact snapshot verified in storage, even if RAM changed meanwhile.
    pub fn mark_saved(&mut self, snapshot: Snapshot) {
        self.saved = Some(snapshot);
    }
}

/// An automatic schema tag catches registry/order/range changes without a version bump.
pub fn schema_tag() -> u32 {
    schema_tag_for(REGISTRY)
}

/// Compact storage key: a record names its entries, so a registry that grew or
/// was reordered still loads every value it can recognise. CRC-16/CCITT-FALSE
/// is small enough for the 16-bit key field and cheap enough for a 1 KiB
/// record on a Cortex-M7. `parameter_store` asserts that no two registry names
/// collide.
pub const fn name_hash(name: &str) -> u16 {
    let bytes = name.as_bytes();
    let mut crc = 0xffffu16;
    let mut index = 0;
    while index < bytes.len() {
        crc ^= (bytes[index] as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
            bit += 1;
        }
        index += 1;
    }
    crc
}

/// The prior 11 parameters keep their order/ranges. Old valid FRAM records can
/// be read without writing or discarding the engineer's logging settings.
pub fn legacy_schema_tag() -> u32 {
    schema_tag_for(&REGISTRY[..11])
}

fn schema_tag_for(registry: &[Definition]) -> u32 {
    let mut hash = 0x811c_9dc5u32;
    let mut feed = |bytes: &[u8]| {
        for &byte in bytes {
            hash = (hash ^ u32::from(byte)).wrapping_mul(0x0100_0193);
        }
    };
    for entry in registry {
        feed(entry.name.as_bytes());
        feed(&[0, entry.kind as u8, entry.access as u8]);
        for value in [entry.default, entry.min, entry.max] {
            feed(&value.to_bits().to_le_bytes());
        }
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_and_validation_enforce_the_public_contract() {
        for (index, entry) in REGISTRY.iter().enumerate() {
            assert!(!entry.name.is_empty() && entry.name.len() <= 16 && entry.name.is_ascii());
            assert!(entry
                .name
                .bytes()
                .all(|byte| byte.is_ascii_uppercase() || byte.is_ascii_digit() || byte == b'_'));
            assert_eq!(Id::from_name(entry.name), Some(entry.id));
            assert_eq!(Id::from_index(index), Some(entry.id));
            assert!(!REGISTRY[..index]
                .iter()
                .any(|other| other.name == entry.name));
            assert_eq!(entry.validate(entry.default), Ok(()));
        }
        assert_eq!(Id::from_name("UNKNOWN"), None);
        assert_eq!(Id::from_index(COUNT), None);
        assert_eq!(
            Id::MavSysId.definition().validate(1.5),
            Err(Error::NotInteger)
        );

        let mut parameters = Parameters::default();
        assert_eq!(parameters.set(Id::MavSysId, 2.0), Err(Error::ReadOnly));
        for (value, error) in [
            (f32::NAN, Error::NonFinite),
            (f32::INFINITY, Error::NonFinite),
            (f32::NEG_INFINITY, Error::NonFinite),
            (0.0, Error::OutOfRange),
            (10.1, Error::OutOfRange),
        ] {
            assert_eq!(parameters.set(Id::LinkLedHz, value), Err(error));
            assert_eq!(parameters.get(Id::LinkLedHz), 1.0);
        }
        for value in [0.1, 2.0, 10.0] {
            assert_eq!(parameters.set(Id::LinkLedHz, value), Ok(true));
            assert_eq!(parameters.get(Id::LinkLedHz), value);
            assert_eq!(parameters.set(Id::LinkLedHz, value), Ok(false));
        }
        let mut invalid = Snapshot::default().0;
        invalid[Id::MavSysId as usize] = 2.0;
        assert_eq!(Snapshot::from_values(invalid), Err(Error::ReadOnly));
        invalid = Snapshot::default().0;
        invalid[Id::LinkLedHz as usize] = f32::NAN;
        assert_eq!(Snapshot::from_values(invalid), Err(Error::NonFinite));
    }

    #[test]
    fn saving_an_older_snapshot_does_not_hide_new_changes() {
        let mut parameters = Parameters::default();
        assert!(parameters.is_dirty());
        parameters.restore(Snapshot::default());
        assert!(!parameters.is_dirty());
        parameters.set(Id::LinkLedHz, 2.0).unwrap();
        let saving = parameters.snapshot();
        parameters.set(Id::LinkLedHz, 3.0).unwrap();
        parameters.mark_saved(saving);
        assert!(parameters.is_dirty());
        parameters.set(Id::LinkLedHz, 2.0).unwrap();
        assert!(!parameters.is_dirty());
        parameters.restore(saving);
        assert_eq!(parameters.get(Id::LinkLedHz), 2.0);
    }
}
