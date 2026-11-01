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

pub struct Definition {
    pub id: Id,
    pub name: &'static str,
    pub kind: Kind,
    pub default: f32,
    pub min: f32,
    pub max: f32,
    pub access: Access,
}

// Generate typed keys and the wire-order table together: one entry per parameter.
macro_rules! parameter_table {
    ($($id:ident { name: $name:literal, kind: $kind:ident, default: $default:expr,
        min: $min:expr, max: $max:expr, access: $access:ident }),+ $(,)?) => {
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #[repr(usize)]
        pub enum Id { $($id),+ }

        pub const REGISTRY: &[Definition] = &[
            $(Definition { id: Id::$id, name: $name, kind: Kind::$kind, default: $default,
                min: $min, max: $max, access: Access::$access }),+
        ];
    };
}

parameter_table! {
    MavSysId {
        name: "MAV_SYS_ID", kind: Integer, default: 1.0,
        min: 1.0, max: 255.0, access: ReadOnly
    },
    LinkLedHz {
        name: "LINK_LED_HZ", kind: Real, default: 1.0,
        min: 0.1, max: 10.0, access: NextCycle
    },
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
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    ReadOnly,
    NonFinite,
    OutOfRange,
    NotInteger,
}

impl Error {
    pub fn message(self) -> &'static str {
        match self {
            Self::ReadOnly => "PARAM_SET: read-only",
            Self::NonFinite => "PARAM_SET: non-finite value",
            Self::OutOfRange => "PARAM_SET: out of range",
            Self::NotInteger => "PARAM_SET: expected integer",
        }
    }
}

impl Definition {
    fn validate(&self, value: f32) -> Result<(), Error> {
        if !value.is_finite() {
            return Err(Error::NonFinite);
        }
        if value < self.min || value > self.max {
            return Err(Error::OutOfRange);
        }
        if self.kind == Kind::Integer && (value as i32) as f32 != value {
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

    /// Mark the exact snapshot verified in storage, even if RAM changed meanwhile.
    pub fn mark_saved(&mut self, snapshot: Snapshot) {
        self.saved = Some(snapshot);
    }
}

/// An automatic schema tag catches registry/order/range changes without a version bump.
pub fn schema_tag() -> u32 {
    let mut hash = 0x811c_9dc5u32;
    let mut feed = |bytes: &[u8]| {
        for &byte in bytes {
            hash = (hash ^ u32::from(byte)).wrapping_mul(0x0100_0193);
        }
    };
    for entry in REGISTRY {
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
