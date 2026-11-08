#![forbid(unsafe_code)]

//! Fixed-capacity magnetic source identity, calibration and health selection.
//! Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.
//! Based on pinned PX4 d6f12ad1c4f70ad3230afd7d86e971421e02fef4:
//! sensor_calibration/Magnetometer, DataValidator{,Group}, and FMUv6X buses.
//! No drivers, fitting, automatic parameter writes or navigation qualification.

use crate::{
    parameters::{Id, Snapshot},
    px4_messages::{SensorMag, VehicleMagnetometer},
};

pub const MAX_SOURCES: usize = 4;
pub const TIMEOUT_US: u64 = 300_000;
pub const NO_DATA: u32 = 1;
pub const STALE: u32 = 2;
pub const TIMEOUT: u32 = 4;
pub const ERROR_COUNT: u32 = 8;
pub const ERROR_DENSITY: u32 = 16;
pub const INVALID: u32 = 32;
const IDENTITY: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    DeviceId,
    Capacity,
    Timestamp,
    Data,
    Calibration,
    DuplicateId,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeviceId(u32);

impl DeviceId {
    pub fn decode(raw: u32) -> Result<Self, Error> {
        if raw == 0 || raw > 0xff_ffff || raw >> 16 == 0 {
            return Err(Error::DeviceId);
        }
        let id = Self(raw);
        match (id.bus_type(), id.bus()) {
            (1, 1..=4) if (1..=127).contains(&id.address()) => Ok(id),
            (2, 1..=3 | 5..=6) => Ok(id),
            (3..=6, _) => Ok(id),
            _ => Err(Error::DeviceId),
        }
    }
    pub const fn raw(self) -> u32 {
        self.0
    }
    pub const fn bus_type(self) -> u8 {
        (self.0 & 7) as u8
    }
    pub const fn bus(self) -> u8 {
        ((self.0 >> 3) & 31) as u8
    }
    pub const fn address(self) -> u8 {
        ((self.0 >> 8) & 255) as u8
    }
    pub const fn device_type(self) -> u8 {
        (self.0 >> 16) as u8
    }
    pub const fn external(self) -> bool {
        match self.bus_type() {
            1 => self.bus() != 4,
            2 => self.bus() == 6,
            4 => false,
            _ => true,
        }
    }
}

// PX4 rotation.h rot_lookup, body R = Rz(yaw) Ry(pitch) Rx(roll).
const ROTATIONS: [[u16; 3]; 41] = [
    [0, 0, 0],
    [0, 0, 45],
    [0, 0, 90],
    [0, 0, 135],
    [0, 0, 180],
    [0, 0, 225],
    [0, 0, 270],
    [0, 0, 315],
    [180, 0, 0],
    [180, 0, 45],
    [180, 0, 90],
    [180, 0, 135],
    [0, 180, 0],
    [180, 0, 225],
    [180, 0, 270],
    [180, 0, 315],
    [90, 0, 0],
    [90, 0, 45],
    [90, 0, 90],
    [90, 0, 135],
    [270, 0, 0],
    [270, 0, 45],
    [270, 0, 90],
    [270, 0, 135],
    [0, 90, 0],
    [0, 270, 0],
    [0, 180, 90],
    [0, 180, 270],
    [90, 90, 0],
    [180, 90, 0],
    [270, 90, 0],
    [90, 180, 0],
    [270, 180, 0],
    [90, 270, 0],
    [180, 270, 0],
    [270, 270, 0],
    [90, 180, 90],
    [90, 0, 270],
    [90, 68, 293],
    [0, 315, 0],
    [90, 315, 0],
];

fn euler_matrix(angles_deg: [f32; 3]) -> [[f32; 3]; 3] {
    let [roll, pitch, yaw] = angles_deg.map(f32::to_radians);
    let (sr, cr) = (libm::sinf(roll), libm::cosf(roll));
    let (sp, cp) = (libm::sinf(pitch), libm::cosf(pitch));
    let (sy, cy) = (libm::sinf(yaw), libm::cosf(yaw));
    [
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp, sr * cp, cr * cp],
    ]
}

fn rotate(matrix: [[f32; 3]; 3], v: [f32; 3]) -> [f32; 3] {
    matrix.map(|r| r[0] * v[0] + r[1] * v[1] + r[2] * v[2])
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Calibration {
    pub device_id: u32,
    /// -1 selects the PX4 defaults (internal 50, external 75); 0 disables.
    pub priority: i32,
    pub offset: [f32; 3],
    pub scale: [f32; 3],
    /// XY, XZ, YZ elements of the symmetric soft-iron matrix.
    pub offdiagonal: [f32; 3],
    pub rotation: i32,
    pub angles_deg: [f32; 3],
}

impl Calibration {
    const DEFAULT: Self = Self {
        device_id: 0,
        priority: -1,
        offset: [0.0; 3],
        scale: [1.0; 3],
        offdiagonal: [0.0; 3],
        rotation: -1,
        angles_deg: [0.0; 3],
    };

    fn matrix(self) -> [[f32; 3]; 3] {
        let [a, b, c] = self.scale;
        let [d, e, f] = self.offdiagonal;
        [[a, d, e], [d, b, f], [e, f, c]]
    }

    fn validate(self) -> Result<(), Error> {
        if self.device_id != 0 {
            DeviceId::decode(self.device_id)?;
        }
        let [a, b, c] = self.scale;
        let [d, e, f] = self.offdiagonal;
        let det = a * b * c + 2.0 * d * e * f - a * f * f - b * e * e - c * d * d;
        if !(-1..=100).contains(&self.priority)
            || !(-1..=40).contains(&self.rotation) && self.rotation != 100
            || self
                .offset
                .iter()
                .chain(&self.scale)
                .chain(&self.offdiagonal)
                .chain(&self.angles_deg)
                .any(|v| !v.is_finite())
            || a <= 0.0
            || a * b - d * d <= 1e-6
            || !det.is_finite()
            || det <= 1e-6
        {
            return Err(Error::Calibration);
        }
        Ok(())
    }

    fn correction(self, id: DeviceId, bound: bool) -> Correction {
        // The project's board/body mounting contract is currently rotation 0,
        // shared with its IMU. An internal CAL_MAGn_ROT must not diverge from it.
        let rotation = if !id.external() {
            IDENTITY
        } else if self.rotation == 100 || self.angles_deg.iter().any(|v| *v != 0.0) {
            euler_matrix(self.angles_deg)
        } else {
            euler_matrix(ROTATIONS[self.rotation.max(0) as usize].map(f32::from))
        };
        Correction {
            offset: self.offset,
            scale: self.matrix(),
            rotation,
            bound,
        }
    }
    fn resolved_priority(self, id: DeviceId) -> u8 {
        if self.priority < 0 {
            if id.external() {
                75
            } else {
                50
            }
        } else {
            self.priority as u8
        }
    }
}

impl Default for Calibration {
    fn default() -> Self {
        Self::DEFAULT
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    pub slots: [Calibration; MAX_SOURCES],
}
impl Default for Config {
    fn default() -> Self {
        Self {
            slots: [Calibration::DEFAULT; MAX_SOURCES],
        }
    }
}
impl Config {
    pub fn validate(self) -> Result<Self, Error> {
        for (i, slot) in self.slots.iter().enumerate() {
            slot.validate()?;
            if slot.device_id != 0
                && self.slots[..i]
                    .iter()
                    .any(|s| s.device_id == slot.device_id)
            {
                return Err(Error::DuplicateId);
            }
        }
        Ok(self)
    }
    pub fn from_parameters(values: Snapshot) -> Result<Self, Error> {
        let mut config = Self::default();
        for (i, slot) in config.slots.iter_mut().enumerate() {
            // One contiguous 15-field block per slot, appended to the registry.
            // The name/order contract is checked by a unit test below.
            let get = |offset| {
                values.get(Id::from_index(Id::CalMag0Id as usize + i * 15 + offset).unwrap())
            };
            *slot = Calibration {
                device_id: get(0) as u32,
                priority: get(1) as i32,
                offset: [get(2), get(3), get(4)],
                scale: [get(5), get(6), get(7)],
                offdiagonal: [get(8), get(9), get(10)],
                rotation: get(11) as i32,
                angles_deg: [get(12), get(13), get(14)],
            };
        }
        config.validate()
    }
    fn calibration(self, id: DeviceId) -> (Calibration, bool) {
        self.slots
            .iter()
            .find(|s| s.device_id == id.raw())
            .map_or((Calibration::DEFAULT, false), |s| (*s, true))
    }
    pub fn priority(self, device_id: u32) -> Result<u8, Error> {
        let id = DeviceId::decode(device_id)?;
        let (cal, _) = self.calibration(id);
        Ok(cal.resolved_priority(id))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Correction {
    offset: [f32; 3],
    scale: [[f32; 3]; 3],
    rotation: [[f32; 3]; 3],
    bound: bool,
}
impl Correction {
    fn apply(self, raw: SensorMag) -> [f32; 3] {
        let v = [
            raw.x - self.offset[0],
            raw.y - self.offset[1],
            raw.z - self.offset[2],
        ];
        rotate(self.rotation, rotate(self.scale, v))
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Validator {
    last: u64,
    value: [f32; 3],
    errors: u32,
    density: f32,
    equal: u32,
    invalid: bool,
}
impl Validator {
    fn put(&mut self, timestamp: u64, value: [f32; 3], errors: u32) {
        if errors > self.errors {
            self.density += (errors - self.errors) as f32;
        } else if self.density > 0.0 {
            self.density -= 1.0;
        }
        if self.last != 0 {
            // Preserve PX4's per-axis accumulation/order, not a per-vector count.
            for (previous, next) in self.value.iter().zip(value) {
                if (*previous - next).abs() < 1e-6 {
                    self.equal = self.equal.saturating_add(1);
                } else {
                    self.equal = 0;
                }
            }
        }
        self.last = timestamp;
        self.value = value;
        self.errors = errors;
        self.invalid = false;
    }
    fn status(&mut self, now: u64) -> (f32, u32) {
        let flags = if self.invalid {
            INVALID
        } else if self.last == 0 {
            NO_DATA
        } else if now < self.last || now - self.last > TIMEOUT_US {
            TIMEOUT
        } else if self.equal > 1000 {
            STALE
        } else if self.errors > 10000 {
            ERROR_COUNT
        } else {
            0
        };
        if flags != 0 {
            return (0.0, flags);
        }
        self.density = self.density.min(100.0);
        let confidence = 1.0 - self.density / 100.0;
        (confidence, if confidence > 0.0 { 0 } else { ERROR_DENSITY })
    }
}

#[derive(Clone, Copy, Debug)]
struct Source {
    id: DeviceId,
    raw: SensorMag,
    correction: Correction,
    validator: Validator,
    priority: u8,
    degraded: bool,
    calibration_count: u8,
    correction_generation: u32,
    calibration_invalid: bool,
}
impl Source {
    fn effective_priority(self) -> u8 {
        if self.degraded && self.priority != 0 {
            1
        } else {
            self.priority
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SourceStatus {
    pub device_id: u32,
    pub external: bool,
    pub calibrated: bool,
    pub priority: u8,
    pub effective_priority: u8,
    pub confidence: f32,
    pub flags: u32,
    pub timestamp_sample: u64,
}

#[derive(Clone, Copy, Debug)]
pub struct Selected {
    pub data: VehicleMagnetometer,
    pub index: usize,
    pub external: bool,
    pub reset: bool,
    pub generation: u32,
    pub correction_generation: u32,
}

pub struct Selector {
    config: Config,
    sources: [Option<Source>; MAX_SOURCES],
    current: Option<usize>,
    generation: u32,
    delivered: Option<(u32, u32, u32)>,
    pub switches: u32,
    pub failovers: u32,
}

impl Default for Selector {
    fn default() -> Self {
        Self::new()
    }
}
impl Selector {
    pub const fn new() -> Self {
        Self {
            config: Config {
                slots: [Calibration::DEFAULT; MAX_SOURCES],
            },
            sources: [None; MAX_SOURCES],
            current: None,
            generation: 0,
            delivered: None,
            switches: 0,
            failovers: 0,
        }
    }

    pub fn configure(&mut self, config: Config) -> Result<(), Error> {
        let config = config.validate()?;
        self.config = config;
        for source in self.sources.iter_mut().flatten() {
            let (cal, bound) = config.calibration(source.id);
            let priority = cal.resolved_priority(source.id);
            if priority != source.priority || bound != source.correction.bound {
                source.degraded = false;
            }
            let correction = cal.correction(source.id, bound);
            // Binding/priority alone do not change the field or reset EKF bias.
            if correction.offset != source.correction.offset
                || correction.scale != source.correction.scale
                || correction.rotation != source.correction.rotation
            {
                source.calibration_count = source.calibration_count.wrapping_add(1);
                source.correction_generation = source.correction_generation.wrapping_add(1);
                source.validator.equal = 0;
            }
            source.correction = correction;
            source.priority = priority;
            let field = correction.apply(source.raw);
            source.validator.value = field;
            // Recalibration may fix a field transform, but it must never clear
            // a driver's fault or revive old data without a new sensor report.
            source.calibration_invalid = !valid_field(field);
        }
        Ok(())
    }

    pub fn put(&mut self, raw: SensorMag, now: u64) -> Result<usize, Error> {
        let id = DeviceId::decode(raw.device_id)?;
        let existing = self
            .sources
            .iter()
            .position(|s| s.is_some_and(|s| s.id == id));
        let index = existing
            .or_else(|| self.sources.iter().position(Option::is_none))
            .ok_or(Error::Capacity)?;
        if raw.timestamp == 0
            || raw.timestamp_sample == 0
            || raw.timestamp > now
            || raw.timestamp_sample > raw.timestamp
            || now - raw.timestamp_sample > TIMEOUT_US
        {
            return Err(Error::Timestamp);
        }
        if let Some(source) = self.sources[index] {
            if raw.timestamp_sample <= source.raw.timestamp_sample {
                return Err(Error::Timestamp);
            }
        }
        let (cal, bound) = self.config.calibration(id);
        let correction = cal.correction(id, bound);
        let field = correction.apply(raw);
        if !valid_field([raw.x, raw.y, raw.z]) {
            if let Some(source) = self.sources[index].as_mut() {
                source.validator.invalid = true;
            }
            return Err(Error::Data);
        }
        let priority = cal.resolved_priority(id);
        let source = self.sources[index].get_or_insert(Source {
            id,
            raw,
            correction,
            validator: Validator::default(),
            priority,
            degraded: false,
            calibration_count: 0,
            correction_generation: 0,
            calibration_invalid: !valid_field(field),
        });
        source.raw = raw;
        source.calibration_invalid = !valid_field(field);
        source.validator.put(raw.timestamp, field, raw.error_count);
        Ok(index)
    }

    pub fn invalidate(&mut self, device_id: u32) {
        for source in self
            .sources
            .iter_mut()
            .flatten()
            .filter(|s| s.id.raw() == device_id)
        {
            source.validator.invalid = true;
        }
    }

    pub fn status(&mut self, now: u64) -> [SourceStatus; MAX_SOURCES] {
        self.sources.each_mut().map(|source| {
            source.as_mut().map_or(SourceStatus::default(), |s| {
                let (confidence, flags) = if s.calibration_invalid {
                    (0.0, INVALID)
                } else if now < s.raw.timestamp_sample || now - s.raw.timestamp_sample > TIMEOUT_US
                {
                    (0.0, TIMEOUT)
                } else {
                    s.validator.status(now)
                };
                SourceStatus {
                    device_id: s.id.raw(),
                    external: s.id.external(),
                    calibrated: s.correction.bound,
                    priority: s.priority,
                    effective_priority: s.effective_priority(),
                    confidence,
                    flags,
                    timestamp_sample: s.raw.timestamp_sample,
                }
            })
        })
    }

    pub fn select(&mut self, now: u64) -> Option<Selected> {
        let status = self.status(now);
        let mut best = self.current.filter(|i| status[*i].effective_priority != 0);
        let (mut confidence, mut priority) = best.map_or((-1.0, 0), |i| {
            (status[i].confidence, status[i].effective_priority)
        });
        for (i, s) in status.iter().enumerate() {
            if s.device_id == 0 || s.effective_priority == 0 || s.confidence <= 0.0 {
                continue;
            }
            if confidence < 0.9 && s.confidence >= 0.9
                || s.confidence > confidence && s.effective_priority >= priority
                || (s.confidence - confidence).abs() < 0.01 && s.effective_priority > priority
            {
                best = Some(i);
                confidence = s.confidence;
                priority = s.effective_priority;
            }
        }
        if confidence <= 0.0 {
            best = None;
        }
        if best != self.current {
            if let Some(previous) = self.current {
                self.switches = self.switches.saturating_add(1);
                if status[previous].flags != 0 || status[previous].confidence <= 0.0 {
                    self.failovers = self.failovers.saturating_add(1);
                    self.sources[previous].as_mut().unwrap().degraded = true;
                }
            }
            self.current = best;
            self.generation = self.generation.wrapping_add(1);
        }
        let index = best?;
        let source = self.sources[index].unwrap();
        let key = (
            source.id.raw(),
            source.correction_generation,
            self.generation,
        );
        Some(Selected {
            data: VehicleMagnetometer {
                timestamp: now,
                timestamp_sample: source.raw.timestamp_sample,
                device_id: source.id.raw(),
                magnetometer_ga: source.validator.value,
                calibration_count: source.calibration_count,
            },
            index,
            external: source.id.external(),
            reset: self.delivered != Some(key),
            generation: self.generation,
            correction_generation: source.correction_generation,
        })
    }

    /// Called only after the selected measurement enters the estimator channel.
    pub fn delivered(&mut self, selected: Selected) {
        let source = self.sources[selected.index].unwrap();
        self.delivered = Some((
            selected.data.device_id,
            source.correction_generation,
            selected.generation,
        ));
    }
    pub fn reset_delivery(&mut self) {
        self.delivered = None;
    }
}

fn valid_field(v: [f32; 3]) -> bool {
    v.iter().all(|x| x.is_finite()) && v.iter().any(|x| *x != 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    const INTERNAL: u32 = crate::bmm150::DEVICE_ID;
    const EXTERNAL: u32 = 0x431009;
    fn raw(id: u32, time: u64) -> SensorMag {
        SensorMag {
            timestamp: time,
            timestamp_sample: time,
            device_id: id,
            x: 0.22,
            y: 0.07,
            z: 0.43,
            temperature: f32::NAN,
            error_count: 0,
        }
    }
    fn bound(id: u32) -> Calibration {
        Calibration {
            device_id: id,
            ..Calibration::default()
        }
    }

    #[test]
    fn identity_decodes_bus_and_address_instead_of_assuming_slot_zero_is_internal() {
        let internal = DeviceId::decode(INTERNAL).unwrap();
        assert_eq!(
            (
                internal.device_type(),
                internal.bus_type(),
                internal.bus(),
                internal.address()
            ),
            (0x43, 1, 4, 0x10)
        );
        assert!(!internal.external());
        assert!(DeviceId::decode(EXTERNAL).unwrap().external());
        assert!(DeviceId::decode((0x43 << 16) | (20 << 8) | 3)
            .unwrap()
            .external());
        assert!(!DeviceId::decode((0x43 << 16) | (3 << 3) | 2)
            .unwrap()
            .external());
        assert!(DeviceId::decode((0x43 << 16) | (6 << 3) | 2)
            .unwrap()
            .external());
        for id in [0, 0x01000000, (0x43 << 16) | (4 << 3) | 2, 0x431000] {
            assert_eq!(DeviceId::decode(id), Err(Error::DeviceId));
        }
    }

    #[test]
    fn calibration_is_id_bound_applies_soft_iron_then_rotation_and_internal_keeps_board_axes() {
        let mut config = Config::default();
        // Deliberately use calibration slot 3 for the first external source.
        config.slots[3] = Calibration {
            device_id: EXTERNAL,
            offset: [0.02, 0.01, 0.03],
            scale: [2.0, 3.0, 4.0],
            offdiagonal: [0.1, 0.2, 0.3],
            rotation: 2,
            ..Calibration::default()
        };
        config.slots[1] = Calibration {
            device_id: INTERNAL,
            rotation: 2,
            ..Calibration::default()
        };
        let mut selector = Selector::new();
        selector.configure(config).unwrap();
        assert_eq!(selector.put(raw(EXTERNAL, 1_000_000), 1_000_000), Ok(0));
        let selected = selector.select(1_000_000).unwrap();
        let expected = [-0.32, 0.486, 1.658]; // S * [0.20,0.06,0.40], followed by yaw 90.
        for (got, want) in selected.data.magnetometer_ga.into_iter().zip(expected) {
            assert!((got - want).abs() < 1e-6, "{got} != {want}");
        }
        let (cal, bound) = config.calibration(DeviceId::decode(INTERNAL).unwrap());
        assert!(bound);
        assert_eq!(
            cal.correction(DeviceId::decode(INTERNAL).unwrap(), bound)
                .apply(raw(INTERNAL, 1)),
            [0.22, 0.07, 0.43]
        );
    }

    #[test]
    fn external_priority_only_wins_when_healthy_and_failed_source_does_not_flap_back() {
        let mut selector = Selector::new();
        selector.put(raw(INTERNAL, 1_000_000), 1_000_000).unwrap();
        assert_eq!(selector.select(1_000_000).unwrap().data.device_id, INTERNAL);
        selector.put(raw(EXTERNAL, 1_010_000), 1_010_000).unwrap();
        assert_eq!(selector.select(1_010_000).unwrap().data.device_id, EXTERNAL);
        assert_eq!(selector.failovers, 0);
        selector.invalidate(EXTERNAL);
        assert_eq!(selector.select(1_020_000).unwrap().data.device_id, INTERNAL);
        assert_eq!(selector.failovers, 1);
        selector.put(raw(EXTERNAL, 1_030_000), 1_030_000).unwrap();
        assert_eq!(selector.select(1_030_000).unwrap().data.device_id, INTERNAL);
        assert_eq!(selector.status(1_030_000)[1].effective_priority, 1);
        let mut config = Config::default();
        config.slots[0] = Calibration {
            priority: 80,
            ..bound(EXTERNAL)
        };
        selector.configure(config).unwrap();
        assert_eq!(selector.select(1_030_000).unwrap().data.device_id, EXTERNAL);
    }

    #[test]
    fn disabled_and_stale_sources_do_not_produce_a_selected_or_zero_field() {
        let mut selector = Selector::new();
        let mut config = Config::default();
        config.slots[0] = Calibration {
            priority: 0,
            ..bound(EXTERNAL)
        };
        selector.configure(config).unwrap();
        selector.put(raw(INTERNAL, 1_000_000), 1_000_000).unwrap();
        selector.put(raw(EXTERNAL, 1_000_001), 1_000_001).unwrap();
        assert_eq!(selector.select(1_000_001).unwrap().data.device_id, INTERNAL);
        assert!(selector.select(1_000_001 + TIMEOUT_US).is_none());
        assert!(Selector::new().select(1_000_000).is_none());
    }

    #[test]
    fn reset_is_retained_until_delivered_and_calibration_change_resets_but_priority_does_not() {
        let mut selector = Selector::new();
        selector.put(raw(INTERNAL, 1_000_000), 1_000_000).unwrap();
        let selected = selector.select(1_000_000).unwrap();
        assert!(selected.reset);
        assert!(selector.select(1_000_001).unwrap().reset); // simulates a full estimator channel
        selector.delivered(selected);
        assert!(!selector.select(1_000_001).unwrap().reset);
        let mut config = Config::default();
        config.slots[2] = Calibration {
            offset: [0.01, 0.0, 0.0],
            ..bound(INTERNAL)
        };
        selector.configure(config).unwrap();
        let selected = selector.select(1_000_001).unwrap();
        assert!(selected.reset);
        assert_eq!(selected.data.calibration_count, 1);
        selector.delivered(selected);
        config.slots[2].priority = 55;
        selector.configure(config).unwrap();
        assert!(!selector.select(1_000_001).unwrap().reset);
        selector.reset_delivery();
        assert!(selector.select(1_000_001).unwrap().reset);
    }

    #[test]
    fn bad_future_duplicate_and_fifth_source_reports_are_explicitly_rejected() {
        let mut selector = Selector::new();
        selector.put(raw(INTERNAL, 1_000_000), 1_000_000).unwrap();
        assert_eq!(
            selector.put(raw(INTERNAL, 1_000_000), 1_000_001),
            Err(Error::Timestamp)
        );
        assert_eq!(
            selector.put(raw(EXTERNAL, 1_010_000), 1_000_000),
            Err(Error::Timestamp)
        );
        let mut bad = raw(INTERNAL, 1_000_002);
        bad.x = f32::NAN;
        assert_eq!(selector.put(bad, 1_000_002), Err(Error::Data));
        assert!(selector.select(1_000_002).is_none());
        for i in 0..3 {
            selector
                .put(raw(EXTERNAL + (i << 8), 1_010_000), 1_010_000)
                .unwrap();
        }
        assert_eq!(
            selector.put(raw(EXTERNAL + (3 << 8), 1_020_000), 1_020_000),
            Err(Error::Capacity)
        );
    }

    #[test]
    fn confidence_checks_error_density_equal_values_and_timeout_boundaries() {
        let mut validator = Validator::default();
        validator.put(1_000_000, [1.0, 2.0, 3.0], 0);
        assert_eq!(validator.status(1_000_000 + TIMEOUT_US), (1.0, 0));
        assert_eq!(validator.status(1_000_001 + TIMEOUT_US), (0.0, TIMEOUT));
        validator.put(1_001_000, [1.0, 2.0, 3.0], 101);
        assert_eq!(validator.status(1_001_000), (0.0, ERROR_DENSITY));
        validator.put(1_002_000, [1.1, 2.0, 3.0], 101);
        assert!(validator.status(1_002_000).0 > 0.0);
        validator.errors = 10001;
        assert_eq!(validator.status(1_002_000), (0.0, ERROR_COUNT));
        validator = Validator::default();
        for i in 0..=335 {
            validator.put(1_000_000 + i, [1.0, 2.0, 3.0], 0);
        }
        assert_eq!(validator.status(1_000_335), (0.0, STALE));
    }

    #[test]
    fn all_rotation_codes_and_custom_angles_form_valid_rotations() {
        for rotation in 0..=40 {
            let cal = Calibration {
                rotation,
                ..bound(EXTERNAL)
            };
            let r = cal
                .correction(DeviceId::decode(EXTERNAL).unwrap(), true)
                .rotation;
            for i in 0..3 {
                for j in 0..3 {
                    let dot = (0..3).map(|k| r[i][k] * r[j][k]).sum::<f32>();
                    assert!(
                        (dot - if i == j { 1.0 } else { 0.0 }).abs() < 1e-6,
                        "rotation {rotation}"
                    );
                }
            }
        }
        let cal = Calibration {
            angles_deg: [0.0, 0.0, 90.0],
            ..bound(EXTERNAL)
        };
        let field = cal
            .correction(DeviceId::decode(EXTERNAL).unwrap(), true)
            .apply(raw(EXTERNAL, 1));
        assert!((field[0] + 0.07).abs() < 1e-6 && (field[1] - 0.22).abs() < 1e-6);
    }

    #[test]
    fn parameter_blocks_are_stable_and_invalid_configs_leave_values_unchanged() {
        assert_eq!(
            Config::from_parameters(Snapshot::default()).unwrap(),
            Config::default()
        );
        let suffixes = [
            "ID", "PRIO", "XOFF", "YOFF", "ZOFF", "XSCALE", "YSCALE", "ZSCALE", "XODIAG", "YODIAG",
            "ZODIAG", "ROT", "ROLL", "PITCH", "YAW",
        ];
        for slot in 0..4 {
            for (i, suffix) in suffixes.iter().enumerate() {
                let id = Id::from_index(Id::CalMag0Id as usize + slot * 15 + i).unwrap();
                assert_eq!(id.definition().name, std::format!("CAL_MAG{slot}_{suffix}"));
            }
        }
        let mut values = crate::parameters::Parameters::default();
        values.set(Id::CalMag0Id, INTERNAL as f32).unwrap();
        let before = values.snapshot();
        assert!(values.set(Id::CalMag1Id, INTERNAL as f32).is_err());
        assert!(values.set(Id::CalMag0Rot, 50.0).is_err());
        assert!(values.set(Id::CalMag0Xodiag, 5.0).is_err());
        assert_eq!(before, values.snapshot());
    }

    #[test]
    fn configuration_cannot_clear_driver_faults_or_select_an_old_sample_with_fresh_publication_time(
    ) {
        let mut selector = Selector::new();
        selector.put(raw(INTERNAL, 1_000_000), 1_000_000).unwrap();
        selector.select(1_000_000);
        selector.invalidate(INTERNAL);
        let mut config = Config::default();
        config.slots[0] = bound(INTERNAL);
        selector.configure(config).unwrap();
        assert!(selector.select(1_000_010).is_none());
        let mut delayed = raw(INTERNAL, 1_110_000);
        delayed.timestamp_sample = 1_010_000;
        selector.put(delayed, 1_110_000).unwrap();
        assert!(selector.select(1_010_000 + TIMEOUT_US).is_some());
        assert!(selector.select(1_010_001 + TIMEOUT_US).is_none());
    }
}
