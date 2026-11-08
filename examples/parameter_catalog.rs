//! Authoritative host export for compile-time QGC metadata generation.

use v6x_rust_led::{
    estimator::{ekf2_parameter, runtime_params::Params},
    parameters::{Access, Definition, Group, Id, Kind, Snapshot, REGISTRY},
};

fn inspection_params() -> Params {
    // Inspect possible EKF mappings independently of the default enable/selector.
    let values = Snapshot::default()
        .with(Id::EstEnable, 1.0)
        .unwrap()
        .with(Id::EstAlgorithm, 1.0)
        .unwrap()
        .with(Id::Ekf2En, 1.0)
        .unwrap();
    Params::from_parameters(values).expect("inspection profile is valid")
}

fn integration(entry: &Definition, params: &Params) -> &'static str {
    if entry.group != Group::Ekf2 {
        return "runtime";
    }
    if !params.maps(entry.id) {
        return "unmapped";
    }
    let name = entry.name;
    if name.starts_with("EKF2_BARO_") {
        "conditional:baro"
    } else if name.starts_with("EKF2_MAG_") || name == "EKF2_HEAD_NOISE" {
        "conditional:mag"
    } else if name.starts_with("EKF2_GPS_") || name.starts_with("EKF2_REQ_") {
        "conditional:gps"
    } else if name.starts_with("EKF2_RNG_")
        || name.starts_with("EKF2_TERR_")
        || name == "EKF2_MIN_RNG"
    {
        "conditional:range"
    } else if name.starts_with("EKF2_GND_") {
        "conditional:ground"
    } else if name == "EKF2_HGT_REF" {
        "conditional:height"
    } else {
        "mapped"
    }
}

fn units(entry: &Definition) -> &'static str {
    if let Some(definition) = ekf2_parameter(entry.name) {
        definition.unit
    } else if entry.group == Group::Compass && entry.name.ends_with("OFF") {
        "gauss"
    } else if entry.group == Group::Compass
        && ["ROLL", "PITCH", "YAW"]
            .iter()
            .any(|s| entry.name.ends_with(s))
    {
        "deg"
    } else if entry.name.ends_with("_HZ") || entry.name.starts_with("MAV_RATE_") {
        "Hz"
    } else {
        ""
    }
}

fn main() {
    if std::env::args().any(|arg| arg == "--sizes") {
        println!(
            "Runtime host bytes={}",
            core::mem::size_of::<v6x_rust_led::estimator::product::ProductRuntime>()
        );
        return;
    }
    let params = inspection_params();
    for entry in REGISTRY {
        let group = entry.group.name();
        let access = match entry.access {
            Access::ReadOnly => "read-only",
            Access::NextCycle => "next-cycle",
        };
        let usage = integration(entry, &params);
        let units = units(entry);
        if entry.kind == Kind::Integer {
            println!(
                "{},{},Int32,{},{},{},{},{},{}",
                entry.name,
                group,
                entry.default as i32,
                entry.min as i32,
                entry.max as i32,
                units,
                usage,
                access,
            );
        } else {
            println!(
                "{},{},Float,{},{},{},{},{},{}",
                entry.name, group, entry.default, entry.min, entry.max, units, usage, access,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_integration_label_follows_the_actual_configuration_reads() {
        let params = inspection_params();
        let mut mapped = 0;
        let mut unmapped = 0;
        for entry in REGISTRY {
            let label = integration(entry, &params);
            if entry.group == Group::Ekf2 {
                assert_eq!(label != "unmapped", params.maps(entry.id), "{}", entry.name);
                if label == "unmapped" {
                    unmapped += 1;
                } else {
                    mapped += 1;
                }
            } else {
                assert_eq!(label, "runtime");
            }
        }
        assert_eq!((mapped, unmapped), (78, 64));
    }

    #[test]
    fn unused_features_are_not_mislabelled_as_conditional_sensor_features() {
        let params = inspection_params();
        for (name, expected) in [
            ("EKF2_GYR_NOISE", "mapped"),
            ("EKF2_BARO_NOISE", "conditional:baro"),
            ("EKF2_GPS_CTRL", "conditional:gps"),
            ("EKF2_MAG_NOISE", "conditional:mag"),
            ("EKF2_RNG_CTRL", "conditional:range"),
            ("EKF2_OF_CTRL", "unmapped"),
            ("EKF2_LOG_VERBOSE", "unmapped"),
        ] {
            assert_eq!(
                integration(Id::from_name(name).unwrap().definition(), &params),
                expected
            );
        }
        assert_eq!(units(Id::Ekf2GyrNoise.definition()), "rad/s");
        assert_eq!(units(Id::MavRateHeartbeat.definition()), "Hz");
    }
}
