//! Authoritative host export for compile-time QGC metadata generation.

use v6x_rust_led::parameters::{Kind, REGISTRY};

fn main() {
    if std::env::args().any(|arg| arg == "--sizes") {
        println!(
            "Runtime host bytes={}",
            core::mem::size_of::<v6x_rust_led::estimator::product::ProductRuntime>()
        );
        return;
    }
    // The group travels with each definition, so the generator never has to
    // infer it from the name.
    for entry in REGISTRY {
        let group = entry.group.name();
        if entry.kind == Kind::Integer {
            println!(
                "{},{},Int32,{},{},{}",
                entry.name, group, entry.default as i32, entry.min as i32, entry.max as i32
            );
        } else {
            println!(
                "{},{},Float,{},{},{}",
                entry.name, group, entry.default, entry.min, entry.max
            );
        }
    }
}
