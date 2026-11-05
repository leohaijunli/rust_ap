//! Authoritative host export for compile-time QGC metadata generation.

use v6x_rust_led::parameters::{Kind, REGISTRY};

fn main() {
    if std::env::args().any(|arg| arg == "--sizes") {
        println!(
            "Runtime host bytes={}",
            core::mem::size_of::<v6x_rust_led::estimator::runtime::Runtime>()
        );
        return;
    }
    for entry in REGISTRY {
        if entry.kind == Kind::Integer {
            println!(
                "{},Int32,{},{},{}",
                entry.name, entry.default as i32, entry.min as i32, entry.max as i32
            );
        } else {
            println!(
                "{},Float,{},{},{}",
                entry.name, entry.default, entry.min, entry.max
            );
        }
    }
}
