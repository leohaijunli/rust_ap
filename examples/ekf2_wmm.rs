//! Host probe of the exact fixed-table WMM Rust lookup.
use std::{env, error::Error};
use v6x_rust_ap::estimator::wmm;

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = env::args().collect();
    if args.len() != 3 {
        return Err("Usage: ekf2_wmm LATITUDE LONGITUDE".into());
    }
    let field = wmm::lookup(args[1].parse()?, args[2].parse()?)
        .map_err(|fault| format!("WMM: {fault:?}"))?;
    println!(
        "{:.9e},{:.9e},{:.9e}",
        field.declination_deg, field.inclination_deg, field.strength_gauss
    );
    Ok(())
}
