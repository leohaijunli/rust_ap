//! Reproduce the board reconfigure path on defaults to find the failing stage.
use v6x_rust_ap::estimator::runtime::Params;
fn main() {
    let values = v6x_rust_ap::parameters::Snapshot::default();
    let params = Params::from_parameters(values).expect("catalog defaults validate");
    // Mirror drivers/estimator.rs: construct a dormant runtime and reconfigure.
    let mut runtime = v6x_rust_ap::estimator::runtime::Runtime::dormant();
    match runtime.reconfigure(params) {
        Ok(()) => println!("reconfigure ok"),
        Err(fault) => println!("reconfigure failed: {fault:?}"),
    }
}
