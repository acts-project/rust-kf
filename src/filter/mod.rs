#[macro_use]
pub mod macros;

pub mod constant_magnetic_field;
pub mod linear;

pub mod angles;
pub mod jacobian;
pub mod utils;
pub mod runge_kutta;

pub mod filter_gain;
pub mod filter_means;
pub mod prediction;
pub mod smoothing;
