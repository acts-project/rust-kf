#[macro_use]
pub mod macros;

pub mod linear;
pub mod constant_magnetic_field;

pub mod angles;
pub mod jacobian;
pub mod utils;

pub mod prediction;
pub mod filter_gain;
pub mod filter_means;
pub mod smoothing;