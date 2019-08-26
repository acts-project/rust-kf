//! Data generation for testing the validity of the kalman filter's outputs based on residual information
//!
//! Exports csvs and plots for Kalman filter tracks generated
//!
//! `run` contains helper functions for gathering and exporting data in a pre-configured format
//!
//! `setup` creates truth tracks and the smeared ("experiemental simulation") data that is passed into the KF
//!
//! `statistics` calls pre-built `setup` functions for generating unique KF tracks to run. Additionally, it has helper
//! functions for calculating the residuals between the "truth" values before smearing and the KF outputs
//!
//! `structs` containts
//!

// allows us to ignore returned `Result<T, U>`'s that is often done needed when creating
// folders to store data in (since we dont care if the folder already exists)
#![allow(unused_must_use)]

#[macro_use]
pub mod macros;

pub mod run;
pub mod setup;
pub mod statistics;
pub mod store;
pub mod structs;
pub mod traits;
