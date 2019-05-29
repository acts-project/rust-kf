//! # kalman_rs
//! 
//! `kalman_rs` is a collection of utilities for using the kalman filter
//! 

pub mod config;
pub mod geometry;
pub mod filter_gain_matrix;
pub mod filter_weighted_means;

pub use geometry::rectangle::Rectangle;
// pub use geometry::trapezoid::Trapezoid;
pub use geometry::traits as sensor_traits;

