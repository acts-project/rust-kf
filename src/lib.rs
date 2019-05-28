//! # kalman_rs
//! 
//! `kalman_rs` is a collection of utilities for using the kalman filter
//! 


pub mod geometry;

pub use geometry::rectangle::Rectangle;
pub use geometry::trapezoid::Trapezoid;
pub use geometry::traits as sensor_traits;
