//! # kalman_rs
//!
//! `kalman_rs` is a collection of utilities for using the kalman filter
//!

#[macro_use]
mod macros;

pub mod config;
pub mod geometry;
#[macro_use]
pub mod filter;
pub mod error;
pub mod generate_data;

pub use geometry::rectangle::Rectangle;
pub use geometry::traits as sensor_traits;
pub use geometry::trapezoid::Trapezoid;

pub mod ffi;
pub use ffi::rust_headers::make_array;

// #[no_mangle]
// pub extern fn ffi_test() {

//     // let arr : &[f64] = &[1., 2., 3., 4.];

//     // let ptr = arr.as_ptr();

//     // let slice = unsafe {
//     //     std::slice::from_raw_parts(ptr, 4)
//     // };

//     // slice.to_vec();
// }

// #[no_mangle]
// pub extern fn float_test(value: c_double) {
//     // value as f64;
// }

// #[no_mangle]
// pub extern fn int_test(value: c_uint ) {
    
// }

// #[no_mangle]
// pub extern fn float_ptr_test(value: *const c_double) {

// }

// #[no_mangle]
// extern fn add(a: c_double, b: c_double) -> c_double {
//     let a = a as f64;
//     let b = b as f64;

//     return (a + b) as c_double
// }