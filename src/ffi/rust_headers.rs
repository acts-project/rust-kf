use super::super::config::*;

use std::ffi::c_void;
use std::os::raw::{
    c_double,
    c_uint
};


#[no_mangle]
/// Takes a poitner to the start of an array and its length and converts it to a slice
/// to be used in the kf operations
pub extern fn make_array(arr_start: *const c_double, length: c_uint) -> c_double {
    let float_ptr = arr_start as *const f64;
    let length = length as usize; 

    // let x = 10;

    let vec = unsafe {
        std::slice::from_raw_parts(float_ptr, length)
    };

    let vec=  vec.to_vec();

    let mut sum : f64= vec.iter().sum();
    sum += 6.;

    dbg!{&vec};

    sum as c_double
}
