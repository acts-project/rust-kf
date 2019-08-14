use super::super::config::*;

use std::ffi::c_void;
use std::os::raw::{
    c_double,
    c_uint
};

#[no_mangle]
/// Takes a poitner to the start of an array and its length and converts it to a &[f64]
/// to be used in the kf operations
/// returns the sum of the values 
pub unsafe extern fn make_array(arr_start: *const c_double, length: c_uint) -> c_double {
    let arr_start = arr_start as *const Real;
    let length = length as usize; 

    // convert the pointer and length to a slice
    let slice = std::slice::from_raw_parts(arr_start, length);

    dbg!{&slice};

    // sum, convert to c++ double, return
    slice.iter().sum::<f64>() as c_double
}

#[no_mangle]
/// Takes a poitner to an Eigen c++ Eigen 3x3 matrix and makes a nalgebra equivalent matrix
pub unsafe extern "C" fn eigen_to_nalgebra(matrix_ptr: *const c_double) {
    // convert to rust types
    let float_ptr = matrix_ptr as *const Real;

    let array = std::slice::from_raw_parts(float_ptr, 9);

    let mut m = Mat3::zeros();
    m.copy_from_slice(array);

    print_!{m};
}

#[no_mangle]
/// Takes pointer to a vector<Vector2d> and converts to rust Vec<nalgebra::Vector2>
pub unsafe extern fn eigen_hits_to_nalgebra_hits(vector_start: *const c_double, length: c_uint) {
    // convert to rust types
    let vector_start = vector_start as *const Real;
    let length = length as usize;

    let slice = std::slice::from_raw_parts(vector_start, length);

    let mut rust_vector = Vec::with_capacity(length);

    for i in (0..length).into_iter().step_by(2) {
        let val_1 = slice.get_unchecked(i);
        let val_2 = slice.get_unchecked(i+1);

        rust_vector.push(
            Vec2::new(*val_1, *val_2)
            );

    }

    dbg!{rust_vector};    
}