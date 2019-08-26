use super::super::config::*;
use super::super::filter::{constant_magnetic_field, linear};
// containers for KF output data
use super::super::filter::utils::{Data, DataPtr, SuperData};
use super::super::generate_data::{setup, structs};

use nalgebra as na;

use std::os::raw::{c_double, c_uint};

// prevent Rust from destructing the data before returning to cpp
use std::mem::ManuallyDrop;

#[no_mangle]
/// Takes a poitner to the start of an array and its length and converts it to a &[f64]
/// to be used in the kf operations
/// returns the sum of the values
pub unsafe extern "C" fn make_array(arr_start: *const c_double, length: c_uint) -> c_double {
    let arr_start = arr_start as *const Real;
    let length = length as usize;

    // convert the pointer and length to a slice
    let slice = std::slice::from_raw_parts(arr_start, length);

    dbg! {&slice};

    // sum, convert to c++ double, return
    slice.iter().sum::<f64>() as c_double
}

#[no_mangle]
/// Takes a poitner to an Eigen c++ Eigen 3x3 matrix and makes a nalgebra equivalent matrix
pub unsafe extern "C" fn eigen_to_nalgebra(matrix_ptr: *const c_double) {
    // convert to rust types
    let float_ptr = matrix_ptr as *const Real;

    let slice = std::slice::from_raw_parts(float_ptr, 9);

    let matrix : Mat3 = na::MatrixSlice3::from_slice(slice).into();

    print_! {matrix};
}

#[no_mangle]
/// Takes pointer to a vector<Vector2d> and converts to rust Vec<nalgebra::Vector2>
pub unsafe extern "C" fn eigen_hits_to_nalgebra_hits(
    vector_start: *const c_double,
    length: c_uint,
) {
    // convert to rust types
    let vector_start = vector_start as *const Real;
    let length = length as usize;

    let slice = std::slice::from_raw_parts(vector_start, length);

    let mut rust_vector = Vec::with_capacity(length);

    for i in (0..length).into_iter().step_by(2) {
        get_unchecked! {vector; slice;
            i => val_1,
            i+1 => val_2
        }

        rust_vector.push(Vec2::new(*val_1, *val_2));
    }

    dbg! {rust_vector};
}

#[no_mangle]
// Performs KF operations for linear beam experiments
pub unsafe extern "C" fn run_linear_kf(
    hits_ptr: *const c_double,
    measurement_covariance_ptr: *const c_double,
    sensor_number: c_uint,
) -> DataPtr {
    let sensor_number = sensor_number as usize;

    let (hits, meas) =
        measurement_and_hits_from_ptrs(hits_ptr, measurement_covariance_ptr, sensor_number);
    let start_location = P3::origin();

    // make a sensor along the x axis every 0.001 units
    let sensors = (0..)
        .take(sensor_number + 1)
        .map(|x| setup::gen_sensor(x as Real / 1000.))
        .collect::<Vec<_>>();

    let kf_outputs = linear::run(
        &start_location, // starting position in R3
        &meas,           // vector of measurement covariances
        &hits,           // vector of hits of sensors
        &sensors,        // vector of sensors
        None,            // initial track parameters
    );

    let kf_outputs = ManuallyDrop::new(kf_outputs);

    // return struct of poitners to the matricies
    kf_outputs.smth.ptrs()
}

#[no_mangle]
/// Performs KF operations for constant magnetic fields
pub unsafe extern "C" fn run_const_b_kf(
    hits_ptr: *const c_double,
    measurement_covariance_ptr: *const c_double,
    b_field_ptr: *const c_double,
    sensor_number: c_uint,
) -> DataPtr {
    // convert to rust types
    let sensor_number = sensor_number as usize;
    let b_field_ptr = b_field_ptr as *const Real;

    // get nalgebra hits and measurement covariances from pointers
    let (hits, meas) =
        measurement_and_hits_from_ptrs(hits_ptr, measurement_covariance_ptr, sensor_number);

    let start_location = P3::origin();

    // serialize the Eigen b_field vector into nalgebra b_field vector
    let b_field = ManuallyDrop::new(
        na::MatrixSlice3x1::from_slice(std::slice::from_raw_parts(b_field_ptr, 3)).into(),
    );

    // make a sensor along the x axis every 0.001 units
    let sensors = (0..)
        .take(sensor_number + 1) // +1 since we use a virual sensor at the global starting position to make things nice
        .map(|x| setup::gen_sensor(x as Real / 1000.))
        .collect::<Vec<_>>();

    let kf_outputs = constant_magnetic_field::run(
        &start_location, // starting position in R3
        &meas,           // vector of measurement covariances
        &hits,           // vector of hits of sensors
        &sensors,        // vector of sensors
        None,            // initial track parameters,
        &b_field,
    );

    kf_outputs.smth.ptrs()

    // ffi to c++ here
}

/// utility function to convert C++ Eigen hits / measurement covariances into Rust matricies
/// NOTE: this function is not zero-copy. All matrix data must 
unsafe fn measurement_and_hits_from_ptrs(
    hits_ptr: *const c_double,
    measurement_cov_ptr: *const c_double,
    sensor_count: usize,
) -> (ManuallyDrop<Vec<Vec2>>, ManuallyDrop<Vec<Mat2>>) {
    // convert to rust types
    let hits_ptr = hits_ptr as *const Real;
    let meas_cov_ptr = measurement_cov_ptr as *const Real;

    // sensor hits and measurement hits
    let mut hits: Vec<Vec2> = Vec::with_capacity(sensor_count);
    let mut meas: Vec<Mat2> = Vec::with_capacity(sensor_count);

    for i in 0..sensor_count as isize {
        // get pointers to the current position in memory
        let curr_hits_ptr = hits_ptr.offset(i * 2);
        let curr_meas_cov_ptr = meas_cov_ptr.offset(i * 4);

        // slice 2 values for hits (2x1 mat), 4 values for measurement covariance (2x2 matrix)
        let hits_slice = std::slice::from_raw_parts(curr_hits_ptr, 2);
        let meas_slice = std::slice::from_raw_parts(curr_meas_cov_ptr, 4);

        // Make static MatrixSlice from the bytes slice, call .into() to convert MatrixSlice -> Matrix
        let curr_hit = na::MatrixSlice2x1::from_slice(hits_slice).into();
        let curr_meas_cov = na::MatrixSlice2::from_slice(meas_slice).into();

        // pretty print to stdout
        print_! {&curr_hit, &curr_meas_cov};

        push! {
            curr_hit => hits,
            curr_meas_cov => meas
        }
    }

    // wrap types in `Manually Drop` to prevent destructors
    (ManuallyDrop::new(hits), ManuallyDrop::new(meas))
}
