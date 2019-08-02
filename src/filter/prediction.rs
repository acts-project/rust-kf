use super::super::config::*;
use super::super::error::*;
use super::super::geometry::traits::{Plane, Transform};
use super::super::geometry::*;
use super::angles;
use super::jacobian::RungeKuttaStep;
#[macro_use]
use super::macros;

// extrapolating state vector
// NOTE: this can only be used for linear systems
pub fn state_vector(
    jacobian: &Mat5,            // J or F_k-1
    prev_filt_state_vec: &Vec5, // prev filt x
) -> Vec5 {
    // pred x

    return jacobian * prev_filt_state_vec;
}

// prediction of covariance matrix C
pub fn covariance_matrix(
    jacobian: &Mat5,                 // J or F_k-1
    prev_filt_covariance_mat: &Mat5, // prev filt C
) -> Mat5 {
    // pred C

    return jacobian * prev_filt_covariance_mat * jacobian.transpose();
}

// just below eq. 7
// residual covariance of predicted results
pub fn residual_mat(
    V: &Mat2,                    // V
    sensor_mapping_mat: &Mat2x5, // H
    pred_covariance_mat: &Mat5,  // pred C
) -> Mat2 {
    // pred R

    return V + (sensor_mapping_mat * pred_covariance_mat * sensor_mapping_mat.transpose());
}

pub fn residual_vec(
    measurement_vec: &Vec2,      // m_k
    sensor_mapping_mat: &Mat2x5, // H
    pred_state_vec: &Vec5,       // pred x
) -> Vec2 {
    // pred r

    let prod = sensor_mapping_mat * pred_state_vec;
    let diff = measurement_vec - prod;

    return diff;
}

/// Calculates the predicted location of the hit on the following sensor
// based on this equation set https://i.imgur.com/mWC0qkj.png
pub fn linear_state_vector<T: Transform + Plane>(
    start_sensor: &T,
    end_sensor: &T,
    prev_filt_state_vec: &Vec5,
) -> (Vec5, Real) {
    get_unchecked! {
        prev_filt_state_vec[eLOC_0] => start_local_x_hit,
        prev_filt_state_vec[eLOC_1] => start_local_y_hit
    }

    let start_local_point = P3::new(*start_local_x_hit, *start_local_y_hit, 0.0);
    let start_global_point = start_sensor.to_global(start_local_point);

    // used so we can be generic over planar sensors
    let normal = end_sensor.plane_normal_vec();

    let end_plane_constant = end_sensor.plane_constant();

    linear_from_one_sensor(prev_filt_state_vec, start_global_point, end_sensor)
}

pub fn linear_from_one_sensor<T: Transform + Plane>(
    prev_filt_state_vec: &Vec5,
    global_start: P3,
    end_sensor: &T,
) -> (Vec5, Real) {
    //

    get_unchecked! {
        prev_filt_state_vec[eTHETA] => theta,
        prev_filt_state_vec[ePHI] => phi,
        prev_filt_state_vec[eQOP]=> qop
    }

    let mut angles = angles::Angles::new_from_angles(*phi, *theta);

    let (global_pred_point, distance) = linear_global_hit_estimation(
        end_sensor.plane_normal_vec(),
        &global_start,
        &mut angles,
        &end_sensor.plane_constant(),
    );

    let local_pred_point = end_sensor.to_local(global_pred_point);

    let new_state = Vec5::new(local_pred_point.x, local_pred_point.y, *phi, *theta, *qop);

    (new_state, distance)
}

/// Encapsultes all the global calculations for `linear_state_vector` without
/// needing start and end sensors. This is done so it is easier to construct unit tests
/// for this particularly troubling part of code.
pub fn linear_global_hit_estimation(
    plane_normal_vector: &Vec3,
    start_global_point: &P3,
    angles: &mut angles::Angles,
    end_plane_constant: &Real,
) -> (P3, Real) {
    // to be honest i am not sure _at all_ why this vector has to be reversed,
    // but the tests where x and y are not both positive will fail without this.
    let normal_vector = -plane_normal_vector;

    // find the slopes of the unit vector from the starting point to the end sensor
    let x_slope = angles.tx;
    let y_slope = angles.ty;
    let z_slope = angles.tz;

    // print!{normal_vecotr, angles, x_slope, y_slope, z_slope}

    //generic numerator for repetitive calculations
    let gen_num_1 = normal_vector.x * start_global_point.x;
    let gen_num_2 = normal_vector.y * start_global_point.y;
    let gen_num_3 = normal_vector.z * start_global_point.z;
    let gen_num = gen_num_1 + gen_num_2 + gen_num_3 + end_plane_constant;

    // print!{gen_num_1, gen_num_2, gen_num_3}

    // generic denominator for repetitive calculations
    let gen_den_1 = normal_vector.x * x_slope;
    let gen_den_2 = normal_vector.y * y_slope;
    let gen_den_3 = normal_vector.z * z_slope;
    let gen_den = gen_den_1 + gen_den_2 + gen_den_3;

    // print!{gen_den_1,gen_den_2, gen_den_3}

    let gen_division = gen_num / gen_den;

    // print!{gen_num, gen_den, gen_division}

    // calculate predicted points of intersection on ending plane
    let pred_x = start_global_point.x - (x_slope * gen_division);
    let pred_y = start_global_point.y - (y_slope * gen_division);
    let pred_z = start_global_point.z - (z_slope * gen_division);

    // print!{pred_x, pred_y, pred_z};

    let global_pred_point = P3::new(pred_x, pred_y, pred_z);

    let distance = (global_pred_point - start_global_point).norm();

    return (global_pred_point, distance);
}

/// Adjusts the Runge-Kutta state vector based on the current state vector.
///NOTE: Since u and u' are 4-row vectors and k1-4 are 3-row vectors I interpret
///      This to mean the top 3 components of u and u' are mutated since there is no
///      energy loss. I am not entirely sure if this is correct.
pub fn rk_current_global_location(
    step_data: &RungeKuttaStep,
    previous_rk_state_vector: &mut Vec8,
) -> () {
    // print!{"before RK state adjust: ", previous_rk_state_vector}

    // adjust u
    let uprime_slice = previous_rk_state_vector.fixed_slice::<U3, U1>(4, 0); // <U3, U1> is a potential source of error. better ask about.
    let sum = (step_data.h * uprime_slice)
        + ((step_data.h * step_data.h / 6.) * (step_data.k1 + step_data.k2 + step_data.k3));

    let mut u_slice = previous_rk_state_vector.fixed_slice_mut::<U3, U1>(0, 0); // same here

    u_slice += sum;

    // adjust u'
    let mut uprime_slice = previous_rk_state_vector.fixed_slice_mut::<U3, U1>(4, 0);
    uprime_slice += (step_data.h / 6.)
        * (step_data.k1 + (2. * step_data.k2) + (2. * step_data.k3) + step_data.k4);

    // print!{"after RK state adjust: ", previous_rk_state_vector}
}
