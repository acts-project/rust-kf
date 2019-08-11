/*

    Tests to assert that filter_means.rs == filter_gains.rs
    to ensure our filtering steps are correct

*/

use kalman_rs as krs;
use krs::config::*;
use krs::filter::{filter_gain, filter_means};
use krs::print_;

fn run_state_vector() {
    let pred_cov_mat = Mat5::new_random();
    let sensor_map_mat = Mat2x5::new(1., 0., 0., 0., 0., 0., 1., 0., 0., 0.);

    let measurement = Vec2::new_random();
    let v = Mat2::new_random();

    let pred_state_vec = Vec5::new_random();

    // gain
    let _kalman_gain = filter_gain::kalman_gain(&pred_cov_mat, &sensor_map_mat, &v);

    let _gain_state_vector = filter_gain::state_vector(
        &pred_state_vec,
        &_kalman_gain,
        &measurement,
        &sensor_map_mat,
    );

    // means
    let inv_v = v.try_inverse().expect("V not unwrappable");

    // we need to calculate the means matrix before finding the state vector.
    // if run_covariance_matrix() fails then this test _must_ also fail
    let _means_covariance_matrix =
        filter_means::covariance_matrix(&pred_cov_mat, &sensor_map_mat, &inv_v);
    let _means_state_vector = filter_means::state_vector(
        &_means_covariance_matrix,
        &pred_cov_mat,
        &pred_state_vec,
        &sensor_map_mat,
        &inv_v,
        &measurement,
    );

    let mut min_vector = Vec5::zeros();
    min_vector.fill(0.00005);

    print_! {
        _means_state_vector,
        _gain_state_vector
    }

    assert! {(_means_state_vector - _gain_state_vector) < min_vector}
}

fn run_covariance_matrix() {
    let filt_cov_mat = Mat5::new_random();
    let pred_cov_mat = Mat5::new_random();
    let sensor_map_mat = Mat2x5::new(1., 0., 0., 0., 0., 0., 1., 0., 0., 0.);

    let measurement = Vec2::new_random();
    let v = Mat2::new_random();

    let pred_state_vec = Vec5::new_random();
    let inv_v = v.try_inverse().expect("V not unwrappable");

    // gains

    let _kalman_gain = filter_gain::kalman_gain(&pred_cov_mat, &sensor_map_mat, &v);

    let _gain_cov_mat =
        filter_gain::covariance_matrix(&_kalman_gain, &sensor_map_mat, &pred_cov_mat);

    // means

    let _means_covariance_matrix =
        filter_means::covariance_matrix(&pred_cov_mat, &sensor_map_mat, &inv_v);

    let mut min_mat = Mat5::zeros();
    min_mat.fill(0.00005);

    print_! {
        _means_covariance_matrix,
        _gain_cov_mat
    }

    assert! {_means_covariance_matrix - _gain_cov_mat < min_mat}
}

const RUNS: u32 = 100;

#[test]
fn state_vector() {
    (0..RUNS).into_iter().for_each(|_| run_state_vector());
}

#[test]
fn covariance_matrix() {
    (0..RUNS).into_iter().for_each(|_| run_covariance_matrix());
}
