use super::super::{
    statistics, store,
    structs::{Residuals, State, StorageData},
};

use super::general;

use std::fs;

use super::super::super::config::*;

use rayon::{self, prelude::*};

pub fn scaling_sensor_dist() -> () {
    let distances = [0.1, 0.01, 0.001, 0.0001, 0.00001, 0.000001, 0.0000001];

    // folder csvs are dumped to ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼    ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼name of generated histogram
    generate_data! {distances, "scale_sensor_distance_{}", "{}_sensor_distance.png", sensor_distance}
    //             ^^^^^^^^^               field of State that we modify with this test^^^^^^^^^^^^^
} //    non default value we edit into the struct

pub fn scaling_point_std_dev() -> () {
    let std_devs = [
        0.1, 0.01, 0.001, 0.0001, 0.001, 0.00001, 0.000001, 0.0000001,
    ];

    generate_data! {std_devs, "scaling_point_std_dev{}", "{}_hit_standard_deviation.png", stdevs.point_std}
}

pub fn scaling_diagonal_mean() -> () {
    let std_devs = [
        0., 0.5, 1., 1.5, 2., 2.5, 3., 3.5, 4., 4.5, 5., 5.5, 6., 6.5,
    ];

    generate_data! {std_devs, "scaling_diagonal_mean{}", "{}_diagonal_mean.png", stdevs.diag_mean}
}

pub fn scaling_corner_mean() -> () {
    let std_devs = [
        0., 0.5, 1., 1.5, 2., 2.5, 3., 3.5, 4., 4.5, 5., 5.5, 6., 6.5,
    ];

    generate_data! {std_devs, "scaling_corner_mean{}", "{}_corner_mean.png", stdevs.corner_mean}
}

pub fn scaling_sensor_count() -> () {
    let counts = [5, 10, 15, 20, 40, 50, 60];

    generate_data! {counts, "scaling_sensor_count{}", "{}_sensor_count.png", num_sensors}
}

// residuals between truth vs smeared values
fn test_generated_residuals() -> () {
    let state = State::default(
        "generated_truth_smear_residuals",
        "_truth_smear_residuals.png",
    );
    general::fetch_kf_randomness_residuals(&state);
}

// residuals between truth and sensor (pred/  filt/ smth) at each sensor
fn test_initial_predictions() -> () {
    let state = State::default(r".\data\initial_prediction_data\", "");
    general::fetch_separated_kf_data(&state);
}

// Runs a singular test with default State parameters
fn run_one_test() -> () {
    let state = State::default(r"E:\kf_csvs\default_parameters", "default_parameters.png");
    general::run(state);
}

fn ridder_algo() -> () {
    let mut state = State::default(r".\data\ridder_algo_data\", "ridder_data.png");
    state.num_sensors = 2;
    state.angles = (0., PI / 2.);
    general::sensor_separated_with_truth(&state);
}

pub fn run_all_stats() {
    // scaling_corner_mean();
    // scaling_point_std_dev();
    // scaling_diagonal_mean();
    // scaling_sensor_dist();
    // scaling_sensor_count();

    // test_generated_residuals();
    test_initial_predictions();

    // run_one_test();
}
