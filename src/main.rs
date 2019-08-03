use kalman_rs::config::*;
use kalman_rs::geometry::*;
use kalman_rs::test_data;

use nalgebra as na;

use kalman_rs::generate_data::run;
use kalman_rs::print;

use rayon;

use rand::rngs::SmallRng;
use rand::SeedableRng;

fn main() {
    // run::const_b::runge_kutta_global_locations();
    run::const_b::run_all_stats();
    // run::linear::run_all_stats();

    // gen_track()
}

use traits::Plane;
fn gen_track() {
    let mut state = kalman_rs::generate_data::structs::State::default_const_b("", "");
    state.iterations = 10;
    state.num_sensors = 10;
    state.sensor_distance = 0.01;

    let rng = SmallRng::from_entropy();
    let track = kalman_rs::generate_data::setup::generate_const_b_track(&state, rng);

    print! {track.smear_initial_vector, track.smear_initial_vector}
    let data = kalman_rs::filter::constant_magnetic_field::run(
        &track.start,
        &track.cov,
        &track.smear_hits,
        &track.sensors,
        Some(&track.smear_initial_vector),
        &state.b_field,
    );

    // let track = kalman_rs::generate_data::setup::generate_linear_track(&state, rng);
    // let data = kalman_rs::filter::linear::run(&track.start, &track.cov, &track.smear_hits, &track.sensors, Some(&track.smear_initial_vector));

    print! {data.pred.state_vec.len(), data.filt.state_vec.len(), data.smth.state_vec.len()}

    let init = track.smear_initial_vector.clone();
    let zero_pred = data.pred.state_vec.get(0).unwrap().clone();

    let diff = init - zero_pred;

    print! {init, zero_pred, diff}
    dbg! {state.angles};

    print! {track.sensors[0].global_center()}
}
