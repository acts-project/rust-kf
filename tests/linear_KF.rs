use kalman_rs as krs;
use krs::config::*;
use krs::generate_data::{
    setup::generate_linear_track,
    structs::{self, State},
};

use krs::{get_unchecked, print_};

use rand::rngs::SmallRng;
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};

/*

    This is the unit test for the entire linear kalman filter

*/

macro_rules! group_assert {
    ($left:ident, $right:ident, $($index:ident),+) => {
        $(
            let left = $left.$index;
            let right = $right.$index;

            let diff = right - left;
            let diff = diff.abs()

            // print_!{left, right, diff}

            assert!{diff <= 0.15}
        )+
    };
}

/*

    TODO: Actually write tests that verify the locations of the points based on the standard deviations of
            the predicted / smoothed / filtered results at each sensor. Currently these tests just set up
            a future situation where this would be done

            For the most part, tests are done based on manually examining graphs of KF-generated outputs

*/

#[test]
fn linear_1() {
    let mut state = structs::State::default("_", "_");
    let mut rng = SmallRng::from_entropy();

    state.angles = (0., PI / 2.);

    let data = generate_linear_track(&state, rng);

    let kf_result = krs::filter::linear::run(
        &data.start,
        &data.cov,
        &data.smear_hits,
        &data.sensors,
        None,
    );
}
#[test]
fn linear_2() {
    let mut state = structs::State::default("_", "_");
    let mut rng = SmallRng::from_entropy();

    state.angles = (0., PI / 2.);

    let data = generate_linear_track(&state, rng);

    let kf_result = krs::filter::linear::run(
        &data.start,
        &data.cov,
        &data.smear_hits,
        &data.sensors,
        None,
    );
}
