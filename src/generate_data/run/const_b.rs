use super::super::super::{
    config::*,
    filter::{self, angles, jacobian, prediction, utils},
};

//
use super::super::{
    store,
    structs::{self, State},
};

use super::general;

use rand::rngs::SmallRng;
use rand::{thread_rng, SeedableRng};
use rand_distr::{Distribution, Normal};

fn gev_to_joules(gev_val: Real) -> Real {
    let gev_joules: Real = 1.602176487 * 10_f64.powf(-10.);
    // let gev_joules = gev_joules * 10.pow(-10);

    gev_val * gev_joules
}

/// Exports a CSV of x / y / z locations that are produced from RK integration
///
/// This is the foundation of testing the constant magnetic field jacobian
/// calculations.
pub fn runge_kutta_global_locations(data: &State) {
    // let phi = 0.;
    // let theta = PI/2.;
    let qop = gev_to_joules(10.);

    let (phi, theta) = data.angles;
    let mut angles = angles::Angles::new_from_angles(phi, theta);

    let state_vector = Vec5::new(0., 0., phi, theta, data.qop);

    let mut global_state_vec = Vec8::zeros();

    // insert values into the global state vector
    edit_matrix! {global_state_vec;
        [4] = angles.tx,
        [5] = angles.ty,
        [6] = angles.tz,
        [7] = data.qop
    }

    let b_field = data.b_field;

    let step_size = 0.001;
    let iterations = 10_000;
    let mut step_results = Vec::with_capacity(iterations);

    for _ in 0..iterations {
        let step = jacobian::runge_kutta_step(&state_vector, &angles, &b_field, step_size);

        prediction::rk_current_global_location(&step, &mut global_state_vec);

        let global_point = utils::global_point_from_rk_state(&global_state_vec);

        step_results.push(structs::P3Wrap::new(global_point));

        angles = utils::angles_from_rk_state(&global_state_vec);
    }

    store::write_csv(r".\data\rk_points.csv", step_results);
}

pub fn _residuals_after_steps(b_field: Vec3) {
    /*
        Initialzie constants
    */
    let point_count = 10_000;
    let truth = std::iter::repeat(P3::origin()).take(point_count);

    let distr = Normal::new(0., 1.).unwrap();
    let mut rng = SmallRng::from_entropy();

    let phi = 0.;
    let theta = PI / 2.;
    let qop = gev_to_joules(10.);

    let step_size = 0.000000001;
    let iterations = 1000;

    // the only thing the RK takes from the local state vector
    // is the QOP in this situation
    // This is because in the actual jacobian calculation a
    // global state vector is created from the local SV
    // and the global SV is the one being operated on.
    // we hand-code the global SV later
    let state_vec = Vec5::new(0., 0., phi, theta, qop);

    /*

        Create smeared values

    */
    let mut smeared_points = truth
        .map(|x| {
            // create smeared  values
            let x = distr.sample(&mut rng);
            let y = distr.sample(&mut rng);
            let z = distr.sample(&mut rng);

            // create a global state vector, insert the
            // global starting location into it
            let mut global_state_vec = Vec8::zeros();

            edit_matrix! {global_state_vec;
                [0 ,0] = x,
                [1 ,0] = y,
                [2 ,0] = z
            }

            let angles = angles::Angles::new_from_angles(phi, theta);

            (global_state_vec, angles)
        })
        .collect::<Vec<_>>();

    // vector of the results of the RK
    let mut smeared_results_vec = Vec::with_capacity(point_count);

    for _ in 0..point_count {
        let (mut global_state_vec, mut angles) = smeared_points.remove(0);

        // has to be done to avoid compiler error
        let mut global_point = P3::origin();

        for _ in 0..iterations {
            let step = jacobian::runge_kutta_step(&state_vec, &angles, &b_field, step_size);

            prediction::rk_current_global_location(&step, &mut global_state_vec);

            global_point = utils::global_point_from_rk_state(&global_state_vec);

            angles = utils::angles_from_rk_state(&global_state_vec);
        }

        // Note: global_point is in 3 dimensions, but we only store & write 2 of those dimensions
        // to CSVs for python analysis
        smeared_results_vec.push(structs::StorageData::new(global_point.x, global_point.y));
    }

    // write to file
    store::write_csv(
        r".\data\runge_kutta_truth_smear_residuals.csv",
        smeared_results_vec,
    );
}

// residuals between truth vs smeared values
fn test_generated_residuals() -> () {
    let state = State::default_const_b(
        r".\data\generated_truth_smear_residuals",
        "_truth_smear_residuals.png",
    );
    general::fetch_kf_randomness_residuals(&state);
}

// residuals between truth and sensor (pred/  filt/ smth) at each sensor
fn test_initial_predictions() -> () {
    let state = State::default_const_b(r".\data\initial_prediction_data\", "_.png");

    general::fetch_separated_kf_data(&state);
}

fn residuals_after_steps_b_field() {
    let b_magnitude: Real = 2.;
    let indiv_field = (b_magnitude.powf(2.) / 3.).sqrt();
    let b_field = Vec3::new(indiv_field, indiv_field, indiv_field);

    _residuals_after_steps(b_field);
}

/// Does runge-kutta through 3d space and
fn global_prop_zero_field() {
    let mut state = State::default_const_b("_", "_");
    state.b_field = Vec3::zeros();

    runge_kutta_global_locations(&state);
}

/// Sensor-separated data of doing runge-kutta with no b-field
/// This tests if runge-kutta produces a strighat line when there
/// is no  b_field
fn zero_field_sensor_sep_data() {
    let mut state = State::default_const_b(r".\data\zero_field_rk", "_");
    // make a very small field so that statistics::collect_stats still treats it
    // as a requiring runge-kutta
    let field = Vec3::new(0., 0., 0.00000000000000000000000000000000001);

    state.b_field = field;
    general::fetch_separated_kf_data(&state);
}

fn pull_data_all() {
    let mut state = State::default(r".\data\pull_data\", "pull_data.png");

    state.iterations = 100_000;
    state.b_field = Vec3::new(0., 0., 0.00000000000000000000000000000000001);

    general::pull_distribution(&state, false);
}

fn pull_data_one() {
    let mut state = State::default(r".\data\pull_data\", "pull_data.png");

    state.iterations = 1000;
    state.b_field = Vec3::new(0., 0., 0.00000000000000000000000000000000001);

    general::pull_distribution(&state, true);
}

pub fn run_all_stats() {
    // test_generated_residuals();
    // test_initial_predictions();
    // residuals_after_steps_zero_field();
    // global_prop_zero_field();

    // zero_field_sensor_sep_data();

    pull_data_all();
    // pull_data_one();
}
