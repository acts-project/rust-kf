use super::super::{
    config::*,
    filter::{
        constant_magnetic_field, linear,
        utils::{Data, SuperData},
    },
    geometry::Rectangle,
};

use super::{
    setup,
    structs::{KFData, State},
};

use rand::rngs::SmallRng;
use rand::{thread_rng, SeedableRng};

use rayon::{self, prelude::*};

/// Runs batches of kf calculations. Parallelization happens upstream
pub fn collect_stats(state: &State) -> Vec<(KFData<Rectangle>, SuperData)> {
    // ) -> () {
    // let uncertainties = &state.stdevs;

    let mut base_rng = thread_rng();
    let small_rngs_iterator = std::iter::repeat(())
        .map(|_| SmallRng::from_rng(&mut base_rng).unwrap())
        .take(state.iterations);

    // if we use non-linear tracks
    let b_field_calculations: bool = if state.b_field != Vec3::zeros() {
        true
    } else {
        false
    };

    let kf_results_vec: Vec<(KFData<Rectangle>, SuperData)> = 
        small_rngs_iterator
        .collect::<Vec<_>>()
        .into_par_iter()
        .map(|rng| {
            // if there is a magnetic field...
            if b_field_calculations {
                stats_const_b(&state, rng)
            // otherwise we assume linear KF operations
            } else {
                stats_linear(&state, rng)
            }
        })
        .collect();

    kf_results_vec
}

/// Helper function for running linear kf
fn stats_linear(state: &State, rng: SmallRng) -> (KFData<Rectangle>, SuperData) {
    let data = setup::generate_linear_track(&state, rng);
    let kf_outs = linear::run(
        &data.start,
        &data.cov,
        &data.smear_hits,
        &data.sensors,
        Some(&data.smear_initial_vector),
    );
    (data, kf_outs)
}

/// Helper function for running constant magnetic field kf
fn stats_const_b(state: &State, rng: SmallRng) -> (KFData<Rectangle>, SuperData) {
    let data = setup::generate_const_b_track(&state, rng);
    let kf_outs = constant_magnetic_field::run(
        &data.start,
        &data.cov,
        &data.smear_hits,
        &data.sensors,
        Some(&data.smear_initial_vector),
        &state.b_field,
    );
    (data, kf_outs)
}

/// Calculates the differences between the truth values of a point versus the kf
/// predicted / filtered /smoothed value of that point.
pub fn fetch_kf_residuals_all(
    create_statistics_data: &Vec<(KFData<Rectangle>, SuperData)>,
    pull_distribution: bool,
) -> Vec<(Vec<Vec2>, Vec<Vec2>, Vec<Vec2>)> {
    create_statistics_data
        .iter()
        .map(|(truth, kf_ver)| {
            let predicted_residuals =
                calc_residual(&kf_ver.pred, &truth.truth_hits, pull_distribution);
            let filtered_residuals =
                calc_residual(&kf_ver.filt, &truth.truth_hits, pull_distribution);
            let smoothed_residuals =
                calc_residual(&kf_ver.smth, &truth.truth_hits, pull_distribution);

            (predicted_residuals, filtered_residuals, smoothed_residuals)
        })
        .collect::<Vec<_>>()
}

/// Calculates the residuals between truth points and their
/// smeared counterparts. Used to ensure the random values generated
/// are truly random
pub fn smear_residuals(kf_data: &KFData<Rectangle>) -> Vec<Vec2> {
    let smears = &kf_data.smear_hits;
    let truths = &kf_data.truth_hits;

    truths
        .iter()
        .zip(smears.iter())
        .map(|(t, s)| t - s)
        .collect::<Vec<Vec2>>()
}

/// Converts a vector of 5-row vectors to a vector of 2-row vectors (just this hits)
fn vec5_to_vec2_all(vector_sv: &Vec<Vec5>) -> Vec<Vec2> {
    vector_sv
        .into_iter()
        .map(|x| vec5_to_vec2_one(&x))
        .collect::<Vec<_>>()
}

/// Converts a since 5-row vector to a 2-row vector
fn vec5_to_vec2_one(vec: &Vec5) -> Vec2 {
    // let new_vec = Vec2::zeros();
    get_unchecked! {vector;vec;
        eLOC_0 => x,
        eLOC_1 => y
    }
    Vec2::new(*x, *y)
}

/// Helper function to calculate the residuals between two vectors of hits
fn calc_residual(kf_data: &Data, truth_points: &Vec<Vec2>, pull_distr: bool) -> Vec<Vec2> {
    let len = truth_points.len();
    let mut diff_vec = Vec::with_capacity(len);
    let state_vectors = &kf_data.state_vec;

    for i in 0..len {
        get_unchecked! {
            state_vectors[i] => curr_state_vec,
            truth_points[i] => curr_truth_point
        }

        let kf_hit = vec5_to_vec2_one(curr_state_vec);

        let mut diff = kf_hit - curr_truth_point;

        // if we are dealing with a pull plot we adjust the residual
        if pull_distr {
            let cov = &kf_data.cov_mat[i];
            for j in 0..2 {
                get_unchecked! {
                    cov[(j, j )]=> curr_cov
                }

                edit_matrix! {diff;
                    [j] /= curr_cov
                }
            }
        }

        diff_vec.push(diff);
    }

    return diff_vec;
}
