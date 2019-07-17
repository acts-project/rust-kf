use super::super::config::*;
use super::super::geometry::Rectangle;
use super::super::filter;

use super::setup;
use setup::generate_track;

use super::structs::{KFData, Residuals, State};

use filter::{linear, utils::SuperData};

use rand::{thread_rng, SeedableRng};
use rand::rngs::SmallRng;
use rand_distr::Normal;

use itertools::izip;


/// Runs batches of kf calculations. Parallelization happens upstream
pub fn collect_stats(
    state: &State
    ) -> Vec<(KFData<Rectangle>, SuperData)> {
    // ) -> () {
    let uncertainties = &state.stdevs;

    let diagonal_rng = Normal::new(uncertainties.diag_mean, uncertainties.diag_std).unwrap();
    let corner_rng = Normal::new(uncertainties.corner_mean, uncertainties.corner_std).unwrap();
    
    let mut base_rng = thread_rng();
    let small_rngs_iterator = std::iter::repeat(()).map(|_|SmallRng::from_rng(&mut base_rng).unwrap()).take(state.iterations);



    // create iterators of repetitve values
    take!{state.iterations;
        num_sensors, state.num_sensors,
        distances, state.sensor_distance,
        angles, state.angles,
        point_std, state.stdevs.point_std
    }

    // zip the iterators together
    let iter = izip!{num_sensors, distances, angles, point_std, small_rngs_iterator};


    let kf_results_vec : Vec<(KFData<Rectangle>, SuperData)> = 
        iter.map(|(num_sensor, sensor_distance, angles, std_dev,rng)| {

            // generate a truth track
            generate_track(
                num_sensor,
                sensor_distance,
                angles,
                rng,
                std_dev,
                diagonal_rng.clone(),
                corner_rng.clone()
            )

        })
        .map(|data|{

            // put the smeared data from the truth track into the kf
            let kf_data = linear::run(
                &data.start,
                &data.cov,
                &data.smear_hits,
                &data.sensors,
                Some(&data.smear_initial_vector)
            );
            

            // print!{"finish kf pass"}
            (data, kf_data)
        }).collect();

    kf_results_vec
}


/// Parallelizes calculating the difference between the truth value 
/// of a point versus the kf predicted / filtered / smoothed value 
/// of that point. The data from `collect_stats` can be directly 
/// piped into this function
pub fn fetch_kf_residuals(
    create_statistics_data: &Vec<(KFData<Rectangle>, SuperData)>
    ) -> Vec<Residuals> {
    // 

    create_statistics_data.iter()
        .map(|(truth, kf_ver)| {
            create_residuals(truth, kf_ver)
        })
        .collect::<Vec<_>>()
} 

/// Handles calculating all residuals of the truth hits vs KF outputs
/// and returns a struct of all smoothed / filtered / predicted residuals
fn create_residuals(
    truth_data: &KFData<Rectangle>,
    kf_data: &SuperData
    )-> Residuals{

    let truth_points = &truth_data.truth_hits;

    let len = kf_data.smth.state_vec.len();

    // smoothed
    let smth_state_vec = &kf_data.smth.state_vec;
    let smth_resid = calc_residual(smth_state_vec,truth_points, len);

    // predicted
    let pred_state_vec = &kf_data.pred.state_vec;
    let pred_res = calc_residual(pred_state_vec, truth_points, len);

    // filtered
    let filt_state_vec = &kf_data.filt.state_vec;
    let filt_res= calc_residual(filt_state_vec,truth_points, len);

    Residuals {
        smth: smth_resid,
        filt: filt_res,
        pred: pred_res
    }
}


/// Calculates the residuals between truth points and their
/// smeared counterparts
pub fn smear_residuals(
    kf_data: &KFData<Rectangle>
    ) -> Vec<Vec2> {

    let smears = &kf_data.smear_hits;
    let truths = &kf_data.truth_hits;

    truths.iter().zip(smears.iter())
        .map(|(t, s)| {
            t-s
        })
        .collect::<Vec<Vec2>>()
}

pub fn truth_kf_output_residuals(
    output: Vec<(KFData<Rectangle>, SuperData)>
    ) -> Vec<Residuals> {
    //

    output.into_iter()
        .map(|(truth_data, kf_out)| {
            let truth_vals = truth_data.truth_hits.into_iter();
            let prediction = vec5_to_vec2(kf_out.pred.state_vec).into_iter();
            let filtered = vec5_to_vec2(kf_out.filt.state_vec).into_iter();
            let smoothed = vec5_to_vec2(kf_out.smth.state_vec).into_iter();

            // truth_vals.zip(prediction);
            let zipped_iter = izip!{truth_vals, prediction, filtered, smoothed};

            let grouped_residuals = 
            zipped_iter.map(|(truth, pred, filt, smth)|{
                let p_ = truth - pred;
                let f_ = truth - filt;
                let s_ = truth - smth;
                
                (s_, f_, p_)
                })
            .collect::<Vec<_>>();

            Residuals::new_grouped(grouped_residuals)

        })
        .collect::<Vec<_>>()

}

fn vec5_to_vec2(vector_sv: Vec<Vec5>) -> Vec<Vec2> {
    vector_sv.into_iter()
        .map(|x| state_vec_to_hit_vec(x))
        .collect::<Vec<_>>()
}

fn state_vec_to_hit_vec(vec: Vec5)-> Vec2{
    // let new_vec = Vec2::zeros();
    get_unchecked!{vector;vec;
        eLOC_0 => x,
        eLOC_1 => y
    }
    Vec2::new(*x, *y)
}



/// Residual between KF outputs and truth hits
fn calc_residual(
    state_vectors: &Vec<Vec5>,
    truth_points: &Vec<Vec2>,
    len: usize
    ) -> Vec<Vec2>{

    let mut diff_vec = Vec::with_capacity(len);


    for i in 0..len {

        get_unchecked!{
            state_vectors[i] => curr_state_vec,
            truth_points[i] => curr_truth_point
        }

        get_unchecked!{vector;curr_state_vec;
            eLOC_0 => x,
            eLOC_1 => y
        }

        let diff = Vec2::new(*x, *y) - curr_truth_point;
        diff_vec.push(diff);
    }
    

    return diff_vec
}