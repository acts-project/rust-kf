use super::super::config::*;
use super::super::geometry::Rectangle;
use super::setup;
use setup::{generate_track, KFData};

use super::super::filter;
#[macro_use]
use filter::macros;
use filter::{linear, utils::SuperData};

use rand::{thread_rng, Rng, SeedableRng};
use rand::rngs::SmallRng;

use rand_distr::{Normal, Distribution};

use rayon;
use rayon::prelude::*;

use super::run::Uncertainty;

/// Runs batches of kf calculations in parallel and returns their result data
pub fn collect_stats(
    mut num_sensors: Vec<u32>, 
    mut sensor_distance: Vec<Real>, 
    mut base_angles: Vec<Option<(f64, f64)>>,
    mut point_std_dev: Vec<Real>,
    distr_rng: &Uncertainty
    ) -> Vec<(KFData<Rectangle>, SuperData)> {

    let diagonal_rng = Normal::new(distr_rng.diag_mean, distr_rng.diag_std).unwrap();
    let corner_rng = Normal::new(distr_rng.corner_mean, distr_rng.corner_std).unwrap();
    let rng = SmallRng::from_entropy();

    let len = num_sensors.len();
    let mut  iter = Vec::with_capacity(len);
    for i in 0..len{
        let a = num_sensors.remove(0);
        let b = sensor_distance.remove(0);
        let c = base_angles.remove(0);
        let d = point_std_dev.remove(0);
        
        iter.push((a, b, c ,d))
    }

    let kf_results_vec = 
        // iter.par_iter()
        iter.iter()
            .map(|(num_sensor, sensor_distance, angles, std_dev)| {
            generate_track(
                *num_sensor,
                *sensor_distance,
                *angles,
                rng.clone(),
                *std_dev,
                diagonal_rng.clone(),
                corner_rng.clone()
            )
        })
        .map(|data|{
            let kf_data = linear::run(
                &data.start,
                &data.cov,
                &data.smear_hits,
                &data.sensors
            );
            (data, kf_data)
        }).collect::<Vec<_>>();

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

    // create_statistics_data.par_iter()
    create_statistics_data.iter()
        .map(|(truth, kf_ver)| {
            create_residuals(truth, kf_ver)
        })
        .collect::<Vec<_>>()
} 


pub struct Residuals{
    pub smth: Vec<Vec2>,
    pub filt: Vec<Vec2>,
    pub pred: Vec<Vec2>
}

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


fn calc_residual(
    state_vectors: &Vec<Vec5>,
    truth_points: &Vec<Vec2>,
    len: usize
    ) -> Vec<Vec2>{

    let mut diff_vec = Vec::with_capacity(len);


    for i in 0..len {

        // print!{"before"}
        get_unchecked!{
            state_vectors[i] => curr_state_vec,
            truth_points[i] => curr_truth_point
        }
        get_unchecked!{vector;curr_state_vec;
            eLOC_0 => x,
            eLOC_1 => y
        }
        // print!{"after"}

        let diff = Vec2::new(*x, *y) - curr_truth_point;
        diff_vec.push(diff);
    }
    

    return diff_vec
}