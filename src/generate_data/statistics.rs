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

use super::run::{Uncertainty, State};

use itertools::izip;

/// Quickly create a repetitive value
/// $name: name of variable to save to
/// $value: the value that will be repeated $take times
macro_rules! take {
    ($take:expr; $($name:ident , $value:expr),+) => {
        $(
            let $name = std::iter::repeat($value).take($take);
        )+
    };
}


/// Runs batches of kf calculations. Parallelization happens upstream
pub fn collect_stats(
    state: &State
    ) -> Vec<(KFData<Rectangle>, SuperData)> {
    // ) -> () {
    let distr_rng = &state.stdevs;
    let diagonal_rng = Normal::new(distr_rng.diag_mean, distr_rng.diag_std).unwrap();
    let corner_rng = Normal::new(distr_rng.corner_mean, distr_rng.corner_std).unwrap();
    let rng = SmallRng::from_entropy();

    // create iterators of repetitve values
    take!{state.iterations;
        num_sensors, state.num_sensors,
        distances, state.sensor_distance,
        angles, state.angles,
        point_std, state.stdevs.point_std
    }

    // zip the iterators together
    let iter = izip!{num_sensors, distances, angles, point_std};


    let kf_results_vec : Vec<(KFData<Rectangle>, SuperData)> = 
        iter.map(|(num_sensor, sensor_distance, angles, std_dev)| {

            // generate a truth track
            generate_track(
                num_sensor,
                sensor_distance,
                angles,
                rng.clone(),
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
                &data.sensors
            );

            // print!{"made here"}
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
            create_residuals(truth, kf_ver)         // change this line to compare truth with smeared
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