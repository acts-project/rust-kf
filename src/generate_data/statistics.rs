use super::super::config::*;
use super::super::geometry::Rectangle;
use super::setup;
use setup::{generate_track, KFData};

use super::super::filter;
#[macro_use]
use filter::macros;
use filter::{linear, utils::Data};

use rand::{thread_rng, Rng, SeedableRng};
use rand::rngs::SmallRng;

use rand_distr::{Normal, Distribution};

use rayon;
use rayon::prelude::*;


/// Runs batches of kf calculations in parallel and returns their result data
pub fn collect_stats(
    mut num_sensors: Vec<u32>, 
    mut sensor_distance: Vec<Real>, 
    mut base_angles: Vec<Option<(f64, f64)>>,
    mut point_std_dev: Vec<Real>,
    ) -> Vec<(KFData<Rectangle>, Data)> {

    let diagonal_rng = Normal::new(3., 1.5).unwrap();
    let corner_rng = Normal::new(0., 1.).unwrap();
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
        iter.par_iter()
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

