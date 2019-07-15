use kalman_rs as krs;
use krs::config::*;
use krs::generate_data::setup::generate_track;

use krs::{print, get_unchecked};

use rand::rngs::SmallRng;
use rand::SeedableRng;
use rand_distr::{Normal, Distribution};

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

            // print!{left, right, diff}

            assert!{diff <= 0.15}
        )+
    };
}


const UNCERTAINTY : Real = 2.;

fn gen_cov() -> Mat2{
    let base_cov = Mat2::identity();     // high covariance across the main diagonal
    let cov_noise = Mat2::new_random() / UNCERTAINTY;       // every index random # in [0,1] * uncertainty
    let covariance = base_cov + cov_noise;
    covariance
}



// #[test]
fn linear_1() {
    let distance_between_sensors: Real = 0.001;
    let number_of_sensors: u32 = 20;
    let diagonal_rng = Normal::new(3., 1.5).unwrap();
    let corner_rng = Normal::new(0., 1.).unwrap();
    let point_std_dev = 0.01;
    let mut rng = SmallRng::from_entropy();

    print!{ "TEST STATISTICS:",
        distance_between_sensors,
        number_of_sensors,
        point_std_dev
    }

    let data = generate_track(
        number_of_sensors, 
        distance_between_sensors,
        Some((0., PI/2.)),                // along the x axis
        rng, 
        point_std_dev, 
        diagonal_rng,corner_rng
    );
    let kf_result = krs::filter::linear::run(&data.start, &data.cov, &data.smear_hits, &data.sensors);

    
}


// generate track 
#[test]                 
fn linear_3() {
    let distance_between_sensors: Real = 0.001;
    let number_of_sensors: u32 = 20;
    let diagonal_rng = Normal::new(3., 1.5).unwrap();
    let corner_rng = Normal::new(0., 1.).unwrap();
    let point_std_dev = 0.01;
    let rng = SmallRng::from_entropy();

    print!{ "TEST STATISTICS:",
        distance_between_sensors,
        number_of_sensors,
        point_std_dev
    }

    let data = generate_track(
        number_of_sensors, 
        distance_between_sensors,
        None,
        rng, 
        point_std_dev, 
        diagonal_rng,corner_rng
    );
    let kf_result = krs::filter::linear::run(&data.start, &data.cov, &data.smear_hits, &data.sensors);

    

}