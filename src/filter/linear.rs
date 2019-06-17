use nalgebra as na;
use super::super::config::*;
use super::prediction;
use super::filter_gain;
use super::smoothing;

use std::iter;

use super::utils::SmoothedData;

// macro to initialize vectors with a given capacity to reduce copy / paste
macro_rules! store_vec {
    // $name: the name of the variable
    // $type: Type of data stored in vector (Mat5 / Vec5)
    // $capacity: How much space to allocate
    ($name:ident, $type:ty, $capacity:expr) => {
        let mut $name: Vec<$type> = Vec::with_capacity($capacity);
    };
}


#[allow(dead_code)]
pub fn run(
    V_vec: &Vec<Mat5>, 
    H_vec: &Vec<Mat5>,
    m_k_vec: &Vec<Vec5>,
)  -> SmoothedData{
    
    if (V_vec.len() == H_vec.len()) && (H_vec.len() == m_k_vec.len()) {}
    else {
        panic!("vector lengths need to be the same length")
    } 
    

    let L = V_vec.len();
    
    // predictions
    // store_vec!(jacobian_vec,          Mat5,L);
    store_vec!(pred_state_vec_vec,    Vec5, L);
    store_vec!(pred_cov_vec,          Mat5, L);
    store_vec!(pred_residual_mat_vec, Mat5, L);
    store_vec!(pred_residual_vec_vec, Vec5, L);
 
    //let mut jacobian_vec = iter::empty();
    let mut filter_state_vec_vec = iter::empty();
    let mut filter_cov_vec = iter::empty();
    //let mut filter_residual_mat_vec = iter::empty();
    //let mut filter_residual_vec_vec = iter::empty();
    //let mut chi_squared_vec = iter::empty();


    // //filters
    // store_vec!(filter_state_vec_vec,    Vec5, L);
    // store_vec!(filter_cov_vec,          Mat5, L);
    // store_vec!(filter_residual_mat_vec, Mat5, L);
    // store_vec!(filter_residual_vec_vec, Vec5, L);
    // store_vec!(chi_squared_vec,         Real, L);

    //smoothed vectors
    store_vec!(smoothed_state_vec_vec,    Vec5, L);
    store_vec!(smoothed_cov_vec,          Mat5, L);
    store_vec!(smoothed_residual_mat_vec, Mat5, L);
    store_vec!(smoothed_residual_vec_vec, Vec5, L);

    // calculate some seeded values
    let previous_state_vec = super::utils::seed_state_vec();
    let previous_covariance = super::utils::seed_covariance();

    // store initial seeded values
    let filter_state_vec_vec = filter_state_vec_vec.chain(iter::repeat(previous_state_vec).take(1));
    let filter_cov_vec = filter_cov_vec.chain(iter::repeat(previous_covariance).take(1));

    for i in 0..L{
        let jacobian = linear_jacobian();

        let H = &H_vec[i];
        let V = &V_vec[i];
        let m_k=&m_k_vec[i];


        //predictions
        let pred_state_vec = prediction::state_vector(&jacobian, &previous_state_vec);
        let pred_cov_mat = prediction::covariance_matrix(&jacobian, &previous_covariance);
        // let pred_residual_mat = prediction::residual_mat(&V, &H, &pred_cov_mat);
        // let pred_residual_vec = prediction::residual_vec(&m_k, &H, &pred_state_vec);

      
        //filtering
        let kalman_gain = filter_gain::kalman_gain(&pred_cov_mat, &H, &V);
        let filter_state_vec = filter_gain::state_vector(&pred_state_vec, &kalman_gain, &m_k, &H);
        // let filter_cov_mat = filter_gain::covariance_matrix(&kalman_gain, &H, &pred_cov_mat);
        // let filter_residual_vec = filter_gain::residual_vec(&H, &kalman_gain, &pred_residual_vec);
        // let filter_residual_mat = filter_gain::residual_mat(&V, &H, &filter_cov_mat);
        // let chi_squared_inc = filter_gain::chi_squared_increment(&filter_residual_vec, &filter_residual_mat);


        //storage
        let filter_state_vec_vec =filter_state_vec_vec    .chain(iter::repeat(filter_state_vec).take(1));
        // jacobian_vec            .chain(iter::repeat(jacobian).take(1));
        // filter_cov_vec          .chain(iter::repeat(filter_cov_mat).take(1));
        // filter_residual_mat_vec .chain(iter::repeat(filter_residual_mat).take(1));
        // filter_residual_vec_vec .chain(iter::repeat(filter_residual_vec).take(1));
        // chi_squared_vec         .chain(iter::repeat(chi_squared_inc).take(1));
    }

    dbg!{"made it this far"};
    // smoothing
    // cycle through the sensors from highest index to lowest index

    let curr_state_vec = filter_state_vec_vec.next().unwrap();
    // for i in (1..L).into_iter().rev(){
    //     let prev = i+1;
    //     let curr = i;
    //     let next = i-1;


    //     //
    //     // initializing variables
    //     // 

    //     //TODO : value only fetch the newest value and move references
    //     //     instead of re-referencing ever iteration
    //     // this means we can chain iterators instead of using malloc 
    //     // for vectors
    //     let prev_state_vec = &filter_state_vec_vec[prev];
    //     let curr_state_vec = &filter_state_vec_vec[curr];
    //     let next_state_vec = &filter_state_vec_vec[next];

    //     // filtered Covariane matrix C
    //     let prev_cov_mat = &filter_cov_vec[prev];
    //     let curr_cov_mat = &filter_cov_vec[curr];
    //     let next_cov_mat = &filter_cov_vec[next];

    //     // predicted covaraince matrix C
    //     // let pred_curr_cov_mat = &pred_cov_vec[curr];

    //     //F_k at the current time step
    //     let curr_jacobian = &jacobian_vec[curr];

    //     let curr_measurement = &m_k_vec[curr];
    //     let curr_H_k = &H_vec[curr];
    //     let curr_V_k = &V_vec[curr];

    //     // 
    //     // smoothing calculations
    //     //

    //     let gain_matrix = smoothing::gain_matrix(&curr_cov_mat, &curr_jacobian, &prev_cov_mat); 
    //     // NOTE: the next [ ] calculations assume that x^n references the next state vector and x^k references the previous 
    //     // state vector. I am uncertain as to what the actual answer is as andi still has not gotten back to me about it.
    //     let smoothed_state_vec = smoothing::state_vector(&curr_state_vec, &gain_matrix, &next_state_vec, &prev_state_vec);
    //     let smoothed_cov_mat = smoothing::covariance_matrix(&curr_cov_mat, &gain_matrix, &next_cov_mat, &prev_cov_mat);
    //     let smoothed_res_mat = smoothing::residual_mat(&curr_V_k, &curr_H_k, &smoothed_cov_mat);
    //     let smoothed_res_vec = smoothing::residual_vec(&curr_measurement, &curr_H_k, &smoothed_state_vec);

    //     // store smoothed parameters
    //     smoothed_state_vec_vec.push(smoothed_state_vec);
    //     smoothed_cov_vec.push(smoothed_cov_mat);
    //     smoothed_residual_mat_vec.push(smoothed_res_mat);
    //     smoothed_residual_vec_vec.push(smoothed_res_vec);


    // }
    
    // put all data into a struct that will contain all the methods to return 
    // the data back to c++
    // return SmoothedData::new(smoothed_state_vec_vec,
                                // smoothed_cov_vec,
                                // smoothed_residual_mat_vec,
                                // smoothed_residual_vec_vec)
                                unimplemented!()

}


// TODO: Covariance transport here
fn linear_jacobian() -> Mat5 {
    return Mat5::identity()
}