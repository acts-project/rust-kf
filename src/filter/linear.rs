use nalgebra as na;
use super::super::config::*;
use super::prediction;
use super::filter_gain;
use super::smoothing;

use std::iter;

use super::super::geometry::Rectangle;
use super::super::geometry::traits::{Plane, Transform};

use super::super::error::*;
use super::utils::SmoothedData;

#[macro_use]
use super::macros;

/// Monolithic function to handle linear KF calculations
#[allow(dead_code)] 
pub fn run(
    measurement_noise_coarariance_vector: &Vec<Mat2>,  // vector of V from fruhwirth paper
    measurements_vector: &Vec<Vec2>,            // vector of all the measurements that were registered
    sensor_vector: &Vec<Rectangle>,             // the geometric sensors that correspond to each hit 
    )  -> SmoothedData{

    let meas_map_mat = Mat2x5::new(1.0, 0. , 0. , 0. , 0. ,
                                   0. , 0. , 0. , 0. , 0. );
    
    if (measurement_noise_coarariance_vector.len() == measurements_vector.len()) && (measurements_vector.len() == sensor_vector.len()) {}
    else {
        panic!("vector lengths need to be the same length")
    }
    let input_length = measurements_vector.len();


    store_vec!{
        input_length; // since we have n sensors, we should have n filtered values
        
        jacobian_iter: Mat5,

        // storage for filtered values
        filter_res_mat_iter: Mat2,
        filter_res_vec_iter: Vec2,
        chi_squared_iter: Real,

        // storage for smoothed values
        smoothed_state_vec_iter : Vec5,
        smoothed_cov_mat_iter : Mat5,
        smoothed_res_mat_iter : Mat2,
        smoothed_res_vec_iter: Vec2
    }
    store_vec!{input_length+1; // these vectors require initial seeded values so we initialize to len+1
        filter_state_vec_iter: Vec5,
        filter_cov_mat_iter: Mat5
    }

    // calculate some seeded values (seeding improvement suggestions welcome)
    let mut previous_state_vec = super::utils::seed_state_vec();
    let mut previous_covariance = super::utils::seed_covariance();

    // Store the seeded values in their respective iterators
    push!(
        previous_covariance => filter_cov_mat_iter, 
        previous_state_vec => filter_state_vec_iter
    );


    for i in 0..input_length{
        let jacobian = linear_jacobian();

        // fetch the next values of H / V / m_k
        get_unchecked!{i;
            measurement_noise_coarariance_vector => curr_v,
            measurements_vector=> curr_m_k 
        }

        //predictions
        let pred_state_vec = prediction::state_vector(&jacobian, &previous_state_vec);
        let pred_cov_mat = prediction::covariance_matrix(&jacobian, &previous_covariance);
        let pred_residual_mat = prediction::residual_mat(curr_v, &meas_map_mat, &pred_cov_mat);
        let pred_residual_vec = prediction::residual_vec(&curr_m_k, &meas_map_mat, &pred_state_vec);

      
        //filtering
        let kalman_gain = filter_gain::kalman_gain(&pred_cov_mat, &meas_map_mat, curr_v);
        let filter_state_vec = filter_gain::state_vector(&pred_state_vec, &kalman_gain, curr_m_k, &meas_map_mat);
        let filter_cov_mat = filter_gain::covariance_matrix(&kalman_gain, &meas_map_mat, &pred_cov_mat);
        let filter_residual_vec = filter_gain::residual_vec(&meas_map_mat, &kalman_gain, &pred_residual_vec);
        let filter_residual_mat = filter_gain::residual_mat(curr_v, &meas_map_mat, &filter_cov_mat);
        let chi_squared_inc = filter_gain::chi_squared_increment(&filter_residual_vec, &filter_residual_mat);

        // store all the filtered values in their respective iterators
        push!{
            filter_state_vec =>filter_state_vec_iter,
            jacobian => jacobian_iter,
            filter_cov_mat => filter_cov_mat_iter,
            filter_residual_mat => filter_res_mat_iter,
            filter_residual_vec => filter_res_vec_iter,
            chi_squared_inc => chi_squared_iter
        }
        
        // store current filtered values as the "previous" to be used in the
        // prediction calculations in the next iteration
        previous_covariance = filter_cov_mat;
        previous_state_vec = filter_state_vec;
    }

    // remove the last value from the filter vector and push it to the smoothed version
    push!{remove: input_length-1;
        filter_state_vec_iter => smoothed_state_vec_iter,
        filter_cov_mat_iter => smoothed_cov_mat_iter,
        filter_res_mat_iter => smoothed_res_mat_iter,
        filter_res_vec_iter => smoothed_res_vec_iter
    }


    for i in (0..input_length-1).rev(){
        
        //
        // initializing variables
        // 
        
        // fetch the current variables 
        get_unchecked!{i;
            filter_state_vec_iter => curr_filt_state_vec,
            filter_cov_mat_iter => curr_filt_cov_mat,
            measurement_noise_coarariance_vector => curr_v,
            measurements_vector =>curr_measurement,
            jacobian_iter => curr_jacobian
        }

        // since we move backwards, previous variables are at i+1
        get_unchecked!{i+1;
            filter_state_vec_iter => prev_filt_state_vec,
            filter_cov_mat_iter => prev_filt_cov_mat
        }

        // grab variables pushed in the last iteration
        // (i+2) since input_length is based on the function argument lengths
        // and we also .remove() one value from each vec
        get_unchecked!{input_length -(i+2);                         //TODO: double check this indexing
            smoothed_state_vec_iter => prev_smth_state_vec,
            smoothed_cov_mat_iter => prev_smth_cov_mat
        }

        // 
        // smoothing calculations
        //

        // NOTE: the next calculations assume that x^n references the next state vector and x^k references the previous 
        // state vector. I am uncertain as to what the actual answer is as andi still has not gotten back to me about it.
        let gain_matrix = smoothing::gain_matrix(curr_filt_cov_mat, curr_jacobian, prev_filt_cov_mat); 
        let smoothed_state_vec = smoothing::state_vector(curr_filt_state_vec, &gain_matrix, prev_smth_state_vec, prev_filt_state_vec);
        let smoothed_cov_mat = smoothing::covariance_matrix(curr_filt_cov_mat, &gain_matrix, prev_filt_cov_mat, prev_smth_cov_mat);
        let smoothed_res_mat = smoothing::residual_mat(curr_v, &meas_map_mat, &smoothed_cov_mat);
        let smoothed_res_vec = smoothing::residual_vec(curr_measurement, &meas_map_mat, &smoothed_state_vec);

        //
        //  group push variables to vectors
        //
        push!{
            smoothed_state_vec => smoothed_state_vec_iter,
            smoothed_cov_mat => smoothed_cov_mat_iter,
            smoothed_res_mat => smoothed_res_mat_iter,
            smoothed_res_vec => smoothed_res_vec_iter
        }

    }
    
    // put all data into a struct that will contain all the methods to return 
    // the data back to c++
    return SmoothedData::new(smoothed_state_vec_iter,
                                smoothed_cov_mat_iter,
                                smoothed_res_mat_iter,
                                smoothed_res_vec_iter)

    // 
    // unimplemented!()
}

// TODO: figure out partial derivatives for jacobian calculation
fn linear_jacobian() -> Mat5 {
    return Mat5::identity()
}
