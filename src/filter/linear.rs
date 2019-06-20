use nalgebra as na;
use super::super::config::*;
use super::prediction;
use super::filter_gain;
use super::smoothing;

use std::iter;

use super::utils::SmoothedData;

#[macro_use]
use super::macros;

/// Monolithic function to handle linear KF calculations
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
    let input_length = V_vec.len();

    store_vec!{
        input_length + 1; // length all vectors will be initialized to (+! for seeded)
        
        jacobian_iter: Mat5,

        // storage for filtered values
        filter_state_vec_iter: Vec5,
        filter_cov_mat_iter: Mat5,
        filter_res_mat_iter: Mat5,
        filter_res_vec_iter: Vec5,
        chi_squared_iter: Real,

        // storage for smoothed values
        smoothed_state_vec_iter : Vec5,
        smoothed_cov_mat_iter : Mat5,
        smoothed_res_mat_iter : Mat5,
        smoothed_res_vec_iter: Vec5
    }

    // calculate some seeded values (seeding improvement suggestions welcome)
    let mut previous_state_vec = super::utils::seed_state_vec();
    let mut previous_covariance = super::utils::seed_covariance();

    // Store the seeded values in their respective iterators
    push!(
        previous_covariance => filter_cov_mat_iter, 
        previous_state_vec => filter_state_vec_iter
    );

    let mut H_iter = H_vec.iter();
    let mut V_iter = V_vec.iter();
    let mut M_k_iter = m_k_vec.iter();

    for i in 0..input_length{
        let jacobian = linear_jacobian();

        // fetch the next values of H / V / m_k
        next!{init:
            H_iter => curr_H,
            V_iter => curr_V,
            M_k_iter => curr_m_k
        };

        //predictions
        let pred_state_vec = prediction::state_vector(&jacobian, &previous_state_vec);
        let pred_cov_mat = prediction::covariance_matrix(&jacobian, &previous_covariance);
        let pred_residual_mat = prediction::residual_mat(&curr_V, &curr_H, &pred_cov_mat);
        let pred_residual_vec = prediction::residual_vec(&curr_m_k, &curr_H, &pred_state_vec);

      
        //filtering
        let kalman_gain = filter_gain::kalman_gain(&pred_cov_mat, &curr_H, &curr_V);
        let filter_state_vec = filter_gain::state_vector(&pred_state_vec, &kalman_gain, &curr_m_k, &curr_H);
        let filter_cov_mat = filter_gain::covariance_matrix(&kalman_gain, &curr_H, &pred_cov_mat);
        let filter_residual_vec = filter_gain::residual_vec(&curr_H, &kalman_gain, &pred_residual_vec);
        let filter_residual_mat = filter_gain::residual_mat(&curr_V, &curr_H, &filter_cov_mat);
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
            H_vec => curr_H_k,
            V_vec => curr_V_k,
            m_k_vec =>curr_measurement,
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
        get_unchecked!{input_length -(i+2);                                           //TODO: fix this index <IMPORTANT!!!>
            smoothed_state_vec_iter => prev_smth_state_vec,
            smoothed_cov_mat_iter => prev_smth_cov_mat
        }

        // 
        // smoothing calculations
        //

        // NOTE: the next calculations assume that x^n references the next state vector and x^k references the previous 
        // state vector. I am uncertain as to what the actual answer is as andi still has not gotten back to me about it.
        let gain_matrix = smoothing::gain_matrix(&curr_filt_cov_mat, &curr_jacobian, &prev_filt_cov_mat); 
        let smoothed_state_vec = smoothing::state_vector(&curr_filt_state_vec, &gain_matrix, &prev_smth_state_vec, &prev_filt_state_vec);
        let smoothed_cov_mat = smoothing::covariance_matrix(&curr_filt_cov_mat, &gain_matrix, &prev_filt_cov_mat, &prev_smth_cov_mat);
        let smoothed_res_mat = smoothing::residual_mat(&curr_V_k, &curr_H_k, &smoothed_cov_mat);
        let smoothed_res_vec = smoothing::residual_vec(&curr_measurement, &curr_H_k, &smoothed_state_vec);

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


}

// TODO: figure out partial derivatives for jacobian calculation
fn linear_jacobian() -> Mat5 {
    return Mat5::identity()
}