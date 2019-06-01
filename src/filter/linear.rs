use nalgebra as na;
use super::super::config::*;
use super::prediction;
use super::filter_gain;

#[allow(dead_code)]
pub fn run(
    V_vec: &Vec<Mat5>, 
    H_vec: &Vec<Mat5>,
    m_k_vec: &Vec<Vec5>,
    ) {
    
    if (V_vec.len() == H_vec.len()) && (H_vec.len() == m_k_vec.len()) {}
    else {
        panic!("vector lengths need to be the same length")
    } 
    
    let initial_state_vec = super::utils::seed_state_vec();
    let initial_covariance = super::utils::seed_covariance();

    let L = V_vec.len();
    
    let mut state_vector_pred_vec : Vec<Vec5> = Vec::with_capacity(L);
    let mut jacobian_vec : Vec<Mat5> = Vec::with_capacity(L);
    let mut cov_pred_vec: Vec<Mat5> = Vec::with_capacity(L);
    let mut res_cov_mat_vec : Vec<Mat5> = Vec::with_capacity(L);
    let mut res_vector_vec : Vec<Vec5> = Vec::with_capacity(L);

    state_vector_pred_vec.push(initial_state_vec);
    cov_pred_vec.push(initial_covariance);

    for i in 0..V_vec.len() {
        let jacobian = linear_jacobian();

        let H = &H_vec[i];
        let V = &V_vec[i];
        let m_k=&m_k_vec[i];


        //predictions
        let pred_state_vec = prediction::state_vector(&jacobian, &state_vector_pred_vec[i]);
        let pred_cov_mat = prediction::covariance_matrix(&jacobian, &cov_pred_vec[i]);
        let pred_residual_mat = prediction::residual_covariance(&V, &H, &pred_cov_mat);
        let pred_residual_vec = prediction::residual_vec(&m_k, &H, &pred_state_vec);

        //filtering
        let kalman_gain = filter_gain::kalman_gain(&pred_cov_mat, &H, &V);
        let filter_state_vec = filter_gain::update_state_vector(&pred_state_vec, &kalman_gain, &m_k, &H);
        let filter_cov_mat = filter_gain::covariance_matrix(&kalman_gain, &H, &pred_cov_mat);
        let filter_residual_vec = filter_gain::residual_vector(&H, &kalman_gain, &pred_residual_vec);
        let filter_residual_mat = filter_gain::residual_covariance_matrix(&V, &H, &filter_cov_mat);
        let chi_squared_inc = filter_gain::chi_squared_increment(&filter_residual_vec, &filter_residual_mat);


        //storage
        state_vector_pred_vec.push(pred_state_vec);
        jacobian_vec.push(jacobian);
        cov_pred_vec.push(pred_cov_mat);
        res_cov_mat_vec.push(pred_residual_mat);
        res_vector_vec.push(pred_residual_vec);


    }

}


// TODO: Covariance transport here
fn linear_jacobian() -> Mat5 {
    return Mat5::identity()
}