use nalgebra as na;
use super::super::config::*;

pub fn gain_matrix( //A_k
    curr_filt_cov_mat: &Mat5, 
    F_k: &Mat5,
    prev_filt_cov_mat: &Mat5) -> Mat5 {      // TODO: Make sure that the C_k+1 here does indeed mean
                                    // that we use the covariance from the sensor 
                                    // that is chronologically after the current sensor in
                                    // the pred / filter phase

    curr_filt_cov_mat * F_k.transpose() * prev_filt_cov_mat.try_inverse().expect("could not invert in gain matrix")
}  

pub fn state_vector(
    curr_filt_state_vec: &Vec5,
    A_k: &Mat5,
    prev_smth_state_vec: &Vec5,
    prev_filt_state_vec: &Vec5) -> Vec5 {
    
    let parens = prev_smth_state_vec - prev_filt_state_vec;
    let prod = A_k * parens;
    let sum =  curr_filt_state_vec + prod;

    return sum
}

pub fn covariance_matrix(
    curr_filt_cov_mat: &Mat5,
    A_k: &Mat5,
    prev_filt_cov_mat: &Mat5,
    prev_smth_cov_mat: &Mat5) -> Mat5 {

    let parens = prev_smth_cov_mat - prev_filt_cov_mat;
    let prod = A_k * parens * A_k.transpose();
    let sum = curr_filt_cov_mat + prod;
    
    return sum
}


pub fn residual_mat(
    V_k: &Mat5,
    H_k: &Mat5,
    curr_smth_cov_mat: &Mat5) -> Mat5 {

    let prod = H_k * curr_smth_cov_mat * H_k.transpose();
    let diff = V_k - prod;

    return diff;
}

pub fn residual_vec(
    m_k: &Vec5,
    H_k: &Mat5,
    curr_smth_state_vec: &Vec5) -> Vec5 {
    
    let prod = H_k * curr_smth_state_vec;
    let sum = m_k + prod;
    
    return sum;
}
