use nalgebra as na;
use super::super::config::*;

// extrapolating state vector
fn extrapolate_state_vector (jacobian: &Mat5, state_vector: &Vec5) -> Vec5 {
    return jacobian * state_vector
}

// extrapolation of the covariance matrix 
fn update_covariance_matrix(jacobian: &Mat5, previous_covariance: &Mat5, sensor_noise: &Mat5)-> Mat5{
    return jacobian * previous_covariance * jacobian.transpose()
}

// just below eq. 7
// covariance of predicted results
fn predicted_covariance(V: &Mat5, H: &Mat5, C: &Mat5) -> Mat5 {
    return V + (H*C) + H.transpose()
}