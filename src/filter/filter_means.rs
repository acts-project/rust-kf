use nalgebra as na;
use super::super::config::*;


// TODO: fix unwrap
#[allow(dead_code)]
fn state_vector(
    C_k : &Mat5,
    C_prediction : &Mat5,
    extrap_state_vector: &Vec5,
    H : &Mat5,
    G : &Mat5,
    m_k: &Vec5) -> Vec5{
    
    let product_one = C_prediction.try_inverse().unwrap() * extrap_state_vector;
    let product_two = H.transpose() * G * m_k;

    return C_k * (product_one + product_two)
}

//TODO : fix unwrap
#[allow(dead_code)]
fn covariance_mat ( // C_k
    C_prediction: &Mat5,
    H : &Mat5,
    G : &Mat5) -> Mat5 {
    
    let product = H.transpose() * G * H;
    let C_prevoius_inv = C_prediction.try_inverse().unwrap();

    return (C_prevoius_inv + product).try_inverse().unwrap();
}

//TOOD: fix uwrap()
#[allow(dead_code)]
fn chi_squared_increment(
    residual_vec: &Vec5,
    G: &Mat5,
    state_vector: &Vec5,
    extrap_state_vector: &Vec5,
    C_prediction: &Mat5) -> Real {

    let first_term = residual_vec.transpose() * G * residual_vec;

    let second_term_3 = (state_vector - extrap_state_vector);
    let second_term_2 = C_prediction.try_inverse().unwrap();
    let second_term_1 = second_term_3.transpose();
    
    let second_term = second_term_1 * second_term_2 * second_term_3;
    
    return (first_term + second_term)[0]

}