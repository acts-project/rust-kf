use nalgebra as na;
use super::super::config::*;

pub fn gain_matrix( //A_k
    C_k: &Mat5, 
    F_k: &Mat5,
    C_k_next: &Mat5) -> Mat5 {

    C_k * F_k.transpose() * C_k_next.try_inverse().unwrap()
}  

pub fn state_vector(
    x_k: &Vec5,
    A_k: &Mat5,
    x_n_kplus: &Vec5,
    x_k_kplus: &Vec5) -> Vec5 {
    
    let parens = x_n_kplus - x_k_kplus;
    let prod = A_k * parens;
    let sum =  x_k + prod;

    return sum
}

pub fn covariance_matrix(
    C_k: &Mat5,
    A_k: &Mat5,
    C_n_kplus: &Mat5,
    C_k_kplus: &Mat5) -> Mat5 {

    let parens = C_n_kplus - C_k_kplus;
    let prod = A_k * parens * A_k.transpose();
    let sum = C_k + prod;
    
    return sum
}

pub fn residual_vec(
    m_k: &Vec5,
    H_k: &Mat5,
    x_n_k: &Vec5) -> Vec5 {
    
    let prod = H_k * x_n_k;
    let sum = m_k + prod;
    
    return sum;
}

pub fn residual_covariance_matrix(
    V_k: &Mat5,
    H_k: &Mat5,
    C_n_k: &Mat5) -> Mat5 {

    let prod = H_k * C_n_k * H_k.transpose();
    let diff = V_k - prod;

    return diff;
}

