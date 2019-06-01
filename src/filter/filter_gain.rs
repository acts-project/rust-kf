use nalgebra as na;
use super::super::config::*;

pub fn measurement_vector(
    jacobian: &Mat5,
    previous_state_vec: &Vec5) -> Vec5{
    return jacobian * previous_state_vec
}

pub fn update_state_vector( // x
    extrap_state_vector: &Vec5, 
    kalman_gain: &Mat5, 
    measurement : &Vec5, 
    sensor_mapping: &Mat5) -> Vec5 {

    let parens = measurement - (sensor_mapping * extrap_state_vector);
    let kalman_product = kalman_gain * parens;
    return extrap_state_vector + kalman_product;
}

//TODO: remove `unwrap` on the inverse
#[allow(dead_code)]
pub fn kalman_gain ( // K
    C : &Mat5,
    H : &Mat5,
    V : &Mat5) -> Mat5 {

    let parens = V + ( H * C * H.transpose() );
    let kalman_gain = C * H.transpose() * parens.try_inverse().unwrap();
    
    kalman_gain
}

//TODO add lazy static for identity
//update covariance matrix C
pub fn covariance_matrix( // C
    K : &Mat5,
    H : &Mat5,
    C : &Mat5) -> Mat5 {
    let parens = Mat5::identity() - (K*H);

    return C * parens;
}

//TODO  add second method for this calculation
//      since the second one looks to have less matmul

//TODO add lazy static for identity
pub fn residual_vector(  //r
    H : &Mat5,
    K : &Mat5,
    residual_preiction : &Vec5) -> Vec5 {

    let ident = Mat5::identity();
    let parens = ident - (H * K);

    return  parens * residual_preiction;
}

pub fn residual_covariance_matrix( //R
    V : &Mat5,
    H : &Mat5,
    C : &Mat5) -> Mat5{
    
    let product = H * C * H.transpose();
    return V - product;
}


pub fn chi_squared_increment(
    residual_vec : &Vec5,
    residual_covariance : &Mat5 ) -> Real {
    
    let prod = residual_vec.transpose() * residual_covariance.try_inverse().unwrap() * residual_vec;
    return prod[0]
}


pub fn update_chi_squared(
    previous_chi_squaread: Real,
    increment: Real) -> Real {
    
    previous_chi_squaread + increment
}