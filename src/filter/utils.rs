use nalgebra as na;
use super::super::config::*;

/// Placeholder function for some form of effective seeding for Mat5's
pub fn seed_covariance() -> Mat5 {
    // na::U4;
    return Mat5::new_random_generic(na::U5,na::U5);    
}

/// Placeholder function for some form of effective seeding of Mat5s
pub fn seed_state_vec() -> Vec5 {
    return Vec5::new_random_generic(na::U5, na::U1)
}

/// Creates a vector of `num` length with Mat5 components
pub fn vec_of_mat(num: usize) -> Vec<Mat5> {
    
    let mut return_vec: Vec<Mat5> = Vec::with_capacity(num);
    (0..num).into_iter()
        .for_each(|_| return_vec.push(seed_covariance()));
    
    return return_vec
}

/// Creates a vector of `num` length with Vec5 components
pub fn vec_of_vec(num: usize) -> Vec<Vec5> {
    
    let mut return_vec: Vec<Vec5> = Vec::with_capacity(num);
    (0..num).into_iter()
        .for_each(|_| return_vec.push(seed_state_vec()));
    
    return return_vec
}


pub struct SmoothedData {
    state_vec: Vec<Vec5>,
    cov_mat: Vec<Mat5>,
    res_mat: Vec<Mat2>,
    res_vec: Vec<Vec2>
}

impl SmoothedData{
    pub fn new(state_vec: Vec<Vec5>,
            cov_mat: Vec<Mat5>,
            res_mat: Vec<Mat2>,
            res_vec: Vec<Vec2>) -> Self {

        return SmoothedData{state_vec: state_vec, 
                            cov_mat: cov_mat, 
                            res_mat: res_mat, 
                            res_vec:res_vec}
    }
    pub fn FFI_return() {
        unimplemented!()
    }
}

