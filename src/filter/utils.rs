use nalgebra as na;
use super::super::config::*;

//placeholder for future more inteligent seeding
pub fn seed_covariance() -> Mat5 {
    // na::U4;
    return Mat5::new_random_generic(na::U5,na::U5);    
}

pub fn seed_state_vec() -> Vec5 {
    return Vec5::new_random_generic(na::U5, na::U1)
}


// utility function for creating a bunch of random data for testing the run functions
pub fn vec_of_mat(num: usize) -> Vec<Mat5> {
    
    let mut return_vec: Vec<Mat5> = Vec::with_capacity(num);
    (0..num).into_iter()
        .for_each(|_| return_vec.push(seed_covariance()));
    
    return return_vec
}


// utility function for creating a bunch of random data for testing the run functions
pub fn vec_of_vec(num: usize) -> Vec<Vec5> {
    
    let mut return_vec: Vec<Vec5> = Vec::with_capacity(num);
    (0..num).into_iter()
        .for_each(|_| return_vec.push(seed_state_vec()));
    
    return return_vec
}