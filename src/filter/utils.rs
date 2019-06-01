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