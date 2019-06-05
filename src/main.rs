extern crate nalgebra as na;
// use na::Point3;
mod config;
mod geometry;

mod filter;

// use geometry::{trapezoid::Trapezoid, rectangle::Rectangle};
// use geometry::traits::{Plane, Transform};

use config::*;


fn main() {
    // let k = filter::utils::seed_state_vec();
    // dbg!{k};
    // let mat = Mat4::identity();
    // dbg!{mat};
    
    let test_mat = filter::utils::vec_of_mat(5);
    let test_vec = filter::utils::vec_of_vec(5);

    filter::linear::run(&test_mat, &test_mat, &test_vec);
}
