extern crate nalgebra as na;
// use na::Point3;
mod config;
mod geometry;

mod filter;

// use geometry::{trapezoid::Trapezoid, rectangle::Rectangle};
// use geometry::traits::{Plane, Transform};

use config::*;


fn main() {
    #![allow(warnings)]
    // let k = filter::utils::seed_covariance();
    // dbg!{k};
    let mat = Mat4::identity();
    dbg!{mat};
    
}
