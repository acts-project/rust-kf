use nalgebra as na;
mod config;
mod geometry;
mod filter;
use config::*;
mod error;


fn main() {
    
    let test_mat = filter::utils::vec_of_mat(5);
    let test_vec = filter::utils::vec_of_vec(5);

    filter::linear::run(&test_mat, &test_mat, &test_vec);
}
