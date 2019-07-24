use kalman_rs::test_data;
use kalman_rs::config::*;
use kalman_rs::geometry::*;

use nalgebra as na;

use kalman_rs::generate_data::run;

use rayon::{self, prelude::*};


fn main() {
    
    test_data!{5;
        V_vec, Mat2, 
        m_k_vec, Vec2
    }

    test_data!{mat; 5; sensor_vec, Rectangle}

    let start = P3::new(0., 0. , 0.);

    // kalman_rs::filter::linear::run(&start, &V_vec, &m_k_vec, &sensor_vec);

    // run::const_b::runge_kutta_global_locations();
    run::const_b::run_all_stats();

}
