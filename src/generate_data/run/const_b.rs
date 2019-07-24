

use super::super::super::{
    config::*,
    filter::{
        self,
        jacobian,
        prediction,
        utils,
        angles,
    }
};
#[macro_use]
use filter::macros;

use super::super::{
    store,
    structs
};


use rand::{thread_rng, SeedableRng};
use rand::rngs::SmallRng;
use rand_distr::{Normal, Distribution};



fn gev_to_joules(gev_val: Real) -> Real{
    let gev_joules: Real = 1.602176487 * 10_f64.powf(-10.);
    // let gev_joules = gev_joules * 10.pow(-10);

    gev_val * gev_joules
}


/// Exports a CSV of x / y / z locations that are produced from RK integration
/// 
/// This is the foundation of testing the constant magnetic field jacobian 
/// calculations.
pub fn runge_kutta_global_locations() {
    let phi = 0.;
    let theta = PI/2.;
    let qop = gev_to_joules(10.);


    let mut angles = angles::Angles::new_from_angles(phi, theta);

    let state_vector = 
        Vec5::new(
            0.,
            0.,
            phi, 
            theta,
            qop
        );

        
    let mut global_state_vec = Vec8::zeros();


    // magnitude of the magnetic field is set, the vector associated with it 
    // is calculated by assuming all components x / y / z are of equal magnitude
    let b_magnitude :Real = 2.;
    let indiv_field = (b_magnitude.powf(2.)/3.).sqrt();
    let b_field = 
        Vec3::new(
            indiv_field,
            indiv_field,
            indiv_field
        );

    dbg!{&qop};
    print!{&b_field};

    let step_size = 0.001;
    let iterations = 10_000;
    let mut step_results = Vec::with_capacity(iterations);

    for _ in 0..iterations {
        let step = jacobian::runge_kutta_step(&state_vector,&angles, &b_field, step_size);

        prediction::rk_current_global_location(&step, &mut global_state_vec);

        let global_point = utils::global_point_from_rk_state(&global_state_vec);

        step_results.push(global_point);

        angles = utils::angles_from_rk_state(&global_state_vec);
    }

    store::write_csv(r".\data\rk_points.csv", step_results);

}


pub fn residuals_after_steps() {

    /*
        Initialzie constants
    */  
    let point_count = 10_000;
    let truth = std::iter::repeat(P3::origin()).take(point_count);

    let distr = Normal::new(0. ,1.).unwrap();
    let mut rng = SmallRng::from_entropy();



    let phi = 0.;
    let theta = PI/2.;
    let qop = gev_to_joules(10.);

    let step_size = 0.05;
    let iterations = 1000;

    let b_magnitude :Real = 2.;
    let indiv_field = (b_magnitude.powf(2.)/3.).sqrt();
    let b_field = 
        Vec3::new(
            indiv_field,
            indiv_field,
            indiv_field
        );
    
    // the only thing the RK takes from the local state vector
    // is the QOP in this situation
    // This is because in the actual jacobian calculation a 
    // global state vector is created from the local SV
    // and the global SV is the one being operated on.
    // we hand-code the global SV later
    let state_vec = 
        Vec5::new(
            0.,
            0.,
            phi, 
            theta,
            qop
        );

    /*
      
        Create smeared values  

    */
    let mut smeared_points = 
    truth
        .map(|x| {
            // create smeared  values
            let x = distr.sample(&mut rng);
            let y = distr.sample(&mut rng);
            let z = distr.sample(&mut rng);

            // create a global state vector, insert the 
            // global starting location into it
            let mut global_state_vec = Vec8::zeros();
            
            edit_matrix!{global_state_vec;
                [0 ,0] = x,
                [1 ,0] = y, 
                [2 ,0] = z
            }

            let angles = angles::Angles::new_from_angles(phi, theta);
            
            (global_state_vec,angles)

        })
        .collect::<Vec<_>>();

    // vector of the results of the RK
    let mut smeared_results_vec = Vec::with_capacity(point_count);

    for _ in  0..point_count {
        let (mut global_state_vec, mut angles) = smeared_points.remove(0);

        // has to be done to avoid compiler error
        let mut global_point = P3::origin();

        for _ in 0..iterations {
            let step = jacobian::runge_kutta_step(&state_vec,&angles, &b_field,step_size);

            prediction::rk_current_global_location(&step, &mut global_state_vec);

            global_point = utils::global_point_from_rk_state(&global_state_vec);

            angles = utils::angles_from_rk_state(&global_state_vec);
        }

        // Note: global_point is in 3 dimensions, but we only store & write 2 of those dimensions
        // to CSVs for python analysis 
        smeared_results_vec.push(structs::StorageData::new(global_point.x, global_point.y));
    }

    // write to file
    store::write_csv(r".\data\runge_kutta_truth_smear_residuals.csv", smeared_results_vec);
}
