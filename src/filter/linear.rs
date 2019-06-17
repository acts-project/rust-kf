use nalgebra as na;
use super::super::config::*;
use super::prediction;
use super::filter_gain;
use super::smoothing;

use std::iter;

use super::utils::SmoothedData;

// macro to initialize vectors with a given capacity to reduce copy / paste
macro_rules! store_vec {
    // $name: the name of the variable
    // $type: Type of data stored in vector (Mat5 / Vec5)
    // $capacity: How much space to allocate
    ($name:ident, $type:ty, $capacity:expr) => {
        let mut $name: Vec<$type> = Vec::with_capacity($capacity);
    };
}

/// This macro is used to chain a value to an iterator. It is primarily used to chain
/// matricies / vectors to their respective iterators while reducing code duplication.
/// Not using this macro would mean we do something like this for every single iterator: 
/// `iterable.chain(std::iter::repeat(value).take(1))`
 
// item: the value that will be turn into an iterator and chained
// location: the iterator the value is being chained to
macro_rules! chain {
    ($($item:expr => $location:ident),*) => {
        $(
            _chain(&mut $location, take_one($item));
        )*
    };
}

/// Used for getting the next value of the iterator. Additionally, this macro allows
/// us to move the ownership of the data to "stagger" it. This does the following:
/// `current` at n => `previous` at n+1,
/// `next` at n => `current` at n+1,
/// new `next` value is fetched from the iterator. 
macro_rules! next {
    // move the value from `next` to `current`, from `current` to `previous`, and 
    // fetch a new value from the iterator for `next`. This is done for the staggered
    // steps of the smoothing calculations
    //
    // storage: iterator we store values in
    // previous / current / next: variables from scope we are modifying
    //                            to new values  
    ($($storage:ident => $previous:ident, $current:ident, $next:ident),+) => {
        $(
            let $previous = $current;
            let $current = $next;
            next!{init: $storage => $next};
        )+
    };
    //this unwraps each value of the iterator. useful for initializing values for `next`
    // and `current` before the start of the smoothing loop since the first iteration
    // of the smoothing loop will call `next!` and immediatly shift the variables.
    // This is also important for future code where we handle for "holes" since we
    // would have to inline code for matching Some(_) or None in the loop.
    (init: $($iterator:ident => $($store_location:ident):+),+) => {
        $(
            $(let $store_location = $iterator.next().unwrap();)+
        )+
    };
}

/// Reverse every iterator passed in. This is useful for the smoothing calculations
/// since we pass from the end to the front, but add items to iterators from 
/// front to back
macro_rules! reverse {
    ($($iterator:ident),+) => {
        $(
            let mut $iterator = $iterator.rev();
        )+
    };
}

/// Creates a variable amount of mutable empty iterators
macro_rules! empty {
    ($($name:ident),+) => {
        $(
            let mut $name = std::iter::empty();
        )+
    };
}


/// Monolithic function to handle linear KF calculations
pub fn run(
    V_vec: &Vec<Mat5>, 
    H_vec: &Vec<Mat5>,
    m_k_vec: &Vec<Vec5>,
)  -> SmoothedData{
    
    if (V_vec.len() == H_vec.len()) && (H_vec.len() == m_k_vec.len()) {}
    else {
        panic!("vector lengths need to be the same length")
    } 
    let input_length = V_vec.len();

    // create empty iterators to store values
    empty!{
        jacobian_iter,
        
        // for filtered values
        filter_state_vec_iter,
        filter_cov_mat_iter,
        filter_res_mat_iter,
        filter_res_vec_iter,
        chi_squared_iter,

        //for smoothed values
        smoothed_state_vec_iter,
        smoothed_cov_mat_iter,
        smoothed_res_mat_iter,
        smoothed_res_vec_iter
    };
    
    // calculate some seeded values (seeding improvement suggestions welcome)
    let mut previous_state_vec = super::utils::seed_state_vec();
    let mut previous_covariance = super::utils::seed_covariance();

    // Store the seeded values in their respective iterators
    chain!(
        previous_covariance => filter_cov_mat_iter, 
        previous_state_vec => filter_state_vec_iter
    );

    let mut H_iter = H_vec.iter();
    let mut V_iter = V_vec.iter();
    let mut M_k_iter = m_k_vec.iter();

    for i in 0..input_length{
        let jacobian = linear_jacobian();

        // fetch the next values of H / V / m_k
        next!{init:
            H_iter => curr_H,
            V_iter => curr_V,
            M_k_iter => curr_m_k
        };

        //predictions
        let pred_state_vec = prediction::state_vector(&jacobian, &previous_state_vec);
        let pred_cov_mat = prediction::covariance_matrix(&jacobian, &previous_covariance);
        let pred_residual_mat = prediction::residual_mat(&curr_V, &curr_H, &pred_cov_mat);
        let pred_residual_vec = prediction::residual_vec(&curr_m_k, &curr_H, &pred_state_vec);

      
        //filtering
        let kalman_gain = filter_gain::kalman_gain(&pred_cov_mat, &curr_H, &curr_V);
        let filter_state_vec = filter_gain::state_vector(&pred_state_vec, &kalman_gain, &curr_m_k, &curr_H);
        let filter_cov_mat = filter_gain::covariance_matrix(&kalman_gain, &curr_H, &pred_cov_mat);
        let filter_residual_vec = filter_gain::residual_vec(&curr_H, &kalman_gain, &pred_residual_vec);
        let filter_residual_mat = filter_gain::residual_mat(&curr_V, &curr_H, &filter_cov_mat);
        let chi_squared_inc = filter_gain::chi_squared_increment(&filter_residual_vec, &filter_residual_mat);

        // store all the filtered values in their respective iterators
        chain!{
            filter_state_vec =>filter_state_vec_iter,
            jacobian => jacobian_iter,
            filter_cov_mat => filter_cov_mat_iter,
            filter_residual_mat => filter_res_mat_iter,
            filter_residual_vec => filter_res_vec_iter,
            chi_squared_inc => chi_squared_iter
        }
        
        // store current filtered values as the "previous" to be used in the
        // prediction calculations in the next iteration
        previous_covariance = filter_cov_mat;
        previous_state_vec = filter_state_vec;
    }

    dbg!{"made it this far"};
    // smoothing
    // cycle through the sensors from highest index to lowest index

    next!(init: 
        filter_state_vec_iter => curr_state_vec : next_state_vec,
        filter_cov_mat_iter => curr_cov_mat : next_cov_mat
    );
    
    let mut H_iter = H_vec.iter();
    let mut V_iter = V_vec.iter();
    let mut M_k_iter = m_k_vec.iter();

    // reverse all iterators so we move from the end sensor to the first sensor
    reverse!(H_iter, V_iter, M_k_iter, jacobian_iter, filter_state_vec_iter, filter_cov_mat_iter, filter_res_mat_iter, filter_res_vec_iter);

    
    for i in 1..input_length{

        //
        // initializing variables
        // 

        next!{
            filter_state_vec_iter => prev_state_vec, curr_state_vec, next_state_vec,
            filter_cov_mat_iter => prev_cov_mat, curr_cov_mat, next_cov_mat}

        next!{init:
            M_k_iter => curr_measurement,
            H_iter => curr_H_k,
            V_iter => curr_V_k,
            jacobian_iter => curr_jacobian}

        // 
        // smoothing calculations
        //

        // // NOTE: the next [ ] calculations assume that x^n references the next state vector and x^k references the previous 
        // // state vector. I am uncertain as to what the actual answer is as andi still has not gotten back to me about it.
        let gain_matrix = smoothing::gain_matrix(&curr_cov_mat, &curr_jacobian, &prev_cov_mat); 
        let smoothed_state_vec = smoothing::state_vector(&curr_state_vec, &gain_matrix, &next_state_vec, &prev_state_vec);
        let smoothed_cov_mat = smoothing::covariance_matrix(&curr_cov_mat, &gain_matrix, &next_cov_mat, &prev_cov_mat);
        let smoothed_res_mat = smoothing::residual_mat(&curr_V_k, &curr_H_k, &smoothed_cov_mat);
        let smoothed_res_vec = smoothing::residual_vec(&curr_measurement, &curr_H_k, &smoothed_state_vec);

        //
        //  Store variables in iterators
        //
        chain!{
            smoothed_state_vec => smoothed_state_vec_iter,
            smoothed_cov_mat => smoothed_cov_mat_iter,
            smoothed_res_mat => smoothed_res_mat_iter,
            smoothed_res_vec => smoothed_res_vec_iter
        }
    }
    
    // put all data into a struct that will contain all the methods to return 
    // the data back to c++
    return SmoothedData::new(smoothed_state_vec_iter.collect(),
                                smoothed_cov_mat_iter.collect(),
                                smoothed_res_mat_iter.collect(),
                                smoothed_res_vec_iter.collect())

}

// TODO: figure out partial derivatives for jacobian calculation
fn linear_jacobian() -> Mat5 {
    return Mat5::identity()
}

/// Chains two iterators together
fn _chain<T>(main_iter: &mut impl Iterator<Item=T>, to_chain: impl Iterator<Item=T>) {
    main_iter.chain(to_chain);
}

/// Convert `item` to an iterator of one element
fn take_one<T: Clone>(item: T) -> impl Iterator<Item=T> {
    iter::repeat(item).take(1)
}