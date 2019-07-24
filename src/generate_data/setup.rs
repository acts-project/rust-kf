use nalgebra::base::Unit;
use super::super::config::*;
use super::super::geometry;
use geometry::traits::{Plane, Transform};
use geometry::Rectangle;

use super::super::filter;
use filter::{prediction, jacobian};

use rand::Rng;
use rand::rngs::SmallRng;
use rand_distr::{Normal, Distribution};

use super::structs::{KFData, State};


pub fn generate_linear_track(
    state: &State,
    mut rng: SmallRng,
    ) -> KFData<Rectangle> {


    let (phi, theta) = state.angles;

    // print!{phi, theta};

    // create a virtual sensor at the starting position. this is used to easily calculate
    // the hits on the sensors
    let virtual_sensor = gen_sensor(0.);
    let start_state_vec = 
        Vec5::new(
            0.,
            0.,
            phi,
            theta,
            1.
        );

    // generate sensors along x axis
    let mut sensor_vec = Vec::new();
    for i in 0..state.num_sensors {
        let ip1 = (i + 1) as Real;
        let x_loc_of_sensor = ip1 * state.sensor_distance;
        sensor_vec.push(gen_sensor(x_loc_of_sensor));
    }

    // print!{sensor_vec.len()}



    // find the locations of the true sensor hits
    let mut truth_hits = Vec::new();

    for i in 0..sensor_vec.len() {
        let curr_sensor = &sensor_vec[i];

        let (pred_sv, _) = 
            prediction::linear_state_vector(
                &virtual_sensor,
                &curr_sensor,
                &start_state_vec
            ).expect("out of bounds when generating track");

        get_unchecked!{
            pred_sv[eLOC_0] => x_hit,
            pred_sv[eLOC_1] => y_hit
        }

        truth_hits.push(Vec2::new(*x_hit, *y_hit))
    }
    
    // smear the hit locations
    let smeared_hits = 
        truth_hits.iter()
            .map(|point|{
                let smear_x = state.stdevs.smear_hit(&mut rng, point.x);
                let smear_y = state.stdevs.smear_hit(&mut rng, point.y);

                Vec2::new(smear_x,  smear_y)
            })
            .collect::<Vec<_>>();

    // // print out truth vs smeared hits
    // smeared_hits.iter()
    //     .zip( truth_hits.iter() )
    //     .for_each(|(smear, truth)| {
    //         let x_diff = (smear.x - truth.x).abs();
    //         let y_diff = (smear.y - truth.y).abs();

    //         // println!{"truth x:{}\tsmear x: {}\tdiff:{}  \t truth y: {}\tsmear y: {}\t diff: {}",truth.x, smear.x, x_diff, truth.y, smear.y, y_diff}
    //     });
    

    // TODO hold these constant  - Mr Paul

    let covariance_vec = 
        (0..sensor_vec.len()).into_iter()
        .map(|_|{
            let a = state.stdevs.diagonal_value(&mut rng).abs();
            let d = state.stdevs.diagonal_value(&mut rng).abs();

            let b = state.stdevs.corner_value(&mut rng).abs();
            let c = state.stdevs.corner_value(&mut rng).abs();
            
            let m = Mat2::new(a, b, c , d);
            m
        })
        .collect::<Vec<_>>();


    let smear_state_vec = smear_state_vector(&mut rng, state.stdevs.point_std, &start_state_vec);

    KFData::new(sensor_vec, covariance_vec, smeared_hits, truth_hits, (phi, theta), smear_state_vec, start_state_vec, None)
}


pub fn generate_const_b_track(
    state: &State,
    mut rng: SmallRng,
    ) -> KFData<Rectangle> {


    let (phi, theta) = state.angles;

    // print!{phi, theta};

    // create a virtual sensor at the starting position. this is used to easily calculate
    // the hits on the sensors
    let virtual_sensor = gen_sensor(0.);
    let start_state_vec = 
        Vec5::new(
            0.,
            0.,
            phi,
            theta,
            1.
        );

    // generate sensors along x axis
    let mut sensor_vec = Vec::new();
    sensor_vec.push(virtual_sensor);

    for i in 0..state.num_sensors {
        let ip1 = (i + 1) as Real;
        let x_loc_of_sensor = ip1 * state.sensor_distance;
        sensor_vec.push(gen_sensor(x_loc_of_sensor));
    }

    // print!{sensor_vec.len()}



    // find the locations of the true sensor hits
    let mut truth_hits = Vec::new();
    let mut prev_filt_state_vec = start_state_vec.clone();

    for i in 1..sensor_vec.len() {
        let start_sensor = &sensor_vec[i-1];
        let end_sensor   = &sensor_vec[i];
        
        let (_, pred_sv) = jacobian::constant_field(&prev_filt_state_vec, &state.b_field, start_sensor, end_sensor);

        get_unchecked!{
            pred_sv[eLOC_0] => x_hit,
            pred_sv[eLOC_1] => y_hit
        }
        
        prev_filt_state_vec = pred_sv;

        truth_hits.push(Vec2::new(*x_hit, *y_hit))
    }

    // pop off the initial state vector we used for the calculation in the above for loop
    // since its not a sensor on the actual track
    sensor_vec.remove(0);


    
    // smear the hit locations
    let smeared_hits = 
        truth_hits.iter()
            .map(|point|{
                let smear_x = state.stdevs.smear_hit(&mut rng, point.x);
                let smear_y = state.stdevs.smear_hit(&mut rng, point.y);

                Vec2::new(smear_x,  smear_y)
            })
            .collect::<Vec<_>>();

    // TODO hold these constant  - Mr Paul

    let covariance_vec = 
        (0..sensor_vec.len()).into_iter()
        .map(|_|{
            let a = state.stdevs.diagonal_value(&mut rng).abs();
            let d = state.stdevs.diagonal_value(&mut rng).abs();

            let b = state.stdevs.corner_value(&mut rng).abs();
            let c = state.stdevs.corner_value(&mut rng).abs();
        
            let m = Mat2::new(a, b, c , d);
            m
        })
        .collect::<Vec<_>>();


    let smear_state_vec = smear_state_vector(&mut rng, state.stdevs.point_std, &start_state_vec);

    KFData::new(sensor_vec, covariance_vec, smeared_hits, truth_hits, (phi, theta), smear_state_vec, start_state_vec, Some(state.b_field))

    // unimplemented!()
}



fn smear_state_vector(rng: &mut SmallRng, std_dev: Real, state_vec: &Vec5) -> Vec5{
    let mut new_vec = Vec5::zeros();

    for i in 0..5{
        get_unchecked!{vector;state_vec; i=> var}
        let distr = Normal::new(*var, std_dev).unwrap();
        let new_val = distr.sample(rng);
        edit_matrix!{new_vec; [i, 0] = new_val}
    }

    new_vec
}

fn gen_sensor(x_point: Real) -> Rectangle {
    // sensor dimensions
    let base = 1000.;
    let height = 1000.;


    let y_axis = Vec3::new(0. , 1., 0.);
    let j = Unit::try_new(y_axis, 0.).unwrap();

    let l2g_rot = Mat4::from_axis_angle(&j, PI/2.);
    let g2l_rot = l2g_rot.try_inverse().expect("rotation matrix non invert test");

    let trans = Trl3::new(x_point, 0., 0.).to_homogeneous();

    let mat = trans * l2g_rot;


    let to_global = Aff3::from_matrix_unchecked(mat);
    let to_local = to_global.try_inverse().unwrap();


    // print!{"SENSOR CENTER WILL BE AT ", to_global * P3::origin() }

    let p1 = P3::new(x_point, 1., 1.);
    let p2 = P3::new(x_point, 0., 1.);

    // new_test_sensor() is created to hard code values (transformation matricies, points)
    // instead of using new() which uses inverses
    Rectangle::new_test_sensor(
        base,
        height,
        to_global,
        to_local,
        l2g_rot,
        g2l_rot,
        p1,
        p2
    )

}

