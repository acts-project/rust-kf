use kalman_rs as krs;
use krs::config::*;
use krs::geometry::Rectangle;                   // rect sensors
use krs::geometry::traits::{Plane, Transform};

use nalgebra::base::Unit;
use krs::print;

use rand::distributions::Normal;

/*

    This is the unit test for the entire linear kalman filter

*/

macro_rules! group_assert {
    ($left:ident, $right:ident, $($index:ident),+) => {
        $(
            let left = $left.$index;
            let right = $right.$index;

            let diff = right - left;

            assert!{diff.abs() < DOT_PRODUCT_EPSILON}
        )+
    };
}


const UNCERTAINTY : Real = 2.;

fn gen_cov(cov_vec: &mut Vec<Mat2>) -> (){
    let base_cov = Mat2::identity();     // high covariance across the main diagonal
    let cov_noise = Mat2::new_random() / UNCERTAINTY;       // every index random # in [0,1] * uncertainty
    let covariance = base_cov + cov_noise;

    cov_vec.push(covariance);
}

fn gen_sensor(sensor_vec: &mut Vec<Rectangle>, index: Real) {
    // sensor dimensions
    let base = 10.;
    let height = 10.;

    // make a sensor every 5 units on the x axis
    let x_point = (index+1.)*5.;


    let y_axis = Vec3::new(0. , 1., 0.);
    let j = Unit::try_new(y_axis, 0.).unwrap();

    let rot = Mat4::from_axis_angle(&j, PI/2.);
    let trans = Trl3::new(x_point, 0., 0.).to_homogeneous();

    let mat = trans * rot;


    let to_global = Aff3::from_matrix_unchecked(mat);
    let to_local = to_global.try_inverse().unwrap();


    print!{"SENSOR CENTER WILL BE AT ", to_global * P3::origin() }

    let p1 = P3::new(x_point, 1., 1.);
    let p2 = P3::new(x_point, 0., 1.);

    // new_test_sensor() is created to hard code values (transformation matricies, points)
    // instead of using new() which uses inverses
    let sensor = 
        Rectangle::new_test_sensor(
            base,
            height,
            to_global,
            to_local,
            p1,
            p2
        );
    sensor_vec.push(sensor);

}

fn random_hit(meas_vec: &mut Vec<Vec2>) {
    let hit = Vec2::new_random();
    
    meas_vec.push(hit);
}

#[test]
fn linear_1() {
    let start = P3::new(0. , 0. , 0.);

    let mut cov_vec = Vec::new();
    let mut sensor_vec = Vec::new();
    let mut meas_vec = Vec::new();


    let iterations = 5;
    for i in 0..iterations {
        gen_cov(&mut cov_vec);
        gen_sensor(&mut sensor_vec, i as Real);
        random_hit(&mut meas_vec);

    }

    let kf_result = krs::filter::linear::run(&start, &cov_vec, &meas_vec, &sensor_vec);

    let last_state = kf_result.state_vec.get(iterations-1).unwrap();

    group_assert!{last_state, start, x, y}
}


// This test fails currently
#[test]                 
fn linear_2() {
    // parallel to x axis
    let start = P3::new(0. , 1. , 1.);

    let mut cov_vec = Vec::new();
    let mut sensor_vec = Vec::new();
    let mut meas_vec = Vec::new();


    let iterations = 5;
    for i in 0..iterations {
        gen_cov(&mut cov_vec);
        gen_sensor(&mut sensor_vec, i as Real);
        random_hit(&mut meas_vec);

    }

    let kf_result = krs::filter::linear::run(&start, &cov_vec, &meas_vec, &sensor_vec);

    dbg!{&kf_result};

    let last_state = kf_result.state_vec.get(iterations-1).unwrap();

    group_assert!{last_state, start, x, y}
}


// generate track 
#[test]                 
fn linear_3() {
    // parallel to x axis
    let start = P3::new(0. , 0. , 0.);

    let mut cov_vec = Vec::new();
    let mut sensor_vec = Vec::new();
    let mut meas_vec = Vec::new();


    let iterations = 5;
    for i in 0..iterations {
        gen_cov(&mut cov_vec);
        gen_sensor(&mut sensor_vec, i as Real);
        gen_hit(&mut meas_vec);

    }

    let kf_result = krs::filter::linear::run(&start, &cov_vec, &meas_vec, &sensor_vec);

    dbg!{&kf_result};

    let last_state = kf_result.state_vec.get(iterations-1).unwrap();

    group_assert!{last_state, start, x, y}
}