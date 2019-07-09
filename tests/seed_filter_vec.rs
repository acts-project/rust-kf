use kalman_rs::config::*;
use kalman_rs::filter::utils::seed_state_vec_from_points as seed_vec;
use kalman_rs::geometry::Rectangle;


/**
 *  Tests for initial seeding of the filtered state vector since the prediction phase
 *  requires a previous filtered vector. 
 * 
 */
fn start_location() -> P3 {
    P3::new(0.,0.,0.)
}


// general form of tests;
fn run_test(
    global_start: P3,
    local_end: P2,
    global_end: P3,
    phi :Real,
    theta : Real
    ) -> () {

    let expected_vec = 
        Vec5::new(
            local_end.x,
            local_end.y,
            phi,
            theta,
            1.,
        );

    let filt_vec =seed_vec(&global_start, &global_end, &local_end); 

    dbg!{expected_vec};
    dbg!{filt_vec};

    for i in 0..4 {

        let predicted = expected_vec.get(i).unwrap();
        let actual = filt_vec.get(i).unwrap();

        dbg!{predicted};
        dbg!{actual};

        assert!{(predicted - actual) < DOT_PRODUCT_EPSILON};
    }
}

#[test]
fn phi_1() {
    let global_start = start_location();

    let local_center = P2::new(0., 0.);
    let global_center= P3::new(1., 0., 0.);

    let phi =  0.;
    let theta = PI/2.;


    run_test(global_start, local_center, global_center, phi,theta)
}


#[test]
fn phi_2() {
    let global_start = start_location();

    let local_center = P2::new(0., 0.);
    let global_center= P3::new(-1., 0., 0.);

    let phi =  PI;
    let theta = PI/2.;


    run_test(global_start, local_center, global_center, phi,theta)
}

#[test]
fn phi_3() {
    let global_start = start_location();

    let local_center = P2::new(0., 0.);
    let global_center= P3::new(-1., -1., 0.);

    let phi =  5.*PI/ 4.;
    let theta = PI/2.;


    run_test(global_start, local_center, global_center, phi,theta)
}


#[test]
fn phi_4() {
    let global_start = start_location();

    let local_center = P2::new(0., 0.);
    let global_center= P3::new(1., 1., 0.);

    let phi =  PI/ 4.;
    let theta = PI/2.;


    run_test(global_start, local_center, global_center, phi,theta)
}

#[test]
fn phi_5() {
    let global_start = start_location();

    let local_center = P2::new(0., 0.);
    let global_center= P3::new(0., -1., 0.);

    let phi =  1.5 *PI;
    let theta = PI/2.;


    run_test(global_start, local_center, global_center, phi,theta)
}


#[test]
fn theta_1() {
    let global_start = start_location();

    let local_center = P2::new(0., 0.);
    let global_center= P3::new(1., 1., 1.);

    let phi =  PI/ 4.;
    let theta = PI/4.;


    run_test(global_start, local_center, global_center, phi,theta)
}


#[test]
fn theta_2() {
    let global_start = start_location();

    let local_center = P2::new(0., 0.);
    let global_center= P3::new(1., 0., -1.);

    let phi = 0.  ;
    let theta = 3.*PI/4.;


    run_test(global_start, local_center, global_center, phi,theta)
}