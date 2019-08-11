use kalman_rs::config::*;
use kalman_rs::filter::utils::seed_state_vec_from_points as seed_vec;
use kalman_rs::geometry::Rectangle;
use kalman_rs::get_unchecked;
use kalman_rs::print_;
use nalgebra::base::Unit;

/**
 *  Tests for initial seeding of the filtered state vector since the prediction phase
 *  requires a previous filtered vector.
 *
 */
fn start_location() -> P3 {
    P3::new(0., 0., 0.)
}

// general form of tests;
fn run_test(global_start: P3, local_end: P2, global_end: P3, phi: Real, theta: Real) -> () {
    let expected_vec = Vec5::new(local_end.x, local_end.y, phi, theta, 1.);

    let filt_vec = seed_vec(&global_start, &global_end, &local_end);

    dbg! {expected_vec};
    dbg! {filt_vec};

    for i in 0..4 {
        let predicted = expected_vec.get(i).unwrap();
        let actual = filt_vec.get(i).unwrap();

        dbg! {predicted};
        dbg! {actual};

        assert! {(predicted - actual) < DOT_PRODUCT_EPSILON};
    }
}

#[test]
fn phi_1() {
    let global_start = start_location();

    let local_center = P2::new(0., 0.);
    let global_center = P3::new(1., 0., 0.);

    let phi = 0.;
    let theta = PI / 2.;

    run_test(global_start, local_center, global_center, phi, theta)
}

#[test]
fn phi_2() {
    let global_start = start_location();

    let local_center = P2::new(0., 0.);
    let global_center = P3::new(-1., 0., 0.);

    let phi = PI;
    let theta = PI / 2.;

    run_test(global_start, local_center, global_center, phi, theta)
}

#[test]
fn phi_3() {
    let global_start = start_location();

    let local_center = P2::new(0., 0.);
    let global_center = P3::new(-1., -1., 0.);

    let phi = 5. * PI / 4.;
    let theta = PI / 2.;

    run_test(global_start, local_center, global_center, phi, theta)
}

#[test]
fn phi_4() {
    let global_start = start_location();

    let local_center = P2::new(0., 0.);
    let global_center = P3::new(1., 1., 0.);

    let phi = PI / 4.;
    let theta = PI / 2.;

    run_test(global_start, local_center, global_center, phi, theta)
}

#[test]
fn phi_5() {
    let global_start = start_location();

    let local_center = P2::new(0., 0.);
    let global_center = P3::new(0., -1., 0.);

    let phi = 1.5 * PI;
    let theta = PI / 2.;

    run_test(global_start, local_center, global_center, phi, theta)
}

#[test]
fn theta_1() {
    let global_start = start_location();

    let local_center = P2::new(0., 0.);
    let global_center = P3::new(1., 1., 1.);

    let phi = PI / 4.;
    let theta = PI / 4.;

    run_test(global_start, local_center, global_center, phi, theta)
}

#[test]
fn theta_2() {
    let global_start = start_location();

    let local_center = P2::new(0., 0.);
    let global_center = P3::new(1., 0., -1.);

    let phi = 0.;
    let theta = 3. * PI / 4.;

    run_test(global_start, local_center, global_center, phi, theta)
}

fn gen_sensor(x_point: Real) -> Rectangle {
    // sensor dimensions
    let base = 10000.;
    let height = 10000.;

    let y_axis = Vec3::new(0., 1., 0.);
    let j = Unit::try_new(y_axis, 0.).unwrap();

    let l2g_rot = Mat4::from_axis_angle(&j, PI / 2.);
    let g2l_rot = l2g_rot
        .try_inverse()
        .expect("rotation matrix non invert test");

    let trans = Trl3::new(x_point, 0., 0.).to_homogeneous();

    let mat = trans * l2g_rot;

    let to_global = Aff3::from_matrix_unchecked(mat);
    let to_local = to_global.try_inverse().unwrap();

    print_! {"SENSOR CENTER WILL BE AT ", to_global * P3::origin() }

    let p1 = P3::new(x_point, 1., 1.);
    let p2 = P3::new(x_point, 0., 1.);

    // new_test_sensor() is created to hard code values (transformation matricies, points)
    // instead of using new() which uses inverses
    Rectangle::new_test_sensor(base, height, to_global, to_local, l2g_rot, g2l_rot, p1, p2)
}

#[test]

fn seed_state_vec_sensor_1() {
    let start_location = P3::new(0., 1000000000., 0.);
    let end_sensor = gen_sensor(10.);

    let hit = &Vec2::new(start_location.y, start_location.z);

    let filt_vec =
        kalman_rs::filter::utils::seed_state_vec_from_sensor(&start_location, &end_sensor, &hit);
    print_! {filt_vec};

    get_unchecked! {vector;filt_vec;
        eLOC_0 => xhit,
        eLOC_1 => yhit,
        ePHI => phi,
        eTHETA => theta
    }

    assert! {*xhit - hit.x < DOT_PRODUCT_EPSILON}
    assert! {*yhit - hit.y < DOT_PRODUCT_EPSILON}
    assert!(phi - (3. * PI / 2.) < DOT_PRODUCT_EPSILON);
    assert! {theta - (PI/ 2.) < DOT_PRODUCT_EPSILON};
}

#[test]
fn seed_state_vec_sensor_2() {
    let start_location = P3::new(0., 0., 1000000000.);
    let end_sensor = gen_sensor(10.);

    let hit = &Vec2::new(start_location.y, start_location.z);

    let filt_vec =
        kalman_rs::filter::utils::seed_state_vec_from_sensor(&start_location, &end_sensor, &hit);
    print_! {filt_vec};

    get_unchecked! {vector;filt_vec;
        eLOC_0 => xhit,
        eLOC_1 => yhit,
        ePHI => phi,
        eTHETA => theta
    }

    assert! {*xhit - hit.x < DOT_PRODUCT_EPSILON}
    assert! {*yhit - hit.y < DOT_PRODUCT_EPSILON}
    assert!(*phi < DOT_PRODUCT_EPSILON);
    assert! {*theta - PI < DOT_PRODUCT_EPSILON};
}
