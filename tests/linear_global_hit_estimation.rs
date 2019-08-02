use kalman_rs as krs;
use krs::config::*;
use krs::filter::{angles::Angles, prediction};

/*

    These are extensive tests for kalman_rs::filter::prediciton::linear_global_hit_estimation
    which is used to extrapolate the phi / theta and the current global position of the particle.
    These tests are pretty extensive as the function was very error prone when making the unit
    test for the linear KF

*/

/// basic macro for asserting multiple fields equaling each other
macro_rules! group_assert {
    ($var1:ident, $var2:ident, $($field:ident),+) => {
        $(
            let left = $var1.$field;//.abs();
            let right = $var2.$field;//.abs();
            dbg!{left};
            dbg!{right};

            assert!(left - right < DOT_PRODUCT_EPSILON);
        )+
    };
}

#[test]
fn estimation_1() {
    // along x-axis
    let phi = 0.;
    let theta = (PI) / 2.;

    let global_start = P3::new(0., 0., 0.);

    let sensor_center = Vec3::new(5., 0., 0.);
    let p1 = Vec3::new(5., 1., 1.);
    let p2 = Vec3::new(5., 0., 1.);

    let v1 = p1 - sensor_center;
    let v2 = p2 - sensor_center;
    let normal = v1.cross(&v2);

    let plane_const = normal.dot(&sensor_center);
    dbg! {plane_const};

    let mut angles = Angles::new_from_angles(phi, theta);

    let (global_est, distance) =
        prediction::linear_global_hit_estimation(&normal, &global_start, &mut angles, &plane_const);

    group_assert! {global_est, sensor_center, x,y,z}
}

#[test]
fn estimation_2() {
    // along y axis
    let phi = PI / 2.;
    let theta = (PI) / 2.;

    let global_start = P3::new(0., 0., 0.);

    let sensor_center = Vec3::new(0., 5., 0.);
    let p1 = Vec3::new(1., 5., 1.);
    let p2 = Vec3::new(0., 5., 1.);

    let v1 = p1 - sensor_center;
    let v2 = p2 - sensor_center;
    let normal = v1.cross(&v2);

    dbg! {normal};

    let plane_const = normal.dot(&sensor_center);
    dbg! {plane_const};

    let mut angles = Angles::new_from_angles(phi, theta);

    let (global_est, distance) =
        prediction::linear_global_hit_estimation(&normal, &global_start, &mut angles, &plane_const);

    group_assert! {global_est, sensor_center, x,y,z}
}

#[test]
fn estimation_3() {
    // along z axis
    let phi = PI / 2.;
    let theta = 0.;

    let global_start = P3::new(0., 0., 0.);

    let sensor_center = Vec3::new(0., 0., 1.);
    let p1 = Vec3::new(1., 5., 10.);
    let p2 = Vec3::new(1., 0., 10.);

    let v1 = p1 - sensor_center;
    let v2 = p2 - sensor_center;
    let normal = v1.cross(&v2);

    let plane_const = normal.dot(&sensor_center);
    dbg! {plane_const};

    let mut angles = Angles::new_from_angles(phi, theta);

    let (global_est, distance) =
        prediction::linear_global_hit_estimation(&normal, &global_start, &mut angles, &plane_const);

    group_assert! {global_est, sensor_center, x,y,z}
}

#[test]
fn estimation_4() {
    // between +x and +y axis
    let phi = PI / 4.;
    let theta = PI / 2.;

    let global_start = P3::new(1., 1., 0.);

    let sensor_center = Vec3::new(5., 5., 0.);
    let p1 = Vec3::new(6., 4., 1.);
    let p2 = Vec3::new(4., 6., -6.);

    let v1 = p1 - sensor_center;
    let v2 = p2 - sensor_center;
    let normal = v1.cross(&v2);

    let plane_const = normal.dot(&sensor_center);
    dbg! {normal};
    dbg! {plane_const};

    let mut angles = Angles::new_from_angles(phi, theta);

    let (global_est, distance) =
        prediction::linear_global_hit_estimation(&normal, &global_start, &mut angles, &plane_const);

    dbg! {angles};
    dbg! {sensor_center};
    dbg! {global_est};

    group_assert! {global_est, sensor_center, x,y,z}
}

#[test]
fn estimation_5() {
    // between +x and -y axis
    let phi = 7. * PI / 4.;
    let theta = PI / 2.;

    let global_start = P3::new(0., 0., 0.);

    let sensor_center = Vec3::new(5., -5., 0.);
    let p1 = Vec3::new(4., -6., 1.);
    let p2 = Vec3::new(6., -4., -6.);

    let v1 = p1 - sensor_center;
    let v2 = p2 - sensor_center;
    let normal = v1.cross(&v2);

    let plane_const = normal.dot(&sensor_center);
    dbg! {normal};
    dbg! {plane_const};

    let mut angles = Angles::new_from_angles(phi, theta);

    let (global_est, distance) =
        prediction::linear_global_hit_estimation(&normal, &global_start, &mut angles, &plane_const);

    dbg! {angles};
    dbg! {sensor_center};
    dbg! {global_est};

    group_assert! {global_est, sensor_center, x,y,z}
}

#[test]
fn estimation_6() {
    // between -x and +y axis
    let phi = 3. * PI / 4.;
    let theta = PI / 2.;

    let global_start = P3::new(0., 0., 0.);

    let sensor_center = Vec3::new(-5., 5., 0.);
    let p1 = Vec3::new(-6., 4., 1.);
    let p2 = Vec3::new(-4., 6., -6.);

    let v1 = p1 - sensor_center;
    let v2 = p2 - sensor_center;
    let normal = v1.cross(&v2);

    let plane_const = normal.dot(&sensor_center);
    dbg! {normal};
    dbg! {plane_const};

    let mut angles = Angles::new_from_angles(phi, theta);

    let (global_est, distance) =
        prediction::linear_global_hit_estimation(&normal, &global_start, &mut angles, &plane_const);

    dbg! {angles};
    dbg! {sensor_center};
    dbg! {global_est};

    group_assert! {global_est, sensor_center, x,y,z}
}

#[test]
fn estimation_7() {
    // between -x and -y axis
    let phi = 5. * PI / 4.;
    let theta = PI / 2.;

    let global_start = P3::new(0., 0., 0.);

    let sensor_center = Vec3::new(-5., -5., 0.);
    let p1 = Vec3::new(-6., -4., 1.);
    let p2 = Vec3::new(-4., -6., -6.);

    let v1 = p1 - sensor_center;
    let v2 = p2 - sensor_center;
    let normal = v1.cross(&v2);

    let plane_const = normal.dot(&sensor_center);
    dbg! {normal};
    dbg! {plane_const};

    let mut angles = Angles::new_from_angles(phi, theta);

    let (global_est, distance) =
        prediction::linear_global_hit_estimation(&normal, &global_start, &mut angles, &plane_const);

    dbg! {angles};
    dbg! {sensor_center};
    dbg! {global_est};

    group_assert! {global_est, sensor_center, x,y,z}
}

#[test]
fn estimation_8() {
    // along -x axis
    let phi = PI / 2.;
    let theta = PI / 2.;

    let global_start = P3::new(0., 0., 0.);

    let sensor_center = Vec3::new(-5., 0., 0.);
    let p1 = Vec3::new(-5., 5., 10.);
    let p2 = Vec3::new(-5., 0., 10.);

    let v1 = p1 - sensor_center;
    let v2 = p2 - sensor_center;
    let normal = v1.cross(&v2);

    let plane_const = normal.dot(&sensor_center);
    dbg! {plane_const};

    let mut angles = Angles::new_from_angles(phi, theta);

    let (global_est, distance) =
        prediction::linear_global_hit_estimation(&normal, &global_start, &mut angles, &plane_const);

    group_assert! {global_est, sensor_center, x,y,z}
}

#[test]
fn estimation_9() {
    // along -y axis
    let phi = 3. * PI / 2.;
    let theta = PI / 2.;

    let global_start = P3::new(0., 0., 0.);

    let sensor_center = Vec3::new(0., -5., 0.);
    let p1 = Vec3::new(-5., -5., 10.);
    let p2 = Vec3::new(0., -5., 10.);

    let v1 = p1 - sensor_center;
    let v2 = p2 - sensor_center;
    let normal = v1.cross(&v2);

    let plane_const = normal.dot(&sensor_center);
    dbg! {plane_const};

    let mut angles = Angles::new_from_angles(phi, theta);

    let (global_est, distance) =
        prediction::linear_global_hit_estimation(&normal, &global_start, &mut angles, &plane_const);

    group_assert! {global_est, sensor_center, x,y,z}
}

#[test]
fn estimation_10() {
    // along -z axis
    let phi = 0.;
    let theta = PI;

    let global_start = P3::new(0., 0., 0.);

    let sensor_center = Vec3::new(0., 0., -5.);
    let p1 = Vec3::new(-5., -5., -5.);
    let p2 = Vec3::new(0., -5., -5.);

    let v1 = p1 - sensor_center;
    let v2 = p2 - sensor_center;
    let normal = v1.cross(&v2);

    let plane_const = normal.dot(&sensor_center);
    dbg! {plane_const};

    let mut angles = Angles::new_from_angles(phi, theta);

    let (global_est, distance) =
        prediction::linear_global_hit_estimation(&normal, &global_start, &mut angles, &plane_const);

    group_assert! {global_est, sensor_center, x,y,z}
}
