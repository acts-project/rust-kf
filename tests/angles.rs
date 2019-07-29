use kalman_rs::config::*;
use kalman_rs::filter::angles::Angles;
use std::ops::Sub;
use std::cmp::PartialOrd;

/*

    tests for kalman_rs::filter::angles::Angle{} which is used to lazily evaluate various angle combinations.
    This was primarily created to isolate problems with the linear KF.

*/

fn calc_global(phi: Real, theta: Real) -> (Real, Real, Real) {
    let tx = theta.sin() * phi.cos();
    let ty = theta.sin() * phi.sin();
    let tz = theta.cos();

    (tx, ty, tz)
}

fn assert(left: Real, right: Real){
    println!{"left is {} \t right is {}", left, right}
    let left = (left-right).abs();
    dbg!{left};
    assert!(left < DOT_PRODUCT_EPSILON)
}

#[test]
fn local_1 () {
    let phi = PI/4.;
    let theta = 5. * PI / 6.;
    let (tx, ty, tz) = calc_global(phi, theta);


    let mut ang = Angles::new_from_angles(phi, theta);
    assert(ang.tx(), tx);
    assert(ang.ty(), ty);
    assert(ang.tz(), tz);

    assert(ang.cos_theta, theta.cos());
    assert(ang.cos_phi, phi.cos());
    assert(ang.sin_theta, theta.sin());
    assert(ang.sin_phi, phi.sin())

}

#[test]
fn local_2 () {
    let phi = 5. *PI /4.;
    let theta = PI/6.;
    let (tx, ty, tz) = calc_global(phi, theta.clone());
    

    dbg!{tx}; dbg!{ty}; dbg!{tz};

    let mut ang = Angles::new_from_angles(phi, theta);
    assert(ang.tx(), tx);
    assert(ang.ty(), ty);
    assert(ang.tz(), tz);

    assert(ang.cos_theta, theta.cos());
    assert(ang.cos_phi, phi.cos());
    assert(ang.sin_theta, theta.sin());
    assert(ang.sin_phi, phi.sin())

}

#[test]
fn local_3 () {
    let phi = 11.*PI /6.;
    let theta = 3.*PI/4.;
    let (tx, ty, tz) = calc_global(phi, theta.clone());
    

    dbg!{tx}; dbg!{ty}; dbg!{tz};

    let mut ang = Angles::new_from_angles(phi, theta);
    assert(ang.tx(), tx);
    assert(ang.ty(), ty);
    assert(ang.tz(), tz);

    assert(ang.cos_theta, theta.cos());
    assert(ang.cos_phi, phi.cos());
    assert(ang.sin_theta, theta.sin());
    assert(ang.sin_phi, phi.sin())

}


#[test]
fn global_1 () {
    let phi = PI/4.;
    let theta = 5. * PI / 6.;
    let (tx, ty, tz) = calc_global(phi, theta);


    let mut ang = Angles::new_from_angles(phi, theta);
    assert(ang.tx(), tx);
    assert(ang.ty(), ty);
    assert(ang.tz(), tz);

    assert(ang.cos_theta, theta.cos());
    assert(ang.cos_phi, phi.cos());
    assert(ang.sin_theta, theta.sin());
    assert(ang.sin_phi, phi.sin())

}

#[test]
fn global_2 () {
    let phi = 1.5* PI;
    let theta = -1.2 * PI / 8.;
    let (tx, ty, tz) = calc_global(phi, theta);


    let mut ang = Angles::new_from_angles(phi, theta);
    assert(ang.tx(), tx);
    assert(ang.ty(), ty);
    assert(ang.tz(), tz);

    assert(ang.cos_theta, theta.cos());
    assert(ang.cos_phi, phi.cos());
    assert(ang.sin_theta, theta.sin());
    assert(ang.sin_phi, phi.sin())

}