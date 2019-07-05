use kalman_rs::config::*;
use kalman_rs::filter::angles::Angles;
use std::ops::Sub;
// use std::cmp::PartialOrd;
// use std::ops::Outpus

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


    let mut ang = Angles::new_from_unit_direction(tx, ty, tz);
    dbg!{"tx"};
    assert(ang.tx(), tx);
    dbg!{"ty"};
    assert(ang.ty(), ty);
    dbg!{"tz"};
    assert(ang.tz(), tz);

    dbg!{"cos theta"};
    assert(ang.cos_theta, theta.cos());
    dbg!{"cos phi"};
    assert(ang.cos_phi, phi.cos());
    dbg!{"sin tehta"};
    assert(ang.sin_theta, theta.sin());
    dbg!{"sin phi"};
    assert(ang.sin_phi, phi.sin())

}

#[test]                     // This test fails
fn local_2 () {
    let phi = PI /4.;
    let theta = -PI/4.;
    let (tx, ty, tz) = calc_global(phi, theta.clone());
    
    // let sin_theta = theta.sin();
    // let abs = sin_theta.clone().abs();

    dbg!{tx}; dbg!{ty}; dbg!{tz};

    let mut ang = Angles::new_from_unit_direction(tx, ty, tz);
    println!{"tx"};
    assert(ang.tx(), tx);
    println!{"ty"};
    assert(ang.ty(), ty);
    println!{"tz"};
    assert(ang.tz(), tz);

    println!{"cos theta"};
    assert(ang.cos_theta, theta.cos());
    println!{"cos phi"};
    assert(ang.cos_phi, phi.cos());
    println!{"sin tehta"};
    assert(ang.sin_theta, theta.sin());
    println!{"sin phi"};
    assert(ang.sin_phi, phi.sin())

}

#[test]
fn global_1 () {
    let phi = PI/4.;
    let theta = 5. * PI / 6.;
    let (tx, ty, tz) = calc_global(phi, theta);


    let mut ang = Angles::new_from_angles(phi, theta);
    dbg!{"tx"};
    assert(ang.tx(), tx);
    dbg!{"ty"};
    assert(ang.ty(), ty);
    dbg!{"tz"};
    assert(ang.tz(), tz);

    dbg!{"cos theta"};
    assert(ang.cos_theta, theta.cos());
    dbg!{"cos phi"};
    assert(ang.cos_phi, phi.cos());
    dbg!{"sin tehta"};
    assert(ang.sin_theta, theta.sin());
    dbg!{"sin phi"};
    assert(ang.sin_phi, phi.sin())

}

#[test]
fn global_2 () {
    let phi = 1.5* PI;
    let theta = -1.2 * PI / 8.;
    let (tx, ty, tz) = calc_global(phi, theta);


    let mut ang = Angles::new_from_angles(phi, theta);
    dbg!{"tx"};
    assert(ang.tx(), tx);
    dbg!{"ty"};
    assert(ang.ty(), ty);
    dbg!{"tz"};
    assert(ang.tz(), tz);

    dbg!{"cos theta"};
    assert(ang.cos_theta, theta.cos());
    dbg!{"cos phi"};
    assert(ang.cos_phi, phi.cos());
    dbg!{"sin tehta"};
    assert(ang.sin_theta, theta.sin());
    dbg!{"sin phi"};
    assert(ang.sin_phi, phi.sin())

}