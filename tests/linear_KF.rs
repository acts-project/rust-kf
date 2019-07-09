use kalman_rs as krs;
use krs::config::*;
use krs::geometry::Rectangle;                   // rect sensors
use krs::geometry::traits::{Plane, Transform};


fn gen_cov(cov_vec: &mut Vec<Mat2>) -> (){
    let base_cov = Mat2::identity();          // high covariance across the main diagonal
    let cov_noise = Mat2::new_random() / 10.; // every index random # in [0,1]
    let covariance = base_cov + cov_noise;

    cov_vec.push(covariance);
}

fn gen_sensor(sensor_vec: &mut Vec<Rectangle>, index: Real) {
    let base = 6.;
    let height = 6.;

    // transform (0,0,0) local to (index*5, 0, 0)
    let to_global = Mat4::new(
        1.,1.,1., index *2.,
        0.,0.000001,0.,0.,
        0.,0.,0.0001,0.,
        0.,0.,0.,1.,
    );

    let aff = Aff3::from_matrix_unchecked(to_global.clone());

    let sensor = 
        Rectangle::new(
            base,
            height,
            to_global
        ).unwrap();
    sensor_vec.push(sensor);

}//

fn gen_hit(meas_vec: &mut Vec<Vec2>) {
    let hit = Vec2::new_random() / 10.;
    
    meas_vec.push(hit);
}

// #[test]
fn linear_1() {
    let start = P3::new(0.01, 0.01, 0.01);

    let mut cov_vec = Vec::new();
    let mut sensor_vec = Vec::new();
    let mut meas_vec = Vec::new();

    for i in 1..5 {
        println!{"here"}
        gen_cov(&mut cov_vec);
        gen_sensor(&mut sensor_vec, i as Real);
        gen_hit(&mut meas_vec);

    }

    let KF_result = krs::filter::linear::run(&start, &cov_vec, &meas_vec, &sensor_vec);



}