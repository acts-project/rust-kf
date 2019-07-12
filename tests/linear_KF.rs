use kalman_rs as krs;
use krs::config::*;
use krs::geometry::Rectangle;                   // rect sensors
use krs::geometry::traits::{Plane, Transform};
use krs::filter::prediction;

use nalgebra::base::Unit;
use krs::{print, get_unchecked};

use rand::{thread_rng, Rng};
use rand_distr::{Normal, Distribution};

/*

    This is the unit test for the entire linear kalman filter

*/

macro_rules! group_assert {
    ($left:ident, $right:ident, $($index:ident),+) => {
        $(
            let left = $left.$index;
            let right = $right.$index;

            let diff = right - left;

            print!{left, right, diff}

            assert!{diff.abs() < DOT_PRODUCT_EPSILON}
        )+
    };
}


const UNCERTAINTY : Real = 2.;

fn gen_cov() -> Mat2{
    let base_cov = Mat2::identity();     // high covariance across the main diagonal
    let cov_noise = Mat2::new_random() / UNCERTAINTY;       // every index random # in [0,1] * uncertainty
    let covariance = base_cov + cov_noise;
    covariance
}

fn gen_sensor(x_point: Real) -> Rectangle {
    // sensor dimensions
    let base = 100.;
    let height = 100.;


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
    Rectangle::new_test_sensor(
        base,
        height,
        to_global,
        to_local,
        p1,
        p2
    )

}

fn random_hit() -> Vec2{
    Vec2::new_random()
    
}

#[test]
fn linear_1() {
    let start = P3::new(0. , 0. , 0.);

    let mut cov_vec = Vec::new();
    let mut sensor_vec = Vec::new();
    let mut meas_vec = Vec::new();


    let iterations = 5;
    for i in 0..iterations {
        let x_point = ((i+1) * 5) as Real;

        cov_vec.push(gen_cov());
        sensor_vec.push(gen_sensor(x_point));
        meas_vec.push(random_hit())

    }

    let kf_result = krs::filter::linear::run(&start, &cov_vec, &meas_vec, &sensor_vec);

    let last_state = kf_result.state_vec.get(iterations-1).unwrap();

    group_assert!{last_state, start, x, y}
}


// This test fails currently
// #[test]                 
fn linear_2() {
    // parallel to x axis
    let start = P3::new(0. , 1. , 1.);

    let mut cov_vec = Vec::new();
    let mut sensor_vec = Vec::new();
    let mut meas_vec = Vec::new();


    let iterations = 5;
    for i in 0..iterations {
        let x_point = ((i+1) * 5) as Real;

        cov_vec.push(gen_cov());
        sensor_vec.push(gen_sensor(x_point));
        meas_vec.push(random_hit())

    }

    let kf_result = krs::filter::linear::run(&start, &cov_vec, &meas_vec, &sensor_vec);

    dbg!{&kf_result};

    let last_state = kf_result.state_vec.get(iterations-1).unwrap();

    group_assert!{last_state, start, x, y}
}


const STD_DEV : Real = 0.5;

struct KFData <T: Transform + Plane>{
    start: P3,
    original_angles: (Real, Real),
    sensors: Vec<T>,
    cov: Vec<Mat2>,
    smear_hits: Vec<Vec2>,
    truth_hits: Vec<Vec2>
}
impl <T> KFData<T> where T: Transform + Plane {
    fn new(sensors: Vec<T>, 
    covariance_mat: Vec<Mat2>,
    smeared_measurements: Vec<Vec2>,
    truth_measurements: Vec<Vec2>,
    original_angles: (Real, Real)) -> Self{
        KFData {
            start: P3::origin(),
            sensors: sensors, 
            cov: covariance_mat, 
            smear_hits: smeared_measurements, 
            truth_hits: truth_measurements,
            original_angles: original_angles
            }
    }

    fn compare_hits(&self, kf_state_vec: &Vec<Vec5>) {
        let kf_hits =
            kf_state_vec.iter()
            .map(|vec|{
                get_unchecked!{
                    vec[eLOC_0] => x,
                    vec[eLOC_1] => y
                }
                Vec2::new(*x, *y)
            })
            .collect::<Vec<_>>();

        self.truth_hits.iter()
            .zip(kf_hits.iter())
            .map(|(truth, smear)| {
                let diff = truth - smear;
                println!{"total diff: {}\ttruth x:{}\tsmear x: {}\tdiff:{}  \t truth y: {}\tsmear y: {}\t diff: {}", diff.norm(),truth.x, smear.x, diff.x, truth.y, smear.y, diff.y}
                (truth, smear)
            })
            .collect::<Vec<_>>()
            .into_iter()
            .for_each(|(truth, smear)|{
                println!{""}
                group_assert!{truth, smear, x, y}
            });
        
            
    }
}

fn generate_truth_track() -> KFData<Rectangle>{
    let mut rng = thread_rng();

    // generates phi as  0 < x < 20 || 340 < x < 360 
    // first make angle between 0 and 40 degrees
    let _phi = rng.gen_range(0., 0.6981317);
    let phi = 
        if _phi >= 0.3490659 { // if phi between 20 and 40 degrees
            // shifts phi by 320 degrees. this moves 20 < _phi < 40 to 340 < x< 360.
            _phi + 5.585054   
        }
        else {
            _phi
        };

    // between 70 deg and 110 deg (+/- 20 from 90 which is along x axis)
    let theta = rng.gen_range(1.22173, 1.919862); 

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
    for i in 0..5 {
        let x = ((i+1) * 5) as Real;
        sensor_vec.push(gen_sensor(x));
    }


    // find the locations of the true sensor hits
    let mut truth_hits = Vec::new();

    for i in 0..sensor_vec.len() {
        let curr_sensor = &sensor_vec[i];

        let (pred_sv, _) = 
            prediction::linear_state_vector(
                &virtual_sensor,
                &curr_sensor,
                &start_state_vec
            ).expect("out of bounds");

        get_unchecked!{
            pred_sv[eLOC_0] => x_hit,
            pred_sv[eLOC_1] => y_hit
        }

        truth_hits.push(Vec2::new(*x_hit, *y_hit))
    }
    
    // smear the hit locations
    let smeared_hits = 
        truth_hits.clone().into_iter()
            .map(|point|{
                let xnorm = Normal::new(point.x, STD_DEV).unwrap();
                let ynorm = Normal::new(point.y, STD_DEV).unwrap();

                let smear_x = xnorm.sample(&mut rng);
                let smear_y = ynorm.sample(&mut rng);

                Vec2::new(smear_x,  smear_y)
            })
            .collect::<Vec<_>>();

    // print out truth vs smeared hits
    smeared_hits.iter()
        .zip(truth_hits.iter())
        .for_each(|(truth, smear)| {
            let x_diff = (smear.x - truth.x).abs();
            let y_diff = (smear.y - truth.y).abs();

            println!{"truth x:{}\tsmear x: {}\tdiff:{}  \t truth y: {}\tsmear y: {}\t diff: {}",truth.x, smear.x, x_diff, truth.y, smear.y, y_diff}
        });

    // create distributions with higher mean on the diagonal and equivalent uncertainty
    let diag_distr = Normal::new(1., STD_DEV).unwrap();
    let other_dirt = Normal::new(0., STD_DEV).unwrap();

    let covariance_vec = 
        (0..sensor_vec.len()).into_iter()
        .map(|_|{
            let a = diag_distr.sample(&mut rng).abs();
            let d = diag_distr.sample(&mut rng).abs();

            let b = other_dirt.sample(&mut rng).abs();
            let c = other_dirt.sample(&mut rng).abs();
            
            let m = Mat2::new(a, b, c , d);
            print!{m}
            m
        })
        .collect::<Vec<_>>();


    KFData::new(sensor_vec, covariance_vec, smeared_hits, truth_hits, (phi, theta))
}

// generate track 
#[test]                 
fn linear_3() {
    let data = generate_truth_track();

    let kf_result = krs::filter::linear::run(&data.start, &data.cov, &data.smear_hits, &data.sensors);

    data.compare_hits(&kf_result.state_vec);

    panic!{"da"}

}