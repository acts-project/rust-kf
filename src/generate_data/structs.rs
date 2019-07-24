use super::super::geometry::traits::{Plane, Transform};
use super::super::config::*;
use serde::Serialize;



use rand::{thread_rng, SeedableRng, Rng};
use rand::rngs::SmallRng;
use rand_distr::{Normal, Distribution};


/// Holds x and y points from the kf, usually either the 
/// residual from truth vs smear or truth vs kf output
#[derive(Debug,Serialize)]
pub struct StorageData {
    x_points: Real,
    y_points: Real
}
impl StorageData {
    pub fn new(x: Real, y: Real) -> Self {
        StorageData{
            x_points: x ,
            y_points: y
        }
    }
}

/// Stores information on the truth and smeared track paramers
pub struct KFData <T: Transform + Plane>{
    pub start: P3,
    pub original_angles: (Real, Real),  // (phi, theta)
    pub sensors: Vec<T>,
    pub cov: Vec<Mat2>,
    pub smear_hits: Vec<Vec2>,
    pub truth_hits: Vec<Vec2>,
    pub smear_initial_vector: Vec5,
    pub truth_initial_vector: Vec5,
    pub b_field : Vec3
}

impl <T> KFData<T> where T: Transform + Plane {
    pub fn new(
        sensors: Vec<T>, 
        covariance_mat: Vec<Mat2>,
        smeared_measurements: Vec<Vec2>,
        truth_measurements: Vec<Vec2>,
        original_angles: (Real, Real),
        smear_state_vec: Vec5,
        truth_state_vec: Vec5,
        b_field: Option<Vec3>
    ) -> Self{

        let b_field = 
        if let Some(field_vector) = b_field{
            field_vector
        }
        else {
            Vec3::zeros()
        };

        KFData {
            start: P3::origin(),
            sensors: sensors, 
            cov: covariance_mat, 
            smear_hits: smeared_measurements, 
            truth_hits: truth_measurements,
            original_angles: original_angles,
            smear_initial_vector: smear_state_vec,
            truth_initial_vector: truth_state_vec,
            b_field: b_field
            }
    }

}


/// Holds all information on what paramers are going to be used for 
/// the generation of a linear track.
#[derive(Serialize)]
pub struct State{
    pub histogram_name: String,
    pub iterations: usize,
    pub num_sensors: u32,
    pub sensor_distance: Real,
    pub angles: (Real, Real),
    pub b_field: Vec3,
    pub qop: Real,
    pub stdevs: Uncertainty,
    pub save_folder: String
}
impl State{
    pub fn default(folder_name : String, hist_name: String) -> Self{
        Self{
            histogram_name: hist_name,
            iterations: 70_000,
            num_sensors: 10,
            sensor_distance: 0.0001,            
            angles: Self::calculate_angles(),
            b_field: Vec3::zeros(),
            qop: 1.,
            stdevs: Uncertainty::default(),
            save_folder: folder_name
        }
    }
    
    pub fn default_const_b(folder_name: String, hist_name:String) -> Self {
        let mut linear = Self::default(folder_name,hist_name);
        
        // 2 Tesla magnitude b field
        linear.b_field = Vec3::new(1.154700, 1.154700,1.154700);
        // 10 Gev qop value
        linear.qop = 0.000000001602176487;

        linear
    }

    fn calculate_angles() -> (Real, Real) {
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

        (phi, theta)

        
    }

}


/// Means and standard deviations of paramers that will be used in 
/// `statistics.rs` and `setup.rs`
/// `diag` corresponds to "a" and "d" in the matrix below
/// `corner` corresponds to "b" and "c" in the matrix below
/// 
/// ```
/// | a , b |
/// | c , d |
/// ```
#[derive(Serialize)]
pub struct Uncertainty {
    pub point_std: Real,
    pub diag_std: Real,
    pub corner_std: Real,
    pub diag_mean: Real,
    pub corner_mean: Real
}

impl Uncertainty {
    pub fn default() -> Self {
        Self{
            point_std: 0.1,
            diag_std: 1.,
            corner_std: 1.,
            diag_mean: 2.,
            corner_mean: 0.,
            
        }
    }
    pub fn diagonal_value(&self, rng: &mut SmallRng) -> Real{
        // TODO: should probably avoid re-initializing this every time
        let distr=  Normal::new(self.diag_mean, self.diag_std).unwrap();
        distr.sample(rng)
    }

    pub fn corner_value(&self, rng: &mut SmallRng) -> Real{
        // let distr=  Normal::new(self.corner_mean, self.corner_std).unwrap();
        // distr.sample(rng)
        0.
    }

    pub fn smear_hit(&self, rng: &mut SmallRng, point_val: Real) -> Real {
        let distr=  Normal::new(point_val, self.point_std).unwrap();
        distr.sample(rng)
    }
}


/// Holds residuals for truth points vs KF output points
pub struct Residuals{
    pub smth: Vec<Vec2>,
    pub filt: Vec<Vec2>,
    pub pred: Vec<Vec2>
}
impl Residuals {
    pub fn new_grouped(grouped_vec: Vec<(Vec2, Vec2, Vec2)>) -> Self{
        let mut s = Vec::new();
        let mut f = Vec::new();
        let mut p = Vec::new();

        grouped_vec.into_iter()
            .for_each(|(smth, filt, pred)| {
                s.push(smth);
                f.push(filt);
                p.push(pred);
            });

        Residuals{
            smth: s,
            filt: f,
            pred: p
        }
    }
}



#[derive(Serialize)]
pub struct SingleField{
    data: Real
}

impl SingleField {
    pub fn new(x: Real) -> Self {SingleField {data: x}}
}
