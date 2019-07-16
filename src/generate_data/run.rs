use super::{statistics, store};
use statistics::Residuals;
use store::StorageData;

use std::fs;

use super::super::config::*;
use std::iter;

use serde::Serialize;

use rayon::{self, prelude::*};

/// Quickly create a repetitive value and collect it to a variable
/// $take: number of times to repeat the value
/// $name: name of variable to save to
/// $value: the value that will be repeated $take times
macro_rules! collect {
    ($take:expr; $($name:ident : $value:expr),+) => {
        $(
            let $name = iter::repeat($value).take($take).collect::<Vec<_>>();
        )+
    };
}


/// Quickly create vectors with a given capacity
macro_rules! with_capacity {
    ($cap:expr; $($name:ident),+) => {
        $(
            let mut  $name = Vec::with_capacity($cap)
        )+
    };
}

/// Quickly clone and push to make paths
macro_rules! path {
    ($base:ident; $($to_push:expr => $varname:ident),+) => {
        $(
            let mut $varname = $base.clone();
            $varname.push_str(r"\");
            $varname.push_str(&$to_push);
        )+
    };
}


#[derive(Serialize)]
pub struct State{
    pub histogram_name: String,
    pub iterations: usize,
    pub num_sensors: u32,
    pub sensor_distance: Real,
    pub angles: Option<(Real, Real)>,
    pub stdevs: Uncertainty,
    pub save_folder: String
}
impl State{
    pub fn default(folder_name : String, hist_name: String) -> Self{
        Self{
            histogram_name: hist_name,
            iterations: 1000,
            num_sensors: 10,
            sensor_distance: 0.01,
            angles: None,
            stdevs: Uncertainty::default(),
            save_folder: folder_name
        }
    }

}


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
            point_std: 0.001,
            diag_std: 1.5,
            corner_std: 1.,
            diag_mean: 3.,
            corner_mean: 0.
        }
    }
}

fn batch_execute(mut data: Vec<State> ) -> () {
    for i in 0..data.len(){
        let curr_data = data.remove(0);

        run(curr_data);
    }
}


fn run(data: State) {

    // let iter = data.iterations;
    collect!{
        data.iterations;
        num_sensors : data.num_sensors,
        sensor_distances : data.sensor_distance,
        angles : data.angles,
        point_stdev: data.stdevs.point_std
    }

    let kf_packaged_data = 
        statistics::collect_stats(num_sensors, sensor_distances, angles, point_stdev, &data.stdevs);

    println!{"got kf data"}

    let mut residuals : Vec<Residuals> = 
        statistics::fetch_kf_residuals(&kf_packaged_data);

    println!{"got residuals"}

    let len = data.num_sensors as usize;

    with_capacity!{len*data.iterations; smth, filt, pred}


    for _ in 0..data.iterations {
        let residual_struct = residuals.remove(0);

        residual_to_vec(&mut smth, &residual_struct.smth);
        residual_to_vec(&mut filt, &residual_struct.filt);
        residual_to_vec(&mut pred, &residual_struct.pred);
    }


    let mut save_folder = data.save_folder.clone();
    save_folder.push_str(r"\");

    fs::create_dir(&save_folder);

    // create extensions on the folder path for each csv
    path!{save_folder;
        "smth.csv" => smth_path,
        "filt.csv" => filt_path,
        "pred.csv" => pred_path
    }


    store::write_csv(&smth_path, smth);
    store::write_csv(&filt_path, filt);
    store::write_csv(&pred_path, pred);

    store::write_json(&data);
}



// TODO move this inside the parallelized calculation
fn residual_to_vec(
    storage: &mut Vec<StorageData>,
    res: &Vec<Vec2>
    ) -> () {
    
    res.iter()
        .for_each(|vec_res|{
            storage.push(StorageData::new(vec_res.x, vec_res.y))
        });
    
}

/// Quickly map and create folder names, histogram names, etc for any changing values.generate_data
/// 
/// $value_iterable: the values that will be mappened into the State struct. These are the values
///                  we are testing. everything else in the struct will remain constant
/// $folder_save_string: the extension for how the folder will be named. This will be concatenated onto the 
///                  save location from config.rs
/// $hist_save_string: what we will name the histogram once it is named (python handles this )
/// $field_to_change: the field on the State structwhose value will be replaced by each index of $value iterable. 
macro_rules! generate_data {

    // alterantive brnach. for modifying something in the stdev fields you need a `.` operator, which the other macro wont pick up on 
    ($value_iterable:ident, $folder_save_string:expr, $hist_save_string:expr, $field_one:ident . $field_two:ident) => {
        let mut counter = 0;
        let iter = 
        $value_iterable.into_iter()
            .map(|x|{
                let hist_save_name = format!{$hist_save_string, counter};
                counter += 1;

                let mut path = CSV_SAVE_LOCATION.clone().to_string();
                path.push_str(&format!{$folder_save_string, x});
                
                let mut data_struct = State::default(path, hist_save_name);
                data_struct.$field_one.$field_two = *x;     // this line is the only difference

                data_struct
            }).collect::<Vec<_>>();


        batch_execute(iter);
    };

    // main branch
    ($value_iterable:ident, $folder_save_string:expr, $hist_save_string:expr, $field_to_change:ident) => {
        let mut counter = 0;

        let iter = 
        $value_iterable.into_iter()
            .map(|x|{
                let hist_save_name = format!{$hist_save_string, counter};
                counter += 1;

                let mut path = CSV_SAVE_LOCATION.clone().to_string();
                path.push_str(&format!{$folder_save_string, x});

                let mut data_struct = State::default(path, hist_save_name);
                data_struct.$field_to_change = *x;

                data_struct
            }).collect::<Vec<_>>();


        batch_execute(iter);
        
    };
}


pub fn scaling_sensor_dist() ->() {

    let distances = [0.1, 0.01, 0.001, 0.0001, 0.00001, 0.000001, 0.0000001];

    // folder csvs are dumped to ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼    ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼name of generated histogram
    generate_data!{distances, "scale_sensor_distance_{}", "{}_sensor_distance.png", sensor_distance}
    //             ^^^^^^^^^               field of State that we modify with this test^^^^^^^^^^^^^
}   //    non default value we edit into the struct


pub fn scaling_point_std_dev() ->() {

    let std_devs = [0.1, 0.01, 0.001, 0.0001, 0.001, 0.00001, 0.000001, 0.0000001];

    generate_data!{std_devs, "scaling_point_std_dev{}", "{}_hit_standard_deviation.png", stdevs.point_std}
}


pub fn scaling_diagonal_mean() ->() {

    let std_devs = [0., 0.5, 1., 1.5, 2., 2.5, 3., 3.5, 4., 4.5, 5., 5.5, 6., 6.5];

    generate_data!{std_devs, "scaling_diagonal_mean{}", "{}_diagonal_mean.png", stdevs.diag_mean}
}


pub fn scaling_corner_mean() ->() {

    let std_devs = [0., 0.5, 1., 1.5, 2., 2.5, 3., 3.5, 4., 4.5, 5., 5.5, 6., 6.5];

    generate_data!{std_devs, "scaling_corner_mean{}", "{}_corner_mean.png", stdevs.corner_mean}
}


pub fn scaling_sensor_count() -> (){
    let counts = [5, 10, 15, 20, 40, 50, 60];

    generate_data!{counts, "scaling_sensor_count{}", "{}_sensor_count.png", num_sensors}
}


pub fn run_all_stats() {

    // scaling_corner_mean();
    // scaling_diagonal_mean();
    // scaling_point_std_dev();
    // scaling_sensor_dist();
    scaling_sensor_count();

}
