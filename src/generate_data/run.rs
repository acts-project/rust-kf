use super::{statistics, store};

use super::structs::{self, StorageData, State, Residuals};

use std::fs;

use super::super::config::*;

use rayon::{self, prelude::*};

use itertools::izip;

fn batch_execute(mut data: Vec<State> ) -> () {
    for i in 0..data.len(){
        let curr_data = data.remove(0);

        run(curr_data);
    }
}


fn run(data: State) {
    
    let kf_packaged_data = 
        statistics::collect_stats(&data);

    let mut residuals : Vec<Residuals> = 
        statistics::fetch_kf_residuals(&kf_packaged_data);

    println!{"finished KF operations for {}", &data.histogram_name}

    let len = (data.num_sensors as usize) * data.iterations;

    let mut smth = Vec::with_capacity(len);
    let mut filt = Vec::with_capacity(len);
    let mut pred = Vec::with_capacity(len);


    residuals.iter().for_each(|res| {
        residual_to_vec(&mut smth, &res.smth);
        residual_to_vec(&mut filt, &res.filt);
        residual_to_vec(&mut pred, &res.pred);
        });


    let mut save_folder = data.save_folder.clone().to_string();
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

    println!{"finished {}", &data.histogram_name}

}


/// Calls all child functions for calculating the residuals for truth vs smeared
/// points
fn fetch_kf_randomness_residuals(data: &State) {
    let kf_packaged_data = statistics::collect_stats(data);

    let kf_data :Vec<StorageData>= 
        kf_packaged_data.iter().map(|(x, _ )| {
           
            statistics::smear_residuals(&x)

        })
        .flatten()
        .map(|x| {
            StorageData::new(x.x, x.y)
        })
        .collect::<Vec<_>>();

    /*

        configure folders and save destinations

    */
    let mut save_folder = data.save_folder.clone().to_string();
    save_folder.push_str(r"\");

    fs::create_dir(&save_folder);

    // create extensions on the folder path for each csv
    path!{save_folder;
        "smth.csv" => smth_path
    }

    /*
        write data to files
    */

    store::write_csv(&smth_path, kf_data);

    store::write_json(&data);

}


type nest_vec = Vec<Vec<Vec2>>;
fn residuals_by_sensor(vec_res: Vec<Residuals>,num_sensors: usize ) -> (nest_vec, nest_vec, nest_vec) {
    let mut sensor_predictions = 
        (0..num_sensors).into_iter()
        .map(|_| Vec::with_capacity(num_sensors))
        .collect::<Vec<_>>();

    let mut sensor_filters = sensor_predictions.clone();
    let mut sensor_smoothes = sensor_predictions.clone();

    for i in 0..num_sensors {
        let mut inn_pred = sensor_predictions.get_mut(i).expect("statistics out of bounds");
        let mut inn_filt = sensor_filters.get_mut(i).expect("asd");
        let mut inn_smth = sensor_smoothes.get_mut(i).expect("sdf");

        vec_res.iter()
            .for_each(|x|{
                inn_pred.push(x.pred[i]);
                inn_filt.push(x.filt[i]);
                inn_smth.push(x.smth[i]);
            });
        
    }

    (sensor_predictions, sensor_filters, sensor_smoothes)

}


/// linear kalman filter exporting data based on individual sensors
fn fetch_separated_kf_data(data: &State) {
    let kf_packaged_data = statistics::collect_stats(&data);

    let vec_residuals = 
        statistics::truth_kf_output_residuals(kf_packaged_data);

    let (sensor_predictions, sensor_filters, sensor_smoothes) = residuals_by_sensor(vec_residuals, data.num_sensors as usize);
    

    let counts = 0..data.num_sensors;
    let zipped_data = izip!{sensor_predictions, sensor_filters, sensor_smoothes, counts };

    // serialize into vector of structs to serialize

    zipped_data.into_iter()
        .map(|(pred, filt, smth, count)|{
            // converts Vec<Vec2> to Vec<StorageData> to use in serializing to csv
            let to_storage = |x: Vec<Vec2>| x.into_iter().map(|res| StorageData::from_vec2(res) ).collect::<Vec<_>>();
            
            let p_ = to_storage(pred);
            let f_ = to_storage(filt);
            let s_ = to_storage(smth);
        
            (p_, f_, s_, count)
        
        })
        .for_each(move|(pred, filt, smth, count)|{

            let make_path_and_write = |storage_data, subfolder_name, count| {
                // make the subdirectoy
                let folder_path = data.save_folder.clone().to_string() + &format!{r"\{}\",subfolder_name};
                // path to the actual csv we are going to write
                std::fs::create_dir(&folder_path);
                let path = folder_path + &format!{r"sensor_{}.csv", count};
                //write the csv
                store::write_csv(&path, storage_data);
            };

            make_path_and_write(pred, stringify!{pred}, count);
            make_path_and_write(filt, stringify!{filt}, count);
            make_path_and_write(smth, stringify!{smth}, count);
            
        });
        
}


/// smear / predicted / covariance diagonal elements for the KF
fn sensor_separated_with_truth(data: &State) -> () {
    let mut kf_packaged_data = statistics::collect_stats(&data);

    let len = kf_packaged_data.len();
    let mut truth_vec = Vec::with_capacity(len);
    let mut kf_pred_vec = Vec::with_capacity(len);
    let mut covariance = Vec::with_capacity(len);

    let sep_data = 
    kf_packaged_data.into_iter()
        .map(|(mut kf_data, mut super_data)| {
            let mut kf_smear = structs::SerStateVec::new(kf_data.smear_initial_vector);
            let mut pred_ser = structs::SerStateVec::new(super_data.pred.state_vec.remove(1));

            let cov_mat = super_data.pred.cov_mat.remove(1);

            get_unchecked!{cov_mat[(0,0)] => a, cov_mat[(1,1)] => b}

            let cov = StorageData::new(*a,*b);

            truth_vec.push(kf_smear);
            kf_pred_vec.push(pred_ser);
            covariance.push(cov);

            
        })
        .collect::<Vec<_>>();

    let path = data.save_folder.to_string() + "\\pred.csv";
    store::write_csv(&path, kf_pred_vec);
    let path = data.save_folder.to_string() + "\\truth.csv";
    store::write_csv(&path, truth_vec);
    let path = data.save_folder.to_string() + "\\cov.csv";
    store::write_csv(&path, covariance);

}

fn pull_distribution(data: &State) {
    let mut kf_packaged_data = statistics::collect_stats(&data);
    
    let vals = kf_packaged_data.into_iter()
        .map(|(mut kf_data, mut super_data)| {
            // get difference between initial filtered state vector
            // and the predicted state vector at the first sensor
            let init = &kf_data.smear_initial_vector;
            let kf_pred = &super_data.pred.state_vec.remove(1);
            let mut diff = init-kf_pred;

            // get the predicted covariance at the first sensor
            let cov = super_data.pred.cov_mat.remove(1);

            // fetch diagonal elements
            get_unchecked!{
                cov[(0,0)] => x,
                cov[(1,1)] => y,
                cov[(2,2)] => p,
                cov[(3,3)] => t,
                cov[(4,4)] => qop
            }
            
            // (initial_track_parameters - predicted_track_param_sensor_1) / corresponding_covariance_diagonal
            edit_matrix!{diff;
                [0] /= x,
                [1] /= y,
                [2] /= p,
                [3] /= t,
                [4] /= qop
            }

            // Load the difference into a struct that can be written to csv
            structs::SerStateVec::new(diff)
        })
        .collect::<Vec<_>>();

    let path = data.save_folder.to_string() + "pull_data.csv";
    store::write_csv(&path, vals);

}


fn residual_to_vec(
    storage: &mut Vec<StorageData>,
    res: &Vec<Vec2>
    ) -> () {
    
    res.iter()
        .for_each(|vec_res|{
            storage.push(StorageData::new(vec_res.x, vec_res.y))
        });
    
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


// residuals between truth vs smeared values 
fn test_generated_residuals() -> () {
    let state = State::default("generated_truth_smear_residuals",  "_truth_smear_residuals.png");
    fetch_kf_randomness_residuals(&state);
}

// residuals between truth and sensor (pred/  filt/ smth) at each sensor
fn test_initial_predictions() -> () {
    print!{"here"}
    let state = State::default(r".\data\initial_prediction_data\", "this_does_not_matter.png");
    fetch_separated_kf_data(&state);
}

// Runs a singular test with default State parameters
fn run_one_test() -> () {
    let state = State::default(r"E:\kf_csvs\default_parameters", "default_parameters.png");
    run(state);
}

fn ridder_algo() -> () {
    let mut state = State::default(r".\data\ridder_algo_data\", "ridder_data.png");
    state.num_sensors = 2;
    state.angles = Some((0., PI/2.));
    sensor_separated_with_truth(&state);
}

fn pull_data() {
    let mut state = State::default(r".\data\pull_data\", "pull_data.png");
    state.num_sensors = 2;
    state.angles = Some((0., PI/2.));

    // state.sensor_distance = 0.0000000001;
    // state.stdevs.point_std = 0.000000001;
    // state.stdevs.diag_mean = 10.;

    dbg!{&state};

    pull_distribution(&state);
}


pub fn run_all_stats() {

    // scaling_corner_mean();
    // scaling_point_std_dev();
    // scaling_diagonal_mean();
    // scaling_sensor_dist();
    // scaling_sensor_count();

    // test_generated_residuals();
    // test_initial_predictions();

    // ridder_algo();

    // run_one_test();

    pull_data();
}
