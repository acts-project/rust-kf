use super::{statistics, store};

use super::structs::{StorageData, State, Residuals};

use std::fs;

use super::super::config::*;

use rayon::{self, prelude::*};


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
    let mut save_folder = data.save_folder.clone();
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


fn fetch_separated_kf_data(data: &State) {
    let kf_packaged_data = statistics::collect_stats(&data);

    let vec_residuals = 
        statistics::truth_kf_output_residuals(kf_packaged_data);

    
    let mut sensor_separated_residuals = 
        (0..(data.num_sensors as usize)).into_iter()
        .map(|_| Vec::new())
        .collect::<Vec<_>>();

    for i in 0..(data.num_sensors as usize) {
        let inner = sensor_separated_residuals.get_mut(i).expect("statistics out of bounds");
        
        vec_residuals.iter()
            .for_each(|x|{
                inner.push(x.smth[i]);               // this line changes what field we are looking at
            });
    }

    // serialize into vector of structs to serialize

    sensor_separated_residuals.into_iter().zip(0..data.num_sensors)
        .map(|(vector, count)|{
            let vec = vector.into_iter().map(|resid| {StorageData::new(resid.x, resid.y)}).collect::<Vec<_>>();

            (vec, count)
        
        })
        .for_each(move|(vec_vec2, count)|{
            let sub_path = format!{r"\sensor_{}.csv", count};
            let path = data.save_folder.clone() + &sub_path;
            print!{path};
            store::write_csv(&path, vec_vec2)
        });
        

    
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
    let state = State::default("generated_truth_smear_residuals".to_string(),  "_truth_smear_residuals.png".to_string());
    fetch_kf_randomness_residuals(&state);
}

// residuals between truth and sensor (pred/  filt/ smth) at each sensor
fn test_initial_predictions() -> () {
    print!{"here"}
    let state = State::default(r".\data\initial_prediction_data\".to_string(), "this_does_not_matter.png".to_string());
    fetch_separated_kf_data(&state);
}

fn run_one_test() -> () {
    let state = State::default(r"E:\kf_csvs\default_parameters".to_string(), "default_parameters.png".to_string());
    run(state);
}
pub fn run_all_stats() {

    // scaling_corner_mean();
    // scaling_point_std_dev();
    // scaling_diagonal_mean();
    // scaling_sensor_dist();
    // scaling_sensor_count();

    // test_generated_residuals();
    // test_initial_predictions();

    run_one_test();
}
