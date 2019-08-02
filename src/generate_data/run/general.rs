use super::super::super::{
    config::*,
    filter::utils::{self, Data},
};
use super::super::{
    statistics, store,
    structs::{self, Residuals, State, StorageData},
};

use std::fs;

use itertools::izip;
use rayon::{self, prelude::*};

pub fn run(data: State) {
    let kf_packaged_data = statistics::collect_stats(&data);

    let mut residuals: Vec<Residuals> = statistics::fetch_kf_residuals(&kf_packaged_data);

    println! {"finished KF operations for {}", &data.histogram_name}

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
    path! {save_folder;
        "smth.csv" => smth_path,
        "filt.csv" => filt_path,
        "pred.csv" => pred_path
    }

    store::write_csv(&smth_path, smth);
    store::write_csv(&filt_path, filt);
    store::write_csv(&pred_path, pred);

    store::write_json(&data);

    println! {"finished {}", &data.histogram_name}
}

type nest_vec = Vec<Vec<Vec2>>;
pub fn residuals_by_sensor(
    vec_res: Vec<Residuals>,
    num_sensors: usize,
) -> (nest_vec, nest_vec, nest_vec) {
    let mut sensor_predictions = (0..num_sensors)
        .into_iter()
        .map(|_| Vec::with_capacity(num_sensors))
        .collect::<Vec<_>>();

    let mut sensor_filters = sensor_predictions.clone();
    let mut sensor_smoothes = sensor_predictions.clone();

    for i in 0..num_sensors {
        let mut inn_pred = sensor_predictions
            .get_mut(i)
            .expect("statistics out of bounds");
        let mut inn_filt = sensor_filters.get_mut(i).expect("asd");
        let mut inn_smth = sensor_smoothes.get_mut(i).expect("sdf");

        vec_res.iter().for_each(|x| {
            inn_pred.push(x.pred[i]);
            inn_filt.push(x.filt[i]);
            inn_smth.push(x.smth[i]);
        });
    }

    (sensor_predictions, sensor_filters, sensor_smoothes)
}

/// linear kalman filter exporting data based on individual sensors
pub fn fetch_separated_kf_data(data: &State) {
    let kf_packaged_data = statistics::collect_stats(&data);

    let vec_residuals = statistics::truth_kf_output_residuals(kf_packaged_data);

    let (sensor_predictions, sensor_filters, sensor_smoothes) =
        residuals_by_sensor(vec_residuals, data.num_sensors as usize);

    let counts = 0..data.num_sensors;
    let zipped_data = izip! {sensor_predictions, sensor_filters, sensor_smoothes, counts };

    // serialize into vector of structs to serialize

    zipped_data
        .into_iter()
        .map(|(pred, filt, smth, count)| {
            // converts Vec<Vec2> to Vec<StorageData> to use in serializing to csv
            let to_storage = |x: Vec<Vec2>| {
                x.into_iter()
                    .map(|res| StorageData::from_vec2(res))
                    .collect::<Vec<_>>()
            };

            let p_ = to_storage(pred);
            let f_ = to_storage(filt);
            let s_ = to_storage(smth);

            (p_, f_, s_, count)
        })
        .for_each(move |(pred, filt, smth, count)| {
            let make_path_and_write = |storage_data, subfolder_name, count| {
                // make the subdirectoy
                let folder_path =
                    data.save_folder.clone().to_string() + &format! {r"\{}\",subfolder_name};
                // path to the actual csv we are going to write
                std::fs::create_dir(&folder_path);
                let path = folder_path + &format! {r"sensor_{}.csv", count};
                //write the csv
                store::write_csv(&path, storage_data);
            };

            make_path_and_write(pred, stringify! {pred}, count);
            make_path_and_write(filt, stringify! {filt}, count);
            make_path_and_write(smth, stringify! {smth}, count);
        });
}

/// smear / predicted / covariance diagonal elements for the KF
/// This is to be used w/ ridder's algo
pub fn sensor_separated_with_truth(data: &State) -> () {
    let mut kf_packaged_data = statistics::collect_stats(&data);

    let len = kf_packaged_data.len();
    let mut truth_vec = Vec::with_capacity(len);
    let mut kf_pred_vec = Vec::with_capacity(len);
    let mut covariance = Vec::with_capacity(len);

    let sep_data = kf_packaged_data
        .into_iter()
        .map(|(mut kf_data, mut super_data)| {
            let mut kf_smear = structs::SerStateVec::new(kf_data.smear_initial_vector);
            let mut pred_ser = structs::SerStateVec::new(super_data.pred.state_vec.remove(1));

            let cov_mat = super_data.pred.cov_mat.remove(1);

            get_unchecked! {cov_mat[(0,0)] => a, cov_mat[(1,1)] => b}

            let cov = StorageData::new(*a, *b);

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

/// create a pull distribution of normalized data based of the intial track parameters, the prediction
/// at the first sensor, and normlaized by the corresponding element in the diagonal of the covariance
/// matrix
pub fn pull_distribution(data: &State) {
    let kf_packaged_data = statistics::collect_stats(&data);

    let len = kf_packaged_data.len();
    let mut predictions = Vec::with_capacity(len);
    let mut filters = Vec::with_capacity(len);
    let mut smoothes = Vec::with_capacity(len);

    kf_packaged_data
        .into_iter()
        .for_each(|(kf_data, mut super_data)| {
            // get the initial track parameters
            let init = &kf_data.smear_initial_vector;
            let _hit = &kf_data.truth_hits[0];
            let hit = Vec5::new(_hit.x, _hit.y, 0., 0., 0.);

            // use closure here to remove repetitive code.
            // fetches the covariance and state vector for the type of data we want (filt, smth, pred)
            // and calculates the pull distribution of it based on the diagonals of the covariance
            let normalize = |data: &mut Data, res_vec: &mut Vec<structs::SerStateVec>, i| {
                // let i = 0;
                let state = data.state_vec.remove(i);
                let cov = data.cov_mat.remove(i);

                // difference between initial track parameters and the current data we are handling
                // let mut diff = init - state;
                let mut diff = hit - state;

                // For every value in the diagonal
                for i in 0..5 {
                    // fetch the current diagonal element
                    get_unchecked! {cov[(i,i)] => curr_diagonal}

                    // divide the difference by the diagonal of the covariance
                    edit_matrix! {diff;
                        [i] /= curr_diagonal
                    }
                }

                res_vec.push(structs::SerStateVec::new(diff));
            };

            normalize(&mut super_data.pred, &mut predictions, 0);
            normalize(&mut super_data.filt, &mut filters, 0);
            normalize(&mut super_data.smth, &mut smoothes, 0);
        });

    // concat a sub path onto the save folder and write the vector of serializable structs to the path
    let write_from_string = |save_string, ser_data| {
        let path = data.save_folder.to_string() + save_string;
        store::write_csv(&path, ser_data);
    };

    write_from_string("predicted.csv", predictions);
    write_from_string("filtered.csv", filters);
    write_from_string("smoothed.csv", smoothes);
}

fn residual_to_vec(storage: &mut Vec<StorageData>, res: &Vec<Vec2>) -> () {
    res.iter()
        .for_each(|vec_res| storage.push(StorageData::new(vec_res.x, vec_res.y)));
}

/// Calls all child functions for calculating the residuals for truth vs smeared
/// points
pub fn fetch_kf_randomness_residuals(data: &State) {
    let kf_packaged_data = statistics::collect_stats(data);

    let kf_data: Vec<StorageData> = kf_packaged_data
        .iter()
        .map(|(x, _)| statistics::smear_residuals(&x))
        .flatten()
        .map(|x| StorageData::new(x.x, x.y))
        .collect::<Vec<_>>();

    /*

        configure folders and save destinations

    */
    let mut save_folder = data.save_folder.to_string();
    save_folder.push_str(r"\");

    fs::create_dir(&save_folder);

    // create extensions on the folder path for each csv
    path! {save_folder;
        "truth_smear_residuals.csv" => smth_path
    }

    /*
        write data to files
    */

    store::write_csv(&smth_path, kf_data);

    store::write_json(&data);
}

// residuals between truth vs smeared values
fn test_generated_residuals() -> () {
    let state = State::default(
        "generated_truth_smear_residuals",
        "_truth_smear_residuals.png",
    );
    fetch_kf_randomness_residuals(&state);
}

// residuals between truth and sensor (pred/  filt/ smth) at each sensor
fn test_initial_predictions() -> () {
    print! {"here"}
    let state = State::default(
        r".\data\initial_prediction_data\",
        "this_does_not_matter.png",
    );
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
    state.angles = (0., PI / 2.);
    sensor_separated_with_truth(&state);
}

pub fn run_all_stats() {
    // test_generated_residuals();
    // test_initial_predictions();

    // ridder_algo();

    // run_one_test();
}
