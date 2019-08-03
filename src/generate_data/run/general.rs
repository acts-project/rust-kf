use super::super::super::{
    config::*,
    filter::utils::{self, Data},
};
use super::super::{
    statistics, store,
    structs::{self, Residuals, State, StorageData},
    traits::{NestedGroupedData, NestedVectorData, SensorHelper},
};

use std::fs;

use itertools::izip;
use rayon::{self, prelude::*};

pub fn run(data: State) {
    let kf_packaged_data = statistics::collect_stats(&data);

    let mut residuals_vector: Vec<(Vec<Vec2>, Vec<Vec2>, Vec<Vec2>)> =
        statistics::fetch_kf_residuals_all(&kf_packaged_data, false);

    println! {"finished KF operations for {}", &data.histogram_name}

    let len = (data.num_sensors as usize) * data.iterations;

    let mut smth = Vec::with_capacity(len);
    let mut filt = Vec::with_capacity(len);
    let mut pred = Vec::with_capacity(len);

    residuals_vector
        .iter()
        .for_each(|(pred_res, filt_res, smth_res)| {
            residual_to_vec(&mut smth, pred_res);
            residual_to_vec(&mut filt, filt_res);
            residual_to_vec(&mut pred, smth_res);
        });

    let mut save_folder = data.save_folder.clone().to_string();
    save_folder.push_str(r"\");

    #[allow(unused_must_use)]
    fs::create_dir(&save_folder);

    // create extensions on the folder path for each csv
    path! {save_folder;
        "smth.csv" => smth_path,
        "filt.csv" => filt_path,
        "pred.csv" => pred_path
    }

    store::write_csv(&data.make_file_path("pred.csv"), pred);
    store::write_csv(&data.make_file_path("filt.csv"), filt);
    store::write_csv(&data.make_file_path("smth.csv"), smth);

    store::write_json(&data);

    println! {"finished {}", &data.histogram_name}
}

type NestVec = Vec<Vec<Vec2>>;
pub fn residuals_by_sensor(
    vec_res: Vec<Residuals>,
    num_sensors: usize,
) -> (NestVec, NestVec, NestVec) {
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

    let vec_residuals = statistics::fetch_kf_residuals_all(&kf_packaged_data, false);

    let (mut pred, mut filt, mut smth) = vec_residuals.by_sensor();

    // make subfolders that store the data
    // since sensor_0.csv for predictions will overwrite
    // sensor_0.csv for filters. Makes the analysis code simpler
    data.make_subfolder("pred");
    data.make_subfolder("filt");
    data.make_subfolder("smth");

    // removes the first entry of teh vector (Vector of things to serialize) and then writes them to the path
    let helper =
        |vec_to_remove_from: &mut Vec<Vec<StorageData>>, subfolder_name, file_name: &String| {
            let ser_data = vec_to_remove_from.remove(0);
            let path = data.make_subfolder_file_path(subfolder_name, &file_name);

            store::write_csv(&path, ser_data);
        };

    for i in 0..data.num_sensors {
        let name = format! {"sensor_{}.csv", i};

        helper(&mut pred, "pred", &name);
        helper(&mut filt, "filt", &name);
        helper(&mut smth, "smth", &name);
    }
}

// / smear / predicted / covariance diagonal elements for the KF
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
type NestStorage = Vec<Vec<StorageData>>;
pub fn pull_distribution_general(data: &State) -> (NestStorage, NestStorage, NestStorage) {
    let kf_packaged_data = statistics::collect_stats(&data);

    // TODO: do action pull calculation down stream of `fetch_kf_residuals_all`
    let vec_pred_filt_smths: Vec<(Vec<Vec2>, Vec<Vec2>, Vec<Vec2>)> =
        statistics::fetch_kf_residuals_all(&kf_packaged_data, true);

    // traits::NestedGroupedData, converts Vec< predicted_residuals,
    // filter_residuals, smooth_residuals> into three
    // Vec<Vec<structs::StorageData>>
    vec_pred_filt_smths.by_sensor()

    // return (pred_vec, filt_vec, smth_vec);
}

pub fn pull_distribution(data: &State, first_only: bool) {
    let (mut pred, mut filt, mut smth) = pull_distribution_general(&data);

    let make_path_and_write = |storage_data: &mut Vec<Vec<StorageData>>, subfolder_name, count| {
        let curr_sotrage = storage_data.remove(0);
        // make the folder we are writing to
        data.make_subfolder(subfolder_name);

        let path =
            data.make_subfolder_file_path(subfolder_name, &format! {r"sensor_{}.csv", count});
        //write the csv
        store::write_csv(&path, curr_sotrage);
    };

    for i in 0..pred.len() {
        make_path_and_write(&mut pred, "predicted", i);
        make_path_and_write(&mut filt, "filtered", i);
        make_path_and_write(&mut smth, "smoothed", i);

        // if we only want the first sensor
        if i == 0 && first_only == true {
            break;
        }
    }
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

    #[allow(unused_must_use)]
    fs::create_dir(&save_folder);

    // create extensions on the folder path for each csv
    let smth_path = data.make_file_path("truth_smear_residuals.csv");

    /*
        write data to files
    */

    store::write_csv(&smth_path, kf_data);

    store::write_json(&data);
}
