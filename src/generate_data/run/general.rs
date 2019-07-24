
use super::super::{
    statistics, 
    store,
    structs::{StorageData, State, Residuals}
    };

use itertools::izip;

use std::fs;

use super::super::super::config::*;

use rayon::{self, prelude::*};


pub fn fetch_separated_kf_data(data: &State) {
    let kf_packaged_data = statistics::collect_stats(&data);

    let vec_residuals = 
        statistics::truth_kf_output_residuals(kf_packaged_data);

    
    let mut sensor_predictions = 
        (0..(data.num_sensors as usize)).into_iter()
        .map(|_| Vec::new())
        .collect::<Vec<_>>();

    let mut sensor_filters = sensor_predictions.clone();
    let mut sensor_smoothes = sensor_predictions.clone();

    for i in 0..(data.num_sensors as usize) {
        let mut inn_pred = sensor_predictions.get_mut(i).expect("statistics out of bounds");
        let mut inn_filt = sensor_filters.get_mut(i).expect("asd");
        let mut inn_smth = sensor_smoothes.get_mut(i).expect("sdf");

        vec_residuals.iter()
            .for_each(|x|{
                inn_pred.push(x.pred[i]);               // this line changes what field we are looking at
                inn_filt.push(x.filt[i]);
                inn_smth.push(x.smth[i]);
            });
    }
    

    let counts = 0..data.num_sensors;
    let zipped_data = izip!{sensor_predictions, sensor_filters, sensor_smoothes, counts };

    // serialize into vector of structs to serialize

    zipped_data.into_iter()
        .map(|(pred, filt, smth, count)|{
            // converts Vec<Vec2> to Vec<StorageData> to use in serializing to csv
            let to_storage = |x: Vec<Vec2>| x.into_iter().map(|res| StorageData::new(res.x, res.y)).collect::<Vec<_>>();
            
            let p_ = to_storage(pred);
            let f_ = to_storage(filt);
            let s_ = to_storage(smth);
        
            (p_, f_, s_, count)
        
        })
        .for_each(move|(pred, filt, smth, count)|{

            let make_path_and_write = |storage_data, subfolder_name, count| {
                // make the subdirectoy
                let folder_path = data.save_folder.clone() + &format!{r"\{}\",subfolder_name};
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


fn residual_to_vec(
    storage: &mut Vec<StorageData>,
    res: &Vec<Vec2>
    ) -> () {
    
    res.iter()
        .for_each(|vec_res|{
            storage.push(StorageData::new(vec_res.x, vec_res.y))
        });
    
}


/// Calls all child functions for calculating the residuals for truth vs smeared
/// points
pub fn fetch_kf_randomness_residuals(data: &State) {
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


pub fn run(data: State) {
    
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