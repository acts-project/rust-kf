use super::structs::State;

use std::fs;

use csv::Writer;
use serde_json;
use serde::Serialize;

/// Write a vector of serializeable structs to a given file path
pub fn write_csv<T:Serialize>(name: &str, data: Vec<T>) -> () {

    let mut wtr = Writer::from_path(name).expect("file could not be used");

    for i in data{
        wtr.serialize(i).expect("could not serialize");
    }

    wtr.flush().expect("CSV could not be flushed");

}


/// Serializes a `State` (which contains information on how tracks should be generated)
/// to a json file and places it into a folder. This, in conjunction with `store::write_csv()`
/// places all csv data from a batch of KF operatoins with the metadata .json file that was used
/// to create them. 
/// 
/// This is done so that the data can quickly be plotted with Python & matplotlib
pub fn write_json(json_data: &State) -> () {
    let mut save_base = json_data.save_folder.clone().to_string();
    save_base.push_str(r"\info.json");

    print!{save_base}

    let file = fs::File::create(save_base).expect("json file could not be made");

    serde_json::to_writer(file, json_data).expect("json could not be serialized to the file");
}
