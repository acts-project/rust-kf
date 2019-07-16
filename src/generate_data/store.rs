use super::super::config::*;
use std::io::prelude;
use std::fs;

use csv::Writer;
use serde::Serialize;
use serde_json;


use super::run::State;


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

pub fn write_csv(name: &str, data: Vec<StorageData>) -> () {
    // let file = fs::File::create(name).expect("file could not be created");
    let mut wtr = Writer::from_path(name).expect("file could not be used");

    for i in data{
        wtr.serialize(i).expect("could not serialize");
    }

    // wtr.serialize(data).expect("could not serialize");

    wtr.flush().expect("CSV could not be flushed");


}

pub fn write_json(json_data: &State) -> () {
    let mut save_base = json_data.save_folder.clone();
    save_base.push_str(r"\info.json");

    dbg!{&save_base};

    let file = fs::File::create(save_base).expect("json file could not be made");

    serde_json::to_writer(file, json_data).expect("json could not be serialized to the file");
}