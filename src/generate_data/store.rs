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


    let file = fs::File::create(save_base).expect("json file could not be made");

    serde_json::to_writer(file, json_data).expect("json could not be serialized to the file");
}



pub fn write_random_values(x: &Vec<Vec2>, i: u32) {
    
    let mut w = 
        if i == 0{
            Writer::from_path(r".\data\smth.csv").unwrap()
        }
        else if i == 1{
            Writer::from_path(r".\data\filt.csv").unwrap()
        }
        else if i == 2{
            Writer::from_path(r".\data\pred.csv").unwrap()
        }
        else{
            panic!{"asdasd"}
        };
    
    x.into_iter()
        .for_each(|x|{
            w.serialize(StorageData::new(x.x, x.y)).expect("coult not seriailze")
        });

    w.flush().expect("coult not flush");
}