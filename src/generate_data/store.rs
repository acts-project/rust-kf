use super::super::config::*;
use std::io::prelude;
use std::fs;

use csv::Writer;
use serde::Serialize;

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