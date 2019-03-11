extern crate nalgebra;
mod lib;


#[allow(dead_code)]

// takes in a global location and outputs the position relative to local scope
fn transform_to_local(location: lib::Point) -> () {
    

}

// takes in a local location and outputs the position to a global scope
fn transform_to_global() -> () {


}

// checks if point is within sensor bounds
fn check_if_on_sensor <T>(sensor_struct: T, sensor_type: lib::SensorType) -> () {

    match sensor_type {
        lib::SensorType::Rectangle => {
            
        },// rectangle
        lib::SensorType::Trapezoid => {

        }// trapezoid
    }
}

fn main() {
    println!("Hello, world!");
}


