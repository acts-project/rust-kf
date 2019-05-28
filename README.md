# kalman_rs
Work pertaining to KF implementation for GSoC 2019

## Usage
Currently only basic functionality of sensors is implemented
* Converting from global/local to local/global frame
* Checking if a local/global point is within a sensor's bounds
* Basic sensor structs for rectangular and trapezoidal geometry

## Building
clone the repository

`git clone https://VanillaBrooks/kalman_rs`

build with rustc:

`cargo build`

unit tests can be run with the command

`cargo test`

documentation can be run with

`cargo doc --open`
