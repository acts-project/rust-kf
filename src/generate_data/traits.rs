//! # Data Wrangling traits
//! These common function are all implemented as traits since it will make generics of data down
//! the road more straight forward.
//!
//! For example, Vec<Vec5> and Vec<Vec2> (state vector and hit vector) will both
//! be handle-able by these traits without too many struggles.
//!
//! Doing this data conversion with functions would be less clear

use super::super::config::*;
use super::structs::StorageData;

/// Initialize a vector of vectors. This is the core data structure for separating data based on what
/// sensor number the current data is being held at
pub trait SensorHelper<T> {
    fn empty_sensors(sensor_count: usize, capacity: usize) -> Vec<Vec<T>>;
}

impl<T> SensorHelper<T> for Vec<Vec<T>> {
    fn empty_sensors(sensor_count: usize, capacity: usize) -> Vec<Vec<T>> {
        let mut all: Vec<Vec<T>> = Vec::with_capacity(sensor_count);

        (0..sensor_count)
            .into_iter()
            .map(|x| Vec::with_capacity(capacity))
            .collect::<Vec<Vec<T>>>()
    }
}

/// Convert a Vector of T into a struct that we can serialize to a .csv
pub trait NestedVectorData<T> {
    fn by_sensor(self) -> Vec<StorageData>;
}

impl<T> NestedVectorData<T> for Vec<T>
where
    T: Into<StorageData>,
{
    fn by_sensor(self) -> Vec<StorageData> {
        self.into_iter().map(|x| x.into()).collect::<Vec<_>>()
    }
}

/// This trait is invoked to handle the repetitive code of converting the
/// (different ways of calculating residuals) that all return Vec< Vec<T>, Vec<T>, Vec<T>>
/// into data that is organized per-sensor and serializable
pub trait NestedGroupedData<T> {
    fn by_sensor(
        self,
    ) -> (
        Vec<Vec<StorageData>>,
        Vec<Vec<StorageData>>,
        Vec<Vec<StorageData>>,
    );
}

impl<T> NestedGroupedData<T> for Vec<(Vec<T>, Vec<T>, Vec<T>)>
where
    T: Into<StorageData>,
{
    fn by_sensor(
        self,
    ) -> (
        Vec<Vec<StorageData>>,
        Vec<Vec<StorageData>>,
        Vec<Vec<StorageData>>,
    ) {
        let sensor_count = self[0].0.len();
        let capacity = self.len();

        let mut pred_vec: Vec<Vec<StorageData>> = Vec::empty_sensors(sensor_count, capacity);
        let mut filt_vec = pred_vec.clone();
        let mut smth_vec = pred_vec.clone();

        self.into_iter().for_each(|(mut pred, mut filt, mut smth)| {
            for i in 0..sensor_count {
                // remove an entry from the residuals (remove_vec), push it to the
                // sensor-separated vector (storage_vec) at the current sensor
                let push_help =
                    |storage_vec: &mut Vec<Vec<StorageData>>, remove_vec: &mut Vec<T>, index| {
                        // vector containing all the residuals at the current sensor
                        let curr_storage: &mut Vec<StorageData> = storage_vec
                            .get_mut(index)
                            .expect("out of bounds for some reason");
                        // value removed from the current residuals we are handling
                        let rmv: T = remove_vec.remove(0);
                        // serialize T to something we can write to a file
                        let storage_ = rmv.into();
                        // move the serializable data to the container for the current sensor
                        curr_storage.push(storage_);
                    };

                push_help(&mut pred_vec, &mut pred, i);
                push_help(&mut filt_vec, &mut filt, i);
                push_help(&mut smth_vec, &mut smth, i);
            }
        });

        (pred_vec, filt_vec, smth_vec)
    }
}
