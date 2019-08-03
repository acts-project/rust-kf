// pub trait SensorVector<T> {
//     fn per_sensor(&self, sensor_count: usize, capacity: usize) -> Vec<Vec<T>>;
// }

// impl<T> SensorVector<T> for Vec<Vec<T>> {
//     fn per_sensor(&self, sensor_count: usize, capacity: usize) -> Vec<Vec<T>> {
//         let mut all: Vec<Vec<T>> = Vec::with_capacity(sensor_count);

//         (0..sensor_count)
//             .into_iter()
//             .map(|x| Vec::with_capacity(capacity))
//             .collect::<Vec<Vec<T>>>()
//     }
// }
