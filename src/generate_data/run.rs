use super::{statistics, store};
use statistics::Residuals;
use store::StorageData;


use super::super::config::*;
use std::iter;

/// Quickly create a repetitive value and collect it to a variable
/// $take: number of times to repeat the value
/// $name: name of variable to save to
/// $value: the value that will be repeated $take times
macro_rules! collect {
    ($take:expr; $($name:ident : $value:expr),+) => {
        $(
            let $name = iter::repeat($value).take($take).collect::<Vec<_>>();
        )+
    };
}


macro_rules! with_capacity {
    ($cap:expr; $($name:ident),+) => {
        $(
            let mut  $name = Vec::with_capacity($cap)
        )+
    };
}
pub fn run() {
    let iterations = 100;
    let sens_len = 30;

    collect!{
        iterations;
        num_sensors : sens_len,
        sensor_distances :0.01,
        angles : None,
        point_stdev: 0.001
    }

    let kf_packaged_data = 
        statistics::collect_stats(num_sensors, sensor_distances, angles, point_stdev);

    let mut residuals : Vec<Residuals> = 
        statistics::fetch_kf_residuals(&kf_packaged_data);

    let len = sens_len as usize;

    with_capacity!{len*iterations; smth, filt, pred}

    // let k: u32 = residuals.remove(0);

    for _ in 0..iterations {
        let residual_struct = residuals.remove(0);

        residual_to_vec(&mut smth, &residual_struct.smth);
        residual_to_vec(&mut filt, &residual_struct.filt);
        residual_to_vec(&mut pred, &residual_struct.pred);
    }
    

    store::write_csv(r".\smth.csv", smth);
    store::write_csv(r".\filt.csv", filt);
    store::write_csv(r".\pred.csv", pred)

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