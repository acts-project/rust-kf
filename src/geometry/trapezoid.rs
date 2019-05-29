extern crate nalgebra as na;
use na::{Point3, Matrix4 as Matrix, Vector3 as Vector};
use super::traits::{self, Transform, Plane};
use super::utils;

use super::super::config::*;

/*


    NOTE: this file is incomplete since I shifted the implementation to
    bounds checking based on half lengths + angles. 

    Trapezoidal bounds checking with the complex angles has not been implemented 
    yet. 


*/



/// A struct for sensors of trapezoidal geometry
pub struct Trapezoid{
    top_base_half : Real,
    bottom_base_half : Real,
    alpha : Real,
    normal: Vec3, // the normal vector is not initially calculated
    global_center: P3,
    to_global: Aff3,
    to_local : Aff3,
}

impl Trapezoid{
    /// This is the constructor for the rectangular geometry. It expects a 4x4 `nalgebra::Matrix4<f64>` that is invertible 
    /// and a 4 element array of `nalgebra::Point3<f64>`. If the matrix is not invertible it will return `Err(&str)`.
    /// The provided matrix should be an affine transformation for converting from R2->R3
    /// 
    /// # Examples
    /// ```
    /// use nalgebra as na;
    /// use na::Point3;
    /// let trapezoid_points = [Point3::new(0.0, 0.0, 0.0), 
    ///                         Point3::new(5.0,1.0,0.0), 
    ///                         Point3::new(5.0, 9.0,0.0), 
    ///                         Point3::new(0.0,10.0,0.0)];
    /// let tfm_matrix : na::Matrix4<f64>= na::Matrix4::new(1.0,5.0,7.0,2.0,  3.0,5.0,7.0,4.0,  8.0,4.0,1.0,9.0, 2.0,6.0,4.0,8.0);
    /// let mut trap_sensor = kalman_rs::Trapezoid::new(trapezoid_points, tfm_matrix).unwrap();
    /// ```
    pub fn new(base_top: Real, base_bot: Real, alpha: Real, to_global_tfm_matrix: Mat4) -> Result<Trapezoid, &'static str>{
        
        let to_global_transform = Aff3::from_matrix_unchecked(to_global_tfm_matrix);

        match to_global_transform.try_inverse(){
            
            //TODO: find noraml vector calculation of the trapezoid

            Some(to_local_transform) => {

                // let half_b1 = base_top/(2 as Real);
                // let half_b2 = base_bot/(2 as Real);

                // let orig = P3::new(0.0, 0.0, 0.0);

                // // TODO : recalculate a normal vector here
                // let v1 = orig - P3::new(half_base, 0.0, 0.0);
                // let v2 = orig -  P3::new(0.0, half_height, 0.0);
                // let normal_vector = v1.cross(&v2);


                // let rect = Rectangle{half_base: half_base, 
                //              half_height: half_height,
                //              normal: normal_vector,
                //              global_center : P3::new(0.0, 0.0, 0.0),
                //              to_global: to_global_transform,
                //              to_local: to_local_transform};
                             
                // dbg!{&rect};
                // Ok(rect)

                unimplemented!()
                },
            None => return Err("matrix was not invertable")

        }
    }
}


impl Transform for Trapezoid{
    /// Converts a point in the global reference frame to a point in the local reference frame of the sensor.
    /// 
    /// # Examples
    /// ```
    /// use nalgebra as na;
    /// use na::Point3;
    /// use kalman_rs::sensor_traits::Transform;
    /// let trapezoid_points = [Point3::new(0.0, 0.0, 0.0), 
    ///                         Point3::new(5.0,1.0,0.0), 
    ///                         Point3::new(5.0, 9.0,0.0), 
    ///                         Point3::new(0.0,10.0,0.0)];
    /// let tfm_matrix : na::Matrix4<f64>= na::Matrix4::new(1.0,5.0,7.0,2.0,  3.0,5.0,7.0,4.0,  8.0,4.0,1.0,9.0, 2.0,6.0,4.0,8.0);
    /// let mut trap_sensor = kalman_rs::Trapezoid::new(trapezoid_points, tfm_matrix).unwrap();
    /// 
    /// let global_point = trap_sensor.to_global(na::Point3::new(1.0, 2.0, 0.0));
    /// ```
    fn to_global(&self, input_point: P3)-> P3{
        self.to_global * input_point
    }
    
    /// Converts a point in the local refernce frame of the sensor to the global reference frame.
    /// 
    /// # Examples
    /// 
    /// ```
    /// use nalgebra as na;
    /// use na::Point3;
    /// use kalman_rs::sensor_traits::Transform;
    /// 
    /// let trapezoid_points = [Point3::new(0.0, 0.0, 0.0), 
    ///                         Point3::new(5.0,1.0,0.0), 
    ///                         Point3::new(5.0, 9.0,0.0), 
    ///                         Point3::new(0.0,10.0,0.0)];
    /// let tfm_matrix : na::Matrix4<f64>= na::Matrix4::new(1.0,5.0,7.0,2.0,  3.0,5.0,7.0,4.0,  8.0,4.0,1.0,9.0, 2.0,6.0,4.0,8.0);
    /// let mut trap_sensor = kalman_rs::Trapezoid::new(trapezoid_points, tfm_matrix).unwrap();
    /// 
    /// let local_point = trap_sensor.to_local(na::Point3::new(4.0, 5.0, 6.0));
    /// ```
    fn to_local(&self, input_point: P3) -> P2{
        self.to_local * input_point
    }


    /// Checks if a local point is contained within the bounds of a sensor.
    /// NOTE: `plane()` must be called before checking for bounds of the sensor since the normal 
    /// vector must be calculated first. 
    /// # Examples
    /// ```
    /// use nalgebra as na;
    /// use na::Point3;
    /// use kalman_rs::sensor_traits::Transform;
    /// 
    /// let trapezoid_points = [Point3::new(0.0, 0.0, 0.0), 
    ///                         Point3::new(5.0,1.0,0.0), 
    ///                         Point3::new(5.0, 9.0,0.0), 
    ///                         Point3::new(0.0,10.0,0.0)];
    /// let tfm_matrix : na::Matrix4<f64>= na::Matrix4::new(1.0,5.0,7.0,2.0,  3.0,5.0,7.0,4.0,  8.0,4.0,1.0,9.0, 2.0,6.0,4.0,8.0);
    /// let mut trap_sensor = kalman_rs::Trapezoid::new(trapezoid_points, tfm_matrix).unwrap();
    /// 
    /// let is_point_on_sensor = trap_sensor.contains_from_local(&na::Point2::new(1.0, 6.0));
    /// ```
    fn contains_from_local(&self, input: &P2) -> bool {
        unimplemented!()
    }
}

impl Plane for Trapezoid{


    /// Check if a given point is located on the same plane as the sensor
    /// NOTE: `plane()` must be called becuase the normal vector is not currently known
    /// # Examples
    /// ```
    /// use nalgebra as na;
    /// use na::Point3;
    /// use kalman_rs::sensor_traits::Plane;
    /// 
    /// let trapezoid_points = [Point3::new(0.0, 0.0, 0.0), 
    ///                         Point3::new(5.0,1.0,0.0), 
    ///                         Point3::new(5.0, 9.0,0.0), 
    ///                         Point3::new(0.0,10.0,0.0)];
    /// let tfm_matrix : na::Matrix4<f64>= na::Matrix4::new(1.0,5.0,7.0,2.0,  3.0,5.0,7.0,4.0,  8.0,4.0,1.0,9.0, 2.0,6.0,4.0,8.0);
    /// let mut trap_sensor =kalman_rs::Trapezoid::new(trapezoid_points, tfm_matrix).unwrap();
    /// 
    /// let on_sensor_plane = trap_sensor.on_plane(&Point3::new(1.0, 1.0, 0.0)); //true
    /// ```
    fn on_plane(&self, input_point: &P3) -> Result<bool, &'static str> {
        let pv = utils::vector3_from_points(&self.global_center, &input_point);
        //TODO : this function should probably not return result
        if self.normal.dot(&pv) == 0.0 {
            Ok(true)
        }
        else{
            Ok(false)
        }
        
    }
}
