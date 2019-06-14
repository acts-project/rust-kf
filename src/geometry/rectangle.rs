extern crate nalgebra as na;
// use na::{Point3, Matrix4 as Matrix, Vector3 as Vector};
use super::traits::{self, Transform, Plane};
use super::utils;
use super::super::config::*;

/// A struct for sensors of rectangular geometry
#[derive(Debug)]
pub struct Rectangle {
    half_base: Real,
    half_height: Real,
    normal : Vec2,
    to_global: Aff3,
    to_local: Aff3,
    measurement_to_state_vector: na::Matrix5x2<Real>
}

impl Rectangle {
    /// This is the constructor for the rectangular geometry. It expects a 4x4 `nalgebra::Matrix4<f32>` that is invertible 
    /// and a 4 element array of `nalgebra::Point3<f32>`. If the matrix is not invertible it will return `Err(&str)`.
    /// The provided matrix should be an affine transformation for converting from R2->R3
    /// 
    ///  # Examples
    /// ```
    /// use nalgebra as na;
    /// use na::Point3;
    /// use kalman_rs::geometry::rectangle::Rectangle;
    /// 
    /// let base = 3.0;
    /// let height = 3.0;
    /// let tfm_matrix : na::Matrix4<f64>= na::Matrix4::new(1.0,0.0,0.0,0.0,  0.0,1.0,0.0,0.0,  0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0);
    /// let mut rectangle_sensor = Rectangle::new(base, height, tfm_matrix).unwrap();
    /// ```
    pub fn new(base: Real, height: Real, to_global_tfm_matrix: Mat4, projection_mat: na::Matrix5x2<Real>) -> Result<Rectangle, &'static str>{
    
        let to_global_transform = Aff3::from_matrix_unchecked(to_global_tfm_matrix);

        match to_global_transform.try_inverse(){
            
            Some(to_local_transform) => {

                let half_base = base/(2 as Real);
                let half_height = height/(2 as Real);

                let orig = P2::new(0.0, 0.0);
                let v1 = orig - P2::new(half_base, 0.0);
                let v2 = orig -  P2::new(0.0, half_height);
                let normal_vector = v1.cross(&v2);


                let rect = Rectangle{half_base: half_base, 
                             half_height: half_height,
                             normal: normal_vector,
                             to_global: to_global_transform,
                             to_local: to_local_transform,
                             measurement_to_state_vector: projection_mat};
                             
                dbg!{&rect};
                Ok(rect)
                },
            None => return Err("matrix was not invertable")

        }
    }

}
impl Transform for Rectangle{
    /// Converts a point in the global reference frame to a point in the local reference frame of the sensor.
    /// 
    /// # Examples
    /// ```
    /// use nalgebra as na;
    /// use na::Point3;
    /// use kalman_rs::geometry::rectangle::Rectangle;
    /// use kalman_rs::sensor_traits::Transform;
    /// 
    /// let base = 3.0;
    /// let height = 3.0;
    /// let tfm_matrix : na::Matrix4<f64>= na::Matrix4::new(1.0,0.0,0.0,0.0,  0.0,1.0,0.0,0.0,  0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0);
    /// let mut rectangle_sensor = Rectangle::new(base, height, tfm_matrix).unwrap();
    /// 
    /// let global_point = rectangle_sensor.to_global(na::Point3::new(1.0, 2.0, 0.0));
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
    /// use kalman_rs::geometry::rectangle::Rectangle;
    /// use kalman_rs::sensor_traits::Transform;
    /// 
    /// let base = 3.0;
    /// let height = 3.0;
    /// let tfm_matrix : na::Matrix4<f64>= na::Matrix4::new(1.0,0.0,0.0,0.0,  0.0,1.0,0.0,0.0,  0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0);
    /// let mut rectangle_sensor = Rectangle::new(base, height, tfm_matrix).unwrap();
    /// 
    /// let global_point = rectangle_sensor.to_local(na::Point3::new(6.0, 3.0, 5.0));
    /// ```
    fn to_local(&self, input_point: P3) -> P2{
        let local = self.to_local * input_point;
        return P2::new(local.x, local.y)
    }


    /// Checks if a local point is contained within the bounds of a sensor.
    /// NOTE: `plane()` must be called before checking for bounds of the sensor since the normal 
    /// vector must be calculated first. 
    /// 
    /// # Examples
    /// ```
    /// use nalgebra as na;
    /// use na::Point3;
    /// use kalman_rs::sensor_traits::Transform;
    /// use kalman_rs::geometry::rectangle::Rectangle;
    /// 
    /// let base = 3.0;
    /// let height = 3.0;
    /// let tfm_matrix : na::Matrix4<f64>= na::Matrix4::new(1.0,0.0,0.0,0.0,  0.0,1.0,0.0,0.0,  0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0);
    /// let mut rectangle_sensor = Rectangle::new(base, height, tfm_matrix).unwrap();
    /// 
    /// let is_point_on_sensor = rectangle_sensor.contains_from_local(&na::Point2::new(1.0, 6.0));
    /// ```
    fn contains_from_local(&self, input: &P2) -> bool {
        
        if (input.x < self.half_base) && (input.y < self.half_height) {
            true
        }
        else {
            false
        }
    }
}


impl Plane for Rectangle{

    /// Check if a given point is located on the same plane as the sensor
    /// NOTE: `plane()` must be called becuase the normal vector is not currently known
    /// # Examples
    /// 
    /// ```
    /// use nalgebra as na;
    /// use na::Point3;
    /// use kalman_rs::sensor_traits::Plane;
    /// use kalman_rs::geometry::rectangle::Rectangle;
    /// 
    /// let base = 3.0;
    /// let height = 3.0;
    /// let tfm_matrix : na::Matrix4<f64>= na::Matrix4::new(1.0,0.0,0.0,0.0,  0.0,1.0,0.0,0.0,  0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0);
    /// let mut rectangle_sensor = Rectangle::new(base, height, tfm_matrix).unwrap();
    /// 
    /// let on_plane = rectangle_sensor.on_plane(&na::Point3::new(1.0, 3.0, 0.0)); //true
    /// ```
    fn on_plane(&self, input_point: &P2) -> Result<bool, &'static str> {
        let pv = P2::new(0.0, 0.0) - input_point;
        //TODO : this function should probably not return result
        if self.normal.dot(&pv) == 0.0 {
            Ok(true)
        }
        else{
            Ok(false)
        }
    }
}
