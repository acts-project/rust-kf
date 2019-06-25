use nalgebra as na;
use super::traits::{Transform, Plane};
use super::super::config::*;
use super::super::error::*;
use super::utils;

/// A struct for sensors of rectangular geometry
#[derive(Debug)]
pub struct Rectangle {
    pub gloabl_center: P3,  //center of the sensor (not used in bound checks)
    pub normal : Vec3,      // normal vector of plane
    pub plane_constant: Real, // D in Ax +By + Cz +D =0 

    half_base: Real,
    half_height: Real,

    to_global: Aff3,    // L => G for point
    to_local: Aff3,     // G => L for point
    measurement_to_state_vector: Mat2x5
}

impl Rectangle {
    /*
    /// This is the constructor for the rectangular geometry. It expects a 4x4 `nalgebra::Matrix4<f32>` that is invertible 
    /// and a 4 element array of `nalgebra::Point3<f32>`. If the matrix is not invertible it will return `Err(kalman_rs::Error)`.
    /// The provided matrix should be an affine transformation for converting from R2->R3 since we assume the local coordiante system
    /// is on the x-y plane.
    /// 
    ///  # Examples
    /// ```
    /// use nalgebra as na;
    /// use kalman_rs::geometry::Rectangle;
    /// 
    ///let base_len = 3.0;
    ///let height_len = 3.0;
    ///let tfm_matrix= na::Matrix4::<f64>::identity(); //arbitrary transform matrix
    ///let projection = na::Matrix5x2::<f64>::zeros();
    ///Rectangle::new(base_len, height_len, tfm_matrix, projection);
    /// 
    /// 
    /// ```
    /// */
    pub fn new(base: Real, height: Real, to_global_tfm_matrix: Mat4, projection_mat: Mat2x5) -> Result<Rectangle, MatrixError>{
    
        let to_global_transform = Aff3::from_matrix_unchecked(to_global_tfm_matrix);

        match to_global_transform.try_inverse(){
            
            Some(to_local_transform) => {

                let half_base = base/(2.0);
                let half_height = height/(2.0);

                let orig = P3::new(0.0, 0.0, 0.0);
                let p1 = P3::new(half_base, 0.0,0.0);
                let p2 = P3::new(0.0, half_height, 0.0);
                let normal_vector = utils::plane_normal_vector(half_base, half_height);


                let rect = Rectangle{half_base: half_base, 
                             half_height: half_height,
                             normal: normal_vector,
                             plane_constant: 0.,         //TODO fix this one
                             gloabl_center: orig,
                             to_global: to_global_transform,
                             to_local: to_local_transform,
                             measurement_to_state_vector: projection_mat};
                             
                dbg!{&rect};
                Ok(rect)
                },
            None => return Err(MatrixError::NonInvertible)

        }
    }

}
impl Transform for Rectangle{
    /*
    /// Converts a point in the global reference frame to a point in the local reference frame of the sensor.
    /// 
    /// # Examples
    /// ```
    /// use nalgebra as na;
    /// use na::Point3;
    /// use nalgebra as na;
    /// use kalman_rs::geometry::Rectangle;
    /// 
    /// let base_len = 3.0;
    /// let height_len = 3.0;
    /// let tfm_matrix= na::Matrix4::<f64>::identity(); //arbitrary transform matrix
    /// let projection = na::Matrix5x2::<f64>::zeros();
    /// Rectangle::new(base_len, height_len, tfm_matrix, projection).unwrap();
    /// 
    /// let global_point = rectangle_sensor.to_global(na::Point3::new(1.0, 2.0, 0.0));
    /// ```*/
    fn to_global(&self, input_point: P3)-> P3{
        self.to_global * input_point
    }
    
    /*
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
    /// ```*/
    fn to_local(&self, input_point: P3) -> P2{
        let local = self.to_local * input_point;
        return P2::new(local.x, local.y)
    }

    /*
    /// Checks if a local point is contained within the bounds of a sensor.
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
    /// ```*/
    fn inside(&self, input: &P2) -> bool {
        
        if (input.x.abs() < self.half_base.abs()) && (input.y.abs() < self.half_height.abs()) {
            true
        }
        else {
            false
        }
    }
}


impl Plane for Rectangle{

    /*
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
    /// ```*/
    fn on_plane(&self, input_point: &P3) -> bool {
        let pv : Vec3= P3::new(0.0, 0.0, 0.0) - input_point;

        if self.normal.dot(&pv) == DOT_PRODUCT_EPSILON{
            true
        }
        else{
            false
        }
    }

    fn plane_normal_vec(&self) -> &Vec3 {
        return &self.normal
    }
}
