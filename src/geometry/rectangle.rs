use nalgebra as na;
use super::traits::{Transform, Plane};
use super::super::config::*;
use super::super::error::*;
use super::utils;

#[macro_use]
use super::super::filter;

/// A struct for sensors of rectangular geometry
#[derive(Debug)]
pub struct Rectangle {
    pub center_global: P3,  //center of the sensor (not used in bound checks)
    pub normal : Vec3,      // normal vector of plane
    pub plane_constant: Real, // D in Ax +By + Cz +D =0 

    half_base: Real,
    half_height: Real,

    pub to_global: Aff3,    // L => G for point
    pub to_local: Aff3,     // G => L for point
    
    pub to_global_rot: Mat4,
    pub to_local_rot: Mat4
}

impl Rectangle {
    
    /// This is the constructor for the rectangular geometry. It expects a 4x4 `nalgebra::Matrix4<f32>` that is invertible 
    /// and a 4 element array of `nalgebra::Point3<f32>`. If the matrix is not invertible it will return `Err(kalman_rs::Error)`.
    /// The provided matrix should be an affine transformation for converting from R2->R3 since we assume the local coordiante system
    /// is on the x-y plane.
    /// 
    /// # Examples
    /// ```
    /// use nalgebra as na;
    /// use na::Point3;
    /// use kalman_rs::config::*;
    /// use kalman_rs::geometry::Rectangle;
    /// 
    /// let transform_mat = Mat4::identity();
    /// let base = 5.;
    /// let height = 4.;
    /// 
    /// let sensor = Rectangle::new(base, height, transform_mat, transform_mat);
    /// ```
    pub fn new(
        base: Real, 
        height: Real, 
        to_global_translation: Mat4,
        to_global_rotation: Mat4,
        ) -> Result<Rectangle, MatrixError>{

        
        let compose_transform = to_global_translation * to_global_rotation;

        let to_local_rotation = to_global_rotation.try_inverse().expect("rotation sensor matrix non invertible");
    
        let to_global_transform = Aff3::from_matrix_unchecked(compose_transform);

        match to_global_transform.try_inverse(){
            
            Some(to_local_transform) => {

                let half_base = base/(2.0);
                let half_height = height/(2.0);

                let orig = P3::new(0.0, 0.0, 0.0);

                let normal_vector = utils::plane_normal_vector(half_base, half_height);
                
                
                let center_global = to_global_transform * orig;
                let plane_const = (center_global.x * normal_vector.x + center_global.y * normal_vector.y + center_global.z * normal_vector.z);

                let rect = Rectangle{half_base: half_base, 
                             half_height: half_height,
                             normal: normal_vector,
                             plane_constant: plane_const,
                             center_global: center_global,
                             to_global: to_global_transform,
                             to_local: to_local_transform,
                             to_global_rot: to_global_rotation,
                             to_local_rot: to_local_rotation};
                             
                // dbg!{&rect};
                Ok(rect)
                },
            None => return Err(MatrixError::NonInvertible)

        }
    }

    ///quickly generates arbitrary sensor data
    pub fn default() -> Self {
        let base = 4.;
        let height = 4.;
        let to_global = Mat4::new_random();
        let rot = Mat4::new_random();

        Self::new(base, height, to_global, rot).expect("could not generate rect. sensor")
    }

    pub fn new_test_sensor(
        base: Real,
        height: Real,
        to_global: Aff3,
        to_local: Aff3,
        to_global_rot: Mat4,
        to_local_rot: Mat4,
        non_center_p1: P3,
        non_center_p2: P3
    )-> Self {
        let local_center = P3::origin();
        let global_center = to_global * local_center;

        // dbg!{global_center};

        let v1 = non_center_p1 - global_center;
        let v2 = non_center_p2 - global_center;
        let normal = v1.cross(&v2);

        // dbg!{normal};

        // dbg!{v1}; dbg!{v2}; dbg!{normal};

        let plane_constant = 
            ((normal.x * global_center.x )+ (normal.y *global_center.y) + (normal.z * global_center.z));

        // dbg!{plane_constant};
        Rectangle{
            half_base: base/2.,
            half_height: height / 2.,
            normal: normal ,
            plane_constant: plane_constant,
            center_global: global_center,
            to_global: to_global,
            to_local: to_local,
            to_global_rot: to_global_rot,
            to_local_rot: to_local_rot
        }
    }


}
impl Transform for Rectangle{

    /// Converts a point in the global reference frame to a point in the local reference frame of the sensor.
    /// 
    /// # Examples
    /// ```
    /// use kalman_rs as krs;
    /// use krs::config::*;
    /// use krs::geometry::traits::*;
    /// use krs::geometry::Rectangle;
    /// 
    /// let sensor = Rectangle::default();
    /// let local_point = P3::origin();
    /// let global_point = sensor.to_global(local_point);
    /// ```
    fn to_global(&self, input_point: P3)-> P3{
        self.to_global * input_point
    }
    
    
    /// Converts a point in the local refernce frame of the sensor to the global reference frame.
    /// 
    ///
    /// # Examples
    /// ```
    /// use kalman_rs as krs;
    /// use krs::config::*;
    /// use krs::geometry::traits::*;
    /// use krs::geometry::Rectangle;
    /// 
    /// let sensor = Rectangle::default();
    /// let global_point = P3::origin();
    /// let local_point = sensor.to_global(global_point);
    /// ```
    fn to_local(&self, input_point: P3) -> P2{
        let local = self.to_local * input_point;
        return P2::new(local.x, local.y)
    }


    /// Checks if a local point is contained within the bounds of a sensor.
    /// 
    /// # Examples
    /// ```
    /// use kalman_rs as krs;
    /// use krs::config::*;
    /// use krs::geometry::traits::*;
    /// use krs::geometry::Rectangle;
    /// 
    /// let sensor = Rectangle::default();
    /// let local_point = P2::origin();
    /// let is_inside_bounds: bool = sensor.inside(&local_point);
    /// ```
    fn inside(&self, input: &P2) -> bool {
        
        if (input.x.abs() < self.half_base.abs()) && (input.y.abs() < self.half_height.abs()) {
            true
        }
        else {
            false
        }
    }
    fn rotation_to_global(&self) -> &Mat4{
        &self.to_global_rot
    }
    fn rotation_to_local(&self) -> &Mat4{
        &self.to_local_rot
    }
}


impl Plane for Rectangle{

    
    /// Check if a given point is located on the same plane as the sensor
    /// NOTE: `plane()` must be called becuase the normal vector is not currently known
    /// 
    /// # Examples
    /// ```
    /// use kalman_rs as krs;
    /// use krs::config::*;
    /// use krs::geometry::traits::*;
    /// use krs::geometry::Rectangle;
    /// 
    /// let sensor = Rectangle::default();
    /// let local_point = P3::origin();
    /// let is_on_sensor_plane : bool = sensor.on_plane(&local_point);
    /// ```
    fn on_plane(&self, input_point: &P3) -> bool {
        let pv : Vec3= self.center_global - input_point;
       
        // if self.normal.dot(&pv).abs() <= DOT_PRODUCT_EPSILON{
        if self.normal.dot(&pv).abs() <= 0.01{
        // if self.normal.dot(&pv).abs() <= 0.0000000000001{
            true
        }
        else{
            false
        }
    }

    fn plane_normal_vec(&self) -> &Vec3 {
        return &self.normal
    }

    fn global_center(&self) -> &P3 {
        &self.center_global
    }
    fn plane_constant(&self) -> Real {
        self.plane_constant
    }

}
