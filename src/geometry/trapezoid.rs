extern crate nalgebra as na;
use na::{Point3, Matrix4 as Matrix, Vector3 as Vector};
use super::traits::{self, Transform, Plane};
use super::utils;

use std::cmp::{self, Ordering};

use super::super::config::*;

/*


    NOTE: this file is incomplete since I shifted the implementation to
    bounds checking based on half lengths + angles. 

    Trapezoidal bounds checking with the complex angles has not been implemented 
    yet. 


*/


// Struct to calculate the y value at any given x
// This is used instead of a closure since closures require 
// heap allocation w/ trait objects (dynamic dispatch)
#[derive(Debug)]
struct Line {
    pub yint: Real,
    pub slope: Real
}
impl Line {//
    fn new_from_points(p1: &P2, p2: &P2) -> Self{
        let slope = (p2.y - p1.y)/ (p2.x - p1.x);

        // y = m*x + [-m*x_0 + y_0] <- yint
        let prod = -1 as Real *(p1.x*slope);
        let yint = prod + p1.y;

        Line{yint: yint, slope:slope}
    }

    // a known y intercept / slope
    fn new_from_values(yint: Real, slope: Real) -> Self {
        Line{yint: yint, slope: slope}
    }

    fn new_from_y_axis_reflection(line: &Line) -> Self{
        let new_slope = (-1 as Real) * line.slope;
        Line{yint: line.yint , slope: new_slope }
    }

    fn call(&self, point: &Real) -> Real {
        (self.slope * point) + self.yint
    }
}

/// A struct for sensors of trapezoidal geometry
#[derive(Debug)]
pub struct Trapezoid{
    half_height: Real,
    normal: Vec2,
    to_global: Aff3,
    to_local : Aff3,
    left_line: Line,    // equation of line used for bounds checking 
    right_line: Line,  //   ''
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
    pub fn new(base_top: Real, 
            base_bot: Real, 
            to_global_tfm_matrix: Mat4, 
            height: Real) -> Result<Trapezoid, &'static str> {
        
        let to_global_transform = Aff3::from_matrix_unchecked(to_global_tfm_matrix);

        match to_global_transform.try_inverse(){
            
            //TODO: find noraml vector calculation of the trapezoid

            Some(to_local_transform) => {

                // calculate half lengths
                let half_b1 = base_top/(2 as Real);
                let half_b2 = base_bot/(2 as Real);
                let half_height = height / (2 as Real);

                // normal vector calculation
                let orig = P2::new(0.0, 0.0);
                // points on the trapezoid used for normal vector & line eq. calc
                let point_1 = P2::new(half_b1, 0.0); // top right corner
                let point_2 = P2::new(0.0, half_b2); // not on surface, used for normal vec
                let point_3 = P2::new(half_b2, 0.0); // bottom right corner
                
                //normal vector
                let v1 = orig - point_1;
                let v2 = orig -  point_2;
                let normal_vector = v1.cross(&v2);
                

                // NOTE: we flip the slopes here since we assume the trapezoid is symmetrical
                // line following the slanted right side of the trapezoid
                let right_line_eq = Line::new_from_points(&point_1, &point_2);
                // line following the slanted left edge of the trapezoid
                let left_line_eq = Line::new_from_y_axis_reflection(&right_line_eq);

                let trap = Trapezoid{
                            half_height: half_height,
                            normal: normal_vector,
                            to_global: to_global_transform,
                            to_local: to_local_transform,
                            left_line: left_line_eq,
                            right_line: right_line_eq};
                             
                Ok(trap)

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
        let local = self.to_local * input_point;
        return P2::new(local.x, local.y)
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

        if input.y < self.half_height{ // bound check the top
            if (0 as Real) > input.x { // bound check right
                let value = self.right_line.call(&input.x);

                if value < input.x {true} // make sure we are to the left of right bound
                else{false}
            }
            else { // bounds check the left
                let value = self.left_line.call(&input.x);

                if value > input.x {true} // make sure we ar eto the right of the left bound
                else {false}
            }
        }
        else{false}

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
