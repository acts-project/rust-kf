extern crate nalgebra as na;

use super::traits::{Transform, Plane};
use super::utils;

use super::super::config::*;
use super::super::error::*;

// Struct to calculate the y value at any given x
// This is used instead of a closure since closures require 
// heap allocation w/ trait objects (dynamic dispatch)
#[derive(Debug)]
pub struct Line {
    pub yint: Real,
    pub slope: Real
}
impl Line {//
    pub fn new_from_points(p1: &P2, p2: &P2) -> Self{
        let slope = (p2.y - p1.y)/ (p2.x - p1.x);

        // y = m*x + [-m*x_0 + y_0] <- yint
        let prod = -1 as Real *(p1.x*slope);
        let yint = prod + p1.y;

        Line{yint: yint, slope:slope}
    }

    // a known y intercept / slope
    pub fn new_from_values(yint: Real, slope: Real) -> Self {
        Line{yint: yint, slope: slope}
    }

    pub fn new_from_y_axis_reflection(line: &Line) -> Self{
        let new_slope = (-1 as Real) * line.slope;
        Line{yint: line.yint , slope: new_slope }
    }

    pub fn call(&self, point: &P2) -> P2 {
        // "the maximum height we can have with that x value"
        let y = (self.slope * point.x) + self.yint;
        // "the maximum x value we can have when we have that y value"
        let x = (point.y - self.yint)/self.slope;
        P2::new(x, y)
    }
}

/// A struct for sensors of trapezoidal geometry
#[derive(Debug)]
pub struct Trapezoid{
    half_height: Real,
    normal: Vec3,
    to_global: Aff3,
    to_local : Aff3,
    left_line: Line,    // equation of line used for bounds checking 
    right_line: Line,  //   ''
    max_half_width: Real,
    min_half_width: Real
}

impl Trapezoid{
    /*
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
    /// ```*/
    pub fn new(base_top: Real, 
            base_bot: Real, 
            to_global_tfm_matrix: Mat4, 
            height: Real) -> Result<Trapezoid, MatrixError> {
        
        let to_global_transform = Aff3::from_matrix_unchecked(to_global_tfm_matrix);

        match to_global_transform.try_inverse(){
            
            //TODO: find noraml vector calculation of the trapezoid

            Some(to_local_transform) => {

                // calculate half lengths
                let half_b1 = base_top/(2 as Real);
                let half_b2 = base_bot/(2 as Real);
                let half_height = height / (2 as Real);

                // normal vector calculation
                let normal_vector = utils::plane_normal_vector(half_b1, half_height);
                
                // equations of lines along slope of trapezoid for bounding checks
                let top_right_corner = P2::new(half_b1, half_height);
                let bottom_right_corner = P2::new(half_b2, -half_height);
                let right_line_eq = Line::new_from_points(&top_right_corner, &bottom_right_corner);
                let left_line_eq = Line::new_from_y_axis_reflection(&right_line_eq);

                let trap = Trapezoid{
                            half_height: half_height,
                            normal: normal_vector,
                            to_global: to_global_transform,
                            to_local: to_local_transform,
                            left_line: left_line_eq,
                            right_line: right_line_eq,
                            max_half_width: half_b1.max(half_b2),
                            min_half_width: half_b1.min(half_b2)};
                             
                Ok(trap)

                },
            None => return Err(MatrixError::NonInvertible)

        }
    }
}


impl Transform for Trapezoid{
    /*
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
    /// ```*/
    fn to_local(&self, input_point: P3) -> P2{
        let local = self.to_local * input_point;
        return P2::new(local.x, local.y)
    }

    
    /*
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
    /// ```*/
    fn contains_from_local(&self, input: &P2) -> bool {

        let line = 
            if (0 as Real) <= input.x {&self.right_line}
            else {&self.left_line};

        let new_point = line.call(&input);

        let max_height = new_point.y.abs().min(self.half_height);
        let x_abs = new_point.x.abs();

        let max_width =  
            if x_abs <= self.min_half_width {self.min_half_width}
            else if x_abs >= self.max_half_width{self.max_half_width}
            else if (x_abs >= self.min_half_width) && (x_abs <= self.max_half_width) {x_abs}
            else{panic!("problem with bounds checking trapezoid")}; //TODO: fix this panic 

        // let dbg_h = format!{"max height:  {}   actual   {}", &max_height, &$point.y};
        // let dbg_w = format!{"max width:   {}   min width   {} x_abs value:   {}   input value:   {} \
        //             choosen maxium   {}", $max_w, $min_w, x_abs, &$point.x, max_width};
        // dbg!{dbg_h};
        // dbg!{dbg_w};

        if input.y.abs() <= max_height {
            if input.x.abs() <= max_width {return true}
            else{return false}
        }
        else{
            return false
        }

    }
}

impl Plane for Trapezoid{

    /*
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
    /// ```*/
    fn on_plane(&self, input_point: &P3) -> bool {
        let pv = P3::new(0.0, 0.0, 0.0) - input_point;
        //TODO : this function should probably not return result
        if self.normal.dot(&pv) == 0.0 {
            // bounds_check!();
            true
        }
        else{
            false
        }

    }
}
