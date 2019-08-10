use super::traits::{Plane, Transform};
use super::utils;

use super::super::config::*;
use super::super::error::*;

// Struct to calculate the y value at any given x
// This is used instead of a closure since closures require
// heap allocation w/ trait objects (dynamic dispatch)
#[derive(Debug)]
pub struct Line {
    pub yint: Real,
    pub slope: Real,
}
impl Line {
    //
    pub fn new_from_points(p1: &P2, p2: &P2) -> Self {
        let slope = (p2.y - p1.y) / (p2.x - p1.x);

        // y = m*x + [-m*x_0 + y_0] <- yint
        let prod = -1 as Real * (p1.x * slope);
        let yint = prod + p1.y;

        Line {
            yint: yint,
            slope: slope,
        }
    }

    // a known y intercept / slope
    pub fn new_from_values(yint: Real, slope: Real) -> Self {
        Line {
            yint: yint,
            slope: slope,
        }
    }

    pub fn new_from_y_axis_reflection(line: &Line) -> Self {
        let new_slope = (-1 as Real) * line.slope;
        Line {
            yint: line.yint,
            slope: new_slope,
        }
    }

    pub fn call(&self, point: &P2) -> P2 {
        // "the maximum height we can have with that x value"
        let y = (self.slope * point.x) + self.yint;
        // "the maximum x value we can have when we have that y value"
        let x = (point.y - self.yint) / self.slope;
        P2::new(x, y)
    }
}

/// A struct for sensors of trapezoidal geometry
#[derive(Debug)]
pub struct Trapezoid {
    half_height: Real,
    normal: Vec3,
    center_global: P3,

    to_global: Aff3,
    to_local: Aff3,

    left_line: Line,  // equation of line used for bounds checking
    right_line: Line, //   ''

    max_half_width: Real,
    min_half_width: Real,

    pub to_global_rot: Mat4,
    pub to_local_rot: Mat4,
}

impl Trapezoid {
    ///
    /// # Examples
    /// ```
    /// use kalman_rs::config::*;
    /// use kalman_rs::geometry::Trapezoid;
    ///
    /// let transform_mat = Mat4::identity();
    /// let base_top = 5.;
    /// let base_bot = 10.;
    /// let height = 4.;
    ///
    /// let sensor = Trapezoid::new(base_top, base_bot, transform_mat, transform_mat, height);
    /// ```
    pub fn new(
        base_top: Real,
        base_bot: Real,
        to_global_translation: Mat4,
        to_global_rotation: Mat4,
        height: Real,
    ) -> Result<Trapezoid, MatrixError> {
        let compose = to_global_translation * to_global_rotation;

        let to_global_transform = Aff3::from_matrix_unchecked(compose);
        let to_local_transform = to_global_transform
            .try_inverse()
            .expect("local -> global trap matrix non invertible");

        let to_local_rotation = to_global_rotation
            .try_inverse()
            .expect("local -> global trap matrix non invertible");

        // calculate half lengths
        let half_b1 = base_top / (2 as Real);
        let half_b2 = base_bot / (2 as Real);
        let half_height = height / (2 as Real);

        // normal vector calculation
        let normal_vector = utils::plane_normal_vector(half_b1, half_height);

        // equations of lines along slope of trapezoid for bounding checks
        let top_right_corner = P2::new(half_b1, half_height);
        let bottom_right_corner = P2::new(half_b2, -half_height);
        let right_line_eq = Line::new_from_points(&top_right_corner, &bottom_right_corner);
        let left_line_eq = Line::new_from_y_axis_reflection(&right_line_eq);

        let local_center = P3::new(0., 0., 0.);
        let global_center = to_global_transform * local_center;

        let trap = Trapezoid {
            half_height: half_height,
            normal: normal_vector,
            center_global: global_center,
            to_global: to_global_transform,
            to_local: to_local_transform,
            left_line: left_line_eq,
            right_line: right_line_eq,
            max_half_width: half_b1.max(half_b2),
            min_half_width: half_b1.min(half_b2),
            to_global_rot: to_global_rotation,
            to_local_rot: to_local_rotation,
        };

        Ok(trap)
    }
    ///quickly generates arbitrary sensor data
    pub fn default() -> Self {
        let base_top = 2.;
        let base_bot = 5.;
        let height = 2.;

        let to_global = Mat4::new_random();
        let rot = Mat4::new_random();

        Self::new(base_top, base_bot, to_global, rot, height)
            .expect("could not generate trap. sensor")
    }
}

impl Transform for Trapezoid {
    /// Converts a point in the global reference frame to a point in the local reference frame of the sensor.
    ///
    /// # Examples
    /// ```
    /// use kalman_rs as krs;
    /// use krs::config::*;
    /// use krs::geometry::traits::*;
    /// use krs::geometry::Trapezoid;
    ///
    /// let sensor = Trapezoid::default();
    /// let local_point = P3::origin();
    /// let global_point = sensor.to_global(local_point);
    /// ```
    fn to_global(&self, input_point: P3) -> P3 {
        self.to_global * input_point
    }

    /// Converts a point in the local refernce frame of the sensor to the global reference frame.
    ///
    /// # Examples
    /// ```
    /// use kalman_rs as krs;
    /// use krs::config::*;
    /// use krs::geometry::traits::*;
    /// use krs::geometry::Trapezoid;
    ///
    /// let sensor = Trapezoid::default();
    /// let global_point = P3::origin();
    /// let local_point = sensor.to_global(global_point);
    /// ```
    fn to_local(&self, input_point: P3) -> P2 {
        let local = self.to_local * input_point;
        return P2::new(local.x, local.y);
    }

    /// Checks if a local point is contained within the bounds of a sensor.
    /// NOTE: `plane()` must be called before checking for bounds of the sensor since the normal
    /// vector must be calculated first.
    /// # Examples
    /// ```
    /// use kalman_rs as krs;
    /// use krs::config::*;
    /// use krs::geometry::traits::*;
    /// use krs::geometry::Trapezoid;
    ///
    /// let sensor = Trapezoid::default();
    /// let local_point = P2::origin();
    /// let is_inside: bool = sensor.inside(&local_point);
    /// ```
    fn inside(&self, input: &P2) -> bool {
        let line = if (0 as Real) <= input.x {
            &self.right_line
        } else {
            &self.left_line
        };

        let new_point = line.call(&input);

        let max_height = new_point.y.abs().min(self.half_height);
        let x_abs = new_point.x.abs();

        let max_width = if x_abs <= self.min_half_width {
            self.min_half_width
        } else if x_abs >= self.max_half_width {
            self.max_half_width
        } else if (x_abs >= self.min_half_width) && (x_abs <= self.max_half_width) {
            x_abs
        } else {
            panic!("problem with bounds checking trapezoid")
        }; //TODO: fix this panic

        // let dbg_h = format!{"max height:  {}   actual   {}", &max_height, &$point.y};
        // let dbg_w = format!{"max width:   {}   min width   {} x_abs value:   {}   input value:   {} \
        //             choosen maxium   {}", $max_w, $min_w, x_abs, &$point.x, max_width};
        // dbg!{dbg_h};
        // dbg!{dbg_w};

        if input.y.abs() <= max_height {
            if input.x.abs() <= max_width {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
    fn rotation_to_global(&self) -> &Mat4 {
        &self.to_global_rot
    }
    fn rotation_to_local(&self) -> &Mat4 {
        &self.to_local_rot
    }
}

impl Plane for Trapezoid {
    /// Check if a given point is located on the same plane as the sensor
    /// NOTE: `plane()` must be called becuase the normal vector is not currently known
    /// # Examples
    /// ```
    /// use kalman_rs as krs;
    /// use krs::config::*;
    /// use krs::geometry::traits::*;
    /// use krs::geometry::Trapezoid;
    ///
    /// let sensor = Trapezoid::default();
    /// let local_point = P3::origin();
    /// let is_on_plane : bool = sensor.on_plane(&local_point);
    /// ```
    fn on_plane(&self, input_point: &P3) -> bool {
        let pv = P3::new(0.0, 0.0, 0.0) - input_point;

        if self.normal.dot(&pv).abs() <= DOT_PRODUCT_EPSILON {
            true
        } else {
            false
        }
    }

    fn plane_normal_vec(&self) -> &Vec3 {
        return &self.normal;
    }

    fn global_center(&self) -> &P3 {
        &self.center_global
    }

    fn plane_constant(&self) -> Real {
        unimplemented!()
    }
}
