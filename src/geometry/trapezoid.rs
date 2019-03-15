extern crate nalgebra as na;
use na::{Point3, Matrix4 as Matrix, Vector3 as Vector};
use super::traits::{self, Transform, Plane};

pub struct Trapezoid{
    points : [na::Point3<f32>; 4],
    normal: Option<Vector<f32>>, // the normal vector is not initially calculated
    tfm: na::Affine3<f32>
}

impl Trapezoid{
    pub fn new(points: [Point3<f32>; 4], tfm_matrix: Matrix<f32>) -> Result<Trapezoid, &'static str>{
        
        let affine_transform = na::Affine3::from_matrix_unchecked(tfm_matrix);

        match affine_transform.try_inverse(){
            Some(_x) => {
                Ok(Trapezoid{points: points, normal:None, tfm: affine_transform})},
            None => return Err("matrix was not invertable")

        }
    }
}


impl Transform for Trapezoid{
    fn to_global(&self, input_point: Point3<f32>)-> na::Point3<f32>{
        return self.tfm * input_point;
    }
    
    fn to_local(&self, input_point: Point3<f32>) -> na::Point3<f32>{
        self.tfm.inverse() * input_point
    }

    fn contains_from_local(&self, input: &Point3<f32>) ->bool{
        traits::quadralateral_contains(&self.points, &input)
    }
}

impl Plane for Trapezoid{
    fn plane(&mut self) -> Vector<f32>{
        // calculate the normal vector of the surface if it has not been calculated before
        // if it has been calculated return the original calculation
        // this would need to change if the suface moves 
        match self.normal{
            Some(x)=>x,
            None =>{
                let normal_vector = traits::plane_normal_vector(&self.points[0], &self.points[1], &self.points[2]);
                self.normal = Some(normal_vector);
                normal_vector
            },
        }
    }
    fn on_plane(&mut self, input_point: Point3<f32>) -> bool{
        let pv = traits::vector3_from_points(&self.points[0], &input_point);
        if self.plane().dot(&pv) ==0.0 && self.contains_from_local(&input_point) {
            return true
        }
        false
    }
}
