extern crate nalgebra as na;


enum Sensor{
    Rectangle(Rectangle),
    Trapezoid(Trapezoid),
}


pub enum SensorType{
    Rectangle,
    Trapezoid,
}

#[allow(dead_code)]

pub struct Rectangle {
    points : [na::Point3<f32>; 4],
    tfm: na::Affine3<f32>
}


pub struct Trapezoid{
    points : [na::Point3<f32>; 4],
    tfm: na::Affine3<f32>
}

//Todo: constructors and structs for rectangle and trapezoid are almost identical. should be wrapped in a macro to reduce repetition
//      It is currently not in macro form for readability / easy addition of features

// should add affine transformation matrix to this 
impl Rectangle {
    pub  fn new(point_array: [na::Point3<f32>; 4], transformation_matrix: na::Matrix4<f32>) -> Result<Rectangle, &'static str>{

        let affine_transform = na::Affine3::from_matrix_unchecked(transformation_matrix);

        match affine_transform.try_inverse(){
            Some(x) => return Ok(Rectangle{points: point_array, tfm: affine_transform}),
            None => return Err("matrix was not invertable")

        }

    }

}

// should add affine transformation matrix to this 
impl Trapezoid {
    pub  fn new(point_array: [na::Point3<f32>; 4], transformation_matrix: na::Matrix4<f32>) -> Result<Trapezoid, &'static str>{

        let affine_transform = na::Affine3::from_matrix_unchecked(transformation_matrix);

        match affine_transform.try_inverse(){
            Some(x) => return Ok(Trapezoid{points: point_array, tfm: affine_transform}),
            None => return Err("matrix was not invertable")

        }

    }

}

impl Transform for Rectangle{
    fn to_global(&self, input_point: na::Point3<f32>)-> na::Point3<f32>{
        return self.tfm * input_point;
    }
    fn to_local(&self, input_point: na::Point3<f32>) -> na::Point3<f32>{
        self.tfm.inverse() * input_point

    }

    fn contains_from_global(&self, input_point: na::Point3<f32>) -> bool{
        let local_point = self.to_local(input_point);
        self.contains_from_local(local_point)
    }

    fn contains_from_local(&self, input: na::Point3<f32>) ->bool{
    return true
    }
}

impl Transform for Trapezoid{
    fn to_global(&self, input_point: na::Point3<f32>)-> na::Point3<f32>{
        return self.tfm * input_point;
    }
    fn to_local(&self, input_point: na::Point3<f32>) -> na::Point3<f32>{
        self.tfm.inverse() * input_point

    }

    fn contains_from_global(&self, input_point: na::Point3<f32>) -> bool{
        let local_point = self.to_local(input_point);
        self.contains_from_local(local_point)
    }

    fn contains_from_local(&self, input: na::Point3<f32>) ->bool{
    return true
    }
}

trait Transform{
    fn to_global(&self, input_point: na::Point3<f32>) -> na::Point3<f32>;

    fn to_local(&self, input_point: na::Point3<f32>) -> na::Point3<f32>;

    fn contains_from_global(&self, input_point: na::Point3<f32>) -> bool;

    fn contains_from_local(&self, input: na::Point3<f32>) ->bool;
}