#[allow(dead_code)]

pub enum SensorType{
    Rectanlge,
    Trapezoid,
}

pub struct Rectangle {
    length_x: u32,
    length_y: u32,
}

pub struct Trapezoid{
    length_base_1: u32,
    length_base_2: u32,
    length_height: u32,
    alpha_angle: u32,
    beta_angle: u32,
}

impl Trapezoid {
    pub fn new(base_1: u32, base_2:u32, height:u32, alpha:u32, beta:u32) -> Trapezoid {
        return Trapezoid{length_base_1: base_1,
                        length_base_2: base_2,
                        length_height: height,
                        alpha_angle: alpha,
                        beta_angle: beta }
    }
}

impl Rectangle {
    pub fn new(length_x:u32, length_y:u32) -> Rectangle{
        return Rectangle{
            length_x: length_x,
            length_y: length_y,
        }
    }
}