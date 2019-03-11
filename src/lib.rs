#[allow(dead_code)]

pub enum SensorType{
    Rectangle,
    Trapezoid,
}

pub struct Point{
    x: f32,
    y: f32,
    z: f32,
}

pub struct GenericSensor{
    p1: f32,
    p2: f32,
    p3: f32,
    p4: f32,
    sensor_type: SensorType,
}
pub struct Rectangle {
    length_x: f32,
    length_y: f32,
}

impl Rectangle {
    pub fn new(length_x:f32, length_y:f32) -> Rectangle{
        return Rectangle{
            length_x: length_x,
            length_y: length_y,
        }
    }
}

pub struct Trapezoid{
    length_base_1: f32,
    length_base_2: f32,
    length_height: f32,
    alpha_angle: f32,
    beta_angle: f32,
}

impl Trapezoid {
    pub fn new(base_1: f32, base_2:f32, height:f32, alpha:f32, beta:f32) -> Trapezoid {
        return Trapezoid{length_base_1: base_1,
                        length_base_2: base_2,
                        length_height: height,
                        alpha_angle: alpha,
                        beta_angle: beta }
    }
}

trait fetch_data {
    fn data <T> (&self) -> T;
}