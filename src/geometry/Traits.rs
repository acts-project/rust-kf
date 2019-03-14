extern crate nalgebra as na;
use na::{Point3, Matrix4 as Matrix, Vector3 as Vector};

pub trait Transform{
    fn to_global(&self, input_point: Point3<f32>) -> na::Point3<f32>;

    fn to_local(&self, input_point: Point3<f32>) -> na::Point3<f32>;

    fn contains_from_global(&self, input_point: Point3<f32>) -> bool{
        let local_point = Self::to_local(&self, input_point);
        Self::contains_from_local(&self,&local_point)
    }

    fn contains_from_local(&self, input: &Point3<f32>) ->bool;
}


pub fn quadralateral_contains(points: &[Point3<f32>;4], check_point: &Point3<f32>)->bool{
    // subtract points so we can make position vectors
    let AM =  points[0] - check_point;
    let AB = points[0] - points[1];
    let AD = points[0] - points[3];

    // position vectors
    let AM = Vector::new(AM.x, AM.y, AM.z);
    let AB = Vector::new(AB.x, AB.y, AB.z);
    let AD = Vector::new(AD.x, AD.y, AD.z);

    // 0 < AM * AB < AB*AB and
    // 0 < AM * AD < AD*AD means 
    // A and D are opposite corners, M is the input point
    // if true: the point is inside a quadralateral
    let AM_dot = AM.dot(&AB);
    if 0.0 < AM.dot(&AB){
        println!{"1"}
        if AM_dot < AB.dot(&AB){
            println!{"2"}
            let AM_dot = AM.dot(&AD);

            if 0.0 < AM.dot(&AD){
                println!{"3"}
                if AM_dot < AD.dot(&AD){
                    println!{"4"}
                    return true
                }
            }
        }
    }
    return false
}


fn distance(p1: &Point3<f32>, p2:&Point3<f32>)->f32{
    let pv = p1 - p2; // find the distance between x / y / z values

    return (pv.x.powf(2.0) + pv.y.powf(2.0) + pv.z.powf(2.0)).powf(0.5)
}

// this function exists to organize the points into a known formation
// since the geometry::Trapezoid::Trapezoid + geometry::Rectangle::Rectangle
// both rely on geometry::Traits::quadralateral_contains to quickly calculate if a point
// is inside the shape. This function requires the locations of the two corner points
pub fn organize_points<'a>(input_points: &'a mut [Point3<f32>;4]) -> &'a [Point3<f32>;4]{

    // we collect into a vec here since FromIterator is not implemented for 
    // arrays (unknown size)
    let distances : Vec<_> = input_points[1..4].iter()
                                        .zip([input_points[0];4].iter())
                                        .map(|(x,y)| distance(x,y))
                                        .collect();
    println!{"the distances are : {:?}", distances}
    let mut max_ = distances[0];
    let mut max_index = 1;
    for i in 1..3{
        let current_value = distances[i];
        println!{"current max: {} current value: {} index: {}", max_, current_value, max_index}
        if current_value > max_{
            max_ = current_value;
            max_index = i+1;

        } 

    }
    println!{"ended with max {} and index {}", max_, max_index}
    
    // if the furthest distance away becomes the corner
    if max_index != 3{
        println!{"changing indexes aroujnd {}", max_index}
        let copy = input_points[3];
        input_points[3] = input_points[max_index];
        input_points[max_index] = copy;
    }
    else{
        println!{"array already oriented correctly"}
    }
    return input_points
    
}