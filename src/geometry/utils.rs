
use nalgebra as na;
use nalgebra :: {Point3, Vector3};

/// Checks if an input point is contained within the within the XY bounds of the sensor. A point with 
/// any nonzero Z value needs to also use `traits::Plane::on_plane` to ensure that the point falls
/// on the same plane as the sensor. 
pub fn quadralateral_contains(points: &[Point3<f32>;4], check_point: &Point3<f32>)->bool{
    // subtract points so we can make position vectors
    let am_vec =  vector3_from_points(&points[0] , &check_point);
    let ab_vec = vector3_from_points(&points[0], &points[1]);
    let ad_vec = vector3_from_points(&points[0] , &points[3]);

    // 0 < AM * AB < AB*AB and
    // 0 < AM * AD < AD*AD means 
    // A and D are opposite corners, M is the input point
    // if true: the point is inside a quadralateral
    let am_dot = am_vec.dot(&ab_vec);
    if 0.0 < am_vec.dot(&ab_vec){
        if am_dot < ab_vec.dot(&ab_vec){

            let am_dot = am_vec.dot(&ad_vec);

            if 0.0 < am_vec.dot(&ad_vec){
                if am_dot < ad_vec.dot(&ad_vec){
                    return true
                }
            }
        }
    }
    return false
}

/// Calcuate a `nalgebra::Vector3<f32>` position vector from two input `nalgebra::Point3<f32>`
pub fn vector3_from_points(p1: &Point3<f32>, p2: &Point3<f32>) -> Vector3<f32> {
    let pv = p1 - p2; // difference between points
    
    Vector3::new(pv.x, pv.y, pv.z)
}

/// Calculates the vector normal to three input points
pub fn plane_normal_vector(p1: &Point3<f32>, p2: &Point3<f32> ,p3: &Point3<f32>) -> Vector3<f32> {
    let v1 = vector3_from_points(&p1, &p2); //vector 1 located on the plane
    let v2 = vector3_from_points(&p1, &p3); // vector 2 located on the plane

    v1.cross(&v2) //cross product yields normal vector of the plane
}

/// Find the distance between two `nalgebra::Point3<f32>`s
fn distance(p1: &Point3<f32>, p2:&Point3<f32>)->f32{
    let pv = p1 - p2; // find the distance between x / y / z values

    return (pv.x.powf(2.0) + pv.y.powf(2.0) + pv.z.powf(2.0))
}


/// This function organizes the input points of a sensor into a known order so that `quadralateral_contains`
/// will correctly function. There is a known edge in which a trapezoid with an extremely low height will choose
/// the wrong order of points. This is relatively easy to fix but quadruples the total number of comparissons needed.
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