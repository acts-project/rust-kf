use super::super::config::*;
use nalgebra as na;
use nalgebra::{Point3, Vector3};

// NOTE: since the (slow) math beind this function is 100% certain for symmetric quadralaterals
// I am leaving it in for testing purposes against the currnet (more efficient) bounds checks

/// Checks if an input point is contained within the within the XY bounds of the sensor. A point with
/// any nonzero Z value needs to also use `traits::Plane::on_plane` to ensure that the point falls
/// on the same plane as the sensor.
pub fn quadralateral_contains(points: &[P3; 4], check_point: &P3) -> bool {
    // subtract points so we can make position vectors
    let am_vec = points[0] - check_point;
    let ab_vec = points[0] - points[1];
    let ad_vec = points[0] - points[3];

    // 0 < AM * AB < AB*AB and
    // 0 < AM * AD < AD*AD means
    // A and D are opposite corners, M is the input point
    // if true: the point is inside a quadralateral
    let am_dot = am_vec.dot(&ab_vec);
    if 0.0 < am_vec.dot(&ab_vec) {
        if am_dot < ab_vec.dot(&ab_vec) {
            let am_dot = am_vec.dot(&ad_vec);

            if 0.0 < am_vec.dot(&ad_vec) {
                if am_dot < ad_vec.dot(&ad_vec) {
                    return true;
                }
            }
        }
    }
    return false;
}

/// Calculates the vector normal to three input points
pub fn plane_normal_vector(side_1: Real, side_2: Real) -> Vec3 {
    let point_1 = P3::new(side_1, 0.0, 0.0);
    let point_2 = P3::new(0.0, side_2, 0.0);
    let point_3 = P3::new(0.0, 0.0, 0.0);

    let v1 = point_1 - point_2;
    let v2 = point_1 - point_3;

    v1.cross(&v2) //cross product yields normal vector of the plane
}

// NOTE: since the (slow) math beind this function is 100% certain for symmetric quadralaterals
// I am leaving it in for testing purposes against the currnet (more efficient) bounds checks

/// This function organizes the input points of a sensor into a known order so that `quadralateral_contains`
/// will correctly function. There is a known edge in which a trapezoid with an extremely low height will choose
/// the wrong order of points. This is relatively easy to fix but quadruples the total number of comparissons needed.
#[allow(dead_code)]
pub fn organize_points<'a>(input_points: &'a mut [P3; 4]) -> &'a [P3; 4] {
    // we collect into a vec here since FromIterator is not implemented for
    // arrays (unknown size)
    let distances: Vec<_> = input_points[1..4]
        .iter()
        .zip([input_points[0]; 4].iter())
        .map(|(x, y)| (x - y).norm_squared())
        .collect();
    println! {"the distances are : {:?}", distances}
    let mut max_ = distances[0];
    let mut max_index = 1;
    for i in 1..3 {
        let current_value = distances[i];
        println! {"current max: {} current value: {} index: {}", max_, current_value, max_index}
        if current_value > max_ {
            max_ = current_value;
            max_index = i + 1;
        }
    }
    println! {"ended with max {} and index {}", max_, max_index}

    // if the furthest distance away becomes the corner
    if max_index != 3 {
        println! {"changing indexes aroujnd {}", max_index}
        let copy = input_points[3];
        input_points[3] = input_points[max_index];
        input_points[max_index] = copy;
    } else {
        println! {"array already oriented correctly"}
    }
    return input_points;
}
