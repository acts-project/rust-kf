use kalman_rs::{self, geometry};

use geometry::{Rectangle, Trapezoid};
use geometry::traits::Plane;
use kalman_rs::config::*;


// default rectangle to reduce boilerplate of each test
fn initialize_rect() -> Rectangle {
    let base_len = 3 as Real;
    let height_len = 3 as Real;

    let tfm_matrix= Mat4::identity(); //arbitrary transform matrix
    let projection = Mat2x5::zeros();

    Rectangle::new(base_len, height_len, tfm_matrix, projection).unwrap()
}

// default trapezoid for testing
fn initialize_trap() -> Trapezoid {
    let top_base = 2 as Real;
    let bottom_base = 5 as Real;
    let height = 2 as Real;

    let tfm = Mat4::identity();     

    Trapezoid::new(top_base, bottom_base, tfm, height).unwrap()
}


#[cfg(test)]
mod plane_tests{
 
    use super::{Trapezoid, Rectangle, Plane, initialize_rect, initialize_trap};
    use kalman_rs::config::*;

    #[test]
    fn rectangle_on_plane() {
        let rect = initialize_rect();
        assert_eq!(rect.on_plane(&P3::new(1.0, 1.0, 0.0)), true)
    }

    #[test]
    fn rectangle_off_plane() {
        let rect = initialize_rect();

        assert_eq!(rect.on_plane(&P3::new(2.0, 2.0, 1.0)), false)
    }

    #[test]
    fn trapezoid_on_plane(){
        let trap = initialize_trap();

        let test_point = P3::new(1.0, 1.0, 0.0);
        assert_eq!(trap.on_plane(&test_point), true)
    }

    #[test]
    fn trapezoid_off_plane(){
        let trap = initialize_trap();

        let test_point = P3::new(1.0, 1.0, 1.0);
        assert_eq!(trap.on_plane(&test_point), false)
    }

}