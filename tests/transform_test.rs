
use kalman_rs::{self, geometry};

use geometry::{Rectangle, Trapezoid};
use geometry::traits::Transform;
use kalman_rs::config::*;
use kalman_rs::error::*;

/**
 * Tests for moving points local -> global and global -> local relative to a sensor
 * 
 */

// default rectangle to reduce boilerplate of each test
fn initialize_rect() -> Rectangle {
    let base_len = 3 as Real;
    let height_len = 3 as Real;

    let tfm_matrix= Mat4::identity(); //arbitrary transform matrix

    Rectangle::new(base_len, height_len, tfm_matrix.clone(), tfm_matrix.clone()).unwrap()
}

// default trapezoid for testing
fn initialize_trap() -> Trapezoid {
    let top_base = 2 as Real;
    let bottom_base = 5 as Real;
    let height = 2 as Real;

    let tfm = Mat4::identity();     

    Trapezoid::new(top_base, bottom_base, tfm, tfm, height).unwrap()
}


// testing src::geometry::traits::quadralateral_contains to make sure it detects points inside 
// the boundary correctly (no respect for z)
#[cfg(test)]
mod _quadralateral_contains {

    use super::{Trapezoid, Rectangle, Transform, initialize_rect, initialize_trap};
    use kalman_rs::config::*;

    #[test]
    fn rectangle_outside_1 () {
        let rect = initialize_rect();

        let test_point = P2::new(1.51, 1.51);

        assert_eq!(rect.inside(&test_point), false)
    }

    #[test]
    fn rectangle_outside_2 () {
        let rect = initialize_rect();

        let test_point = P2::new(-3.0, 0.0);

        assert_eq!(rect.inside(&test_point), false)
    }

    #[test]
    fn rectangle_inside_1 () {
        let rect = initialize_rect();

        let test_point = P2::new(1.49, 1.49);

        assert_eq!(rect.inside(&test_point), true)
    }

    #[test]
    fn rectangle_inside_2 () {
        let rect = initialize_rect();

        let test_point = P2::new(-1.0, 1.0);

        assert_eq!(rect.inside(&test_point), true)
    }

    #[test]
    fn trapezoid_outside (){
        let trap = initialize_trap();

        let test_point = P2::new(-1.0, 2.6);

        assert_eq!(trap.inside(&test_point), false)
    }

    
    #[test]
    fn trapezoid_outside_2 (){
        let trap = initialize_trap();

        let test_point = P2::new(-1.0, 1.01);

        assert_eq!(trap.inside(&test_point), false)
    }

    #[test]
    fn trapezoid_outside_3 (){
        let trap = initialize_trap();

        let test_point = P2::new(0.0, 2.0);

        assert_eq!(trap.inside(&test_point), false)
    }

    #[test]
    fn trapezoid_outside_4 (){
        let trap = initialize_trap();

        let test_point = P2::new(0.0, -2.0);

        assert_eq!(trap.inside(&test_point), false)
    }

    #[test]
    fn trapezoid_outside_5 (){
        let trap = initialize_trap();

        let test_point = P2::new(-1.0, 1.1);

        assert_eq!(trap.inside(&test_point), false)
    }

    #[test]
    fn trapezoid_outside_6 (){
        let trap = initialize_trap();

        let test_point = P2::new(0.0, -5.0);

        assert_eq!(trap.inside(&test_point), false)
    }

    #[test]
    fn trapezoid_inside_1 (){
        let trap = initialize_trap();

        let test_point = P2::new(0.0, 0.0);

        assert_eq!(trap.inside(&test_point), true)
    }
    #[test]
    fn trapezoid_inside_2 (){
        let trap = initialize_trap();

        let test_point = P2::new(1.0, 0.0);

        assert_eq!(trap.inside(&test_point), true)
    }
    #[test]
    fn trapezoid_inside_3 (){
        let trap = initialize_trap();       //////////////////////////

        let test_point = P2::new(-1.0, 0.0);

        assert_eq!(trap.inside(&test_point), true)
    }
    #[test]
    fn trapezoid_inside_4 (){
        let trap = initialize_trap();

        let test_point = P2::new(1.5, 0.0);

        assert_eq!(trap.inside(&test_point), true)
    }
    #[test]
    fn trapezoid_inside_5 (){
        let trap = initialize_trap();

        let test_point = P2::new(4.0, 0.0);

        assert_eq!(trap.inside(&test_point), false)
    }
}


