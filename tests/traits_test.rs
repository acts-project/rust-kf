

#[cfg(test)]
mod _organize_points{

    use nalgebra::Point3 ;
    use kalman_rs::geometry;

    #[test]
    // it would seem that there might be an edge case in which the height of the trapezoid is very small
    fn test_trapezoid () -> () {
        // distances from  index 0:               0                     5.09                 10.2                         10.0
        let point_array = [Point3::new(0.0, 0.0, 0.0), Point3::new(5.0,1.0,0.0), Point3::new(5.0, 9.0,0.0), Point3::new(0.0,10.0,0.0)];
        //therefore we expect the last point to be the one from index 2

        assert_eq!(point_array[2], geometry::utils::organize_points(&mut point_array.clone())[3]);
    }

    #[test] //rhombus
    fn test_trapezoid_2(){ 

        let point_array = [Point3::new(0.0, 0.0, 0.0), Point3::new(3.0,4.0,0.0), Point3::new(0.0, 5.0,0.0), Point3::new(-2.0,4.0,0.0)];

        assert_eq!(point_array[1], geometry::utils::organize_points(&mut point_array.clone())[3]);
    }
    #[test]
    fn test_rectange(){
        // Rectangle
        //distances:                  0                             5                          7.07                        7.07 
        let point_array_2 = [Point3::new(0.0, 0.0, 0.0), Point3::new(5.0,0.0,0.0), Point3::new(0.0,5.0,0.0), Point3::new(5.0,5.0,0.0)];
        // we expect index 3 to be the max distance 

        assert_eq!(point_array_2[3], geometry::utils::organize_points(&mut point_array_2.clone())[3]);
    }
}//organize points end

// testing src::geometry::traits::quadralateral_contains to make sure it detects points inside 
// the boundary correctly (no respect for z)
#[cfg(test)]
mod _quadralateral_contains {

    use nalgebra::Point3;
    use kalman_rs::geometry::utils::quadralateral_contains;
    use kalman_rs::config::*;
    use kalman_rs::geometry::traits::Transform;
    use kalman_rs::geometry::rectangle::Rectangle;
    // use geometry::trapezoid::Trapezoid;

    #[test]
    fn rectangle_outside () {
        let base_len = 3 as Real;
        let height_len = 3 as Real;
        let tfm_matrix= Mat4::new(1.0,0.0,0.0,0.0,  0.0,1.0,0.0,0.0,  0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0); //arbitrary transform matrix
        let mut rect = Rectangle::new(base_len, height_len, tfm_matrix).unwrap();

        assert_eq!(rect.contains_from_local(&P2::new(5.0, 5.0)), false)
    }

    #[test]
    fn rectangle_inside () {
        let base_len = 3 as Real;
        let height_len = 3 as Real;
        let tfm_matrix= Mat4::new(1.0,0.0,0.0,0.0,  0.0,1.0,0.0,0.0,  0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0); //arbitrary transform matrix
        let mut rect = Rectangle::new(base_len, height_len, tfm_matrix).unwrap();

        assert_eq!(rect.contains_from_local(&P2::new(1.0, 1.0)), true)
    }

    // #[test] // standarad trapezoid
    // fn trapezoid_outside (){
    //     let point_array = [Point3::new(0.0, 0.0, 0.0), Point3::new(5.0,1.0,0.0), Point3::new(0.0,10.0,0.0) ,Point3::new(5.0, 9.0,0.0)];
    //     let outside_point = Point3::new(-1.0, 0.0, 0.0);

    //     assert_eq!(quadralateral_contains(&point_array, &outside_point), false)
    // }

    // #[test] // standard trapezoid
    // fn trapezoid_inside() {
    //     let point_array = [Point3::new(0.0, 0.0, 0.0), Point3::new(5.0,1.0,0.0), Point3::new(0.0,10.0,0.0), Point3::new(5.0, 9.0,0.0)];
    //     let outside_point = Point3::new(1.0, 0.5, 0.0);

    //     assert_eq!(quadralateral_contains(&point_array, &outside_point), true)
    // }

    // #[test] //rhombus
    // fn trapezoid_outside_2() {
    //     let point_array = [Point3::new(0.0, 0.0, 0.0),  Point3::new(0.0, 5.0,0.0), Point3::new(-2.0,4.0,0.0), Point3::new(3.0,5.0,0.0)];
    //     let outside_point = Point3::new(-1.0, -1.0, -1.0);

    //     assert_eq!(quadralateral_contains(&point_array, &outside_point), false)
    // }

    // #[test] //rhombus
    // fn trapezoid_inside_2() {
    //     let point_array = [Point3::new(0.0, 0.0, 1.0),  Point3::new(0.0, 5.0,1.0), Point3::new(-2.0,4.0,1.0), Point3::new(3.0,5.0,1.0)];
    //     let inside_point = Point3::new(1.0, 1.0, 1.0);

    //     assert_eq!(quadralateral_contains(&point_array, &inside_point), true)
    // }
}


