
#[cfg(test)]
mod traits_test{

    //testing src::geometry::traits::organize_points to ensure it structures points correctly
    // [opposite corners of quadralateral should be at indexes 0 and 3]
    #[cfg(test)]
    mod _organize_points{

        use nalgebra::Point3 ;
        use super::super::super::geometry;

        #[test]
        // it would seem that there might be an edge case in which the height of the trapezoid is very small
        fn test_trapezoid () -> () {
            //distances from  index 0:               0                     5.09                 10.2                         10.0
            let point_array = [Point3::new(0.0, 0.0, 0.0), Point3::new(5.0,1.0,0.0), Point3::new(5.0, 9.0,0.0), Point3::new(0.0,10.0,0.0)];
            //therefore we expect the last point to be the one from index 2

            assert_eq!(point_array[2], geometry::traits::organize_points(&mut point_array.clone())[3]);
        }

        #[test] //rhombus
        fn test_trapezoid_2(){ 

            let point_array = [Point3::new(0.0, 0.0, 0.0), Point3::new(3.0,4.0,0.0), Point3::new(0.0, 5.0,0.0), Point3::new(-2.0,4.0,0.0)];

            assert_eq!(point_array[1], geometry::traits::organize_points(&mut point_array.clone())[3]);
        }
        #[test]
        fn test_rectange(){
            // Rectangle
            //distances:                  0                             5                          7.07                        7.07 
            let point_array_2 = [Point3::new(0.0, 0.0, 0.0), Point3::new(5.0,0.0,0.0), Point3::new(0.0,5.0,0.0), Point3::new(5.0,5.0,0.0)];
            // we expect index 3 to be the max distance 

            assert_eq!(point_array_2[3], geometry::traits::organize_points(&mut point_array_2.clone())[3]);
        }
    }//organize points end

    // testing src::geometry::traits::quadralateral_contains to make sure it detects points inside 
    // the boundary correctly (no respect for z)
    #[cfg(test)]
    mod _quadralateral_contains {

        use nalgebra::Point3 ;
        use super::super::super::geometry::traits::quadralateral_contains;
        #[test]
        fn rectangle_outside () {
            let point_array = [Point3::new(0.0, 0.0, 0.0), Point3::new(5.0,0.0,0.0), Point3::new(0.0,5.0,0.0), Point3::new(5.0,5.0,0.0)];
            let outside_point = Point3::new(-1.0, 0.0, 0.0);

            assert_eq!(quadralateral_contains(&point_array, &outside_point), false)
        }

        #[test]
        fn rectangle_inside () {
            let point_array = [Point3::new(0.0, 0.0, 1.0), Point3::new(5.0,0.0,1.0), Point3::new(0.0,5.0,1.0), Point3::new(5.0,5.0,1.0)];
            let inside_point = Point3::new(4.9, 1.0, 1.0);

            assert_eq!(quadralateral_contains(&point_array, &inside_point), true)
        }

        #[test] // standarad trapezoid
        fn trapezoid_outside (){
            let point_array = [Point3::new(0.0, 0.0, 0.0), Point3::new(5.0,1.0,0.0), Point3::new(0.0,10.0,0.0) ,Point3::new(5.0, 9.0,0.0)];
            let outside_point = Point3::new(-1.0, 0.0, 0.0);

            assert_eq!(quadralateral_contains(&point_array, &outside_point), false)
        }

        #[test] // standard trapezoid
        fn trapezoid_inside() {
            let point_array = [Point3::new(0.0, 0.0, 0.0), Point3::new(5.0,1.0,0.0), Point3::new(0.0,10.0,0.0), Point3::new(5.0, 9.0,0.0)];
            let outside_point = Point3::new(1.0, 0.5, 0.0);

            assert_eq!(quadralateral_contains(&point_array, &outside_point), true)
        }

        #[test] //rhombus
        fn trapezoid_outside_2() {
            let point_array = [Point3::new(0.0, 0.0, 0.0),  Point3::new(0.0, 5.0,0.0), Point3::new(-2.0,4.0,0.0), Point3::new(3.0,5.0,0.0)];
            let outside_point = Point3::new(-1.0, -1.0, -1.0);

            assert_eq!(quadralateral_contains(&point_array, &outside_point), false)
        }

        #[test] //rhombus
        fn trapezoid_inside_2() {
            let point_array = [Point3::new(0.0, 0.0, 1.0),  Point3::new(0.0, 5.0,1.0), Point3::new(-2.0,4.0,1.0), Point3::new(3.0,5.0,1.0)];
            let inside_point = Point3::new(1.0, 1.0, 1.0);

            assert_eq!(quadralateral_contains(&point_array, &inside_point), true)
        }
    }
}

// this module checks that a given point is within the same plane as the surface
// it checks the geometry::traits::Plane implementation (this complements traits::quadralateral_contains)
#[cfg(test)]
mod plane_tests{
    use nalgebra::{Point3};
    use super::super::geometry;
    use crate::geometry::traits::Plane;

    #[test]
    fn rectangle_on_plane() {
        let rect_points = [Point3::new(0.0, 0.0, 0.0), Point3::new(5.0,0.0,0.0), Point3::new(0.0,5.0,0.0), Point3::new(5.0,5.0,0.0)];
        let tfm_matrix : na::Matrix4<f32>= na::Matrix4::new(1.0,5.0,7.0,2.0,  3.0,5.0,7.0,4.0,  8.0,4.0,1.0,9.0, 2.0,6.0,4.0,8.0);
        let mut rect = geometry::rectangle::Rectangle::new(rect_points, tfm_matrix).unwrap();
        rect.plane();

        assert_eq!(rect.on_plane(&Point3::new(1.0, 1.0, 0.0)).unwrap(), true)
    }

    #[test]
    fn rectangle_off_plane() {
        let rect_points = [Point3::new(0.0, 0.0, 0.0), Point3::new(5.0,0.0,0.0), Point3::new(0.0,5.0,0.0), Point3::new(5.0,5.0,0.0)];
        let tfm_matrix : na::Matrix4<f32>= na::Matrix4::new(1.0,5.0,7.0,2.0,  3.0,5.0,7.0,4.0,  8.0,4.0,1.0,9.0, 2.0,6.0,4.0,8.0);
        let mut rect = geometry::rectangle::Rectangle::new(rect_points, tfm_matrix).unwrap();
        rect.plane();

        assert_eq!(rect.on_plane(&Point3::new(1.0, 1.0, 1.0)).unwrap(), false)
    }

    #[test]
    fn trapezoid_on_plane(){
        let trap_points = [Point3::new(0.0, 0.0, 0.0), Point3::new(5.0,1.0,0.0), Point3::new(0.0,10.0,0.0) ,Point3::new(5.0, 9.0,0.0)];
        let tfm_matrix: na::Matrix4<f32>= na::Matrix4::new(1.0,5.0,7.0,2.0,  3.0,5.0,7.0,4.0,  8.0,4.0,1.0,9.0, 2.0,6.0,4.0,8.0);
        let mut rect = geometry::trapezoid::Trapezoid::new(trap_points, tfm_matrix).unwrap();
        rect.plane();
                                          // z is zero should be on plane
        assert_eq!(rect.on_plane(&Point3::new(2.0, 2.0, 0.0)).unwrap(), true)
    }

    #[test]
    fn trapezoid_off_plane(){
        let trap_points = [Point3::new(0.0, 0.0, 0.0), Point3::new(5.0,1.0,0.0), Point3::new(0.0,10.0,0.0) ,Point3::new(5.0, 9.0,0.0)];
        let tfm_matrix: na::Matrix4<f32>= na::Matrix4::new(1.0,5.0,7.0,2.0,  3.0,5.0,7.0,4.0,  8.0,4.0,1.0,9.0, 2.0,6.0,4.0,8.0);
        let mut rect = geometry::rectangle::Rectangle::new(trap_points, tfm_matrix).unwrap();
        rect.plane();
                                                        // z is non zero should be off plane
        assert_eq!(rect.on_plane(&Point3::new(2.0, 2.0, 4.0)).unwrap(), false)
    }

}