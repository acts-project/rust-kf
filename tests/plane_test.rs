use kalman_rs;

#[cfg(test)]
mod plane_tests{
    use kalman_rs::geometry;
    use kalman_rs::geometry::traits::Plane;
    use nalgebra as na;
    use kalman_rs::config::*;

    #[test]
    fn rectangle_on_plane() {
        let rect_points = [P3::new(0.0, 0.0, 0.0), P3::new(5.0,0.0,0.0), P3::new(0.0,5.0,0.0), P3::new(5.0,5.0,0.0)];
        let tfm_matrix= Mat4::new(1.0,5.0,7.0,2.0,  3.0,5.0,7.0,4.0,  8.0,4.0,1.0,9.0, 2.0,6.0,4.0,8.0);
        let mut rect = geometry::rectangle::Rectangle::new(rect_points, tfm_matrix).unwrap();
        rect.plane();

        assert_eq!(rect.on_plane(&P3::new(1.0, 1.0, 0.0)).unwrap(), true)
    }

    #[test]
    fn rectangle_off_plane() {
        let rect_points = [P3::new(0.0, 0.0, 0.0), P3::new(5.0,0.0,0.0), P3::new(0.0,5.0,0.0), P3::new(5.0,5.0,0.0)];
        let tfm_matrix = Mat4::new(1.0,5.0,7.0,2.0,  3.0,5.0,7.0,4.0,  8.0,4.0,1.0,9.0, 2.0,6.0,4.0,8.0);
        let mut rect = geometry::rectangle::Rectangle::new(rect_points, tfm_matrix).unwrap();
        rect.plane();

        assert_eq!(rect.on_plane(&P3::new(1.0, 1.0, 1.0)).unwrap(), false)
    }

    #[test]
    fn trapezoid_on_plane(){
        let trap_points = [P3::new(0.0, 0.0, 0.0), P3::new(5.0,1.0,0.0), P3::new(0.0,10.0,0.0) ,P3::new(5.0, 9.0,0.0)];
        let tfm_matrix = Mat4::new(1.0,5.0,7.0,2.0,  3.0,5.0,7.0,4.0,  8.0,4.0,1.0,9.0, 2.0,6.0,4.0,8.0);
        let mut rect = geometry::trapezoid::Trapezoid::new(trap_points, tfm_matrix).unwrap();
        rect.plane();
                                          // z is zero should be on plane
        assert_eq!(rect.on_plane(&P3::new(2.0, 2.0, 0.0)).unwrap(), true)
    }

    #[test]
    fn trapezoid_off_plane(){
        let trap_points = [P3::new(0.0, 0.0, 0.0), P3::new(5.0,1.0,0.0), P3::new(0.0,10.0,0.0) ,P3::new(5.0, 9.0,0.0)];
        let tfm_matrix =  Mat4::new(1.0,5.0,7.0,2.0,  3.0,5.0,7.0,4.0,  8.0,4.0,1.0,9.0, 2.0,6.0,4.0,8.0);
        let mut rect = geometry::rectangle::Rectangle::new(trap_points, tfm_matrix).unwrap();
        rect.plane();
                                                        // z is non zero should be off plane
        assert_eq!(rect.on_plane(&P3::new(2.0, 2.0, 4.0)).unwrap(), false)
    }

}