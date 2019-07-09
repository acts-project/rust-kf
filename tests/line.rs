use kalman_rs::geometry::trapezoid::Line;
use kalman_rs::config::*;


/* 

    kalman_rs::geometry::trapezoid::Line is used to represent the equation of a line
    since closures stored in a struct would need to be heap allocated. These are basic tests
    to make sure the calculation of the line is as expected. Not a lot to go wrong here.module_path!

*/

#[test]
fn line1 () {
    let p1 = P2::new(2.0, 2.0);
    let p2 = P2::new(0.0, 0.0);

    let line = Line::new_from_points(&p1, &p2);

    assert_eq!(line.slope, 1.0);
    assert_eq!(line.yint, 0.0);
}

#[test]
fn line2 () {
    let p1 = P2::new(2.0, 0.0);
    let p2 = P2::new(0.0, 2.0);

    let line = Line::new_from_points(&p1, &p2);

    assert_eq!(line.slope, -1.0);
    assert_eq!(line.yint, 2.0);
}

// reflect over y axis
#[test] 
fn line3 () {
    let p1 = P2::new(2.0, 0.0);
    let p2 = P2::new(0.0, 2.0);

    let line = Line::new_from_points(&p1, &p2);
    let reflection = Line::new_from_y_axis_reflection(&line);

    assert!(reflection.slope == 1.0);
    assert!(reflection.yint == 2.0);
}

// horiz line
#[test]
fn line4 () {
    let p1 = P2::new(0.0, 0.0);
    let p2 = P2::new(4.0, 0.0);

    let line = Line::new_from_points(&p1, &p2);
    
    assert!(line.slope ==0.0);
    assert!(line.yint == 0.0);
}