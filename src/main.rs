extern crate nalgebra as na;
use na::{Point2, Point3, Affine3};
mod geometry;
mod tests;


fn main() {
    let mut pts = [Point3::new(0.0, 0.0, 0.0),  Point3::new(0.0, 5.0,0.0), Point3::new(-2.0,4.0,0.0), Point3::new(3.0,5.0,0.0)];
    println!{"input is {:?}", pts}
    let ans = geometry::Traits::organize_points(&mut pts);
    println!{"{:?}", ans}
    println!{"{:?}", pts}
}


