
// reusable types to be referenced from functions;
use nalgebra as na;
pub type Real = f64;
pub type Vec3 = na::Vector3<Real>;

pub type P3 = na::Point3<Real>;
pub type P2 = na::Point2<Real>;

pub type Mat3 = na::Matrix3<Real>;
pub type Mat4 = na::Matrix4<Real>;
pub type Trf3 = na::Transform3<Real>;
pub type Trl3 = na::Translation3<Real>;
pub type Rot3 = na::Rotation3<Real>;
pub type Aff3 = na::Affine3<Real>;
