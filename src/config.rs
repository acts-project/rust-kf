
// reusable types to be referenced from functions;
use nalgebra as na;
pub type Real = f64;

pub type Vec2 = na::Vector2<Real>;
pub type Vec3 = na::Vector3<Real>;
pub type Vec5 = na::Vector5<Real>;

pub type P2 = na::Point2<Real>;
pub type P3 = na::Point3<Real>;

pub type Mat2 = na::Matrix2<Real>;
pub type Mat3 = na::Matrix3<Real>;
pub type Mat4 = na::Matrix4<Real>;
pub type Mat5 = na::Matrix5<Real>;

pub type Mat2x5 =na::Matrix2x5<Real>;
pub type Mat5x2 =na::Matrix5x2<Real>;

pub type Trf3 = na::Transform3<Real>;
pub type Trl3 = na::Translation3<Real>;
pub type Rot3 = na::Rotation3<Real>;
pub type Aff3 = na::Affine3<Real>;


pub const DOT_PRODUCT_EPSILON : Real = 0.0005;