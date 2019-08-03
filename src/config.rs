use nalgebra::*;
pub use std::f64::consts::PI; // re-export pi

pub use nalgebra::{U1, U2, U3, U4, U5, U6, U7, U8};

// Plane / Transform traits that always need to be brought in for generics
pub use super::geometry::traits::*;

// reusable types to be referenced from functions;
pub type Real = f64;

pub type Vec2 = Vector2<Real>;
pub type Vec3 = Vector3<Real>;
pub type Vec4 = Vector4<Real>;
pub type Vec5 = Vector5<Real>;
pub type Vec8 = VectorN<Real, U8>;

pub type P2 = Point2<Real>;
pub type P3 = Point3<Real>;

pub type Mat2 = Matrix2<Real>;
pub type Mat3 = Matrix3<Real>;
pub type Mat4 = Matrix4<Real>;
pub type Mat5 = Matrix5<Real>;
pub type Mat8 = MatrixMN<Real, U8, U8>;

pub type Mat2x5 = Matrix2x5<Real>;
pub type Mat5x2 = Matrix5x2<Real>;

pub type Mat5x8 = MatrixMN<Real, U5, U8>;
pub type Mat8x5 = MatrixMN<Real, U8, U5>;

pub type Mat1x8 = MatrixMN<Real, U1, U8>;
pub type Mat1x5 = MatrixMN<Real, U1, U5>;

pub type Trf3 = Transform3<Real>;
pub type Trl3 = Translation3<Real>;
pub type Rot3 = Rotation3<Real>;
pub type Aff3 = Affine3<Real>;

// uszie constants
def_constant! {usize;
    eLOC_0 = 0,
    eLOC_1 = 1,
    ePHI = 2,
    eTHETA = 3,
    eQOP = 4,
    eT = 5
}

// f64 constants
def_constant! {Real;
    DOT_PRODUCT_EPSILON = 0.0005
}

def_constant! {&str;
    CSV_SAVE_LOCATION = r"E:\kf_csvs\"

}
