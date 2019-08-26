//! Prelude of constants and types used in most modules.
//!
//! All Matrix types assume f64 type

use nalgebra::*;
pub use std::f64::consts::PI; // re-export pi

pub use nalgebra::{U1, U2, U3, U4, U5, U6, U7, U8};

// Plane / Transform traits that always need to be brought in for generics
pub use super::geometry::traits::*;

// reusable types to be referenced from functions;
pub type Real = f64;

/// Two row vector
pub type Vec2 = Vector2<Real>;
/// Three row vector
pub type Vec3 = Vector3<Real>;
/// Four row vector
pub type Vec4 = Vector4<Real>;
/// Five row vector
pub type Vec5 = Vector5<Real>;
/// Eight row vector
pub type Vec8 = VectorN<Real, U8>;

/// Point in R2
pub type P2 = Point2<Real>;
/// Point in R3
pub type P3 = Point3<Real>;

/// 2x2 Matrx
pub type Mat2 = Matrix2<Real>;
/// 3x3 Matrx
pub type Mat3 = Matrix3<Real>;
/// 4x4 Matrx
pub type Mat4 = Matrix4<Real>;
/// 5x5 Matrx
pub type Mat5 = Matrix5<Real>;
/// 8x8 Matrx
pub type Mat8 = MatrixMN<Real, U8, U8>;

/// 2x5 Matrx
pub type Mat2x5 = Matrix2x5<Real>;
/// 5x2 Matrx
pub type Mat5x2 = Matrix5x2<Real>;

/// 5x8 Matrx
pub type Mat5x8 = MatrixMN<Real, U5, U8>;
/// 8x5 Matrx
pub type Mat8x5 = MatrixMN<Real, U8, U5>;

/// 1x8 Matrx
pub type Mat1x8 = MatrixMN<Real, U1, U8>;
/// 1x5 Matrx
pub type Mat1x5 = MatrixMN<Real, U1, U5>;

/// 3D Transformation
pub type Trf3 = Transform3<Real>;
/// 3D Translation
pub type Trl3 = Translation3<Real>;
/// 3D Rotaiton
pub type Rot3 = Rotation3<Real>;
/// 3D Affine transformation
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
    DOT_PRODUCT_EPSILON = 0.0005,
    MAX_CHI_SQUARED = 1.0000
}

def_constant! {&str;
    CSV_SAVE_LOCATION = r"E:\kf_csvs\"
}
