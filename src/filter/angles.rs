//! Container for reducing the amount of re-computation of phi / theta
use super::super::config::*;

/// Container for reducing the amount of re-computation of phi / theta
#[derive(PartialOrd, PartialEq, Debug)]
pub struct Angles {
    pub cos_theta: Real,
    pub cos_phi: Real,
    pub sin_theta: Real,
    pub sin_phi: Real,
    pub direction: Vec3,
    pub tx: Real,
    pub ty: Real,
    pub tz: Real,
}

impl Angles {
    /// when calculating from Tx, Ty, Tz (global coordinate angles):
    ///
    /// ```
    /// use kalman_rs::filter::angles;
    /// let tx = 0.1;
    /// let ty = 0.2;
    /// let tz = 0.3;
    ///
    /// let angles = angles::Angles::new_from_unit_direction(tx, ty, tz);
    ///
    /// ```
    pub fn new_from_unit_direction(tx: Real, ty: Real, tz: Real) -> Self {
        let cos_theta = tz;
        let sin_theta = ((tx * tx) + (ty * ty)).sqrt();

        let inverse_sin_theta = 1. / sin_theta;
        let cos_phi = tx * inverse_sin_theta;
        let sin_phi = ty * inverse_sin_theta;

        Angles {
            cos_theta: cos_theta,
            cos_phi: cos_phi,
            sin_theta: sin_theta,
            sin_phi: sin_phi,
            direction: Vec3::new(tx, ty, tz),
            tx: tx,
            ty: ty,
            tz: tz,
        }
    }
    ///
    /// Used for calculating based on phi / theta values
    /// ```
    /// use kalman_rs::filter::angles;
    ///
    /// let phi = 3.1115;
    /// let theta = 3.115;
    ///
    /// let angles = angles::Angles::new_from_angles(phi, theta);
    /// ```
    pub fn new_from_angles(phi: Real, theta: Real) -> Self {
        let cos_phi = phi.cos();
        let sin_phi = phi.sin();
        let cos_theta = theta.cos();
        let sin_theta = theta.sin();

        let tx = cos_phi * sin_theta;
        let ty = sin_phi * sin_theta;
        let tz = cos_theta;

        Angles {
            cos_theta: cos_theta,
            cos_phi: cos_phi,
            sin_theta: sin_theta,
            sin_phi: sin_phi,
            direction: Vec3::new(tx, ty, tz),
            tx: tx,
            ty: ty,
            tz: tz,
        }
    }
}
