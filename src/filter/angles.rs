use super::super::config::*;

// TODO: Options enums for new values that are calculated: sin_phi_over_cos_theta... etc
#[derive(PartialOrd, PartialEq, Debug)]
pub struct Angles{
    pub cos_theta: Real,
    pub cos_phi: Real,
    pub sin_theta: Real,
    pub sin_phi: Real,
    pub direction: Vec3,
    pub tx: Real,
    pub ty: Real,
    pub tz: Real
}

impl Angles {
    pub fn new_from_unit_direction(tx: Real, ty: Real, tz: Real) -> Self {
        let cos_theta = tz;
        let sin_theta = ((tx* tx) + (ty*ty)).sqrt();
        
        let inverse_sin_theta = 1./sin_theta;
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
            tz: tz
        }
    }
    pub fn new_from_angles(phi: Real, theta: Real) -> Self {
        let cos_phi = phi.cos();
        let sin_phi = phi.sin();
        let cos_theta = theta.cos();
        let sin_theta = theta.sin();
        
        let tx = cos_phi * sin_theta;
        let ty = sin_phi * sin_theta;
        let tz = cos_theta;


        Angles{
            cos_theta: cos_theta,
            cos_phi: cos_phi,
            sin_theta: sin_theta,
            sin_phi: sin_phi,
            direction: Vec3::new(tx, ty, tz),
            tx: tx,
            ty: ty,
            tz: tz
        }
    }
}