use super::super::config::*;

// TODO: Options enums for new values that are calculated: sin_phi_over_cos_theta... etc
#[derive(PartialOrd, PartialEq)]
pub struct Angles{
    pub cos_theta: Real,
    pub cos_phi: Real,
    pub sin_theta: Real,
    pub sin_phi: Real,
    pub tx: Option<Real>,
    pub ty: Option<Real>,
    pub tz: Option<Real>
}

/// Macro for expanding match expressions used in the lazy evaluation. Checks to see if an
/// option has previously been initialized, and if it has not it initializes it and returns 
/// the result

// $slf: since `.` in self.field makes it not an identifier, $slf captures `self` in the call
// $field '' captures the field name in the call (.tx, .ty, .tz)
// $calculation: the value that the expression will be initialied to should it be `None`
macro_rules! create_match {
    ($slf:ident.$field:ident, $calculation:expr) => {
        let mut data = $slf.$field
        match data{
            Some(field) => field,
            None=> {
                data = Some($calculation);
                $calculation
            }
        }
    };
}


impl Angles {
    pub fn new_from_unit_direction(tx: Real, ty: Real, tz: Real) -> Self {
        let cos_theta = tz;
        let sin_theta = ((tx* tx) + (ty*ty)).sqrt();
        
        let inverse_sin_theta = 1./sin_theta;
        let cos_phi = tx * inverse_sin_theta;
        let sin_phi = ty * inverse_sin_theta;

        // let theta =tz.acos();
        // let sin_theta = theta.sin();
        // let cos_theta = tz;

        // let cos_phi = tx / sin_theta;
        // let sin_phi = ty / sin_theta; 
        
        Angles {
            cos_theta: cos_theta,
            cos_phi: cos_phi,
            sin_theta: sin_theta,
            sin_phi: sin_phi,
            tx: Some(tx),
            ty: Some(ty),
            tz: Some(tz)
        }
    }
    pub fn new_from_angles(phi: Real, theta: Real) -> Self {
        Angles{
            cos_theta: theta.cos(),
            cos_phi: phi.cos(),
            sin_theta: theta.sin(),
            sin_phi: phi.sin(),
            tx: None,
            ty: None,
            tz: None
        }
    }
    pub fn tx(&mut self) -> Real {
        match self.tx {
            Some(tx) => tx,
            None => {
                self.tx = Some(self.cos_phi * self.sin_theta);
                self.tx()
            }
        }
    }
    pub fn tx_(&mut self) -> Real {
        create_match!{self.tx, self.cos_phi * self.sin_theta}
    }
    pub fn ty(&mut self) -> Real {
        create_match!{self.ty, self.sin_phi * self.sin_theta}
    }
    pub fn tz(&mut self) -> Real {
        create_match!{self.tz, self.cos_theta}
    }
}