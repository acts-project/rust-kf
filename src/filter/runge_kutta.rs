const ERR_TOLERANCE: Real = 0.001;


use super::super::{
    config::*,
};

use super::{
    angles,
    utils,
};

#[derive(Debug)]
pub struct RungeKuttaStep {
    pub k1: Vec3,
    pub k2: Vec3,
    pub k3: Vec3,
    pub k4: Vec3,
    pub h: Real,
}

impl RungeKuttaStep {
    fn new(k1: Vec3, k2: Vec3, k3: Vec3, k4: Vec3, step_size: Real) -> Self {
        RungeKuttaStep {
            k1: k1,
            k2: k2,
            k3: k3,
            k4: k4,
            h: step_size,
        }
    }

    pub fn adaptive_step_size(&self, max_distance: Real) -> Real {
        let error = self.h.powf(2.) * (self.k1 - self.k2 - self.k3 + self.k4).norm();


        let next_step_unbounded = self.h * (ERR_TOLERANCE / error).powf(1./4.);

        let _temp = 4.*self.h;
        let max = 
            if  _temp > max_distance {_temp}
            else{max_distance};
        let min = 0.25 * self.h;

        // check the bounds to make sure we are not making the next step too large or small
        if next_step_unbounded > max {max}
        else if next_step_unbounded < min {min}
        else {next_step_unbounded}
    }
}

/// Adjusts the Runge-Kutta state vector *in place* based on the current state vector.
///NOTE: Since u and u' are 4-row vectors and k1-4 are 3-row vectors I interpret
///      This to mean the top 3 components of u and u' are mutated since there is no
///      energy loss. I am not entirely sure if this is correct.
pub fn rk_current_global_location(
    step_data: &RungeKuttaStep,
    previous_rk_state_vector: &mut Vec8,
) -> () {
    // print!{"before RK state adjust: ", previous_rk_state_vector}

    // adjust u
    let uprime_slice = previous_rk_state_vector.fixed_slice::<U3, U1>(4, 0); // <U3, U1> is a potential source of error. better ask about.
    let sum = (step_data.h * uprime_slice)
        + ((step_data.h * step_data.h / 6.) * (step_data.k1 + step_data.k2 + step_data.k3));

    let mut u_slice = previous_rk_state_vector.fixed_slice_mut::<U3, U1>(0, 0); // same here

    u_slice += sum;

    // adjust u'
    let mut uprime_slice = previous_rk_state_vector.fixed_slice_mut::<U3, U1>(4, 0);
    uprime_slice += (step_data.h / 6.)
        * (step_data.k1 + (2. * step_data.k2) + (2. * step_data.k3) + step_data.k4);

}

/// Perform a single step of RKN integration based on a given step size, magnetic field vector, state vector, and angles.
/// Note that the previous filtered state vector is only used for its QOP value. Unsure if this should change.
/// based on
/// https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/DefaultExtension.hpp#L45-59
/// https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/EigenStepper.ipp#L215-253
pub(crate) fn runge_kutta_step(
    prev_filt_state_vec: &Vec5,
    angles: &angles::Angles, // one-time time calculated angles
    b_field: &Vec3,          // magnetic field vector
    h: Real,                 // step size
) -> RungeKuttaStep {
    get_unchecked! {
        prev_filt_state_vec[eQOP] => qop
    }
    let qop = *qop;

    let dir = &angles.direction;
    let half_h = h / 2.;

    let k1 = qop * dir.cross(&b_field);

    let adj_k1 = dir + (half_h * k1);
    let k2 = qop * adj_k1.cross(&b_field);

    let adj_k2 = dir + (half_h * k2);
    let k3 = qop * adj_k2.cross(&b_field);

    let adj_k3 = dir + (h * k3);
    let k4 = qop * adj_k3.cross(&b_field);

    RungeKuttaStep::new(k1, k2, k3, k4, h)
}


/// Caculation of transport jacobian at a single time step
/// in a constant magnetic field.
pub fn constant_magnetic_transport(
    prev_filt_state_vec: &Vec5,
    step_data: &RungeKuttaStep,
    b_field: &Vec3,
    angles: &angles::Angles,
) -> Mat8 {
    get_unchecked! {
        prev_filt_state_vec[eQOP] => qop
    }
    let qop = *qop;

    let h = step_data.h;
    let half_h = h / 2.;
    let mut transport = Mat8::identity();
    let dir = angles.direction;

    let mut dk1_dt = Mat3::zeros();
    let mut dk2_dt = Mat3::identity();
    let mut dk3_dt = Mat3::identity();
    let mut dk4_dt = Mat3::identity();

    let dk1_dl = dir.cross(&b_field);

    let adjust = dir + (half_h * step_data.k1);
    let dk2_dl = adjust.cross(&b_field) + (qop * half_h * dk1_dl.cross(&b_field));

    let adjust = dir + (half_h * step_data.k2);
    let dk3_dl = adjust.cross(&b_field) + (qop * half_h * dk2_dl.cross(&b_field));

    let adjust = dir + (h * step_data.k3);
    let dk4_dl = adjust.cross(&b_field) + (qop * h * dk3_dl.cross(&b_field));

    edit_matrix! {dk1_dt;
        [0,1] =  b_field.z,
        [0,2] = -b_field.y,
        [1,0] = -b_field.z,
        [1,2] =  b_field.x,
        [2,0] =  b_field.y,
        [2,1] = -b_field.x
    }
    dk1_dt *= qop;

    dk2_dt += half_h * dk1_dt;
    utils::matrix_cross_product(&mut dk2_dt, &b_field);
    dk2_dt *= qop;

    dk3_dt += half_h * dk2_dt;
    utils::matrix_cross_product(&mut dk3_dt, &b_field);
    dk3_dt *= qop;

    dk4_dt += half_h * dk3_dt;
    utils::matrix_cross_product(&mut dk4_dt, &b_field);
    dk4_dt *= qop;

    // dF/dT
    let mut df_dt = transport.fixed_slice_mut::<U3, U3>(0, 4);
    df_dt.fill_with_identity();
    df_dt += (h / 6.) * (dk1_dt + dk2_dt + dk3_dt);
    df_dt *= h;

    // dF/dL
    let mut df_dl = transport.fixed_slice_mut::<U3, U1>(0, 7);
    let _temp = (h * h / 6.) * (dk1_dl + dk2_dl + dk3_dl);
    df_dl.copy_from(&_temp);

    // dG/dT
    let mut dg_dt = transport.fixed_slice_mut::<U3, U3>(4, 4);
    dg_dt += (h / 6.) * (dk1_dt + (2. * (dk2_dt + dk3_dt)) + dk4_dt);

    // dG/dL
    let mut dg_dl = transport.fixed_slice_mut::<U3, U1>(4, 7);
    let _temp = h / 6. * (dk1_dl + 2. * (dk2_dl + dk3_dl) + dk4_dl);
    dg_dl.copy_from(&_temp);

    // print!{"RK transport", transport}

    return transport;
}
