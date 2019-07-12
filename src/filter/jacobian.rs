use super::super::config;
use config::*;

use nalgebra as na;

#[macro_use]
use super::macros;
use super::angles;



/// Calculate the jacobian between sensors for a linear case
pub fn linear(
    prev_state_vec: &Vec5,
    distance: Real
    ) -> Mat5{

    // dbg!{"IN JACOBIAN"};

    get_unchecked!{
        prev_state_vec[ePHI] => phi,
        prev_state_vec[eTHETA] => theta
    }


    // struct containing all the required sin / cos of phi / theta
    let mut angles = angles::Angles::new_from_angles(*phi, *theta);

    // dbg!{&angles};


    let loc_2_glob : Mat8x5 = local_to_global_jac(&angles);
    let glob_2_loc : Mat5x8= global_to_local_jac(&angles);
    let transport_jac: Mat8 = linear_transport_jac(&mut angles, distance);


    return glob_2_loc * transport_jac * loc_2_glob
    
}


/// Local => global jacobian used for both linear and constant magnetic field situaitons
/// https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Surfaces/detail/Surface.ipp#L82-106
fn global_to_local_jac(
    trig_angles: &angles::Angles
    ) -> Mat5x8 {

    let mut global_to_local_jacobian = Mat5x8::zeros();

    let inv_sin_theta = 1. / trig_angles.sin_theta;

    let sin_phi_over_sin_theta = trig_angles.sin_phi / trig_angles.sin_theta;
    let cos_phi_over_sin_theta = trig_angles.cos_phi / trig_angles.sin_theta;

    change_mat_val!{
        global_to_local_jacobian;
        // [eT, 3] => 1.,   out of bounds
        [ePHI, 4] => -sin_phi_over_sin_theta,
        [ePHI, 5] => cos_phi_over_sin_theta,
        [eTHETA, 6] => -inv_sin_theta,
        [eQOP, 7] => 1.

    }

    global_to_local_jacobian

}

/// global => local jacobian used for both linear and constant magnetic field situaitons
/// https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Surfaces/detail/Surface.ipp#L46-80
fn local_to_global_jac(
    trig_angles: &angles::Angles
    ) -> Mat8x5{

    let mut local_to_global_jacobian = Mat8x5::zeros();

    // add values into transport jacobian
    change_mat_val!{
        local_to_global_jacobian;
        // [3, eT] => 1.,     // This is from the ACTS code. This might go to zero (?) since time is not being tracked
        [4, ePHI] => (-trig_angles.sin_theta) * trig_angles.sin_phi,
        [4, eTHETA] => trig_angles.cos_theta * trig_angles.cos_phi,
        [5, ePHI] =>  trig_angles.sin_theta * trig_angles.cos_phi,
        [5, eTHETA] => trig_angles.cos_theta * trig_angles.sin_phi,
        [6, eTHETA] =>  -trig_angles.sin_theta,
        [7, eQOP] => 1.
    }

    // dbg!{local_to_global_jacobian};
    local_to_global_jacobian
}

fn linear_transport_jac(
    trig_angles: &mut angles::Angles,
    distance: Real
    ) -> Mat8{

    let mut transport_jac = Mat8::identity(); 

    change_mat_val!{transport_jac;
        [0, 0] => distance * trig_angles.tx,
        [1,1] => distance * trig_angles.ty,
        [2,2] => distance * trig_angles.tz
        // since the other values across the diagonal are 1 and we transport_jac is a identity matrix we leave it here
    }


    transport_jac
}


// This is a secondary function for calculating the linear jacobian to test different implementations. 

/// Mirrors https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/StraightLineStepper.hpp#L307
/// Does not include shift from state derivative calculated https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/StraightLineStepper.hpp#L353
/// Transport is handled outside of function.
pub fn linear_state_derivative(
    prev_state_vec: &Vec5,
    distance: Real
    ) -> Mat5 {

    dbg!{"before"};
    
    get_unchecked!{
        prev_state_vec[ePHI] => phi,
        prev_state_vec[eTHETA] => theta
    }

    dbg!{"made past here"};

    let mut ang = angles::Angles::new_from_angles(*phi, *theta);

    let inv_sin_theta = 1./ang.sin_theta;

    /*

        global => local
        parallels https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/StraightLineStepper.hpp#L322-344

    */
    let mut jac_to_curv = Mat5x8::zeros();

    change_mat_val!{
        jac_to_curv;
        [0,0] => -ang.sin_phi,
        [0,1] => ang.cos_phi,
        [1,0] => ang.cos_phi * ang.cos_theta,
        [1,1] => ang.sin_phi * ang.cos_theta,
        [1,2] => ang.sin_theta,
        // ^^^^^^^ does not account for numerically unstable coordinate system
        // time parameter
        [5,3] => 1.,
        // direction  / momentum parameters for curvilinear
        [2,4] => ang.sin_phi * inv_sin_theta,
        [2,5] => ang.cos_phi * inv_sin_theta,
        [3,6] => -inv_sin_theta,
        [4,7] => 1.
    }

    /*

        local => global
        parallels https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/StraightLineStepper.hpp#L346-378
        
    */

    let mut jac_to_global = Mat8x5::zeros();

    change_mat_val!{
        jac_to_global;

        [0, eLOC_0] => -ang.sin_phi,
        [0, eLOC_1] => -ang.cos_phi * ang.cos_theta,

        [1, eLOC_0] => ang.cos_phi,
        [1, eLOC_1] => -ang.sin_phi * ang.cos_theta,

        [2, eLOC_1] => ang.sin_theta,
        [3, eT] => 1.,

        [4, ePHI] => -ang.sin_theta * ang.sin_phi,
        [4, eTHETA] => ang.cos_theta * ang.cos_phi,

        [5, ePHI] => ang.sin_theta *  ang.cos_phi,
        [5, eTHETA] => ang.cos_theta * ang.sin_phi,

        [6, eTHETA] => -ang.sin_theta,
        [7, eQOP] => 1.
    }

    let transport_jac = linear_transport_jac(&mut ang, distance);

    let full_jacobian = jac_to_curv *transport_jac * jac_to_global;

    return full_jacobian
}


    // dk1dT(0, 1) = sd.B_first.z();
    // dk1dT(0, 2) = -sd.B_first.y();
    // dk1dT(1, 0) = -sd.B_first.z();
    // dk1dT(1, 2) = sd.B_first.x();
    // dk1dT(2, 0) = sd.B_first.y();
    // dk1dT(2, 1) = -sd.B_first.x();


pub fn constant_magnetic_transport(
    prev_filt_state_vec: &Vec5,
    step_data: &RungeKutta,
    b_field: &Vec3,
    angles: &angles::Angles,
    step_size: Real

    ) -> Mat8{
    
    get_unchecked!{
        prev_filt_state_vec[eQOP] => qop
    }
    let qop = *qop;

    let half_step_size = step_size / 2.;
    let mut transport = Mat8::zeros();
    let dir = angles.direction;

    submatrix!{transport;
        0..4 , 0..4 => dFdT,
        0..4 , 4..8 => dFdL,
        4..8 , 0..4 => dGdT,
        4..8 , 4..8 => dGdL
    }

    let mut dk1dT = Mat4::zeros();
    let mut dk2dT = Mat4::identity();
    let mut dk3dT = Mat4::identity();
    let mut dk4dT = Mat4::identity();

    let dk1dL = dir.cross(&b_field);

    let adjust = dir + (half_step_size * step_data.k1);
    let dk2dL = adjust.cross(&b_field) + 
        (qop  * half_step_size * dk1dL.cross(&b_field));

    let adjust = dir + (half_step_size * step_data.k2);
    let dk3dL = adjust.cross(&b_field) + 
        (qop * half_step_size * dk2dL.cross(&b_field));

    let adjust = dir + (step_size * step_data.k3);
    let dk4dL = adjust.cross(&b_field) + 
        (qop * step_size * dk3dL.cross(&b_field));


    
    
    
    unimplemented!()
}


pub struct RungeKutta{
    k1: Vec3,
    k2: Vec3,
    k3: Vec3,
    k4: Vec3
}
impl RungeKutta{
    fn new(k1: Vec3, k2: Vec3, k3: Vec3, k4: Vec3)-> Self{
        RungeKutta{
            k1: k1,
            k2: k2,
            k3: k3,
            k4: k4
        }
    }
}

// based on 
// https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/DefaultExtension.hpp#L45-59
// https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/EigenStepper.ipp#L215-253
fn runge_kutta_step(
    prev_filt_state_vec: &Vec5,
    angles: angles::Angles,         // one-time time calculated angles
    b_field: &Vec3,                 // magnetic field vector
    h: Real                         // step size
    ) -> RungeKutta {

    get_unchecked!{
        prev_filt_state_vec[eQOP] => qop
    }

    let dir = angles.direction;
    let half_h = h/ 2.;

    let k1 = *qop * dir.cross(&b_field);

        //          qop * (stepper.direction(state.stepping) + h * kprev).cross(bField);
    let adj_k1 = dir + (half_h * k1);
    let k2 = *qop * adj_k1.cross(&b_field);

    let adj_k2 = dir + (half_h * k2);
    let k3  = *qop * adj_k2.cross(&b_field);

    let adj_k3 = dir + (h * k3);
    let k4 = *qop * adj_k3.cross(&b_field);


    RungeKutta::new(k1, k2, k3, k4)
} 