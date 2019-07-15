use super::super::config;
use super::super::geometry::traits::{Transform, Plane};

use config::*;
use nalgebra as na;

#[macro_use]
use super::macros;
use super::angles;


/// Calculate the jacobian between sensors for a linear case
pub fn linear<T: Transform + Plane>(
    prev_state_vec: &Vec5,
    distance: Real,
    start_sensor: &T,
    end_sensor: &T
    ) -> Mat5{


    get_unchecked!{
        prev_state_vec[ePHI] => phi,
        prev_state_vec[eTHETA] => theta
    }


    // struct containing all the required sin / cos of phi / theta
    let mut angles = angles::Angles::new_from_angles(*phi, *theta);

    // dbg!{&angles};

        ////////////////////////////////////////////////// CHANGE ME BAKC TO START SENSOR AND END SENSOR CORRECTLY!!!!!!!!
        ////////////////////////////////////////////////// CHANGE ME BAKC TO START SENSOR AND END SENSOR CORRECTLY!!!!!!!!
        ////////////////////////////////////////////////// CHANGE ME BAKC TO START SENSOR AND END SENSOR CORRECTLY!!!!!!!!
        ////////////////////////////////////////////////// CHANGE ME BAKC TO START SENSOR AND END SENSOR CORRECTLY!!!!!!!!
        ////////////////////////////////////////////////// CHANGE ME BAKC TO START SENSOR AND END SENSOR CORRECTLY!!!!!!!!
        ///                                                 . .. . . . .. . . . . 


    // let loc_2_glob : Mat8x5 = local_to_global_jac(&angles, end_sensor.rotation_to_global() );
    // let glob_2_loc : Mat5x8= global_to_local_jac(&angles, start_sensor.rotation_to_local() ); 
    let loc_2_glob : Mat8x5 = local_to_global_jac(&angles, start_sensor.rotation_to_local() );
    let glob_2_loc : Mat5x8= global_to_local_jac(&angles, end_sensor.rotation_to_global() ); 
    let transport_jac: Mat8 = linear_transport_jac(&mut angles, distance);

    print!{"IN JACOBIAN", loc_2_glob, glob_2_loc, transport_jac};

    return glob_2_loc * transport_jac * loc_2_glob
    
}


/// Local => global jacobian used for both linear and constant magnetic field situaitons
/// https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Surfaces/detail/Surface.ipp#L82-106
fn global_to_local_jac(
    trig_angles: &angles::Angles,
    rotation_mat: &Mat4
    ) -> Mat5x8 {

    let mut global_to_local_jacobian = Mat5x8::zeros();

    let mut g2l_slice = global_to_local_jacobian.fixed_slice_mut::<U2, U3>(0,0);
    
    let rot_mat_transposed = rotation_mat.transpose();
    let rot_slice = rot_mat_transposed.fixed_slice::<U2, U3>(0,0);

    g2l_slice.copy_from(&rot_slice);


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
    trig_angles: &angles::Angles,
    rotation_mat: &Mat4
    ) -> Mat8x5{

    let mut local_to_global_jacobian = Mat8x5::zeros();

    let mut l2g_slice = local_to_global_jacobian.fixed_slice_mut::<U3, U2>(0,0);
    let rot_slice = rotation_mat.fixed_slice::<U3, U2>(0,0);

    l2g_slice.copy_from(&rot_slice);

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

    let transport_jac = Mat8::identity(); 
    let mut secondary= Mat8::zeros();

    change_mat_val!{secondary;
        [0, 0] => distance * trig_angles.tx(),
        [1,1] => distance * trig_angles.ty(),
        [2,2] => distance * trig_angles.tz()
        // since the other values across the diagonal are 1 and we transport_jac is a identity matrix we leave it here
    }

    return transport_jac + secondary

}


// This is a secondary function for calculating the linear jacobian to test different implementations. 

/// Mirrors https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/StraightLineStepper.hpp#L307
/// Does not include shift from state derivative calculated https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/StraightLineStepper.hpp#L353
/// Transport is handled outside of function.
pub fn linear_state_derivative(
    prev_state_vec: &Vec5,
    distance: Real
    ) -> Mat5 {

    
    get_unchecked!{
        prev_state_vec[ePHI] => phi,
        prev_state_vec[eTHETA] => theta
    }


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