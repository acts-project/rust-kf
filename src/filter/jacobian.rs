use super::super::config;
use super::utils;
use config::*;
use super::super::geometry::traits::{Transform, Plane};

use config::*;
use nalgebra as na;

#[macro_use]
use super::macros;
use super::angles;

use super::prediction;


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

    let loc_2_glob : Mat8x5 = local_to_global_jac(&angles, end_sensor.rotation_to_global() );
    let glob_2_loc : Mat5x8= global_to_local_jac(&angles, start_sensor.rotation_to_local() ); 

    let transport_jac: Mat8 = linear_transport_jac(&mut angles, distance);

    // print!{"IN JACOBIAN", loc_2_glob, glob_2_loc, transport_jac};

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

    // neglects eT
    edit_matrix!{
        global_to_local_jacobian;
        [ePHI, 4] = -sin_phi_over_sin_theta,
        [ePHI, 5] = cos_phi_over_sin_theta,
        [eTHETA, 6] = -inv_sin_theta,
        [eQOP, 7] = 1.

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
    edit_matrix!{
        local_to_global_jacobian;
        // [3, eT] => 1.,     // This is from the ACTS code. This might go to zero (?) since time is not being tracked
        [4, ePHI] = (-trig_angles.sin_theta) * trig_angles.sin_phi,
        [4, eTHETA] = trig_angles.cos_theta * trig_angles.cos_phi,
        [5, ePHI] =  trig_angles.sin_theta * trig_angles.cos_phi,
        [5, eTHETA] = trig_angles.cos_theta * trig_angles.sin_phi,
        [6, eTHETA] =  -trig_angles.sin_theta,
        [7, eQOP] = 1.
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

    edit_matrix!{secondary;
        [0, 0] = distance * trig_angles.tx,
        [1,1] = distance * trig_angles.ty,
        [2,2] = distance * trig_angles.tz
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

    edit_matrix!{
        jac_to_curv;
        [0,0] = -ang.sin_phi,
        [0,1] = ang.cos_phi,
        [1,0] = ang.cos_phi * ang.cos_theta,
        [1,1] = ang.sin_phi * ang.cos_theta,
        [1,2] = ang.sin_theta,
        // ^^^^^^^ does not account for numerically unstable coordinate system
        // time parameter
        [5,3] = 1.,
        // direction  / momentum parameters for curvilinear
        [2,4] = ang.sin_phi * inv_sin_theta,
        [2,5] = ang.cos_phi * inv_sin_theta,
        [3,6] = -inv_sin_theta,
        [4,7] = 1.
    }

    /*

        local => global
        parallels https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/StraightLineStepper.hpp#L346-378
        
    */

    let mut jac_to_global = Mat8x5::zeros();

    edit_matrix!{
        jac_to_global;

        [0, eLOC_0] = -ang.sin_phi,
        [0, eLOC_1] = -ang.cos_phi * ang.cos_theta,

        [1, eLOC_0] = ang.cos_phi,
        [1, eLOC_1] = -ang.sin_phi * ang.cos_theta,

        [2, eLOC_1] = ang.sin_theta,
        [3, eT] = 1.,

        [4, ePHI] = -ang.sin_theta * ang.sin_phi,
        [4, eTHETA] = ang.cos_theta * ang.cos_phi,

        [5, ePHI] = ang.sin_theta *  ang.cos_phi,
        [5, eTHETA] = ang.cos_theta * ang.sin_phi,

        [6, eTHETA] = -ang.sin_theta,
        [7, eQOP] = 1.
    }

    let transport_jac = linear_transport_jac(&mut ang, distance);

    let full_jacobian = jac_to_curv *transport_jac * jac_to_global;

    return full_jacobian
}


pub fn constant_field<T: Transform + Plane>(
    prev_filt_state_vec: &Vec5,
    b_field: &Vec3,
    start_sensor: &T,
    end_sensor: &T
    )-> Mat5{
    //

    /*
        Build an angles struct that we will mutate depending on the global time step
    */

    get_unchecked!{vector; prev_filt_state_vec;
        eLOC_0 => x,
        eLOC_1 => y,
        ePHI => phi,
        eTHETA => theta,
        eQOP => qop
    }

    let local_point = P3::new(*x, *y, 0.);
    let global_point = start_sensor.to_global(local_point);

    let mut angles = angles::Angles::new_from_angles(*phi, *theta);


    /*
        build a 8x1 global state vector
    */

    let mut global_state_vec = Vec8::zeros();
    let mut u_slice =  global_state_vec.fixed_slice_mut::<U4, U1>(0,0);
    edit_matrix!{u_slice;
        [0,0] = global_point.x,
        [1,0] = global_point.y,
        [2,0] = global_point.z,
        [3,0] = 0.
    }

    let mut u_prime_slice = global_state_vec.fixed_slice_mut::<U4, U1>(4,0);
    edit_matrix!{u_prime_slice;
        [4,0] = angles.tx,
        [5,0] = angles.ty,
        [6,0] = angles.tz,
        [7,0] = *qop
    }


    let global_2_local_rotation = end_sensor.rotation_to_local();
    let local_2_global_rotation = start_sensor.rotation_to_global();


    /*
        
        initialize jacobians

    */

    let mut transport = Mat8::identity();
    let glob_2_loc = global_to_local_jac(&angles,&global_2_local_rotation);
    let loc_2_glob = local_to_global_jac(&angles,&local_2_global_rotation);


    /*
        Run runge-kutta loop until we are on the global place of the end sensor
    */

    // TODO: make this auto-adjust the stepsize
    let step_size : Real = 0.05;

    loop {
        
        // Runge-Kutta step data
        let step_data = runge_kutta_step(
            prev_filt_state_vec,
            &angles,
            b_field,
            step_size
        );

        // fetch transport between RK steps
        let transport_step = constant_magnetic_transport(
            prev_filt_state_vec,
            &step_data,
            b_field,
            &angles
        );

        // updates the global state vector in place
        prediction::rk_current_global_location(&step_data, &mut global_state_vec);

        let global_location = utils::global_point_from_rk_state(&global_state_vec);
        
        // if we have arrived at the ending sensor in the global place we stop
        if end_sensor.inside_global(global_location) {
            break
        } 
        // otherwise update the angles and transport jacobian
        else {
            angles = utils::angles_from_rk_state(&global_state_vec);
            transport = transport * transport_step;
        }

    }    

    let jacobian = glob_2_loc * transport * loc_2_glob;

    jacobian
}



/// Caculation of transport jacobian at a single time step 
/// in a constant magnetic field. 
pub fn constant_magnetic_transport(
    prev_filt_state_vec: &Vec5,
    step_data: &RungeKuttaStep,
    b_field: &Vec3,
    angles: &angles::Angles,
    ) -> Mat8{
    
    get_unchecked!{
        prev_filt_state_vec[eQOP] => qop
    }
    let qop = *qop;

    let h = step_data.h;
    let half_h = h / 2.;
    let mut transport = Mat8::zeros();
    let dir = angles.direction;


    let mut dk1dT = Mat3::zeros();
    let mut dk2dT = Mat3::identity();
    let mut dk3dT = Mat3::identity();
    let mut dk4dT = Mat3::identity();

    let dk1dL = dir.cross(&b_field);

    let adjust = dir + (half_h * step_data.k1);
    let dk2dL = adjust.cross(&b_field) + 
        (qop  * half_h * dk1dL.cross(&b_field));

    let adjust = dir + (half_h * step_data.k2);
    let dk3dL = adjust.cross(&b_field) + 
        (qop * half_h * dk2dL.cross(&b_field));

    let adjust = dir + (h * step_data.k3);
    let dk4dL = adjust.cross(&b_field) + 
        (qop * h * dk3dL.cross(&b_field));


    edit_matrix!{dk1dT;
        [0,1] =  b_field.z,
        [0,2] = -b_field.y,
        [1,0] = -b_field.z,
        [1,2] =  b_field.x,
        [2,0] =  b_field.y,
        [2,1] = -b_field.x
    }
    dk1dT *= qop;

    dk2dT += half_h * dk1dT;
    utils::matrix_cross_product(&mut dk2dT, &b_field);
    dk2dT *= qop;

    dk3dT += half_h * dk2dT;
    utils::matrix_cross_product(&mut dk3dT, &b_field);
    dk3dT *= qop;

    dk4dT += half_h * dk3dT;
    utils::matrix_cross_product(&mut dk4dT, &b_field);
    dk4dT *= qop;

    // dF/dT 
    let mut dFdT = transport.fixed_slice_mut::<U3, U3>(0, 4);
    dFdT.fill_with_identity();
    dFdT += (h/6.) * (dk1dT + dk2dT + dk3dT);
    dFdT *= h;


    // dF/dL
    let mut dFdL = transport.fixed_slice_mut::<U3, U1>(0,7);
    let _temp= (h * h / 6.)   * (dk1dL + dk2dL + dk3dL);
    dFdL.copy_from(&_temp);


    // dG/dT
    let mut dGdT = transport.fixed_slice_mut::<U3, U3>(4,4);
    dGdT += (h / 6.) * (dk1dT + (2. * (dk2dT + dk3dT)) + dk4dT);


    // dG/dL
    let mut dGdL = transport.fixed_slice_mut::<U3, U1>(4,7);
    let _temp = h / 6. * (dk1dL + 2. * (dk2dL + dk3dL) + dk4dL);
    dGdL.copy_from(&_temp);

    print!{"RK transport", transport}
    
    return transport
}


pub struct RungeKuttaStep{
    pub k1: Vec3,
    pub k2: Vec3,
    pub k3: Vec3,
    pub k4: Vec3,
    pub h: Real
}
impl RungeKuttaStep{
    fn new(k1: Vec3, k2: Vec3, k3: Vec3, k4: Vec3, step_size: Real)-> Self{
        RungeKuttaStep{
            k1: k1,
            k2: k2,
            k3: k3,
            k4: k4,
            h: step_size
        }
    }
}

// based on 
// https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/DefaultExtension.hpp#L45-59
// https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/EigenStepper.ipp#L215-253
fn runge_kutta_step(
    prev_filt_state_vec: &Vec5,
    angles: &angles::Angles,         // one-time time calculated angles
    b_field: &Vec3,                 // magnetic field vector
    h: Real                         // step size
    ) -> RungeKuttaStep {

    get_unchecked!{
        prev_filt_state_vec[eQOP] => qop
    }
    let qop = *qop;

    let dir = &angles.direction;
    let half_h = h/ 2.;

    let k1 = qop * dir.cross(&b_field);

    let adj_k1 = dir + (half_h * k1);
    let k2 = qop * adj_k1.cross(&b_field);

    let adj_k2 = dir + (half_h * k2);
    let k3  = qop * adj_k2.cross(&b_field);

    let adj_k3 = dir + (h * k3);
    let k4 = qop * adj_k3.cross(&b_field);


    RungeKuttaStep::new(k1, k2, k3, k4, h)
} 
