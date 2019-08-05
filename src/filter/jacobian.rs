use super::super::{
    config::*,
    geometry::traits::{Plane, Transform},
};

use nalgebra as na;

use super::{
    angles,
    runge_kutta,
    utils,
};

/// Calculate the jacobian between sensors for a linear case
pub fn linear<T: Transform + Plane>(
    prev_state_vec: &Vec5,
    distance: Real,
    start_sensor: &T,
    end_sensor: &T,
) -> Mat5 {
    get_unchecked! {
        prev_state_vec[ePHI] => phi,
        prev_state_vec[eTHETA] => theta
    }

    // struct containing all the required sin / cos of phi / theta
    let mut angles = angles::Angles::new_from_angles(*phi, *theta);

    let loc_2_glob: Mat8x5 = local_to_global_jac(&angles, end_sensor.rotation_to_global());
    let glob_2_loc: Mat5x8 = global_to_local_jac(&angles, start_sensor.rotation_to_local());

    let transport_jac: Mat8 = linear_transport_jac(&mut angles, distance);

    // calculate the effects of the derivative factors
    let transport_and_to_global: Mat8x5 = transport_jac * loc_2_glob;

    let deriv_factors = derivative_factors(
        &angles,
        &transport_and_to_global,
        end_sensor.rotation_to_global(),
    );
    let oath_len_derivative = utils::oath_length_derivatives(&angles);
    let transport_and_to_global: Mat8x5 =
        transport_and_to_global - (oath_len_derivative * deriv_factors);

    // return glob_2_loc * transport_and_to_global;

    return glob_2_loc * transport_jac * loc_2_glob;
}

// https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/StraightLineStepper.hpp#L394
fn derivative_factors(
    angles: &angles::Angles,
    transport_2_glob: &Mat8x5,
    rotation_mat: &Mat4,
) -> Mat1x5 {
    //

    // I _think_ this is supposed to be transposed based on the naming but im not sure
    let rt = rotation_mat.transpose();
    let norm = rt.fixed_slice::<U1, U3>(2, 0);

    // we have to use _temp since nalgbra wont let us divide by a 1x1 matrix
    let _temp = norm * angles.direction;
    get_unchecked! { _temp[0] => prod }
    let norm = norm / *prod;

    let jac_slice = transport_2_glob.fixed_slice::<U3, U5>(0, 0);

    return norm * jac_slice;
}

/// Local => global jacobian used for both linear and constant magnetic field situaitons
/// https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Surfaces/detail/Surface.ipp#L82-106
fn global_to_local_jac(trig_angles: &angles::Angles, rotation_mat: &Mat4) -> Mat5x8 {
    let mut global_to_local_jacobian = Mat5x8::zeros();

    let mut g2l_slice = global_to_local_jacobian.fixed_slice_mut::<U2, U3>(0, 0);

    let rot_mat_transposed = rotation_mat.transpose();
    let rot_slice = rot_mat_transposed.fixed_slice::<U2, U3>(0, 0);

    g2l_slice.copy_from(&rot_slice);

    let inv_sin_theta = 1. / trig_angles.sin_theta;

    let sin_phi_over_sin_theta = trig_angles.sin_phi / trig_angles.sin_theta;
    let cos_phi_over_sin_theta = trig_angles.cos_phi / trig_angles.sin_theta;

    // neglects eT
    edit_matrix! {
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
fn local_to_global_jac(trig_angles: &angles::Angles, rotation_mat: &Mat4) -> Mat8x5 {
    let mut local_to_global_jacobian = Mat8x5::zeros();

    let mut l2g_slice = local_to_global_jacobian.fixed_slice_mut::<U3, U2>(0, 0);
    let rot_slice = rotation_mat.fixed_slice::<U3, U2>(0, 0);

    l2g_slice.copy_from(&rot_slice);

    // add values into transport jacobian
    edit_matrix! {
        local_to_global_jacobian;
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

fn linear_transport_jac(trig_angles: &mut angles::Angles, distance: Real) -> Mat8 {
    let transport_jac = Mat8::identity();
    let mut secondary = Mat8::zeros();

    edit_matrix! {secondary;
        [0, 0] = distance * trig_angles.tx,
        [1,1] = distance * trig_angles.ty,
        [2,2] = distance * trig_angles.tz
        // since the other values across the diagonal are 1 and we transport_jac is a identity matrix we leave it here
    }

    return transport_jac + secondary;
}

// This is a secondary function for calculating the linear jacobian to test different implementations.

/// Mirrors https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/StraightLineStepper.hpp#L307
/// Does not include shift from state derivative calculated https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/StraightLineStepper.hpp#L353
/// Transport is handled outside of function.
pub fn linear_state_derivative(prev_state_vec: &Vec5, distance: Real) -> Mat5 {
    get_unchecked! {
        prev_state_vec[ePHI] => phi,
        prev_state_vec[eTHETA] => theta
    }

    let mut ang = angles::Angles::new_from_angles(*phi, *theta);

    let inv_sin_theta = 1. / ang.sin_theta;

    /*

        global => local
        parallels https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Propagator/StraightLineStepper.hpp#L322-344

    */
    let mut jac_to_curv = Mat5x8::zeros();

    edit_matrix! {
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

    edit_matrix! {
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

    let full_jacobian = jac_to_curv * transport_jac * jac_to_global;

    return full_jacobian;
}

pub fn constant_field<T: Transform + Plane>(
    prev_filt_state_vec: &Vec5,
    b_field: &Vec3,
    start_sensor: &T,
    end_sensor: &T,
) -> (Mat5, Vec5) {
    //

    let (mut global_state_vec, mut angles) =
        utils::local_to_global_state_vector(prev_filt_state_vec, start_sensor);

    let global_2_local_rotation = end_sensor.rotation_to_local();
    let local_2_global_rotation = start_sensor.rotation_to_global();

    /*

        initialize jacobians

    */

    let mut transport = Mat8::identity();
    let glob_2_loc = global_to_local_jac(&angles, &global_2_local_rotation);
    let loc_2_glob = local_to_global_jac(&angles, &local_2_global_rotation);

    /*
        Run runge-kutta loop until we are on the global place of the end sensor
    */

    // TODO: make this auto-adjust the stepsize
    // let step_size: Real = 0.0001;
    let step_size: Real = 0.00001;

    loop {

        // Runge-Kutta step data
        let step_data = runge_kutta::runge_kutta_step(prev_filt_state_vec, &angles, b_field, step_size);

        // fetch transport between RK steps
        let transport_step =
            runge_kutta::constant_magnetic_transport(prev_filt_state_vec, &step_data, b_field, &angles);

        // print!{transport_step}

        // updates the global state vector in place
        runge_kutta::rk_current_global_location(&step_data, &mut global_state_vec);

        // pull the global point from the global state vector
        let global_location = utils::global_point_from_rk_state(&global_state_vec);

        angles = utils::angles_from_rk_state(&global_state_vec);
        transport = transport * transport_step;

        // TODO: REMOVE ME !!!
        if global_location.x < 0. {
            panic! {"we are going backwards for some reason"}
        }

        // if we have arrived at the ending sensor in the global place we stop
        if end_sensor.on_plane(&global_location) {
            break;
        }

    }

    let (local_sv_prediction, _) =
        utils::global_to_local_state_vector(&global_state_vec, end_sensor);

    let jacobian = glob_2_loc * transport * loc_2_glob;

    (jacobian, local_sv_prediction)
}


