use super::super::config;
use config::*;

use nalgebra::*;

#[macro_use]
use super::macros;

pub fn linear(
    prev_state_vec: &Vec5
    ) -> Mat5{

    let mut transport_jac = Mat8::zeros();
    get_unchecked!{
        prev_state_vec[ePHI] => phi,
        prev_state_vec[eTHETA] => theta,
        prev_state_vec[eQOP] => momentum
    }

    let sin_theta = theta.sin();
    let cos_theta = theta.cos();
    let sin_phi = phi.sin();
    let cos_phi = phi.cos();

    // add values into transport jacobian
    // Parallels https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Surfaces/detail/Surface.ipp#L74-79
    change_mat_val!{
        transport_jac: 8;
        [3, eT] => 1.,     // This is from the ACTS code. This might go to zero (?) since time is not being tracked
        [4, ePHI] => (-sin_theta) * sin_phi,
        [4, eTHETA] => cos_theta * cos_phi,
        [5, ePHI] => sin_theta * cos_phi,
        [5, eTHETA] => cos_theta * sin_phi,
        [6, eTHETA] =>  -sin_theta,
        [7, eQOP] => 1.
    }

    let mut local_to_global_jacobian = Mat5x8::zeros();

    let inv_sin_theta = theta.asin();
    let sin_phi_over_sin_theta = sin_phi / sin_theta;
    let cos_phi_over_sin_theta = cos_phi / sin_theta;

    change_mat_val!{
        local_to_global_jacobian : 8;
        [eT, 3] => 1.,
        [ePHI, 4] => -sin_phi_over_sin_theta,
        [ePHI, 5] => cos_phi_over_sin_theta,
        [eTHETA, 6] => -inv_sin_theta,
        [eQOP, 7] => 1.
    }

    unimplemented!()
}
