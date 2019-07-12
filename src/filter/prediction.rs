use super::super::config::*;
use super::super::error::*;
use super::super::geometry::traits::{Plane, Transform};
use super::super::geometry::*;
use super::angles;

#[macro_use]
use super::macros;

// extrapolating state vector
// NOTE: this can only be used for linear systems
pub fn state_vector (
    jacobian: &Mat5,               // J or F_k-1
    prev_filt_state_vec: &Vec5     // prev filt x
    ) -> Vec5 {                    // pred x

    return jacobian * prev_filt_state_vec
}

// prediction of covariance matrix C
pub fn covariance_matrix(
    jacobian: &Mat5,                    // J or F_k-1
    prev_filt_covariance_mat: &Mat5     // prev filt C
    )-> Mat5{                           // pred C

    return jacobian * prev_filt_covariance_mat * jacobian.transpose()
}

// just below eq. 7
// residual covariance of predicted results
pub fn residual_mat(
    V: &Mat2,                       // V
    sensor_mapping_mat: &Mat2x5,    // H
    pred_covariance_mat: &Mat5      // pred C
    ) -> Mat2 {                     // pred R
        
    return V + (sensor_mapping_mat*pred_covariance_mat * sensor_mapping_mat.transpose())
}

pub fn residual_vec(
    measurement_vec: &Vec2,         // m_k
    sensor_mapping_mat: &Mat2x5,    // H
    pred_state_vec: &Vec5           // pred x
    ) -> Vec2 {                     // pred r

    let prod = sensor_mapping_mat * pred_state_vec;
    let diff = measurement_vec - prod;

    return diff;
}

/// Calculates the predicted location of the hit on the following sensor
// based on this equation set https://i.imgur.com/mWC0qkj.png
pub fn linear_state_vector<T: Transform + Plane>(
    start_sensor: &T, 
    end_sensor: &T, 
    prev_filt_state_vec: &Vec5,
    ) -> Result<(Vec5, Real), SensorError> {

    // println!{"IN PREDICTION: filtered state vec:"}
    // dbg!{prev_filt_state_vec};
    
    get_unchecked!{
        prev_filt_state_vec[eLOC_0] => start_local_x_hit,
        prev_filt_state_vec[eLOC_1] => start_local_y_hit,
        prev_filt_state_vec[eTHETA] => theta,
        prev_filt_state_vec[ePHI] => phi
    }

    let mut ang = angles::Angles::new_from_angles(*phi, *theta);

    let start_local_point = P3::new(*start_local_x_hit, *start_local_y_hit, 0.0);
    let start_global_point = start_sensor.to_global(start_local_point);


    // used so we can be generic over planar sensors
    let normal = end_sensor.plane_normal_vec();

    let end_plane_constant = end_sensor.plane_constant();

    // dbg!{end_plane_constant};

    // find predicted 
    let (global_pred_point, global_distance) = 
        linear_global_hit_estimation(
            &normal,
            &start_global_point,
            &mut ang,
            &end_plane_constant
        );

    let local_pred_point  = end_sensor.to_local(global_pred_point);

    // println!{"global predicted point:"}
    // dbg!{global_pred_point};
    // println!{"local predicted point: "}
    // dbg!{local_pred_point};


    // check if the predicted point is on the sensor
    if end_sensor.inside(&local_pred_point) {
        // might be able to avoid cloning here
        let mut new_state_vec = prev_filt_state_vec.clone();

        edit_matrix!{new_state_vec;
            [eLOC_0] = local_pred_point.x,
            [eLOC_1] = local_pred_point.y
        }
        // println!{"full predicted state vector"}
        // dbg!{&new_state_vec};

        Ok((new_state_vec, global_distance))
    }
    else {
        Err(SensorError::OutsideSensorBounds)
    }
}

pub fn constant_magnetic_state_vector<T: Transform + Plane>(
    start_sensor: &T, 
    end_sensor: &T, 
    prev_filt_state_vec: &Vec5,
    magnetic_field : &Vec3
    ) -> Result<Vec5, SensorError> {

    // fetch different indexes from the same vector
    get_unchecked!{vector; prev_filt_state_vec;
        eLOC_0 => x_loc,
        eLOC_1 => y_loc,
        ePHI => phi,
        eTHETA => theta,
        eQOP => qop
    };

    let cos_phi = phi.cos();
    let sin_phi = phi.sin();
    let cos_theta = theta.cos();
    let sin_theta = theta.sin();

    let tx = cos_phi * sin_theta;
    let ty = sin_phi * sin_theta;
    let tz = cos_theta;
    let direction = Vec4::new(tx, ty, tz, 0.);

    // hard coding this for now
    let step_size : Real = 0.05;

    // convert filtered local to hit to global position
    let start_local_point = P3::new(*x_loc, *y_loc, 0.);
    let starting_global_location = start_sensor.to_global(start_local_point);

    // pull x y z coordinates from global location
    get_unchecked!{vector;starting_global_location;
        0=>x,
        1=>y,
        2=>z
    }

    // create new
    let mut global_u = Vec4::new(*x, *y, *z, 0.);
    let mut global_u_prime= Vec4::new(tx, ty, tz, *qop);

    
    // main runge-kutta loop
    loop {
        // fetch the qop value from the previous iteration's u vector
        get_unchecked!{3; global_u=> qop}

        // calculate u'' based on Eq. 17,
        // https://cds.cern.ch/record/1114177/files/ATL-SOFT-PUB-2009-002.pdf
        let upp = eval_diffeq(qop, &magnetic_field, &direction);

        // fetch u'' values 
        // i _think_ this gets compiled away. will do individual tests later
        get_unchecked!{vector; upp;
            0 => u1,
            1 => u2,
            2 => u3,
            3 => u4
        }
        
        /*
            F calculation (Eq. 7)
        */

        // h* u'
        let h_times_u_prime = unsafe {scalar_multiply(&global_u_prime, step_size) };
        
        // h^2 /6 * (u1 + u2 + u3)
        let scalar_step = (u1 + u2 + u3) * step_size.powf(2.) / 6.;
        
        let vec_sum = h_times_u_prime + global_u;
        let pred_u = add_scalar(&vec_sum, scalar_step);

        /*
            G calculation (Eq. 7)
        */

        // u1 + 2 * u2 + 2* u3 + u3
        let parens = u1 + (2.*u2) + (2.*u3) + u4;
        // h/6 * parenthesis 
        let h_over_six = step_size * parens / 6.;

        let pred_u_prime = add_scalar(&global_u_prime, h_over_six);
    

        // fetch the current x / y / z coordinates from the predicted u vector
        get_unchecked!{vector; pred_u;
            0=> x,
            1=> y,
            2 =>z
        }
        let global_point = P3::new(*x,*y,*z);

        // will return true if the point is within an acceptable distance
        
        if end_sensor.on_plane(&global_point) {break}

        // set the parameters for the next iteration
        global_u = pred_u;
        global_u_prime = pred_u_prime;
    }

    // pull global coordinates of ending value
    get_unchecked!{vector; global_u;
        0 => x,
        1 => y,
        2 => z
    }
    
    // convert to local coordinates
    let global_end_point = P3::new(*x, *y, *z);
    let local_end = end_sensor.to_local(global_end_point);

    // fetch values needed to create state vector
    get_unchecked!{vector; local_end;
        0 => l0,
        1 => l1
    }
    get_unchecked!{vector; global_u_prime;
        0 => tx,
        2 => tz,
        3 => qop
    }

    // solve for theta and phi from coordinate direction angles
    // TODO: make this more efficient
    let theta = tz.acos();
    let sin_theta = theta.sin();
    let cos_phi = tx / sin_theta;
    let phi = cos_phi.acos();

    // construct the final return value
    let predicted_state_vec = 
        Vec5::new(
            *l0,
            *l1,
            phi,
            theta,
            *qop
        );

    Ok(predicted_state_vec)

}


fn eval_diffeq(
    qop: &Real,
    magnetic_vec: &Vec3,
    direction_vec: &Vec4
    ) -> Vec4 {

    get_unchecked!{vector;direction_vec;
        0 => tx,
        1 => ty,
        2 => tz

    }
    get_unchecked!{vector;magnetic_vec;
        0 => bx,
        1 => by,
        2 => bz
    }
    
    let xpp = qop * ((ty*bz) - (tz*by));
    let ypp = qop * ((tz * bx) - (tx*bz));
    let zpp = qop * ((tx *  by) - (ty * bx));
    let lambda = 0.;

    Vec4::new(xpp, ypp, zpp, lambda)

}


/// Add a scalar to all elements of a vector
fn add_scalar(vec: &Vec4, scalar: Real) -> Vec4 {
    // cant fill before matrix is initialized
    let mut to_fill = Vec4::zeros();
    to_fill.fill(scalar);
    
    vec + to_fill
}

/// multiply a scalar to every value in a vector
unsafe fn scalar_multiply(vec: &Vec4, scalar: Real) -> Vec4 {
    let mut new_vec = Vec4::zeros();

    for i in 0..3{
        let val = vec.get_unchecked(i);
        let replace = new_vec.get_unchecked_mut(i);
        *replace = (*val) * scalar;
    }

    new_vec

}

/// Encapsultes all the global calculations for `linear_state_vector` without 
/// needing start and end sensors. This is done so it is easier to construct unit tests
/// for this particularly troubling part of code.
pub fn linear_global_hit_estimation(
    plane_normal_vector: &Vec3,
    start_global_point: &P3,
    angles: &mut angles::Angles,
    end_plane_constant: &Real
    ) -> (P3,Real) {

    // to be honest i am not sure _at all_ why this vector has to be reversed,
    // but the tests where x and y are not both positive will fail without this.
    let normal_vector = -plane_normal_vector;

    // find the slopes of the unit vector from the starting point to the end sensor
    let x_slope = angles.tx;
    let y_slope = angles.ty;
    let z_slope = angles.tz;

    // dbg!{normal_vector};dbg!{angles}; dbg!{x_slope}; dbg!{y_slope}; dbg!{z_slope};

    //generic numerator for repetitive calculations
    let gen_num_1 = normal_vector.x * start_global_point.x;
    let gen_num_2 = normal_vector.y * start_global_point.y;
    let gen_num_3 = normal_vector.z * start_global_point.z;
    let gen_num = gen_num_1 + gen_num_2 + gen_num_3 + end_plane_constant;

    // dbg!{gen_num_1}; dbg!{gen_num_2}; dbg!{gen_num_3};

    // generic denominator for repetitive calculations
    let gen_den_1 = normal_vector.x * x_slope;
    let gen_den_2 = normal_vector.y * y_slope;
    let gen_den_3 = normal_vector.z * z_slope;
    let gen_den = gen_den_1 + gen_den_2 + gen_den_3;

    // dbg!{gen_den_1}; dbg!{gen_den_2}; dbg!{gen_den_3};

    let gen_division = gen_num / gen_den;

    // dbg!{gen_num};dbg!{gen_den};dbg!{gen_division};


    // calculate predicted points of intersection on ending plane
    let pred_x = start_global_point.x - (x_slope * gen_division);
    let pred_y = start_global_point.y - (y_slope * gen_division);
    let pred_z = start_global_point.z - (z_slope * gen_division);

    // dbg!{pred_x}; dbg!{pred_y}; dbg!{pred_z};

    let global_pred_point = P3::new(pred_x, pred_y, pred_z);

    let distance = (global_pred_point - start_global_point).norm();

    return (global_pred_point, distance)
}