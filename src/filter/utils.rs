use super::super::config::*;
use super::super::geometry::traits::{Plane, Transform};
use super::angles;

/// Placeholder function for some form of effective seeding for Mat5's
pub fn seed_covariance() -> Mat5 {
    // create a matrix with every element being .1
    let mut base = Mat5::zeros();

    let mut vals = std::iter::repeat(0.07).into_iter();

    for i in 0..5 {
        let val = vals.next().unwrap();

        edit_matrix! {base;
            [i,i] = val
        }
    }
    base
}

/// Calculate the first filtered state vector estimate based on the
/// global starting position of the particle and the the location
/// of the sensor it is supposed to hit

// NOTE: this function is basically a wrapper around `seed_state_vec_from_points`
//          for ease of writing tests
pub fn seed_state_vec_from_sensor<T: Plane + Transform>(
    start_location: &P3,
    first_sensor: &T,
    first_sensor_hit: &Vec2,
) -> Vec5 {
    let local_hit_point = P3::new(first_sensor_hit.x, first_sensor_hit.y, 0.);

    let global_end = first_sensor.global_center();
    let local_end = first_sensor.to_local(local_hit_point); // TODO store local_center so we dont need this conversion

    seed_state_vec_from_points(&start_location, &global_end, &local_end)
}

/// Calculate the first filtered state vector based on global point starting position,
/// local ending location, and global ending location
pub fn seed_state_vec_from_points(
    global_start_location: &P3,
    global_destination: &P3,
    local_destination: &P2,
) -> Vec5 {
    // position vector from begining point to ending point
    let vector_to_sensor = global_destination - global_start_location;

    // fetch local hit measurements
    get_unchecked! {vector;vector_to_sensor;
        eLOC_0 => x,
        eLOC_1 => y
    }

    let z_axis = Vec3::new(0., 0., 1.);
    let x_axis = Vec3::new(1., 0., 0.);

    // on the XY projection we construct, if we are in quadrants 3 or 4 then the angle
    // generated by .angle() will be x instead of 2pi - x (returns smallest angle between vectors).
    // we do this here to reduce copies.
    let adjust_factor = if y < &0. { true } else { false };

    let xy_projection = Vec3::new(*x, *y, 0.);

    let _phi = xy_projection.angle(&x_axis);

    // if we are in quadrants 3/4 we need to adjust for the angle in _phi
    let phi = if adjust_factor {
        (2. * PI) - _phi
    } else {
        _phi
    };

    let theta = vector_to_sensor.angle(&z_axis);

    let mut seed_vec = Vec5::zeros();

    edit_matrix! {seed_vec;
        [eLOC_0] = local_destination.x,
        [eLOC_1] = local_destination.y,
        [ePHI]   = phi,
        [eTHETA] = theta,
        [eQOP] = 1.
    }

    seed_vec
}

// TODO Name is similar to other structs. figure out a new one
#[derive(Debug, Clone)]
pub struct Data {
    pub state_vec: Vec<Vec5>,
    pub cov_mat: Vec<Mat5>,
    pub res_mat: Vec<Mat2>,
    pub res_vec: Vec<Vec2>,
}

impl Data {
    pub fn new(
        state_vec: Vec<Vec5>,
        cov_mat: Vec<Mat5>,
        res_mat: Vec<Mat2>,
        res_vec: Vec<Vec2>,
    ) -> Self {
        return Data {
            state_vec: state_vec,
            cov_mat: cov_mat,
            res_mat: res_mat,
            res_vec: res_vec,
        };
    }
}

/// For every row in a 3x3 matrix equal to the cross product of that row
/// with a given vector.
pub fn matrix_cross_product(matrix: &mut Mat3, vector: &Vec3) {
    for i in 0..3 {
        let mut column = matrix.fixed_slice_mut::<U3, U1>(0, i);

        let cross_result = column.cross(&vector);

        column.copy_from(&cross_result);
    }
}

pub fn simulate_cross_product(
    input_matrix: &mut Mat4,
    b_field: &Vec3,
    direction: &Vec3,
    qop: Real,
) {
    get_unchecked! {vector;b_field;
        0 => mag_x,
        1 => mag_y,
        2 => mag_z
    }

    get_unchecked! {vector;direction;
        0 => dir_x,
        1 => dir_y,
        2 => dir_z
    }

    edit_matrix! {input_matrix;
        [0,1] = qop*mag_z,
        [0,2] = -qop *mag_y,
        [0,3] = (dir_y * mag_z) - (dir_z * mag_y),

        [1,0] = -qop*mag_z,
        [1,2] = qop *mag_x,
        [1,3] = (dir_z * mag_x) - (dir_x * mag_z),

        [2,0] = qop *mag_y,
        [2,1] = -qop * mag_x,
        [2,3] = (dir_x * mag_y) - (dir_y * mag_x)

    }
}

/// pull the global location of a particle from a global state vector
pub fn global_point_from_rk_state(rk_state_vec: &Vec8) -> P3 {
    get_unchecked! {vector;rk_state_vec;
        0 =>  x,
        1 => y,
        2 => z
    }
    P3::new(*x, *y, *z)
}

/// make angles struct from a global state vector
pub fn angles_from_rk_state(rk_staete_vec: &Vec8) -> angles::Angles {
    get_unchecked! {vector;rk_staete_vec;
        4 => tx,
        5 => ty,
        6 => tz
    }

    angles::Angles::new_from_unit_direction(*tx, *ty, *tz)
}

// TODO: come up with a better name for this
#[derive(Debug, Clone)]
pub struct SuperData {
    pub smth: Data,
    pub filt: Data,
    pub pred: Data,
}
impl SuperData {
    pub fn new(smth: Data, filt: Data, pred: Data) -> Self {
        SuperData {
            smth: smth,
            filt: filt,
            pred: pred,
        }
    }
}

/// Transform a 8-row rk state vector in global coordinates to a
/// 5-row vector in local coordinates relative to a destination sensor
pub fn global_to_local_state_vector<T: Transform + Plane>(
    global_sv: &Vec8,
    sensor: &T,
) -> (Vec5, angles::Angles) {
    let global_point = global_point_from_rk_state(&global_sv);
    let angles = angles_from_rk_state(&global_sv);

    get_unchecked! {
        global_sv[7] => qop
    }

    let local = sensor.to_local(global_point);

    let sv = Vec5::new(
        local.x,
        local.y,
        angles.cos_phi.acos(),
        angles.cos_theta.acos(),
        *qop,
    );

    (sv, angles)
}

pub fn local_to_global_state_vector<T: Transform + Plane>(
    local_sv: &Vec5,
    sensor: &T,
) -> (Vec8, angles::Angles) {
    get_unchecked! {vector; local_sv;
        eLOC_0 => x,
        eLOC_1 => y,
        ePHI => phi,
        eTHETA => theta,
        eQOP => qop
    }

    let local_point = P3::new(*x, *y, 0.);
    let global_point = sensor.to_global(local_point);

    let angles = angles::Angles::new_from_angles(*phi, *theta);

    /*
        build a 8x1 global state vector
    */

    // we must do it this way since ::new() does not
    // exist for Vec8 since its custom defined in config.rs
    // and is not pre-defined by nalgbra
    let mut global_state_vec = Vec8::zeros();
    edit_matrix! {global_state_vec;
        [0,0] = global_point.x,
        [1,0] = global_point.y,
        [2,0] = global_point.z,
        [3,0] = 0.,

        [4,0] = angles.tx,
        [5,0] = angles.ty,
        [6,0] = angles.tz,
        [7,0] = *qop
    }

    (global_state_vec, angles)
}

pub fn oath_length_derivatives(angles: &angles::Angles) -> Vec8 {
    let mut vec = Vec8::zeros();

    edit_matrix! {vec;
        [0] = angles.tx,
        [1] = angles.ty,
        [2] = angles.tz
    }

    return vec;
}
