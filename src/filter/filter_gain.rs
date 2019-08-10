use super::super::config::*;

#[inline(always)]
pub fn state_vector(
    pred_state_vec: &Vec5,       //x
    kalman_gain: &Mat5x2,        //K
    measurement: &Vec2,          //m_k
    sensor_mapping_mat: &Mat2x5, // H
) -> Vec5 {
    let parens = measurement - (sensor_mapping_mat * pred_state_vec);
    let kalman_product = kalman_gain * parens;
    return pred_state_vec + kalman_product;
}

#[inline(always)]
pub fn kalman_gain(
    pred_covariance: &Mat5,        //C
    sensor_mapping_mat: &Mat2x5,   //H
    measurement_covariance: &Mat2, //V
) -> Mat5x2 {
    let parens = measurement_covariance
        + (sensor_mapping_mat * pred_covariance * sensor_mapping_mat.transpose());
    let kalman_gain = pred_covariance
        * sensor_mapping_mat.transpose()
        * parens
            .try_inverse()
            .expect("could not inert in kalman_gain");

    kalman_gain
}

#[inline(always)]
pub fn covariance_matrix(
    kalman_gain_mat: &Mat5x2,    //K
    sensor_mapping_mat: &Mat2x5, // H
    pred_covariance: &Mat5,      // pred C
) -> Mat5 {
    let parens = Mat5::identity() - (kalman_gain_mat * sensor_mapping_mat);

    return parens * pred_covariance;
}

//TODO ensure that `identity` is complile-time optimized
#[inline(always)]
pub fn residual_vec(
    sensor_mapping_mat: &Mat2x5, // H
    kalman_gain_mat: &Mat5x2,    // K
    pred_residual_vec: &Vec2,    // pred r
) -> Vec2 {
    let ident = Mat2::identity();
    let parens = ident - (sensor_mapping_mat * kalman_gain_mat);

    return parens * pred_residual_vec;
}

#[inline(always)]
pub fn residual_mat(
    measurement_covariance: &Mat2, // measurement_covariance
    sensor_mapping_mat: &Mat2x5,   // H
    filt_covariance_mat: &Mat5,    //filt C
) -> Mat2 {
    let product = sensor_mapping_mat * filt_covariance_mat * sensor_mapping_mat.transpose();
    return measurement_covariance - product;
}

#[inline(always)]
pub fn chi_squared_increment(filt_residual_vec: &Vec2, filt_residual_mat: &Mat2) -> Real {
    let prod = filt_residual_vec.transpose()
        * filt_residual_mat
            .try_inverse()
            .expect("could not invert residual covairiance matrix")
        * filt_residual_vec;
    return prod[0];
}

#[inline(always)]
pub fn update_chi_squared(previous_chi_squaread: Real, increment: Real) -> Real {
    previous_chi_squaread + increment
}
