use super::super::config::*;

#[inline(always)]
pub fn state_vector(
    filt_covariance_mat: &Mat5,     // filt C
    pred_covariance_mat: &Mat5,     // pred C
    pred_state_vec: &Vec5,          // pred x
    sensor_mapping_mat: &Mat2x5,    // H
    inverse_measurement_cov: &Mat2, // inv(V)
    measurement_vec: &Vec2,         //m_k
) -> Vec5 {
    let product_one = pred_covariance_mat
        .try_inverse()
        .expect("could not invert pred cov mat")
        * pred_state_vec;
    let product_two = sensor_mapping_mat.transpose() * inverse_measurement_cov * measurement_vec;

    return filt_covariance_mat * (product_one + product_two);
}

#[inline(always)]
pub fn covariance_matrix(
    pred_covariance_mat: &Mat5,     // pred C
    sensor_mapping_mat: &Mat2x5,    // H
    inverse_measurement_cov: &Mat2, // inv (V)
) -> Mat5 {
    let product = sensor_mapping_mat.transpose() * inverse_measurement_cov * sensor_mapping_mat;
    let c_prevoius_inv = pred_covariance_mat
        .try_inverse()
        .expect("could not invert previous covariance");

    return (c_prevoius_inv + product)
        .try_inverse()
        .expect("could not invert matrix product");
}

#[inline(always)]
pub fn chi_squared_increment(
    residual_vec: &Vec2,
    inverse_measurement_cov: &Mat2,
    state_vector: &Vec5,
    extrap_state_vector: &Vec5,
    pred_covariance_mat: &Mat5,
) -> Real {
    let first_term = residual_vec.transpose() * inverse_measurement_cov * residual_vec;

    let second_term_3 = state_vector - extrap_state_vector;
    let second_term_2 = pred_covariance_mat.try_inverse().unwrap();
    let second_term_1 = second_term_3.transpose();

    let second_term = second_term_1 * second_term_2 * second_term_3;

    return (first_term + second_term)[0];
}
