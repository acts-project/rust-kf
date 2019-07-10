use super::super::config::*;

pub fn gain_matrix(
    curr_filt_cov_mat: &Mat5,   //filt C
    jacobian: &Mat5,            // F_k or J
    prev_filt_cov_mat: &Mat5    // prev filt C
    ) -> Mat5 {                 // A

    let inv_cov = 
        match prev_filt_cov_mat.try_inverse() {
            Some(inverse) => inverse,
            None => {
                dbg!{"gain matrix inverse error"};
                // dbg!{prev_filt_cov_mat};
                Mat5::identity()
            }
        };

    curr_filt_cov_mat * jacobian.transpose() * inv_cov
}

pub fn state_vector(
    curr_filt_state_vec: &Vec5,     // curr filt x
    gain_mat: &Mat5,                // A
    prev_smth_state_vec: &Vec5,     // prev smth x
    prev_filt_state_vec: &Vec5      // prev filt x
    ) -> Vec5 {                     // smth x
    
    let parens = prev_smth_state_vec - prev_filt_state_vec;
    let prod = gain_mat * parens;
    let sum =  curr_filt_state_vec + prod;

    return sum
}

pub fn covariance_matrix(
    curr_filt_cov_mat: &Mat5,   // curr filt C  
    gain_mat: &Mat5,            // A
    prev_smth_cov_mat: &Mat5,    // prev smth C
    prev_filt_cov_mat: &Mat5,   // prev filt C
    ) -> Mat5 {                 // smth C

    let parens = prev_smth_cov_mat - prev_filt_cov_mat;
    let prod = gain_mat * parens * gain_mat.transpose();
    let sum = curr_filt_cov_mat + prod;
    
    return sum
}


pub fn residual_mat(
    V: &Mat2,                       // V
    sensor_mapping_mat: &Mat2x5,    // H
    curr_smth_cov_mat: &Mat5        // curr smth C
    ) -> Mat2 {                     // smth R

    let prod = sensor_mapping_mat * curr_smth_cov_mat * sensor_mapping_mat.transpose();
    let diff = V - prod;

    return diff;
}

pub fn residual_vec(
    measurement_vec: &Vec2,         // m_k
    sensor_mapping_mat: &Mat2x5,    // H
    curr_smth_state_vec: &Vec5      // curr smth x
    ) -> Vec2 {                     // smth r
    
    let prod = sensor_mapping_mat * curr_smth_state_vec;
    let sum = measurement_vec + prod;
    
    return sum;
}
