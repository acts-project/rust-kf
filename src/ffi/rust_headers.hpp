#ifndef __RUST_HEADERS
#define __RUST_HEADERS

#ifdef __cplusplus
extern "C"{
#endif
    
    struct DataPtr{
        double* state_vec;
        double* cov_mat;
        double* res_mat;
        double* res_vec;
    };
    DataPtr run_linear_kf(double* hits_ptr, double* meas_ptr, int sensor_count);
    DataPtr run_const_b_kf(double* hits_ptr, double* meas_ptr, double* b_field_ptr, int sensor_count);

    double make_array(double* arr_start, int arr_len);
    void eigen_to_nalgebra(double *matrix_ptr);
    void eigen_hits_to_nalgebra_hits(double* first_element, int arr_len);


#ifdef __cplusplus
}
#endif


#endif
