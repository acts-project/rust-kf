#ifndef __RUST_HEADERS
#define __RUST_HEADERS

#ifdef __cplusplus
extern "C"{
#endif

    double make_array(double* arr_start, int arr_len);
    void eigen_to_nalgebra(double *matrix_ptr);
    void eigen_hits_to_nalgebra_hits(double* first_element, int arr_len);
    

#ifdef __cplusplus
}
#endif


#endif
