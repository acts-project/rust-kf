#ifndef __RUST_HEADERS
#define __RUST_HEADERS

#ifdef __cplusplus
extern "C"{
#endif

    double make_array(double* arr_start, int arr_len);
    // void ffi_test();   
    // void float_test(double value);
    // void int_test(double value);
    // void float_ptr_test(double* value);

    double add(double a, double b);

#ifdef __cplusplus
}
#endif


#endif
