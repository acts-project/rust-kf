#include <iostream>
#include "rust_headers.hpp"

int main() {
    const int arr_len = 4;
    double x[arr_len] = {1.6,2,3,4};   
    double* arr_ptr = x;

    // pass the pointer and array to rust which calculates the sum and adds and arbitrary number
    double result = make_array(arr_ptr, 4);

    std::cout << result;

}