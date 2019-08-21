
#include <iostream>
#include <Eigen/Dense>
#include <vector> 

#include "rust_headers.hpp" 

using namespace Eigen;
using namespace std;

void vector_of_hits();
void pass_array();
void pass_matrix();
void run_linear_kf();
void run_const_b_kf();

struct DataPtr{};

DataPtr _test() {
    DataPtr x;
    return x;
}

int main() {
    // vector_of_hits();
    // pass_array();
    // run_linear_kf();
}

// // converting c++ matricies to rust matricies
// void vector_of_hits() {

//     vector<Vector2d> hits_vector;

//     // make a bunch of random entries
//     for (int i = 0; i < 10; i++){
//         Vector2d random_hit = Vector2d::Random();

//         // stdout to compare with the rust stdout
//         cout << random_hit << "\n\n";

//         hits_vector.push_back(random_hit);
//     }

//     // get pointer to the first Vector2d
//     double* start = hits_vector[0].data();

//     int length = hits_vector.size();

//     // rust ffi call
//     eigen_hits_to_nalgebra_hits(start, length);
// }

// // example for sending an array of data to rust
// void pass_array() {
//     const int arr_len = 4;
//     double array[arr_len] = {1, 2, 3 ,4};
//     double* arr_ptr = array;

//     // rust ffi call
//     make_array(arr_ptr, arr_len);
// }

// void pass_matrix() {
//     Matrix3d square_mat = Matrix3d::Random();

//     double* ptr = square_mat.data();

//     // rust ffi call
//     eigen_to_nalgebra(ptr);
// }

// // running a linear beam experiment using measurement covariances and hits
// void run_linear_kf() {
//     vector<Vector2d> hits_vector;
//     vector<Matrix2d> meas_vector;

//     const int number_of_sensors = 3;

//     for (int i =0; i < number_of_sensors; i++) {
//         Vector2d random_hit = Vector2d::Random();
//         Matrix2d random_cov = Matrix2d::Random();

//         cout << random_hit << "\n\n" << random_cov << "\n\n";

//         hits_vector.push_back(random_hit);
//         meas_vector.push_back(random_cov);

//     }

//     double* hits_ptr = hits_vector[0].data();
//     double* meas_ptr = meas_vector[0].data();

//     run_linear_kf(hits_ptr, meas_ptr, number_of_sensors);
//     cout << "made it here";
// }

// void run_const_b_kf() {

//     vector<Vector2d> hits_vector;
//     vector<Matrix2d> meas_vector;

//     const int number_of_sensors = 3;

//     for (int i =0; i < number_of_sensors; i++) {
//         Vector2d random_hit = Vector2d::Random();
//         Matrix2d random_cov = Matrix2d::Random();

//         cout << random_hit << "\n\n" << random_cov << "\n\n";

//         hits_vector.push_back(random_hit);
//         meas_vector.push_back(random_cov);

//     }
    
//     Vector3d b_field = Vector3d::Random();

//     double* hits_ptr = hits_vector[0].data();
//     double* meas_ptr = meas_vector[0].data();
//     double* field_ptr = b_field.data();

//     run_const_b_kf(hits_ptr, meas_ptr, field_ptr, number_of_sensors);
    
// }

