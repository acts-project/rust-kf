
#include <iostream>
#include <Eigen/Dense>
#include <vector> 

#include "rust_headers.hpp"

using namespace Eigen;
using namespace std;

void vector_of_hits();
void pass_array();
void pass_matrix();

int main() {
    vector_of_hits();
    pass_array();
}


void vector_of_hits() {

    vector<Vector2d> hits_vector;

    // make a bunch of random entries
    for (int i = 0; i < 10; i++){
        Vector2d random_hit = Vector2d::Random();

        // stdout to compare with the rust stdout
        cout << random_hit << "\n\n";

        hits_vector.push_back(random_hit);
    }

    // get pointer to the first Vector2d
    double* start = hits_vector[0].data();

    int length = hits_vector.size();

    // rust ffi call
    eigen_hits_to_nalgebra_hits(start, length);
}

void pass_array() {
    const int arr_len = 4;
    double array[arr_len] = {1, 2, 3 ,4};
    double* arr_ptr = array;

    // rust ffi call
    make_array(arr_ptr, arr_len);
}

void pass_matrix() {
    Matrix3d square_mat = Matrix3d::Random();

    double* ptr = square_mat.data();

    // rust ffi call
    eigen_to_nalgebra(ptr);
}
