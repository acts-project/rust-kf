// use kalman_rs::config::*;
// #[macro_use]
// use kalman_rs::change_mat_val;

// /*

//     Unit tests for the macro kalman_rs::filter::macros::change_mat_val.
//     This was needed when the nalgebra docs were unclear on how indexing functioned
//     and required a "linear index" that traversed down every column to the desired element.module_path!

//     change_mat_val!{} now uses an improved indexing version (by row and column instead of linearly)
//     but these tests do not hurt

// */
// fn init_mat() -> (Mat3, usize) {
//     (Mat3::zeros(), 3)
// }

// fn init_mxn() -> (Mat5x2, usize) {
//     (Mat5x2::zeros(), 5)
// }

// // tests for N x N matricies
// mod square_mat {
//     use super::*;

//     #[test]
//     fn mat_test_1() {
//         let (mut mat, dim) = init_mat();
//         let new_val = 10.0;
//         change_mat_val!{mat;
//             [0,0] => new_val           // very first index
//         }

//         dbg!{&mat};

//         assert_eq!(
//             *mat.get(0).expect("out of bounds"),
//             new_val
//         )
//     }

//     #[test]
//     fn mat_test_2() {
//         let (mut mat, dim) = init_mat();
//         let new_val = 10.0;
//         change_mat_val!{mat;
//             [2,2] => 10.0            // very last index
//         }

//         dbg!{&mat};

//         assert_eq!(
//             *mat.get(8).expect("out of bounds"),
//             new_val
//         )
//     }

//     #[test]
//     fn mat_test_3() {
//         let (mut mat, dim) = init_mat();
//         let new_val = 10.0;
//         change_mat_val!{mat;
//             [2,0] => 10.0           // bottom left corner
//         }

//         dbg!{&mat};

//         assert_eq!(
//             *mat.get(2).expect("out of bounds"),
//             new_val
//         )
//     }

//     #[test]
//     fn mat_test_4() {
//         let (mut mat, dim) = init_mat();
//         let new_val = 10.0;
//         change_mat_val!{mat;
//             [0,2] => 10.0       // top right corner
//         }

//         dbg!{&mat};

//         assert_eq!(
//             *mat.get(6).expect("out of bounds"),
//             new_val
//         )
//     }
//     #[test]
//     fn mat_test_5() {
//         let (mut mat, dim) = init_mat();
//         let new_val = 10.0;
//         change_mat_val!{mat;
//             [1,0] => 10.0       // top right corner
//         }

//         dbg!{&mat};

//         assert_eq!(
//             *mat.get(1).expect("out of bounds"),
//             new_val
//         )
//     }
// }

// // uses 5x2 mat
// mod non_square_mat {
//     use super::*;

//     #[test]
//     fn mxn_mat_test_1() {
//         let (mut mat, dim) = init_mxn();
//         let new_val = 1.;
//         let index = 0;

//         change_mat_val!{
//             mat;
//             [0,0] => new_val
//         }

//         assert_eq!{
//             *mat.get(index).expect("out of bounds"),
//             new_val
//         }
//     }

//     #[test]
//     fn mxn_mat_test_2() {
//         let (mut mat, dim) = init_mxn();
//         let new_val = 1.;
//         let index = 9;

//         change_mat_val!{
//             mat;
//             [4,1] => new_val
//         }

//         dbg!{&mat};

//         assert_eq!{
//             *mat.get(index).expect("out of bounds"),
//             new_val
//         }
//     }

//     #[test]
//     fn mxn_mat_test_3() {
//         let (mut mat, dim) = init_mxn();
//         let new_val = 1.;
//         let index = 7;

//         change_mat_val!{
//             mat;
//             [2,1] => new_val
//         }

//         dbg!{&mat};

//         assert_eq!{
//             *mat.get(index).expect("out of bounds"),
//             new_val
//         }
//     }

//     #[test]
//     fn mxn_mat_test_4() {
//         let (mut mat, dim) = init_mxn();
//         let new_val = 1.;
//         let index = 3;

//         change_mat_val!{
//             mat;
//             [3,0] => new_val
//         }

//         dbg!{&mat};

//         assert_eq!{
//             *mat.get(index).expect("out of bounds"),
//             new_val
//         }
//     }

//     #[test]
//     fn mxn_mat_test_5() {
//         let (mut mat, dim) = init_mxn();
//         let new_val = 1.;
//         let index = 5;

//         change_mat_val!{
//             mat;
//             [0,1] => new_val
//         }

//         dbg!{&mat};

//         assert_eq!{
//             *mat.get(index).expect("out of bounds"),
//             new_val
//         }
//     }

// }
