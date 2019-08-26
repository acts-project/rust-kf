# Kalman Filter in Rust

Google Summer of Code 2019 project written entirely by Brooks Karlik.

[Orignal GSoC Proposal](https://summerofcode.withgoogle.com/dashboard/project/5790049365393408/details/)

## Mentors

- Paul Gessinger

- Hadrien Grasland

- Andreas Salzburger

## Project Summary :

The Kalman Filter is a method of iteratively predicting the future state of a system based on previous information. Not only is a Kalman Filter more reliable about predicting future state than traditional extrapolation techniques, It also provides a confidence for the estimate. A Kalman Filter is used both to reduce the impact of sensor noise on estimations, and to determine which sensors can be “trusted” more than others. Whereas more primitive methods for estimation and extrapolation rely on some form of averaging, a Kalman Filter forecasts by developing a weighted covariance for each sensor input.

The aim of this project is to implement a Kalman Filter in Rust. Rust has gained popularity for providing more compile-time checks than other systems-level languages, namely C and C++. Rust’s memory model ensures that there is little to no room for many of the memory pitfalls common in other low level languages, such as double-freeing memory, dangling pointers, and user-after-free errors. This, in conjunction with high runtime performance, leads writing components of a codebase in Rust to be favorable for both speed and stability.

## Building and documentation:

Clone the repo :

`git clone https://github.com/acts-trk/rust-kf && cd rust-kf`

build: 

`cargo build --release`

documentation:

`cargo doc --open`

# Rust-KF in review

problems,  inconvenience, and takeaways.


## Sometimes Ambiguous error messages

Clear error message when using `x` by value:
```
// erorr: the method `clone` exists but the following trait bounds were not satisfied: `std::vec::Vec<T> : std::clone::Clone`
fn error_fn_1<T>(x: Vec<T>) {
   let mut a = x.clone();
}
```

Changing `Vec<T>` to `&Vec<T>` hides the error. It clones the `&` reference and ignores that it is mutable:
```
fn error_fn_2<T>(x: &Vec<T>) {
   let mut a = x.clone();
}
```

actually mutating `a` causes a new and confusing error:
```
fn error_fn_3<T>(x: &Vec<T>) {
   let mut a = x.clone();
   //----- help: consider changing this to be a mutable reference: `&mut std::vec::Vec<T>`
   a.remove(0);
   //^ `a` is a `&` reference, so the data it refers to cannot be borrowed as mutable
}
```

```
// adding Clone trait bound fixes.
fn error_fn_3<T: Clone>(x: &Vec<T>) {
   let mut a = x.clone();
   a.remove(0);
}
```

## Mutable references to different data

Mutable references to mutually exclusive areas of a matrix cannot occur at the same time:
```
use super::config::*;
use nalgebra as na;

// A 5x1 column slice of f64
type Slice<'a> = na::MatrixSliceMut::<'a, Real, U5, U1, U1, U5>;

fn submatrix_slicing() {
   let mut matrix = Mat5::identity();

   // two mutable references to different data
   let mut column_1 : Slice = matrix.fixed_slice_mut::<U5, U1>(0,0);
   let mut column_2 : Slice =  matrix.fixed_slice_mut::<U5, U1>(0,1);

   take_mut_ref(&mut column_1); // Error: cant have two mutable references to `matrix`

}

fn take_mut_ref<T>(a: &mut T) {}

```

However, Rust allows multiple mutable references to different data in other situations:

```
// compiles fine
fn mut_ref_diff_data() {
   let mut array = [1,2,3,4];

   let mut zero = array[0];
   let mut one = array[1];
  
   take_mut_ref(&mut zero);
}
```

This is likely both a Rust and `Nalgebra` maturity issue. This can be pretty easily worked around by editing each submatrix individually. This is done in the rust-kf [here](https://github.com/VanillaBrooks/rust-kf/blob/constant-magnetic-field/src/filter/runge_kutta.rs#L165-L184). However, it would improve code clarity to declare all mutable slices


## FFI

### Error Messages

```
//main.cpp
extern "C" {
   void rust_function();
}

int main() {
   rust_function();
}
```
```
// lib.rs

#[no_mangle]
fn rust_function() {
   println!{"I am in the rust function"}
}
```

Compiling this code with `msvc` on windows:

```
...
Build succeeded.
   0 Warning(s)
   0 Error(s)

Time Elapsed 00:00:01.74
I am in the rust function
```

however, compiling with MinGW on windows causes obscure error messages:

```
../target/release/rust_lib.lib(std-b2f27b8d08c4688f.std.7cxo72xo-cgu.0.rcgu.o):std.7cxo72xo-cgu.0:(.text[_ZN83_$LT$alloc..boxed..Box$LT$F$GT$$u20$as$u20$core..ops..function..FnOnce$LT$A$GT$$GT$9call_once17he29ee55a6a0ae801E]+0x28): undefined reference to `__chkstk'
../target/release/rust_lib.lib(std-b2f27b8d08c4688f.std.7cxo72xo-cgu.0.rcgu.o):std.7cxo72xo-cgu.0:(.text[_ZN3std3sys7windows2fs4File9file_attr17he7a6dcc1a776de37E]+0x12): undefined reference to `__chkstk'
../target/release/rust_lib.lib(std-b2f27b8d08c4688f.std.7cxo72xo-cgu.0.rcgu.o):std.7cxo72xo-cgu.0:(.text[_ZN3std3sys7windows2fs8readlink17h06cd13edb42cebc7E]+0xc): undefined reference to `__chkstk'
../target/release/rust_lib.lib(std-b2f27b8d08c4688f.std.7cxo72xo-cgu.0.rcgu.o):std.7cxo72xo-cgu.0:(.text[_ZN3std3sys7windows2os12error_string17hc99e106c3b167d73E]+0xe): undefined reference to `__chkstk'
../target/release/rust_lib.lib(std-b2f27b8d08c4688f.std.7cxo72xo-cgu.0.rcgu.o):std.7cxo72xo-cgu.0:(.text[_ZN3std3sys7windows5stdio5write17h336e25934477c81eE]+0xf): undefined reference to `__chkstk'
../target/release/rust_lib.lib(std-b2f27b8d08c4688f.std.7cxo72xo-cgu.0.rcgu.o):std.7cxo72xo-cgu.0:(.text[_ZN65_$LT$std..sys..windows..stdio..Stdin$u20$as$u20$std..io..Read$GT$4read17h214a2371cbc3aacbE]+0x12): more undefined references to `__chkstk' follow
../target/release/rust_lib.lib(std-b2f27b8d08c4688f.std.7cxo72xo-cgu.0.rcgu.o):std.7cxo72xo-cgu.0:(.xdata[$cppxdata$_ZN4core3fmt5Write10write_char17h3b8f34858b332aa5E]+0xc): undefined reference to `__CxxFrameHandler3'
../target/release/rust_lib.lib(std-b2f27b8d08c4688f.std.7cxo72xo-cgu.0.rcgu.o):std.7cxo72xo-cgu.0:(.xdata[$cppxdata$_ZN4core3fmt5Write10write_char17hae76db2a45caa3feE]+0xc): undefined reference to `__CxxFrameHandler3'
../target/release/rust_lib.lib(std-b2f27b8d08c4688f.std.7cxo72xo-cgu.0.rcgu.o):std.7cxo72xo-cgu.0:(.xdata[$cppxdata$_ZN4core3fmt5Write10write_char17hfbba2cd6cc2b817eE]+0xc): undefined reference to `__CxxFrameHandler3'
../target/release/rust_lib.lib(std-b2f27b8d08c4688f.std.7cxo72xo-cgu.0.rcgu.o):std.7cxo72xo-cgu.0:(.xdata[$cppxdata$_ZN4core3ops8function6FnOnce40call_once$u7b$$u7b$vtable.shim$u7d$$u7d$17h04d33cd1ad95318eE]+0x10): undefined reference to `__CxxFrameHandler3'
../target/release/rust_lib.lib(std-b2f27b8d08c4688f.std.7cxo72xo-cgu.0.rcgu.o):std.7cxo72xo-cgu.0:(.xdata[$cppxdata$_ZN4core3ops8function6FnOnce40call_once$u7b$$u7b$vtable.shim$u7d$$u7d$17h741c15c496a4df9dE]+0x10): undefined reference to `__CxxFrameHandler3'
../target/release/rust_lib.lib(std-b2f27b8d08c4688f.std.7cxo72xo-cgu.0.rcgu.o):std.7cxo72xo-cgu.0:(.xdata[$cppxdata$_ZN4core3ptr18real_drop_in_place17h033793c0390e7bc7E]+0xc): more undefined references to `__CxxFrameHandler3' follow
../target/release/rust_lib.lib(panic_unwind-9c73c9c2e052b2f1.panic_unwind.3hskofpu-cgu.0.rcgu.o):panic_unwind.3hsko:(.data[_ZN12panic_unwind3imp16TYPE_DESCRIPTOR117h0d905fcdab9b78adE]+0x0): undefined reference to `??_7type_info@@6B@'
../target/release/rust_lib.lib(panic_unwind-9c73c9c2e052b2f1.panic_unwind.3hskofpu-cgu.0.rcgu.o):panic_unwind.3hsko:(.data[_ZN12panic_unwind3imp16TYPE_DESCRIPTOR217ha99a382d3932afd0E]+0x0): undefined reference to `??_7type_info@@6B@'
../target/release/rust_lib.lib(panic_unwind-9c73c9c2e052b2f1.panic_unwind.3hskofpu-cgu.0.rcgu.o):panic_unwind.3hsko:(.xdata[$cppxdata$__rust_maybe_catch_panic]+0xc): undefined reference to `__CxxFrameHandler3'
../target/release/rust_lib.lib(panic_unwind-9c73c9c2e052b2f1.panic_unwind.3hskofpu-cgu.0.rcgu.o):panic_unwind.3hsko:(.xdata[$cppxdata$__rust_maybe_catch_panic]+0x1c): undefined reference to `__CxxFrameHandler3'
../target/release/rust_lib.lib(backtrace-7a588e8fa018f6bc.backtrace.6ykv7qa2-cgu.0.rcgu.o):backtrace.6ykv7qa2:(.text[_ZN9backtrace9symbolize7dbghelp7resolve17h1c616b7013c75969E]+0x12): undefined reference to `__chkstk'
../target/release/rust_lib.lib(backtrace-7a588e8fa018f6bc.backtrace.6ykv7qa2-cgu.0.rcgu.o):backtrace.6ykv7qa2:(.text[_ZN9backtrace9symbolize7dbghelp22resolve_without_inline17hbdd50890cd6579a8E]+0x12): undefined reference to `__chkstk'
../target/release/rust_lib.lib(backtrace-7a588e8fa018f6bc.backtrace.6ykv7qa2-cgu.0.rcgu.o):backtrace.6ykv7qa2:(.xdata[$cppxdata$_ZN9backtrace9symbolize7dbghelp7resolve17h1c616b7013c75969E]+0x1c): undefined reference to `__CxxFrameHandler3'
../target/release/rust_lib.lib(rustc_demangle-74b71f441b8acffe.rustc_demangle.4u9sjqwm-cgu.0.rcgu.o):rustc_demangle.4u9:(.xdata[$cppxdata$_ZN14rustc_demangle2v07Printer10print_type28_$u7b$$u7b$closure$u7d$$u7d$17h65b2c8eaf1497f78E]+0x18): undefined reference to `__CxxFrameHandler3'
../target/release/rust_lib.lib(alloc-f297c401e81b90c6.alloc.67mh9if6-cgu.0.rcgu.o):alloc.67mh9if6-cgu:(.xdata[$cppxdata$_ZN77_$LT$alloc..borrow..Cow$LT$str$GT$$u20$as$u20$core..ops..arith..AddAssign$GT$10add_assign17hc6979b9b05a4434dE]+0x1c): undefined reference to `__CxxFrameHandler3'
../target/release/rust_lib.lib(alloc-f297c401e81b90c6.alloc.67mh9if6-cgu.0.rcgu.o):alloc.67mh9if6-cgu:(.xdata[$cppxdata$_ZN5alloc3fmt6format17h6ff6ffb07b25780bE]+0x14): undefined reference to `__CxxFrameHandler3'
../target/release/rust_lib.lib(alloc-f297c401e81b90c6.alloc.67mh9if6-cgu.0.rcgu.o):alloc.67mh9if6-cgu:(.xdata[$cppxdata$_ZN5alloc3str56_$LT$impl$u20$alloc..borrow..ToOwned$u20$for$u20$str$GT$10clone_into17hb1ab464ba077e8e9E]+0x18): undefined reference to `__CxxFrameHandler3'
../target/release/rust_lib.lib(alloc-f297c401e81b90c6.alloc.67mh9if6-cgu.0.rcgu.o):alloc.67mh9if6-cgu:(.xdata[$cppxdata$_ZN5alloc3str21_$LT$impl$u20$str$GT$12to_lowercase17h79ed27ac3fe70658E]+0x1c): more undefined references to `__CxxFrameHandler3' follow
collect2.exe: error: ld returned 1 exit status
mingw32-make[2]: *** [CMakeFiles\rust_ffi.dir\build.make:87: rust_ffi.exe] Error 1
mingw32-make[1]: *** [CMakeFiles\Makefile2:72: CMakeFiles/rust_ffi.dir/all] Error 2
mingw32-make: *** [Makefile:83: all] Error 2
```

Building with simple type conversions (like a `const* c_double` to `const* f64`) and assignment `let x = 1` does not cause this error message. However, doing thing such as `let x : Vec<usize> = Vec::new()` does. `MinGW` and `msvc` were the only compilers I tested. 

As far as i could find there was no clear solution to this problem. All compilation was done (error free) with `msvc` build tools for the duration of the project. This was not too large of a problem for me, as `msvc` is requred to build the rust compiler on windows.

Additionally, similarly horrible error messages will occur when the rust-function argument type does not match the corresponding c++ header file type on all compilers. Tracking down these issues was extremely time consuming and made calling each function individually (instead of a single FFI function to handle everything) impractical.

```
// main.cpp
extern "C" {
    void rust_function(double x);
}
```
```
// lib.rs
use std::os::raw::c_int;
#[no_mangle]
rust_function(x: c_int) {}
```

will cause wall-of-text style error similar to the one above. # Rust-KF problems,  inconvenience, and takeaways.


### Conversion from array of c++ `Eigen` matrices to Rust `Nalgebra` matrices

A single `Eigen` matrix can be converted to an `Nalgebra` matrix without copying:

```
extern "C" {
   void eigen_to_nalgebra(double* matrix_ptr );
}

void pass_matrix() {
   Matrix3d square_mat = Matrix3d::Random();

   double* ptr = square_mat.data();

   // rust ffi call
   eigen_to_nalgebra(ptr);
}
```

```
use std::os::raw::c_double;

#[no_mangle]
pub unsafe extern "C" fn eigen_to_nalgebra(matrix_ptr: *const c_double) {
   // convert to rust types
   let float_ptr = matrix_ptr as *const f64;

   // slice of memory
   let slice = std::slice::from_raw_parts(float_ptr, 9);

   // convert to MatrixSlice (3x3), call .into() to convert to Mat3
   let matrix : Mat3 = na::MatrixSlice3::from_slice(hits_slice).into();

   print_! {m};
}
```

As far as I have found, there is no way to "reinterpret" the vector of `Eigen` matrices to `Nalgebra` matrices without doing some form of copying, such as in `measurement_and_hits_from_ptrs` in `src/ffi/rust_headers.rs` :

```
// simplified version of the original function
// make a vector of sensor hits ( 2-row vec )
#[no_mangle]
unsafe fn measurement_and_hits_from_ptrs(
   hits_ptr: *const c_double,
   sensor_count: usize ) {

   // convert to rust types
   let hits_ptr = hits_ptr as *const Real;
   let meas_cov_ptr = measurement_cov_ptr as *const Real;

   // sensor hits and measurement hits
   let mut hits: Vec<Vec2> = Vec::with_capacity(sensor_count);

   for i in 0..sensor_count as isize {
       // get pointers to the current position in memory
       let curr_hits_ptr = hits_ptr.offset(i * 2);

       // slice 2 values for hits (2x1 mat)
       let hits_slice = std::slice::from_raw_parts(curr_hits_ptr, 2);

       // Make static MatrixSlice from the bytes slice, call .into() to convert MatrixSlice -> Matrix
       let curr_hit = na::MatrixSlice2x1::from_slice(hits_slice).into();

       // push to vector
       hits.push(curr_hit)
   }

}
```

I suspect there is a way to do this since both `Eigen` and `Nalgebra` use column-major matrices. This might potentially involve `std::mem::transmute`, but I am unsure.

### Key take-aways

It is ideal to keep the surface area of callable functions at a minimum between rust and c++. Not only is there an overhead to every ffi call, compiler errors can be difficult to track down when linking two projects. When possible, it is advisable to write one function to accept c++ values, perform all rust calculations, and then return to c++. This, as opposed to calling each ffi function individually from c++, reduces the amount of headaches that can occur with the aforementioned errors.

## Convenient Macros

Macros were largely used to increase parity with c++ features and code cleanliness.

### `edit_matrix`

This arose out of the need to quickly change individual values of a matrix.

the following pieces of code are equivalent:

```

let mut matrix = Mat4::identity();
// set to two
let mut index = unsafe{ matrix.get_unchecked_mut((0,0))};
*index = 2;
// add 3 to index
let mut index_2 = unsafe{ matrix.get_unchecked_mut((1,1))};
*index_2 += 3;
```
```
   let mut matrix = Mat4::identity();
   edit_matrix!{matrix;
       [0,0] = 2,
       [1,1] += 3
   }
```

One advantage of this approach is the ease of finding out-of-bounds accesses and the locations of occurrence. For debugging purposes this:

```
let x = unsafe {
   matrix.get_unchecked_mut((1,1))
}
```
can be replaced in _one location_ with (a more general form of) this in `edit_matrix`:

```
let x = unsafe {
   match matrix.get_mut((1,1)) {
       Some(valid_index) => valid_index,
       None => {
           println!{"edit_matrix out of bounds:\nfile:{}\tline:{}", file!{}, line!{}}
           panic!{""}
       }
   }
}

```
This will print the file name and line number where the error occurred.

### `get_unchecked`

This quickly gets unchecked references to data in matrices, vectors, and arrays. For similar reasons to `edit_matrix`, opting into unchecked-bounds is made more safe by quickly being able to switch to checked-bounds with clear error messages. The following are equivalent:

```
let matrix = Mat4::identity();
let x = unsafe {matrix.get_unchecked((1,1))};
let y = unsafe {matrix.get_unchedked((2,2))};
let z = unsafe {matrix.get_unchedked((3,3))};
```

```
let matrix = Mat4::identity();
get_unchecked!{matrix;
   [1,1] => x,
   [2,2] => y,
   [3,3] => z
}
```

This helps significantly in isolating problems while increasing the isolation of unsafe code.


### Other

other macros such as `push`, `print_` , `reverse`, and `store_vec` provide convenience methods for working with vectors, iterators, and matrices. Full documentation for these can be found by opening the docs: `cargo doc --open`

One notable disadvantage of this macro approach is a somewhat higher overhead to understanding the codebase. Macros such as `get_unchecked` can be called in several different ways on different data types. This can cause some indirection when first reading through the project, whereas c++ has these features more or less built in.

