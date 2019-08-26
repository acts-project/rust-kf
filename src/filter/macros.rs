#[macro_export]
// macro to initialize vectors with a given capacity to reduce copy / paste
macro_rules! store_vec {
    // $name: the name of the variable
    // $type: Type of data stored in vector (Mat5 / Vec5)
    // $capacity: How much space to allocate
    ($capacity:expr ; $($name:ident : $type:ty),+ ) => {
        $(
            let mut $name : Vec<$type> = Vec::with_capacity($capacity);
        )+
        // let mut $name: Vec<$type> = Vec::with_capacity($capacity);
    };
}

#[macro_export]
/// Push a value into an iterator. Saves repeating vec.push(asdad)
/// Easier to read this way.
macro_rules! push {
    ($($item:expr => $location:ident),*) => {
        $(
            $location.push($item);
        )*
    };
    (clone: $index:expr; $($pop_vec:ident => $dest_vec:ident),+) =>{
        $(
            let removed_val = $pop_vec.get($index).expect("could not pop").clone();
            push!{removed_val => $dest_vec};
        )+
    };
    (remove: $index:expr;$($pop_vec:ident),+ ) => {
        $($pop_vec.remove($index);)+
    };
}

#[macro_export]
/// Get a reference to an item in a vector using .get_unchecked(_)
///
/// # Examples
///
/// Same index from multiple vectors
/// ```
/// use kalman_rs::{config::*, get_unchecked};
///
/// let one_two = Vec2::new(1., 2.);
/// let two_three = Vec2::new(2., 3.);
/// let three_four = Vec2::new(3., 4.);
///
/// // get first index of each vector
/// get_unchecked!{0;
///     one_two => one,
///     two_three => two,
///     three_four=> three
/// }
///
/// assert_eq!{one, &1.}
/// assert_eq!{two, &2.}
/// assert_eq!{three,&3.}
///
/// ```
/// different indexes from different matricies:
/// ```
/// use kalman_rs::{config::*, get_unchecked};
///
/// let matrix = Mat3::identity();
/// let matrix2 = Mat3::zeros();
///  
/// get_unchecked!{
///     matrix[0,0] => top_left_identity,
///     matrix2[1,1] => top_left_zero
/// }
///
/// assert_eq!{top_left_identity, &1.}
/// assert_eq!{top_left_zero, &0.}
///
/// ```
/// Multiple indexes from the same matrix:
/// ```
/// use kalman_rs::{config::*, get_unchecked};
///
/// // | 1. 2. |
/// // | 3. 4. |
///
/// let matrix = Mat2::new(1., 2., 3., 4.);
///
/// get_unchecked!{matrix;
///     [0, 0] => one,
///     [1, 0] => three,
///     [1, 1] => four
///     
/// }
///
/// assert_eq!{one, &1.}
/// assert_eq!{three, &3.}
/// assert_eq!{four, &4.}
/// ```
/// Different indexes from the same vector
/// ```
/// use kalman_rs::{config::*, get_unchecked};
///
/// let sample_vec = Vec5::new(1., 2., 3., 4., 5.);
///
/// // using `vector;` at the start removes the need for brackets
/// // around the index
/// get_unchecked!{vector; sample_vec;
///     3 => four
/// }
///
/// assert_eq!{four, &4.}
///
/// ```
///
macro_rules! get_unchecked {
    // get specific indexes from different vectors
    ($($matrix:ident[$row:expr, $column:expr] => $destination:ident),+) => {
        $(
            get_unchecked!{@VECTOR; $matrix[($row, $column)] => $destination}
        )+
    };
    // get multiple indexes from the same matrix
    ($matrix:ident; $([$row:expr, $column:expr] => $destination:ident),+) => {
        $(
            get_unchecked!{@VECTOR; $matrix[($row, $column)] => $destination}
        )+
    };
    // get specific indexes from different vectors
    ($($vector:ident[$index:expr] => $destination:ident),+) => {
        $(
            get_unchecked!{@VECTOR; $vector[$index] => $destination}
        )+
    };
    // get multiple indexes from the same vector without brackets [] around the index
    (vector; $vector:ident; $($index:expr => $destination:ident),+) => {
        $(
            get_unchecked!{@VECTOR; $vector[$index] => $destination}
        )+
    };
    // get the same index from multiple vectors
    ($index:expr; $($vector:ident => $destination:ident),+) => {
        $(
            get_unchecked!{@VECTOR; $vector[$index] => $destination}
        )+
    };
    // Main branch that all other branches of the macro will call
    (@VECTOR; $vector:ident[$index:expr] => $destination:ident ) => {
        let $destination =
            // unsafe {
                // $vector.get_unchecked($index)
            // };
                match $vector.get($index) {
                    Some(val) => val,
                    None=> {
                        println!{"get unchecked out of bounds:\nfile:{}\tline:{}", file!{}, line!{}}
                        panic!{""}
                    }
                };
    }
}

#[macro_export]
/// Fetch the lengths of all iterators passed in. Used for debug
macro_rules! length {
    ($($var:ident),+) => {
         $(
             let coll: Vec<_> = $var.collect();
             dbg!{coll.len()};
         )+
    };
}

#[macro_export]
/// Used for getting the next value of the iterator. Additionally, this macro allows
/// us to move the ownership of the data to "stagger" it. This does the following:
/// `current` at n => `previous` at n+1,
/// `next` at n => `current` at n+1,
/// new `next` value is fetched from the iterator.
macro_rules! next {
    // move the value from `next` to `current`, from `current` to `previous`, and
    // fetch a new value from the iterator for `next`. This is done for the staggered
    // steps of the smoothing calculations
    //
    // storage: iterator we store values in
    // previous / current / next: variables from scope we are modifying
    //                            to new values
    ($($storage:ident => $previous:ident, $current:ident, $next:ident),+) => {
        $(
            let $previous = $current;
            let $current = $next;
            next!{init: $storage => $next};

        )+
    };
    //this unwraps each value of the iterator. useful for initializing values for `next`
    // and `current` before the start of the smoothing loop since the first iteration
    // of the smoothing loop will call `next!` and immediatly shift the variables.
    // This is also important for future code where we handle for "holes" since we
    // would have to inline code for matching Some(_) or None in the loop.
    (init: $($iterator:ident => $store_location:ident),+) => {
        $(
            let $store_location = $iterator.next().unwrap();
        )+
    };
}

#[macro_export]
macro_rules! into_iter {
    ($iterator:ident) => {
        let $iterator = $iterator.into_iter();
    };
}

#[macro_export]
/// Reverse every iterator passed in. This is useful for the smoothing calculations
/// since we pass from the end to the front, but add items to iterators from
/// front to back
macro_rules! reverse {
    ($($iterator:ident),+) => {
        $(
            let mut $iterator = $iterator.rev();
        )+
    };
    (into: $($iterator:ident),+) =>{
        $(
            let $iterator =  $iterator.into_iter();
            reverse!($iterator);

        )+
    };
    (base: $($iterator:ident),+) => {
        $(
            let mut $iterator = $iterator.rev();
        )+
    };
}

#[macro_export]
/// Edit a [row, column] index in a matrix with a given equality expression (= , /=, +=, ... etc)
///
/// # Examples
///
/// ```
/// use kalman_rs as krs;
/// use krs::config::*;
/// use krs::edit_matrix;
///
/// let mut matrix_zeros = Mat2::zeros();
/// let id_matrix = Mat2::identity();
///
/// // fill the diagonal with 1's
/// edit_matrix!{matrix_zeros;
///     [0,0] = 1.0,
///     [1,1] = 1.0
/// }
///
/// assert_eq!{matrix_zeros, id_matrix}
/// ```
///
/// ```
/// use kalman_rs::{config::*, edit_matrix};
///
/// let id_matrix = Mat2::identity();
///
/// // create a matrix where the diagonals are all 5.
/// let mut five_diagonal = Mat2::zeros();
/// five_diagonal.fill_diagonal(5.);
///
///
/// // divide all indexes by 5.
/// edit_matrix!{five_diagonal;
///     [0,0] /= 5.,
///     [1,1] /= 5.
/// }
///
/// assert_eq!{id_matrix, five_diagonal}
///
/// ```
macro_rules! edit_matrix {
    ($matrix_name:ident; $([$row:expr, $col:expr] $operation:tt $new_value:expr),+) => {
        $(
            // indexing for get() methods is done linearly. Instead of .get(3,3) for the bottom right
            // corner of a 4x4 matrix we must do .get(15). This line calculates what that index is.
            let value =
                // unsafe {
                    // $matrix_name.get_unchecked_mut(($row, $col))
                // };

                    match $matrix_name.get_mut(($row, $col)) {
                        Some(val) => val,
                        None=> {
                            println!{"edit_matrix out of bounds:\nfile:{}\tline:{}", file!{}, line!{}}
                            panic!{""}
                        }
                    };
            *value $operation $new_value;

        )+
    };

    ($vector_name:ident; $([$row:expr] $operation:tt $new_value:expr),+) => {
        $(
            edit_matrix!{$vector_name; [$row, 0] $operation $new_value}

        )+
    };
}

#[macro_export]
/// Poor man's version of dbg!{...} to use `fmt::Display` instead of `Debug`. Prints matricies nicely  
///
/// # Examples
///
/// ```
/// use kalman_rs::{print_, config::*};
///
/// let matrix = Mat3::zeros();
/// let other_mat = Mat2::identity();
///
/// print_!{matrix, other_mat}
///
/// // output:
///
/// //[src\filter\macros.rs:9] matrix =
/// //  ┌       ┐
/// //  │ 0 0 0 │
/// //  │ 0 0 0 │
/// //  │ 0 0 0 │
/// //  └       ┘
/// //
/// //
/// //[src\filter\macros.rs:9] other_mat =
/// //  ┌     ┐
/// //  │ 1 0 │
/// //  │ 0 1 │
/// //  └     ┘
///
/// ```
macro_rules! print_ {
    ($($val:expr),*) => {
        $(

        println!("[{}:{}] {} = {}",
            file!(), line!(), stringify!($val), $val);

        )*
    };
}
