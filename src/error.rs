/// Implement `From` trait for user-specified enums
/// 
/// Calling impl_from!(empty: ... ) implements for a C-like enum
/// Calling impl_from!(....) implements a function like enum

// from_type: Type that will be converted away from
// to_type: Destination enum that we are converting to
// subtype: the path to the branch of to_type that from_type will be converted to
#[macro_export]
macro_rules! impl_from {
    // catch to expand to a function-like enum
    (empty: $from_type:ident, $to_type:ty, $subtype:expr) => {
        impl_from!(full: $from_type, $to_type, $subtype, exp_empty)
    };// will expand to call impl_from!(exp_empty: ...)  ^^^^^^^

    // catch for C-like enums
    ($from_type:ident, $to_type:ty, $subtype:expr) => {
        impl_from!(full: $from_type, $to_type, $subtype, exp_full)
    };// will expand to call impl_from!(exp_full: ...)   ^^^^^^^

    // this branch will be called always. interior macro will expand 
    // depending on if the implementing type expects a C-link enum or not
    (full: $from_type:ident, $to_type:ty, $subtype:expr, $expansion:ident) => {
        impl From<$from_type> for $to_type {
            impl_from!($expansion: $from_type, $subtype);
        }
    };

    //interior expansion function if it is a C-like enum
    (exp_empty: $from_type:ident, $subtype:expr) => {
        fn from(error: $from_type) -> Self {
            $subtype
        }
    };
    // interior expansion function if it is a function-link enum
    (exp_full: $from_type:ident, $subtype:expr) => {
        fn from(error: $from_type) -> Self {
            $subtype(error)
        }
    };
}

#[derive(Debug)]
pub enum Error{
    Matrix(MatrixError),
    Sensor(SensorError)
}

#[derive(Debug)]
pub enum MatrixError {
    NonInvertible,
}

#[derive(Debug)]
pub enum SensorError {
    OutsideSensorBounds
}

// this function is only here to ensure that all `std::From` trait implementations 
// are correctly expanded at compile time. It never needs to be called
#[allow(dead_code)]
fn init_from() {
    // MatrixError
    impl_from!(MatrixError, Error, Error::Matrix);
    impl_from!(empty: i32, MatrixError, MatrixError::NonInvertible);
    
    //SensorError
    impl_from!(SensorError, Error, Error::Sensor);
}