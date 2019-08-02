use super::geometry::{Rectangle, Trapezoid};
#[macro_export]
/// This macro is to quickly define pub constants
/// To suppress warnings and keep variable names parallel to
/// https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Utilities/detail/DefaultParameterDefinitions.hpp
/// we use a macro so we dont need repetitive #[allow(_)] while not allowing them globally.
macro_rules! def_constant {
    // most used branch. allows not specifying the type repetitvely for
    // each item
    //type: type of the constant
    //varname: name of the constant being stored
    //val: the value the constant is set to
    ($type:ty; $($var_name:ident = $val:expr),+) => {
        $(
            // expand to the more specific version of the macro
            def_constant!($var_name :  $type = $val);
        )+
    };
    // General form for the macro used for mixtures of types (specified per constant)
    //varname: name of the constant being stored
    //type: type of the constant
    //val: the value the constant is set to
    ($($var_name:ident : $type:ty = $val:expr),+) => {
        $(
            #[allow(non_upper_case_globals)]
            pub const $var_name : $type = $val;
        )+
    };
}

#[macro_export]
/// Quickly generate test data
macro_rules! test_data {
    ($len:expr; $($name:ident, $type:ident),+) => {
        $(
            let mut $name = Vec::with_capacity($len);
            let var : $type = $type::new_random();
            for _ in 0..$len {
                $name.push(var.clone());
            }
        )+
    };
    (mat; $len:expr; $name:ident, $type:ident) => {
        let mut $name = Vec::with_capacity($len);
        for _ in 0..$len {
            let default = $type::default();
            $name.push(default)
        }
    };
}
