#[macro_export]
/// Quickly define pub constants and ignore uppercase naming warnings
// To suppress warnings and keep variable names parallel to
// https://gitlab.cern.ch/acts/acts-core/blob/master/Core/include/Acts/Utilities/detail/DefaultParameterDefinitions.hpp
// we use a macro so we dont need repetitive #[allow(_)] while not allowing them globally.
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
