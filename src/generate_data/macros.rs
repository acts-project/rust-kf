#[macro_export]
/// Quickly create a repetitive value
/// $name: name of variable to save to
/// $value: the value that will be repeated $take times
macro_rules! take {
    ($take:expr; $($name:ident , $value:expr),+) => {
        $(
            let $name = std::iter::repeat($value).take($take);
        )+
    };
}

#[macro_export]
/// Quickly clone and push to make paths
macro_rules! path {
    ($base:ident; $($to_push:expr => $varname:ident),+) => {
        $(
            let mut $varname = $base.clone().to_string();
            $varname.push_str(r"\");
            $varname.push_str(&$to_push);
        )+
    };
} 


#[macro_export]
/// Quickly map and create folder names, histogram names, etc for any changing values.generate_data
/// 
/// $value_iterable: the values that will be mappened into the State struct. These are the values
///                  we are testing. everything else in the struct will remain constant
/// $folder_save_string: the extension for how the folder will be named. This will be concatenated onto the 
///                  save location from config.rs
/// $hist_save_string: what we will name the histogram once it is named (python handles this )
/// $field_to_change: the field (or field path) on the State structwhose value will be replaced by each index of $value iterable.
///                   this variable is repeating so we can access the Uncertainties struct. For example, passing in stdev.point_std
///                   would error (without this) since the `.` invalidates the ident `stdev`. the repetition will expand to the correct
///                   path in the macro. 
macro_rules! generate_data {

    ($value_iterable:ident, $folder_save_string:expr, $hist_save_string:expr, $($field_to_change:ident).+) => {
        // zip together the value we are replacing the default of State with and the count at which it occurs.generate_data
        // this is done becuase we cant usea mutable reference outside the iterator because its parallel
        let len = $value_iterable.len();
        let new_iter = $value_iterable.into_iter().zip(0..len).collect::<Vec<_>>();


        new_iter.par_iter()
            .for_each(|(x, count)|{
                let hist_save_name = format!{$hist_save_string, count};

                let mut path = CSV_SAVE_LOCATION.clone().to_string();
                path.push_str(&format!{$folder_save_string, x});

                let mut data_struct = State::default(&path, &hist_save_name);

                // expands to the correct field of the struct that we are editing. if $field_to_change is stdev.point_std
                // then the following line expands to datastruct.stdev.point_std = **x
                data_struct$(.$field_to_change)+ = **x;             

                $crate::generate_data::run::general::run(data_struct)
            });

    };
}

