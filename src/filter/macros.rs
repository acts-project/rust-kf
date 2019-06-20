// macro to initialize vectors with a given capacity to reduce copy / paste
#[macro_export]
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


/// Push a value into an iterator. Saves repeating vec.push(asdad)
/// Easier to read this way.
#[macro_export]
macro_rules! push {
    ($($item:expr => $location:ident),*) => {
        $(
            $location.push($item);
        )*
    };
    (remove: $index:expr; $($pop_vec:ident => $dest_vec:ident),+) =>{
        $(
            let removed_val = $pop_vec.remove($index);
            push!{removed_val => $dest_vec};
        )+
    };
    (remove: $index:expr;$($pop_vec:ident),+ ) => {
        $($pop_vec.remove($index);)+
    };
}

#[macro_export]
macro_rules! get_unchecked {
    ($index:expr; $($vector:ident => $destination:ident),+) => {
        $(
            let $destination = 
                unsafe {
                    $vector.get($index).expect("get_unchecked!{} fetched something out of bounds")
                }
        )+
    };
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


/// Used for getting the next value of the iterator. Additionally, this macro allows
/// us to move the ownership of the data to "stagger" it. This does the following:
/// `current` at n => `previous` at n+1,
/// `next` at n => `current` at n+1,
/// new `next` value is fetched from the iterator. 
#[macro_export]
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
            
            // if $previous.is_none() {
            //     next!{$storage => $previous, $current, $next};
            // };
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

/// Reverse every iterator passed in. This is useful for the smoothing calculations
/// since we pass from the end to the front, but add items to iterators from 
/// front to back
#[macro_export]
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


/// Creates a variable amount of mutable empty iterators
#[macro_export]
macro_rules! empty {
    ($($name:ident),+) => {
        $(
            let mut $name = std::iter::empty();
        )+
    };
}
