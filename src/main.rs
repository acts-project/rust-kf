use nalgebra as na;
mod config;
mod geometry;
mod filter;
use config::*;
mod error;

macro_rules! test_data {
    ($len:expr; $($name:ident, $type:ident, $rows:expr, $cols:expr),+) => {
        $(  
            // if $cond == true{

            // }
            let mut $name = Vec::with_capacity($len);
            let var : $type = $type::new_random_generic($rows,$cols);
            for _ in 0..$len {
                $name.push(var.clone());
            }
        )+
    };
}

use na::*;
fn main() {
    
    test_data!{5;
        V_vec, Mat2, U2, U2,
        H_vec, Mat2x5, U2, U5,
        m_k_vec, Vec2, U2, U1
    }

    filter::linear::run(&V_vec, &H_vec, &m_k_vec);
}
