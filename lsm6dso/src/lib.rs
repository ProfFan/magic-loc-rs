#![no_std]
#![feature(generic_const_exprs)]

pub mod ll;
pub mod uninitialized;

pub use uninitialized::LSM6DSO;
