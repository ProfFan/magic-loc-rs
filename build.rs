use std::env;

pub fn main() {
    let target = env::var("TARGET").unwrap();

    println!("cargo:info=TARGET={}", target);
}
