use std::{env, path::PathBuf};

include!("ros_paths.rs");

fn main() {
    let prefixes = ros_prefixes(&ros_distro());

    let mut b = cxx_build::bridge("src/ffi.rs");
    b.file("src/tf2_wrapper.cpp")
        .include("include")
        .includes(include_dirs(&prefixes))
        .flag_if_supported("-std=c++17");
    b.compile("tf2_rs_tf2_wrapper");

    link_search(&prefixes);

    // Core TF2
    println!("cargo:rustc-link-lib=tf2");

    println!("cargo:rerun-if-changed=ros_paths.rs");
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=src/tf2_wrapper.cpp");
    println!("cargo:rerun-if-changed=include/tf2_wrapper.h");
}
