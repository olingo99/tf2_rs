use std::{env, path::PathBuf};

include!("../ros_paths.rs");

fn main() {
    let ros_distro = ros_distro();
    let prefixes = ros_prefixes(&ros_distro);

    let mut b = cxx_build::bridge("src/lib.rs");
    b.file("src/oracle.cpp")
        .include("include")
        .includes(include_dirs(&prefixes))
        .flag_if_supported("-std=c++17");
    // Humble's tf2_sensor_msgs has no Imu doTransform.
    if ros_distro == "humble" {
        b.define("TF2_RS_NO_IMU_TRANSFORM", None);
    }
    b.compile("tf2_rs_oracle");

    link_search(&prefixes);

    // tf2_geometry_msgs applies transforms through KDL.
    println!("cargo:rustc-link-lib=orocos-kdl");

    println!("cargo:rerun-if-changed=../ros_paths.rs");
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/oracle.cpp");
    println!("cargo:rerun-if-changed=include/tf2_oracle.h");
}
