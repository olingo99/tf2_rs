use std::{env, path::PathBuf};

include!("../ros_paths.rs");

fn main() {
    let prefixes = ros_prefixes(&ros_distro());
    let include_dirs = include_dirs(&prefixes);

    let mut b = cxx_build::bridge("src/lib.rs");
    b.file("src/oracle.cpp")
        .include("include")
        .includes(&include_dirs)
        .flag_if_supported("-std=c++17");
    // Older tf2_sensor_msgs (Humble) have no Imu or MagneticField doTransform.
    for (msg, define) in [
        ("sensor_msgs::msg::Imu", "TF2_RS_NO_IMU_TRANSFORM"),
        (
            "sensor_msgs::msg::MagneticField",
            "TF2_RS_NO_MAGNETIC_FIELD_TRANSFORM",
        ),
    ] {
        if !header_contains(&include_dirs, "tf2_sensor_msgs/tf2_sensor_msgs.hpp", msg) {
            b.define(define, None);
        }
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
