use std::{env, path::PathBuf};

include!("ros_paths.rs");

fn main() {
    let prefixes = ros_prefixes(&ros_distro());
    let include_dirs = include_dirs(&prefixes);

    // Newer tf2_geometry_msgs also copy child_frame_id when transforming a
    // TransformStamped.
    println!("cargo:rustc-check-cfg=cfg(tf2_copies_child_frame_id)");
    if header_contains(
        &include_dirs,
        "tf2_geometry_msgs/tf2_geometry_msgs.hpp",
        "t_out.child_frame_id = t_in.child_frame_id",
    ) {
        println!("cargo:rustc-cfg=tf2_copies_child_frame_id");
    }

    // Older tf2_geometry_msgs add the input velocity to a transformed VelocityStamped
    // and keep its header.
    println!("cargo:rustc-check-cfg=cfg(tf2_velocity_adds_input)");
    if header_contains(
        &include_dirs,
        "tf2_geometry_msgs/tf2_geometry_msgs.hpp",
        "t_out.velocity.linear.x = out_vel.x() + t_in.velocity.linear.x",
    ) {
        println!("cargo:rustc-cfg=tf2_velocity_adds_input");
    }

    let mut b = cxx_build::bridge("src/ffi.rs");
    b.file("src/tf2_wrapper.cpp")
        .include("include")
        .includes(&include_dirs)
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
