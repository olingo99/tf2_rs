use std::{env, path::PathBuf};

fn main() {
    let mut b = cxx_build::bridge("src/ffi.rs");
    b.file("src/tf2_wrapper.cpp")
        .include("include")
        .flag_if_supported("-std=c++17");

    // Collect include/lib paths from AMENT_PREFIX_PATH (overlay workspaces) + default ROS prefix.
    let mut prefixes: Vec<PathBuf> = Vec::new();

    if let Ok(ament) = env::var("AMENT_PREFIX_PATH") {
        for p in ament.split(':').filter(|s| !s.is_empty()) {
            prefixes.push(PathBuf::from(p));
        }
    }

    prefixes.push(PathBuf::from("/opt/ros/jazzy"));

    for prefix in &prefixes {
        let inc = prefix.join("include");
        if inc.exists() {
            b.include(&inc);

            // Add nested include dirs when they exist.
            for pkg in [
                "tf2",
                "tf2_ros",
                "rclcpp",
                "rcl_interfaces",
                "service_msgs",
                "rcl",
                "rmw",
                "rcpputils",
                "rcl_yaml_param_parser",
                "type_description_interfaces",
                "rosidl_dynamic_typesupport",
                "tracetools",
                "tf2_geometry_msgs",
                "tf2_sensor_msgs",
                "geometry_msgs",
                "sensor_msgs",
                "std_msgs",
                "builtin_interfaces",
                "rosidl_runtime_cpp",
                "rosidl_runtime_c",
                "rosidl_typesupport_interface",
                "rcutils",
                "libstatistics_collector",
                "statistics_msgs",
                "rosidl_typesupport_introspection_cpp",
            ] {
                let nested = inc.join(pkg);
                if nested.exists() {
                    b.include(&nested);
                }
            }
        }
    }


    for p in ["/usr/include/eigen3", "/usr/local/include/eigen3", "/usr/include"] {
        if std::path::Path::new(p).exists() {
            b.include(p);
        }
    }

    b.compile("tf2_rs_tf2_wrapper");


    // Link search paths
    for prefix in &prefixes {
        let lib = prefix.join("lib");
        if lib.exists() {
            println!("cargo:rustc-link-search=native={}", lib.display());
        }
    }

    // Core TF2
    println!("cargo:rustc-link-lib=tf2");

    // doTransform specializations live here
    // println!("cargo:rustc-link-lib=tf2_geometry_msgs");
    // println!("cargo:rustc-link-lib=tf2_sensor_msgs");
    // println!("cargo:rustc-link-lib=tf2_ros");
    // println!("cargo:rustc-link-lib=rclcpp");


    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=src/tf2_wrapper.cpp");
    println!("cargo:rerun-if-changed=include/tf2_wrapper.h");
}
