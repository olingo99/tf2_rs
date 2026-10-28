// ROS 2 install prefixes and C++ include directories, shared by build.rs and
// oracle/build.rs through `include!`. Both import `std::{env, path::PathBuf}`.

/// The sourced ROS distribution.
fn ros_distro() -> String {
    println!("cargo:rerun-if-env-changed=ROS_DISTRO");
    match env::var("ROS_DISTRO") {
        Ok(a) => a,
        Err(e) => {
            panic!("ROS_DISTRO not defined (source your ROS 2 setup). Error: {e}")
        }
    }
}

/// AMENT_PREFIX_PATH (overlay workspaces), then the default ROS prefix.
fn ros_prefixes(ros_distro: &str) -> Vec<PathBuf> {
    let mut prefixes: Vec<PathBuf> = Vec::new();

    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    if let Ok(ament) = env::var("AMENT_PREFIX_PATH") {
        for p in ament.split(':').filter(|s| !s.is_empty()) {
            prefixes.push(PathBuf::from(p));
        }
    }

    prefixes.push(PathBuf::from(format!("/opt/ros/{ros_distro}")));
    prefixes
}

fn include_dirs(prefixes: &[PathBuf]) -> Vec<PathBuf> {
    let mut dirs = Vec::new();
    for prefix in prefixes {
        let inc = prefix.join("include");
        if inc.exists() {
            dirs.push(inc.clone());

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
                "rosidl_buffer",
                "rosidl_typesupport_interface",
                "rcutils",
                "libstatistics_collector",
                "statistics_msgs",
                "rosidl_typesupport_introspection_cpp",
            ] {
                let nested = inc.join(pkg);
                if nested.exists() {
                    dirs.push(nested);
                }
            }
        }
    }

    for p in [
        "/usr/include/eigen3",
        "/usr/local/include/eigen3",
        "/usr/include",
    ] {
        if std::path::Path::new(p).exists() {
            dirs.push(PathBuf::from(p));
        }
    }
    dirs
}

fn link_search(prefixes: &[PathBuf]) {
    for prefix in prefixes {
        let lib = prefix.join("lib");
        if lib.exists() {
            println!("cargo:rustc-link-search=native={}", lib.display());
        }
    }
}
