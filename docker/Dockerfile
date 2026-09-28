# Build tf2_rs against a ROS 2 binary install, from the repository root:
#   docker build -f docker/Dockerfile --build-arg ROS_DISTRO=jazzy .
# Supported ROS_DISTRO values: humble, jazzy, kilted, lyrical, rolling.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}

# Minimum supported Rust version of rclrs.
ARG RUST_VERSION=1.85.0

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-pip \
    && rm -rf /var/lib/apt/lists/*
RUN PIP_BREAK_SYSTEM_PACKAGES=1 pip3 install --no-cache-dir colcon-ros-cargo

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
    | sh -s -- -y --profile minimal --default-toolchain ${RUST_VERSION}
ENV PATH=/root/.cargo/bin:${PATH}

WORKDIR /workspace
COPY . src/tf2_rs
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-up-to tf2_rs
