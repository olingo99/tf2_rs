FROM ros:kilted
ARG DEBIAN_FRONTEND=noninteractive


RUN apt update && apt install -y \
    curl \
    git \
    libclang-dev \
    python3-pip \
    python3-vcstool \
    ros-kilted-example-interfaces \
    ros-kilted-test-msgs

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain 1.85.0 -y
ENV PATH=/root/.cargo/bin:$PATH

RUN pip install --break-system-packages colcon-cargo colcon-ros-cargo 

RUN mkdir -p /workspace/src
WORKDIR /workspace


RUN git clone -b kilted https://github.com/ros2/common_interfaces.git src/common_interfaces
RUN git clone -b kilted https://github.com/ros2/example_interfaces.git src/example_interfaces
RUN git clone -b kilted https://github.com/ros2/rcl_interfaces.git src/rcl_interfaces
RUN git clone -b kilted https://github.com/ros2/rosidl_core.git src/rosidl_core
RUN git clone -b kilted https://github.com/ros2/rosidl_defaults.git src/rosidl_defaults
RUN git clone -b kilted https://github.com/ros2/unique_identifier_msgs.git src/unique_identifier_msgs
RUN git clone https://github.com/ros2-rust/rosidl_rust.git src/rosidl_rust

RUN git clone -b kilted https://github.com/ros2/geometry2.git src/geometry2
RUN git clone https://github.com/olingo99/tf2_rs src/tf2_rs

RUN . /opt/ros/kilted/setup.sh && colcon build --packages-up-to tf2_rs