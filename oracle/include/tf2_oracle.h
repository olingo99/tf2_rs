#pragma once

// Test-only C++ reference for the Rust transforms, see src/lib.rs.

#include <array>
#include <cstdint>

namespace tf2_rs_oracle {

// Defined in the cxx-generated header (oracle.rs.h).
struct ImuData;

using TransformArray = std::array<double, 7>;

void do_transform_point(
    const std::array<double, 3>& p, const TransformArray& tf, std::array<double, 3>& out);
void do_transform_vector3_stamped(
    const std::array<double, 3>& v, const TransformArray& tf, std::array<double, 3>& out);
void do_transform_pose(
    const std::array<double, 7>& pose, const TransformArray& tf, std::array<double, 7>& out);
void do_transform_pose_with_covariance_stamped(
    const std::array<double, 7>& pose,
    const std::array<double, 36>& covariance,
    const TransformArray& tf,
    std::array<double, 7>& out_pose,
    std::array<double, 36>& out_covariance);
bool do_transform_imu(const ImuData& imu, const TransformArray& tf, ImuData& out);

}  // namespace tf2_rs_oracle
