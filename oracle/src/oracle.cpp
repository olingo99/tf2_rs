// Test-only C++ reference for the Rust transforms, see src/lib.rs. Every
// function converts the arrays to the ROS message and calls the upstream
// tf2::doTransform specialisation for it.

#include "tf2_oracle.h"

#include "tf2_rs_oracle/src/lib.rs.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace tf2_rs_oracle {
namespace {

geometry_msgs::msg::TransformStamped to_msg(const TransformArray& t) {
  geometry_msgs::msg::TransformStamped out;
  out.transform.translation.x = t[0];
  out.transform.translation.y = t[1];
  out.transform.translation.z = t[2];
  out.transform.rotation.x = t[3];
  out.transform.rotation.y = t[4];
  out.transform.rotation.z = t[5];
  out.transform.rotation.w = t[6];
  return out;
}

geometry_msgs::msg::Pose to_pose(const std::array<double, 7>& a) {
  geometry_msgs::msg::Pose out;
  out.position.x = a[0];
  out.position.y = a[1];
  out.position.z = a[2];
  out.orientation.x = a[3];
  out.orientation.y = a[4];
  out.orientation.z = a[5];
  out.orientation.w = a[6];
  return out;
}

std::array<double, 7> from_pose(const geometry_msgs::msg::Pose& p) {
  return {p.position.x, p.position.y, p.position.z,
          p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
}

#ifndef TF2_RS_NO_IMU_TRANSFORM
geometry_msgs::msg::Vector3 to_vector(const std::array<double, 3>& a) {
  geometry_msgs::msg::Vector3 out;
  out.x = a[0];
  out.y = a[1];
  out.z = a[2];
  return out;
}

std::array<double, 3> from_vector(const geometry_msgs::msg::Vector3& v) {
  return {v.x, v.y, v.z};
}
#endif

}  // namespace

void do_transform_point(
    const std::array<double, 3>& p, const TransformArray& tf, std::array<double, 3>& out) {
  geometry_msgs::msg::Point in, result;
  in.x = p[0];
  in.y = p[1];
  in.z = p[2];
  tf2::doTransform(in, result, to_msg(tf));
  out = {result.x, result.y, result.z};
}

void do_transform_vector3_stamped(
    const std::array<double, 3>& v, const TransformArray& tf, std::array<double, 3>& out) {
  geometry_msgs::msg::Vector3Stamped in, result;
  in.vector.x = v[0];
  in.vector.y = v[1];
  in.vector.z = v[2];
  tf2::doTransform(in, result, to_msg(tf));
  out = {result.vector.x, result.vector.y, result.vector.z};
}

void do_transform_pose(
    const std::array<double, 7>& pose, const TransformArray& tf, std::array<double, 7>& out) {
  geometry_msgs::msg::Pose result;
  tf2::doTransform(to_pose(pose), result, to_msg(tf));
  out = from_pose(result);
}

void do_transform_pose_with_covariance_stamped(
    const std::array<double, 7>& pose,
    const std::array<double, 36>& covariance,
    const TransformArray& tf,
    std::array<double, 7>& out_pose,
    std::array<double, 36>& out_covariance) {
  geometry_msgs::msg::PoseWithCovarianceStamped in, result;
  in.pose.pose = to_pose(pose);
  in.pose.covariance = covariance;
  tf2::doTransform(in, result, to_msg(tf));
  out_pose = from_pose(result.pose.pose);
  out_covariance = result.pose.covariance;
}

bool do_transform_imu(const ImuData& imu, const TransformArray& tf, ImuData& out) {
#ifdef TF2_RS_NO_IMU_TRANSFORM
  (void)imu;
  (void)tf;
  (void)out;
  return false;
#else
  sensor_msgs::msg::Imu in, result;
  in.orientation.x = imu.orientation[0];
  in.orientation.y = imu.orientation[1];
  in.orientation.z = imu.orientation[2];
  in.orientation.w = imu.orientation[3];
  in.orientation_covariance = imu.orientation_covariance;
  in.angular_velocity = to_vector(imu.angular_velocity);
  in.angular_velocity_covariance = imu.angular_velocity_covariance;
  in.linear_acceleration = to_vector(imu.linear_acceleration);
  in.linear_acceleration_covariance = imu.linear_acceleration_covariance;
  tf2::doTransform(in, result, to_msg(tf));
  out.orientation = {
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w};
  out.orientation_covariance = result.orientation_covariance;
  out.angular_velocity = from_vector(result.angular_velocity);
  out.angular_velocity_covariance = result.angular_velocity_covariance;
  out.linear_acceleration = from_vector(result.linear_acceleration);
  out.linear_acceleration_covariance = result.linear_acceleration_covariance;
  return true;
#endif
}

}  // namespace tf2_rs_oracle
