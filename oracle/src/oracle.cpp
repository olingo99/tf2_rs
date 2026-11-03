// Test-only C++ reference for the Rust transforms, see src/lib.rs. Every
// function converts the shared structs to the ROS message and calls the upstream
// tf2::doTransform specialisation for it.

#include "tf2_oracle.h"

#include "tf2_rs_oracle/src/lib.rs.h"

#include <stdexcept>
#include <string>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace tf2_rs_oracle {
namespace {

std_msgs::msg::Header to_msg(const HeaderData& h) {
  std_msgs::msg::Header out;
  out.stamp.sec = h.sec;
  out.stamp.nanosec = h.nanosec;
  out.frame_id = std::string(h.frame_id);
  return out;
}

HeaderData from_msg(const std_msgs::msg::Header& h) {
  return {h.stamp.sec, h.stamp.nanosec, h.frame_id};
}

// Point and Vector3 have the same fields.
template<class Msg>
Msg to_xyz(const Vec3Data& v) {
  Msg out;
  out.x = v.x;
  out.y = v.y;
  out.z = v.z;
  return out;
}

template<class Msg>
Vec3Data from_xyz(const Msg& m) {
  return {m.x, m.y, m.z};
}

geometry_msgs::msg::Quaternion to_msg(const QuatData& q) {
  geometry_msgs::msg::Quaternion out;
  out.x = q.x;
  out.y = q.y;
  out.z = q.z;
  out.w = q.w;
  return out;
}

QuatData from_msg(const geometry_msgs::msg::Quaternion& q) {
  return {q.x, q.y, q.z, q.w};
}

geometry_msgs::msg::Pose to_msg(const PoseData& p) {
  geometry_msgs::msg::Pose out;
  out.position = to_xyz<geometry_msgs::msg::Point>(p.position);
  out.orientation = to_msg(p.orientation);
  return out;
}

PoseData from_msg(const geometry_msgs::msg::Pose& p) {
  return {from_xyz(p.position), from_msg(p.orientation)};
}

geometry_msgs::msg::TransformStamped to_msg(const TfData& t) {
  geometry_msgs::msg::TransformStamped out;
  out.header = to_msg(t.header);
  out.child_frame_id = std::string(t.child_frame_id);
  out.transform.translation = to_xyz<geometry_msgs::msg::Vector3>(t.translation);
  out.transform.rotation = to_msg(t.rotation);
  return out;
}

TfData from_msg(const geometry_msgs::msg::TransformStamped& t) {
  return {
    from_msg(t.header), t.child_frame_id, from_xyz(t.transform.translation),
    from_msg(t.transform.rotation)};
}

geometry_msgs::msg::Point32 to_msg(const Point32Data& p) {
  geometry_msgs::msg::Point32 out;
  out.x = p.x;
  out.y = p.y;
  out.z = p.z;
  return out;
}

Point32Data from_msg(const geometry_msgs::msg::Point32& p) {
  return {p.x, p.y, p.z};
}

geometry_msgs::msg::Polygon to_msg(const PolygonData& p) {
  geometry_msgs::msg::Polygon out;
  out.points.reserve(p.points.size());
  for (const auto& point : p.points) {
    out.points.push_back(to_msg(point));
  }
  return out;
}

PolygonData from_msg(const geometry_msgs::msg::Polygon& p) {
  PolygonData out;
  out.points.reserve(p.points.size());
  for (const auto& point : p.points) {
    out.points.push_back(from_msg(point));
  }
  return out;
}

geometry_msgs::msg::Transform to_msg(const TransformMsgData& t) {
  geometry_msgs::msg::Transform out;
  out.translation = to_xyz<geometry_msgs::msg::Vector3>(t.translation);
  out.rotation = to_msg(t.rotation);
  return out;
}

TransformMsgData from_msg(const geometry_msgs::msg::Transform& t) {
  return {from_xyz(t.translation), from_msg(t.rotation)};
}

geometry_msgs::msg::Wrench to_msg(const WrenchData& w) {
  geometry_msgs::msg::Wrench out;
  out.force = to_xyz<geometry_msgs::msg::Vector3>(w.force);
  out.torque = to_xyz<geometry_msgs::msg::Vector3>(w.torque);
  return out;
}

WrenchData from_msg(const geometry_msgs::msg::Wrench& w) {
  return {from_xyz(w.force), from_xyz(w.torque)};
}

geometry_msgs::msg::PoseWithCovariance to_msg(const PoseWithCovarianceData& p) {
  geometry_msgs::msg::PoseWithCovariance out;
  out.pose = to_msg(p.pose);
  out.covariance = p.covariance;
  return out;
}

PoseWithCovarianceData from_msg(const geometry_msgs::msg::PoseWithCovariance& p) {
  return {from_msg(p.pose), p.covariance};
}

sensor_msgs::msg::PointCloud2 to_msg(const CloudData& c) {
  sensor_msgs::msg::PointCloud2 out;
  out.header = to_msg(c.header);
  out.height = c.height;
  out.width = c.width;
  out.fields.reserve(c.fields.size());
  for (const auto& f : c.fields) {
    sensor_msgs::msg::PointField field;
    field.name = std::string(f.name);
    field.offset = f.offset;
    field.datatype = f.datatype;
    field.count = f.count;
    out.fields.push_back(std::move(field));
  }
  out.is_bigendian = c.is_bigendian;
  out.point_step = c.point_step;
  out.row_step = c.row_step;
  out.data.assign(c.data.data(), c.data.data() + c.data.size());
  out.is_dense = c.is_dense;
  return out;
}

CloudData from_msg(const sensor_msgs::msg::PointCloud2& c) {
  CloudData out;
  out.header = from_msg(c.header);
  out.height = c.height;
  out.width = c.width;
  out.fields.reserve(c.fields.size());
  for (const auto& f : c.fields) {
    out.fields.push_back(PointFieldData{f.name, f.offset, f.datatype, f.count});
  }
  out.is_bigendian = c.is_bigendian;
  out.point_step = c.point_step;
  out.row_step = c.row_step;
  out.data.reserve(c.data.size());
  for (uint8_t byte : c.data) {
    out.data.push_back(byte);
  }
  out.is_dense = c.is_dense;
  return out;
}

}  // namespace

Vec3Data do_transform_point(const Vec3Data& p, const TfData& tf) {
  geometry_msgs::msg::Point result;
  tf2::doTransform(to_xyz<geometry_msgs::msg::Point>(p), result, to_msg(tf));
  return from_xyz(result);
}

PointStampedData do_transform_point_stamped(const PointStampedData& p, const TfData& tf) {
  geometry_msgs::msg::PointStamped in, result;
  in.header = to_msg(p.header);
  in.point = to_xyz<geometry_msgs::msg::Point>(p.point);
  tf2::doTransform(in, result, to_msg(tf));
  return {from_msg(result.header), from_xyz(result.point)};
}

Point32Data do_transform_point32(const Point32Data& p, const TfData& tf) {
  geometry_msgs::msg::Point32 result;
  tf2::doTransform(to_msg(p), result, to_msg(tf));
  return from_msg(result);
}

PolygonData do_transform_polygon(const PolygonData& p, const TfData& tf) {
  geometry_msgs::msg::Polygon result;
  tf2::doTransform(to_msg(p), result, to_msg(tf));
  return from_msg(result);
}

PolygonStampedData do_transform_polygon_stamped(const PolygonStampedData& p, const TfData& tf) {
  geometry_msgs::msg::PolygonStamped in, result;
  in.header = to_msg(p.header);
  in.polygon = to_msg(p.polygon);
  tf2::doTransform(in, result, to_msg(tf));
  return {from_msg(result.header), from_msg(result.polygon)};
}

Vec3Data do_transform_vector3(const Vec3Data& v, const TfData& tf) {
  geometry_msgs::msg::Vector3 result;
  tf2::doTransform(to_xyz<geometry_msgs::msg::Vector3>(v), result, to_msg(tf));
  return from_xyz(result);
}

Vector3StampedData do_transform_vector3_stamped(const Vector3StampedData& v, const TfData& tf) {
  geometry_msgs::msg::Vector3Stamped in, result;
  in.header = to_msg(v.header);
  in.vector = to_xyz<geometry_msgs::msg::Vector3>(v.vector);
  tf2::doTransform(in, result, to_msg(tf));
  return {from_msg(result.header), from_xyz(result.vector)};
}

QuatData do_transform_quaternion(const QuatData& q, const TfData& tf) {
  geometry_msgs::msg::Quaternion result;
  tf2::doTransform(to_msg(q), result, to_msg(tf));
  return from_msg(result);
}

QuaternionStampedData do_transform_quaternion_stamped(
    const QuaternionStampedData& q, const TfData& tf) {
  geometry_msgs::msg::QuaternionStamped in, result;
  in.header = to_msg(q.header);
  in.quaternion = to_msg(q.quaternion);
  tf2::doTransform(in, result, to_msg(tf));
  return {from_msg(result.header), from_msg(result.quaternion)};
}

PoseData do_transform_pose(const PoseData& pose, const TfData& tf) {
  geometry_msgs::msg::Pose result;
  tf2::doTransform(to_msg(pose), result, to_msg(tf));
  return from_msg(result);
}

PoseStampedData do_transform_pose_stamped(const PoseStampedData& pose, const TfData& tf) {
  geometry_msgs::msg::PoseStamped in, result;
  in.header = to_msg(pose.header);
  in.pose = to_msg(pose.pose);
  tf2::doTransform(in, result, to_msg(tf));
  return {from_msg(result.header), from_msg(result.pose)};
}

PoseWithCovarianceData do_transform_pose_with_covariance(
    const PoseWithCovarianceData& pose, const TfData& tf) {
  geometry_msgs::msg::PoseWithCovariance result;
  tf2::doTransform(to_msg(pose), result, to_msg(tf));
  return from_msg(result);
}

PoseWithCovarianceStampedData do_transform_pose_with_covariance_stamped(
    const PoseWithCovarianceStampedData& pose, const TfData& tf) {
  geometry_msgs::msg::PoseWithCovarianceStamped in, result;
  in.header = to_msg(pose.header);
  in.pose = to_msg(pose.pose);
  tf2::doTransform(in, result, to_msg(tf));
  return {from_msg(result.header), from_msg(result.pose)};
}

TransformMsgData do_transform_transform(const TransformMsgData& t, const TfData& tf) {
  geometry_msgs::msg::Transform result;
  tf2::doTransform(to_msg(t), result, to_msg(tf));
  return from_msg(result);
}

TfData do_transform_transform_stamped(const TfData& t, const TfData& tf) {
  geometry_msgs::msg::TransformStamped result;
  tf2::doTransform(to_msg(t), result, to_msg(tf));
  return from_msg(result);
}

WrenchData do_transform_wrench(const WrenchData& w, const TfData& tf) {
  geometry_msgs::msg::Wrench result;
  tf2::doTransform(to_msg(w), result, to_msg(tf));
  return from_msg(result);
}

WrenchStampedData do_transform_wrench_stamped(const WrenchStampedData& w, const TfData& tf) {
  geometry_msgs::msg::WrenchStamped in, result;
  in.header = to_msg(w.header);
  in.wrench = to_msg(w.wrench);
  tf2::doTransform(in, result, to_msg(tf));
  return {from_msg(result.header), from_msg(result.wrench)};
}

VelocityStampedData do_transform_velocity_stamped(const VelocityStampedData& v, const TfData& tf) {
  geometry_msgs::msg::VelocityStamped in, result;
  in.header = to_msg(v.header);
  in.body_frame_id = std::string(v.body_frame_id);
  in.reference_frame_id = std::string(v.reference_frame_id);
  in.velocity.linear = to_xyz<geometry_msgs::msg::Vector3>(v.velocity.linear);
  in.velocity.angular = to_xyz<geometry_msgs::msg::Vector3>(v.velocity.angular);
  tf2::doTransform(in, result, to_msg(tf));
  return {
    from_msg(result.header), result.body_frame_id, result.reference_frame_id,
    {from_xyz(result.velocity.linear), from_xyz(result.velocity.angular)}};
}

bool do_transform_imu(const ImuData& imu, const TfData& tf, ImuData& out) {
#ifdef TF2_RS_NO_IMU_TRANSFORM
  (void)imu;
  (void)tf;
  (void)out;
  return false;
#else
  sensor_msgs::msg::Imu in, result;
  in.header = to_msg(imu.header);
  in.orientation = to_msg(imu.orientation);
  in.orientation_covariance = imu.orientation_covariance;
  in.angular_velocity = to_xyz<geometry_msgs::msg::Vector3>(imu.angular_velocity);
  in.angular_velocity_covariance = imu.angular_velocity_covariance;
  in.linear_acceleration = to_xyz<geometry_msgs::msg::Vector3>(imu.linear_acceleration);
  in.linear_acceleration_covariance = imu.linear_acceleration_covariance;
  tf2::doTransform(in, result, to_msg(tf));
  out.header = from_msg(result.header);
  out.orientation = from_msg(result.orientation);
  out.orientation_covariance = result.orientation_covariance;
  out.angular_velocity = from_xyz(result.angular_velocity);
  out.angular_velocity_covariance = result.angular_velocity_covariance;
  out.linear_acceleration = from_xyz(result.linear_acceleration);
  out.linear_acceleration_covariance = result.linear_acceleration_covariance;
  return true;
#endif
}

bool do_transform_magnetic_field(
    const MagneticFieldData& mag, const TfData& tf, MagneticFieldData& out) {
#ifdef TF2_RS_NO_MAGNETIC_FIELD_TRANSFORM
  (void)mag;
  (void)tf;
  (void)out;
  return false;
#else
  sensor_msgs::msg::MagneticField in, result;
  in.header = to_msg(mag.header);
  in.magnetic_field = to_xyz<geometry_msgs::msg::Vector3>(mag.magnetic_field);
  in.magnetic_field_covariance = mag.magnetic_field_covariance;
  tf2::doTransform(in, result, to_msg(tf));
  out.header = from_msg(result.header);
  out.magnetic_field = from_xyz(result.magnetic_field);
  out.magnetic_field_covariance = result.magnetic_field_covariance;
  return true;
#endif
}

CloudData do_transform_pointcloud2(const CloudData& cloud, const TfData& tf) {
  // The upstream iterators take &data.front(), undefined for empty data (an
  // assertion failure where libstdc++ checks it, as on Lyrical and Rolling).
  if (cloud.data.empty()) {
    throw std::invalid_argument("upstream doTransform is undefined for a cloud without data");
  }
  sensor_msgs::msg::PointCloud2 result;
  tf2::doTransform(to_msg(cloud), result, to_msg(tf));
  return from_msg(result);
}

}  // namespace tf2_rs_oracle
