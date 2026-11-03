#pragma once

// Test-only C++ reference for the Rust transforms, see src/lib.rs.

namespace tf2_rs_oracle {

// Defined in the cxx-generated header (lib.rs.h).
struct Vec3Data;
struct QuatData;
struct PoseData;
struct PoseWithCovarianceData;
struct PointStampedData;
struct Point32Data;
struct PolygonData;
struct PolygonStampedData;
struct Vector3StampedData;
struct QuaternionStampedData;
struct PoseStampedData;
struct PoseWithCovarianceStampedData;
struct TransformMsgData;
struct TfData;
struct WrenchData;
struct WrenchStampedData;
struct VelocityStampedData;
struct ImuData;
struct MagneticFieldData;
struct CloudData;

Vec3Data do_transform_point(const Vec3Data& p, const TfData& tf);
PointStampedData do_transform_point_stamped(const PointStampedData& p, const TfData& tf);
Point32Data do_transform_point32(const Point32Data& p, const TfData& tf);
PolygonData do_transform_polygon(const PolygonData& p, const TfData& tf);
PolygonStampedData do_transform_polygon_stamped(const PolygonStampedData& p, const TfData& tf);
Vec3Data do_transform_vector3(const Vec3Data& v, const TfData& tf);
Vector3StampedData do_transform_vector3_stamped(const Vector3StampedData& v, const TfData& tf);
QuatData do_transform_quaternion(const QuatData& q, const TfData& tf);
QuaternionStampedData do_transform_quaternion_stamped(
    const QuaternionStampedData& q, const TfData& tf);
PoseData do_transform_pose(const PoseData& pose, const TfData& tf);
PoseStampedData do_transform_pose_stamped(const PoseStampedData& pose, const TfData& tf);
PoseWithCovarianceData do_transform_pose_with_covariance(
    const PoseWithCovarianceData& pose, const TfData& tf);
PoseWithCovarianceStampedData do_transform_pose_with_covariance_stamped(
    const PoseWithCovarianceStampedData& pose, const TfData& tf);
TransformMsgData do_transform_transform(const TransformMsgData& t, const TfData& tf);
TfData do_transform_transform_stamped(const TfData& t, const TfData& tf);
WrenchData do_transform_wrench(const WrenchData& w, const TfData& tf);
WrenchStampedData do_transform_wrench_stamped(const WrenchStampedData& w, const TfData& tf);
VelocityStampedData do_transform_velocity_stamped(const VelocityStampedData& v, const TfData& tf);
bool do_transform_imu(const ImuData& imu, const TfData& tf, ImuData& out);
bool do_transform_magnetic_field(
    const MagneticFieldData& mag, const TfData& tf, MagneticFieldData& out);
CloudData do_transform_pointcloud2(const CloudData& cloud, const TfData& tf);

}  // namespace tf2_rs_oracle
