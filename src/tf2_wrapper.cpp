#include "tf2_wrapper.h"

#include "tf2_rs/src/ffi.rs.h"

#include <chrono>
#include <cstring>
#include <string>

#if __has_include(<geometry_msgs/msg/transform_stamped.hpp>)
  #include <geometry_msgs/msg/transform_stamped.hpp>
  #include <geometry_msgs/msg/point_stamped.hpp>
  #include <geometry_msgs/msg/pose_stamped.hpp>
#else
  #include <geometry_msgs/geometry_msgs/msg/transform_stamped.hpp>
  #include <geometry_msgs/geometry_msgs/msg/point_stamped.hpp>
  #include <geometry_msgs/geometry_msgs/msg/pose_stamped.hpp>
#endif

#if __has_include(<std_msgs/msg/header.hpp>)
  #include <std_msgs/msg/header.hpp>
#else
  #include <std_msgs/std_msgs/msg/header.hpp>
#endif

#if __has_include(<sensor_msgs/msg/point_cloud2.hpp>)
  #include <sensor_msgs/msg/point_cloud2.hpp>
  #include <sensor_msgs/msg/point_field.hpp>
#else
  #include <sensor_msgs/sensor_msgs/msg/point_cloud2.hpp>
  #include <sensor_msgs/sensor_msgs/msg/point_field.hpp>
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#if __has_include(<tf2/exceptions.hpp>)
  #include <tf2/exceptions.hpp>
#elif __has_include(<tf2/tf2/exceptions.hpp>)
  #include <tf2/tf2/exceptions.hpp>
#else
  #error "Could not find TF2 exception .hpp headers"
#endif


static Tf2Status ok() {
  return Tf2Status{Tf2Errc::Ok, ""};
}

static Tf2Status status(Tf2Errc code, const std::string& msg) {
  Tf2Status s;
  s.code = code;
  s.message = msg;
  return s;
}



template <class Fn>
static Tf2Status with_tf2_status(Fn&& fn) {
  try {
    fn();
    return ok();
  } catch (const tf2::LookupException& e) {
    return status(Tf2Errc::Lookup, e.what());
  } catch (const tf2::ConnectivityException& e) {
    return status(Tf2Errc::Connectivity, e.what());
  } catch (const tf2::ExtrapolationException& e) {
    return status(Tf2Errc::Extrapolation, e.what());
  } catch (const tf2::InvalidArgumentException& e) {
    return status(Tf2Errc::InvalidArgument, e.what());
  } catch (const tf2::TransformException& e) {
    return status(Tf2Errc::Other, e.what());
  } catch (const std::exception& e) {
    return status(Tf2Errc::Other, e.what());
  } catch (...) {
    return status(Tf2Errc::Other, "unknown exception");
  }
}


static tf2::TimePoint to_timepoint(const Tf2Time& t) {
  // TF2 convention: (0,0) means "latest available"
  if (t.sec == 0 && t.nanosec == 0) {
    return tf2::TimePointZero;
  }
  return tf2::TimePoint(std::chrono::seconds(t.sec) + std::chrono::nanoseconds(t.nanosec));
}

static std_msgs::msg::Header to_ros_header(const Tf2Header& h) {
  std_msgs::msg::Header out;
  out.stamp.sec = h.stamp.sec;
  out.stamp.nanosec = h.stamp.nanosec;
  out.frame_id = std::string(h.frame_id);
  return out;
}

static Tf2Header from_ros_header(const std_msgs::msg::Header& h) {
  Tf2Header out;
  out.stamp.sec = h.stamp.sec;
  out.stamp.nanosec = h.stamp.nanosec;
  out.frame_id = h.frame_id;
  return out;
}

static geometry_msgs::msg::TransformStamped to_ros(const Tf2TransformStamped& tf) {
  geometry_msgs::msg::TransformStamped out;
  out.header.stamp.sec = tf.stamp.sec;
  out.header.stamp.nanosec = tf.stamp.nanosec;
  out.header.frame_id = std::string(tf.parent_frame);
  out.child_frame_id = std::string(tf.child_frame);

  out.transform.translation.x = tf.tf.tx;
  out.transform.translation.y = tf.tf.ty;
  out.transform.translation.z = tf.tf.tz;

  out.transform.rotation.x = tf.tf.qx;
  out.transform.rotation.y = tf.tf.qy;
  out.transform.rotation.z = tf.tf.qz;
  out.transform.rotation.w = tf.tf.qw;

  return out;
}

// Header and child frame stay empty: doTransform only reads the transform itself.
static geometry_msgs::msg::TransformStamped to_ros(const Tf2Transform& tf) {
  geometry_msgs::msg::TransformStamped out;
  out.transform.translation.x = tf.tx;
  out.transform.translation.y = tf.ty;
  out.transform.translation.z = tf.tz;

  out.transform.rotation.x = tf.qx;
  out.transform.rotation.y = tf.qy;
  out.transform.rotation.z = tf.qz;
  out.transform.rotation.w = tf.qw;

  return out;
}

static Tf2TransformStamped from_ros(const geometry_msgs::msg::TransformStamped& tf) {
  Tf2TransformStamped out;
  out.stamp.sec = tf.header.stamp.sec;
  out.stamp.nanosec = tf.header.stamp.nanosec;
  out.parent_frame = tf.header.frame_id;
  out.child_frame = tf.child_frame_id;

  out.tf.tx = tf.transform.translation.x;
  out.tf.ty = tf.transform.translation.y;
  out.tf.tz = tf.transform.translation.z;

  out.tf.qx = tf.transform.rotation.x;
  out.tf.qy = tf.transform.rotation.y;
  out.tf.qz = tf.transform.rotation.z;
  out.tf.qw = tf.transform.rotation.w;

  return out;
}

// ---------------- BufferCoreWrapper ----------------

BufferCoreWrapper::BufferCoreWrapper(uint64_t cache_time_ns)
  : buffer_(tf2::Duration(std::chrono::nanoseconds(cache_time_ns))) {}

void BufferCoreWrapper::clear() const{
  buffer_.clear();
}

Tf2Status BufferCoreWrapper::set_transform(
    const Tf2TransformStamped& tf,
    rust::Str authority,
    bool is_static,
    bool& out_ok) const
{
  return with_tf2_status([&] {
    const auto msg = to_ros(tf);
    out_ok = buffer_.setTransform(msg, std::string(authority), is_static);
  });
}

Tf2Status BufferCoreWrapper::can_transform(
    rust::Str target_frame,
    rust::Str source_frame,
    const Tf2Time& time,
    Tf2CanTransformResult& out) const
{
  out.available = false;
  out.diagnostic = "";

  return with_tf2_status([&] {
    const auto tp = to_timepoint(time);

    std::string err;
    out.available = buffer_.canTransform(
        std::string(target_frame),
        std::string(source_frame),
        tp,
        &err);

    if (!out.available) {
      out.diagnostic = std::move(err);
    }
  });
}

Tf2Status BufferCoreWrapper::lookup_transform(
    rust::Str target_frame,
    rust::Str source_frame,
    const Tf2Time& time,
    Tf2TransformStamped& out_tf) const
{
  return with_tf2_status([&] {
    const auto tp = to_timepoint(time);
    auto tf = buffer_.lookupTransform(
        std::string(target_frame),
        std::string(source_frame),
        tp);
    out_tf = from_ros(tf);
  });
}

std::shared_ptr<BufferCoreWrapper> new_buffer_core(uint64_t cache_time_ns) {
  return std::make_shared<BufferCoreWrapper>(cache_time_ns);
}

// ---------------- doTransform wrappers ----------------

// geometry_msgs/PointStamped
static geometry_msgs::msg::PointStamped to_ros(const Tf2PointStamped& v) {
  geometry_msgs::msg::PointStamped out;
  out.header = to_ros_header(v.header);
  out.point.x = v.x;
  out.point.y = v.y;
  out.point.z = v.z;
  return out;
}

static Tf2PointStamped from_ros(const geometry_msgs::msg::PointStamped& v) {
  Tf2PointStamped out;
  out.header = from_ros_header(v.header);
  out.x = v.point.x;
  out.y = v.point.y;
  out.z = v.point.z;
  return out;
}

Tf2Status do_transform_point_stamped(
    const Tf2PointStamped& input,
    const Tf2TransformStamped& tf,
    Tf2PointStamped& out) {
  return with_tf2_status([&] {
    const auto in_ros = to_ros(input);
    const auto tf_ros = to_ros(tf);
    geometry_msgs::msg::PointStamped out_ros;
    tf2::doTransform(in_ros, out_ros, tf_ros);
    out = from_ros(out_ros);
  });
}

// geometry_msgs/PoseStamped
static geometry_msgs::msg::PoseStamped to_ros(const Tf2PoseStamped& v) {
  geometry_msgs::msg::PoseStamped out;
  out.header = to_ros_header(v.header);
  out.pose.position.x = v.position_x;
  out.pose.position.y = v.position_y;
  out.pose.position.z = v.position_z;
  out.pose.orientation.x = v.orientation_x;
  out.pose.orientation.y = v.orientation_y;
  out.pose.orientation.z = v.orientation_z;
  out.pose.orientation.w = v.orientation_w;
  return out;
}

static Tf2PoseStamped from_ros(const geometry_msgs::msg::PoseStamped& v) {
  Tf2PoseStamped out;
  out.header = from_ros_header(v.header);
  out.position_x = v.pose.position.x;
  out.position_y = v.pose.position.y;
  out.position_z = v.pose.position.z;
  out.orientation_x = v.pose.orientation.x;
  out.orientation_y = v.pose.orientation.y;
  out.orientation_z = v.pose.orientation.z;
  out.orientation_w = v.pose.orientation.w;
  return out;
}

Tf2Status do_transform_pose_stamped(
    const Tf2PoseStamped& input,
    const Tf2TransformStamped& tf,
    Tf2PoseStamped& out) {
  return with_tf2_status([&] {
    const auto in_ros = to_ros(input);
    const auto tf_ros = to_ros(tf);
    geometry_msgs::msg::PoseStamped out_ros;
    tf2::doTransform(in_ros, out_ros, tf_ros);
    out = from_ros(out_ros);
  });
}

// sensor_msgs/PointCloud2
Tf2Status do_transform_pointcloud2(
    const Tf2CloudLayout& layout,
    rust::Slice<const Tf2PointField> fields,
    rust::Slice<const uint8_t> data,
    const Tf2Transform& tf,
    rust::Slice<uint8_t> out) {
  return with_tf2_status([&] {
    if (out.size() != data.size()) {
      throw tf2::InvalidArgumentException("PointCloud2 output buffer size does not match input");
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.height = layout.height;
    cloud.width = layout.width;
    cloud.is_bigendian = layout.is_bigendian;
    cloud.point_step = layout.point_step;
    cloud.row_step = layout.row_step;
    cloud.is_dense = layout.is_dense;

    cloud.fields.reserve(fields.size());
    for (const auto& f : fields) {
      sensor_msgs::msg::PointField pf;
      pf.name = std::string(f.name);
      pf.offset = f.offset;
      pf.datatype = f.datatype;
      pf.count = f.count;
      cloud.fields.push_back(std::move(pf));
    }

    // Raw pointers so the copy is a single memmove.
    cloud.data.assign(data.data(), data.data() + data.size());

    // Same object as input and output: tf2 allows this, and it avoids the
    // full copy doTransform would otherwise make with `p_out = p_in`.
    tf2::doTransform(cloud, cloud, to_ros(tf));

    if (!cloud.data.empty()) {
      std::memcpy(out.data(), cloud.data.data(), cloud.data.size());
    }
  });
}
