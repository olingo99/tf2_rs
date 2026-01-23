#pragma once

#include <cstdint>
#include <memory>
#include "rust/cxx.h"

// TF2 BufferCore header path differs across distros; handle both.
#if __has_include(<tf2/buffer_core.h>)
  #include <tf2/buffer_core.h>
#else
  #include <tf2/tf2/buffer_core.h>
#endif

// Forward-declare all cxx-shared structs in the GLOBAL namespace.
// The definitions come from the cxx-generated header (ffi.rs.h).
struct Tf2Time;
struct Tf2Header;
struct Tf2TransformStamped;

struct Tf2PointStamped;
struct Tf2PoseStamped;

struct Tf2PointField;
struct Tf2PointCloud2;

// New: forward declarations for status/error code (defined in cxx-generated header)
enum class Tf2Errc : int32_t;
struct Tf2Status;

class BufferCoreWrapper {
public:
  explicit BufferCoreWrapper(uint64_t cache_time_ns);

  void clear() const;

  // Now returns a status; result is written to out_ok
  Tf2Status set_transform(
      const Tf2TransformStamped& tf,
      rust::Str authority,
      bool is_static,
      bool& out_ok) const;

  // Now returns a status; boolean answer is written to out_ok
  Tf2Status can_transform(
      rust::Str target_frame,
      rust::Str source_frame,
      const Tf2Time& time,
      bool& out_ok) const;

  // Now returns a status; transform is written to out_tf
  Tf2Status lookup_transform(
      rust::Str target_frame,
      rust::Str source_frame,
      const Tf2Time& time,
      Tf2TransformStamped& out_tf) const;

private:
  mutable tf2::BufferCore buffer_;
};

std::shared_ptr<BufferCoreWrapper> new_buffer_core(uint64_t cache_time_ns);

// TF2 built-in doTransform wrappers (free functions) now return status + out-param
Tf2Status do_transform_point_stamped(
    const Tf2PointStamped& input,
    const Tf2TransformStamped& tf,
    Tf2PointStamped& out);

Tf2Status do_transform_pose_stamped(
    const Tf2PoseStamped& input,
    const Tf2TransformStamped& tf,
    Tf2PoseStamped& out);

Tf2Status do_transform_pointcloud2(
    const Tf2PointCloud2& input,
    const Tf2TransformStamped& tf,
    Tf2PointCloud2& out);
