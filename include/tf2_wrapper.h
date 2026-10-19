#pragma once

#include <cstdint>
#include <memory>
#include "rust/cxx.h"

#if __has_include(<tf2/buffer_core.hpp>)
  #include <tf2/buffer_core.hpp>
#elif __has_include(<tf2/tf2/buffer_core.hpp>)
  #include <tf2/tf2/buffer_core.hpp>
#else
  #error "Could not find a TF2 BufferCore .hpp header"
#endif

// Forward-declare all cxx-shared structs in the GLOBAL namespace.
// The definitions come from the cxx-generated header (ffi.rs.h).
struct Tf2Time;
struct Tf2Header;
struct Tf2Transform;
struct Tf2TransformStamped;

struct Tf2PointStamped;
struct Tf2PoseStamped;

struct Tf2PointField;
struct Tf2CloudLayout;

// New: forward declarations for status/error code (defined in cxx-generated header)
enum class Tf2Errc : int32_t;
struct Tf2Status;
struct Tf2CanTransformResult;

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

  // Returns wrapper status for exceptional failures only.
  // The normal canTransform probe result is written to `out`.
  Tf2Status can_transform(
      rust::Str target_frame,
      rust::Str source_frame,
      const Tf2Time& time,
      Tf2CanTransformResult& out) const;

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

// `out` must have the same length as `data`. The header is not touched here;
// the caller sets it from the transform.
Tf2Status do_transform_pointcloud2(
    const Tf2CloudLayout& layout,
    rust::Slice<const Tf2PointField> fields,
    rust::Slice<const uint8_t> data,
    const Tf2Transform& tf,
    rust::Slice<uint8_t> out);
