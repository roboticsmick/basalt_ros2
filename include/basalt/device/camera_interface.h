#pragma once

#include <cstdint>
#include <vector>

#include <basalt/image/image.h>

namespace basalt {

/// Abstract interface for camera devices
class CameraInterface {
 public:
  virtual ~CameraInterface() = default;

  /// Poll for new images
  /// @param t_ns Output timestamp in nanoseconds
  /// @param img Output vector of images (stereo pair: left, right)
  /// @return True if images were available, false if timed out
  virtual bool poll(uint64_t& t_ns, std::vector<ManagedImage<uint16_t>>& img) = 0;
};

}  // namespace basalt
