#pragma once

#include <cstdint>

#include <Eigen/Dense>

namespace basalt {

/// Abstract interface for IMU devices
class ImuInterface {
 public:
  virtual ~ImuInterface() = default;

  /// Poll for next IMU measurement
  /// @param t_ns Output timestamp in nanoseconds
  /// @param accel Output accelerometer reading (m/s²)
  /// @param gyro Output gyroscope reading (rad/s)
  /// @return True if IMU data was available, false if buffer empty
  virtual bool poll(uint64_t& t_ns, Eigen::Vector3d& accel,
                    Eigen::Vector3d& gyro) = 0;
};

}  // namespace basalt
