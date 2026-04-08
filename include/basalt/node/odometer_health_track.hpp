#pragma once

#include <chrono>
#include <mutex>

/// Tracks the health of a single VIO odometer criterion (keypoints, velocity, or acceleration).
///
/// Two phases:
///   - Startup: all criteria must stay healthy for startup_duration before startup_healthy() returns true
///   - Runtime: criterion may be unhealthy for up to timeout_duration before runtime_healthy() returns false
///
/// Ported from basalt_ros (ROS 1) odometer_health_track.hpp.
/// Uses std::chrono::steady_clock instead of ros::Time/ros::Duration.
struct OdometerHealthTrack {

  using TimePoint = std::chrono::steady_clock::time_point;
  using Duration  = std::chrono::duration<double>;

  // Tracker configuration
  Duration timeout_duration;   // Max time unhealthy before runtime_healthy() returns false (0 = immediate)
  Duration startup_duration;   // Min time healthy before startup_healthy() returns true

  // Tracker state (mutex-protected)
  std::mutex status_mutex;
  bool initialised{false};
  bool startup_valid{false};
  bool runtime_valid{false};
  TimePoint last_valid_stamp{};
  TimePoint last_invalid_stamp{};

  /// Update health state with the result of this step and its timestamp.
  void update(bool this_step_valid, TimePoint stamp) {
    std::lock_guard<std::mutex> lock(status_mutex);

    // Record timestamps
    if (this_step_valid) {
      last_valid_stamp = stamp;
    } else {
      last_invalid_stamp = stamp;
    }

    // Initialise on first call — treat time before first update as invalid
    if (!initialised) {
      last_invalid_stamp = stamp;
      initialised = true;
    }

    if (this_step_valid) {

      runtime_valid = true;

      // Startup validity: must be continuously valid for at least startup_duration
      if (stamp - last_invalid_stamp > startup_duration) {
        startup_valid = true;
      }

    } else {

      // Any invalid reading resets startup validity
      startup_valid = false;

      // Runtime validity: only cleared after being invalid for timeout_duration
      if (stamp - last_valid_stamp > timeout_duration) {
        runtime_valid = false;
      }
    }
  }

  /// Reset all state (called on VIO reset before reinitialisation).
  void reset() {
    std::lock_guard<std::mutex> lock(status_mutex);
    initialised    = false;
    startup_valid  = false;
    runtime_valid  = false;
    last_valid_stamp   = TimePoint{};
    last_invalid_stamp = TimePoint{};
  }

  /// True once this criterion has been continuously healthy for startup_duration.
  bool startup_healthy() {
    std::lock_guard<std::mutex> lock(status_mutex);
    return startup_valid;
  }

  /// True while this criterion has not been unhealthy longer than timeout_duration.
  bool runtime_healthy() {
    std::lock_guard<std::mutex> lock(status_mutex);
    return runtime_valid;
  }

  /// @param timeout_sec  Seconds of consecutive failure before runtime_healthy() → false (0 = immediate)
  /// @param startup_sec  Seconds of consecutive success before startup_healthy() → true
  OdometerHealthTrack(double timeout_sec, double startup_sec)
      : timeout_duration(timeout_sec),
        startup_duration(startup_sec) {}
};
