/**
 * @file ros_imu.cpp
 * @brief ROS2 subscriber to an IMU as a basalt-style device.
 *
 * Original ROS1 implementation by Matthew Tavatgis (2025).
 * Ported to ROS2 Jazzy by Michael Venz (2026).
 *
 * @copyright BSD 3-Clause License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <basalt/device/ros_imu.h>

namespace basalt
{

  RosImuDevice::RosImuDevice(rclcpp::Node::SharedPtr node,
                             const std::string &imu_topic)
      : node_(node), imu_topic_(imu_topic)
  {
    // Store topic for later subscription in start()
    // Do NOT create subscription here - the shared_ptr wrapper isn't ready yet
    RCLCPP_INFO(node_->get_logger(), "RosImuDevice created (subscription will be set up in start()) with topic=%s",
                imu_topic.c_str());
  }

  RosImuDevice::~RosImuDevice()
  {
    stop();
  }

  void RosImuDevice::start()
  {
    if (is_running_)
    {
      RCLCPP_WARN(node_->get_logger(), "RosImuDevice already running");
      return;
    }

    // Create subscription here when the shared_ptr is fully initialized
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RosImuDevice::imuCallback, this, std::placeholders::_1));

    is_running_ = true;
    RCLCPP_INFO(node_->get_logger(), "RosImuDevice started - subscription active");
  }

  void RosImuDevice::stop()
  {
    is_running_ = false;
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      last_received_t_ns_ = 0;
    }
    RCLCPP_INFO(node_->get_logger(), "RosImuDevice stopped");
  }

  void RosImuDevice::flushBuffers()
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    imu_buffer_.clear();
    // Intentionally does NOT reset last_received_t_ns_: the monotonicity guard
    // must survive the flush so DDS-delayed messages from the pre-reset cycle
    // are still rejected as out-of-order after the new VIO instance starts.
  }

  void RosImuDevice::imuCallback(
      const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
  {
    // Validate timestamp — reject negative (uninitialized clock) samples
    int64_t t_ns_signed = rclcpp::Time(msg->header.stamp).nanoseconds();
    if (t_ns_signed < 0)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "IMU timestamp negative (%ld ns) — dropping", t_ns_signed);
      return;
    }
    uint64_t t_ns = static_cast<uint64_t>(t_ns_signed);

    // Extract acceleration (m/s^2)
    Eigen::Vector3d accel;
    accel << msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z;

    // Extract angular velocity (rad/s)
    Eigen::Vector3d gyro;
    gyro << msg->angular_velocity.x, msg->angular_velocity.y,
        msg->angular_velocity.z;

    // Push to buffer with thread safety
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    // Reject out-of-order samples (clock jumps, duplicate deliveries)
    if (last_received_t_ns_ > 0 && t_ns <= last_received_t_ns_)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "IMU out-of-order: t=%lu <= last=%lu — dropping", t_ns, last_received_t_ns_);
      return;
    }
    last_received_t_ns_ = t_ns;

    imu_buffer_.push_back({t_ns, {accel, gyro}});

    // Keep buffer bounded — warn on sample drops
    if (imu_buffer_.size() > 100)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "IMU buffer full (%zu samples) — dropping oldest sample", imu_buffer_.size());
      imu_buffer_.pop_front();
    }
  }

  bool RosImuDevice::poll(uint64_t &t_ns, Eigen::Vector3d &accel,
                          Eigen::Vector3d &gyro)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    if (imu_buffer_.empty())
    {
      return false;
    }

    auto front = imu_buffer_.front();
    imu_buffer_.pop_front();

    t_ns = front.first;
    accel = front.second.first;
    gyro = front.second.second;

    return true;
  }

} // namespace basalt
