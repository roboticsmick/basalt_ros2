/********************************************************
 * Basalt ROS2 IMU                                  *
 *                                                  *
 * Author:      Matthew Tavatgis                   *
 * Updated:     March 2026 (ROS2 Migration)        *
 *                                                  *
 * Brief:       Implements a ROS2 subscriber to an *
 *              IMU as a basalt style device.      *
 *                                                  *
 *******************************************************/

#include <basalt/device/ros_imu.h>

namespace basalt {

RosImuDevice::RosImuDevice(rclcpp::Node::SharedPtr node,
                           const std::string& imu_topic)
    : node_(node), imu_topic_(imu_topic) {
  // Store topic for later subscription in start()
  // Do NOT create subscription here - the shared_ptr wrapper isn't ready yet
  RCLCPP_INFO(node_->get_logger(), "RosImuDevice created (subscription will be set up in start()) with topic=%s",
              imu_topic.c_str());
}

RosImuDevice::~RosImuDevice() {
  stop();
}

void RosImuDevice::start() {
  if (is_running_) {
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

void RosImuDevice::stop() {
  is_running_ = false;
  RCLCPP_INFO(node_->get_logger(), "RosImuDevice stopped");
}

void RosImuDevice::imuCallback(
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  // Convert ROS2 timestamp to nanoseconds
  uint64_t t_ns = rclcpp::Time(msg->header.stamp).nanoseconds();

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
  imu_buffer_.push_back({t_ns, {accel, gyro}});

  // Keep buffer bounded (Q4 - warn on IMU sample drops)
  if (imu_buffer_.size() > 100) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "IMU buffer full (%zu samples) — dropping oldest sample", imu_buffer_.size());
    imu_buffer_.pop_front();
  }
}

bool RosImuDevice::poll(uint64_t& t_ns, Eigen::Vector3d& accel,
                        Eigen::Vector3d& gyro) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);

  if (imu_buffer_.empty()) {
    return false;
  }

  auto front = imu_buffer_.front();
  imu_buffer_.pop_front();

  t_ns = front.first;
  accel = front.second.first;
  gyro = front.second.second;

  return true;
}

}  // namespace basalt
