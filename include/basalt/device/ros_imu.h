#pragma once

#include <memory>
#include <deque>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>

#include <basalt/device/imu_interface.h>
#include <basalt/utils/imu_types.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include <tbb/concurrent_queue.h>

namespace basalt {

/// RosImuDevice — captures IMU data from ROS2 topic
class RosImuDevice : public ImuInterface {
 public:
  explicit RosImuDevice(rclcpp::Node::SharedPtr node,
                        const std::string& imu_topic);
  virtual ~RosImuDevice();

  bool poll(uint64_t& t_ns, Eigen::Vector3d& accel, Eigen::Vector3d& gyro) override;

  void start();
  void stop();

  // Queue for IMU input (used by node)
  tbb::concurrent_queue<basalt::ImuData<double>::Ptr>* imu_data_queue = nullptr;

 private:
  rclcpp::Node::SharedPtr node_;
  std::string imu_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::mutex buffer_mutex_;
  std::deque<std::pair<uint64_t, std::pair<Eigen::Vector3d, Eigen::Vector3d>>> imu_buffer_;
  bool is_running_ = false;

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
};

}  // namespace basalt
