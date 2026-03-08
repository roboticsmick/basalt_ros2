#pragma once

#include <memory>
#include <deque>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <basalt/device/camera_interface.h>
#include <basalt/utils/imu_types.h>
#include <basalt/optical_flow/optical_flow.h>
#include <tbb/concurrent_queue.h>

namespace basalt {

/// RosCameraDevice — captures synchronized stereo/mono images from ROS2 topics
class RosCameraDevice : public CameraInterface {
 public:
  RosCameraDevice(rclcpp::Node::SharedPtr node,
                  const std::string& left_topic,
                  const std::string& right_topic);
  virtual ~RosCameraDevice();

  bool poll(uint64_t& t_ns, std::vector<ManagedImage<uint16_t>>& img) override;

  void start();
  void stop();
  std::string getFrameId() const;
  basalt::Calibration<double> exportCalibration();

  // Queue for optical flow input (used by node)
  tbb::concurrent_queue<basalt::OpticalFlowInput::Ptr>* image_data_queue = nullptr;

 private:
  rclcpp::Node::SharedPtr node_;
  std::string left_topic_;
  std::string right_topic_;
  std::mutex buffer_mutex_;
  std::deque<std::pair<uint64_t, std::unique_ptr<std::vector<ManagedImage<uint16_t>>>>> image_buffer_;
  std::string frame_id_;
  bool is_running_ = false;

  std::unique_ptr<image_transport::ImageTransport> it_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
  std::unique_ptr<message_filters::TimeSynchronizer<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Pre-allocated scratch mats for image conversion (O3 - processing optimisation)
  cv::Mat convert_scratch_gray8_;
  cv::Mat convert_scratch_gray16_;

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr& right_msg);

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);

  void convertImageMsg(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                       ManagedImage<uint16_t>& out);
};

}  // namespace basalt
