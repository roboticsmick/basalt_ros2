#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <basalt/calibration/calibration.hpp>
#include <opencv2/calib3d.hpp>

namespace basalt {

/**
 * @brief CalibrationPublisherNode
 * Reads a Basalt calibration file (YAML or JSON format), computes stereo rectification
 * matrices using cv::stereoRectify(), and publishes CameraInfo messages for left and
 * right cameras on a fixed interval.
 *
 * This node enables downstream stereo processing nodes (image_proc::RectifyNode,
 * stereo_image_proc) to use the Basalt-computed calibration instead of the device's
 * on-board calibration.
 *
 * Parameters:
 *   calib_path (string): Path to calibration YAML or JSON file
 *   left_camera_info_topic (string): Topic for left CameraInfo (default: /oak/left/camera_info)
 *   right_camera_info_topic (string): Topic for right CameraInfo (default: /oak/right/camera_info)
 *   publish_interval_hz (int): Publication frequency in Hz (default: 10)
 */
class CalibrationPublisherNode : public rclcpp::Node {
 public:
  explicit CalibrationPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  virtual ~CalibrationPublisherNode();

 private:
  // Async initialization (after constructor completes)
  void initializeAsync();

  // Load calibration from YAML or JSON
  Calibration<double> loadCalibration(const std::string& calib_path);

  // Publish CameraInfo messages
  void publishCameraInfo();

  // Convert Basalt camera model to ROS K, D matrices
  void cameraToKD(const GenericCamera<double>& cam,
                  cv::Mat& K,
                  cv::Mat& D,
                  const cv::Size& image_size);

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;

  // Timer for periodic publishing
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Initialization timer
  rclcpp::TimerBase::SharedPtr init_timer_;

  // Cached calibration data
  Calibration<double> calib_;
  sensor_msgs::msg::CameraInfo left_info_msg_;
  sensor_msgs::msg::CameraInfo right_info_msg_;

  // State
  bool is_initialized_ = false;
};

}  // namespace basalt
