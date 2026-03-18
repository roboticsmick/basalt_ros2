#pragma once

#include <memory>
#include <thread>
#include <tbb/global_control.h>
#include <tbb/concurrent_queue.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <image_transport/image_transport.hpp>

#include <basalt/vi_estimator/vio_estimator.h>
#include <basalt/optical_flow/optical_flow.h>
#include <basalt/calibration/calibration.hpp>
#include <basalt/device/ros_camera.h>
#include <basalt/device/ros_imu.h>

namespace basalt {

/// VisualOdometerNode — main ROS2 node for VIO odometry
class VisualOdometerNode : public rclcpp::Node {
 public:
  explicit VisualOdometerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  virtual ~VisualOdometerNode();

 private:
  // Parameters
  int thread_limit_;
  bool publish_cloud_;
  bool publish_images_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string sensor_frame_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
  image_transport::Publisher left_img_pub_;
  image_transport::Publisher right_img_pub_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // VIO components
  std::shared_ptr<RosCameraDevice> camera_;
  std::shared_ptr<RosImuDevice> imu_;
  std::shared_ptr<OpticalFlowBase> optical_flow_;
  std::shared_ptr<VioEstimatorBase> vio_estimator_;

  // Threading and control
  std::thread camera_processing_thread_;
  std::thread processing_thread_;
  std::thread status_thread_;

  // Data queues
  tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> out_state_queue_;
  tbb::concurrent_bounded_queue<basalt::VioVisualizationData::Ptr> out_vis_queue_;

  // Reliability metric (Item 5)
  std::atomic<int64_t> last_odom_ns_{0};
  int32_t odom_watchdog_timeout_ms_{5000};
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr keypoint_ratio_pub_;

  // Thread functions
  void cameraProcessingThread();
  void runtimeThread();
  void statusThread();
  void initializeAsync();

  geometry_msgs::msg::TransformStamped lookupTransform(
      const std::string& target,
      const std::string& source,
      const rclcpp::Time& time_stamp);

  // Initialization timer
  rclcpp::TimerBase::SharedPtr init_timer_;
  bool is_initialized_ = false;

  // Thread shutdown control (NASA §VIII)
  std::atomic<bool> should_stop_{false};
};

}  // namespace basalt
