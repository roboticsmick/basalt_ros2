#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <tbb/global_control.h>
#include <tbb/concurrent_queue.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
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
#include "basalt/node/odometer_health_track.hpp"

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
  bool publish_transform_{true};
  std::string odom_frame_;
  std::string base_frame_;
  std::string sensor_frame_;

  // VIO health parameters (loaded from vio_config, defaults match basalt_ros)
  int    min_keypoints_{1};
  double max_velocity_{1.5};
  double max_acceleration_{0.2};
  double health_startup_duration_{0.3};
  double health_keypoint_timeout_{1.0};

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  image_transport::Publisher left_img_pub_;
  image_transport::Publisher right_img_pub_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Stored for VIO re-initialisation across resets
  Calibration<double> calib_;
  VioConfig vio_config_;
  bool use_imu_{false};

  // VIO components
  std::shared_ptr<RosCameraDevice> camera_;
  std::shared_ptr<RosImuDevice> imu_;
  std::shared_ptr<OpticalFlowBase> optical_flow_;
  std::shared_ptr<VioEstimatorBase> vio_estimator_;

  // Mutex protecting vio_estimator_ during reset (camera thread reads, runtime thread writes)
  std::mutex vio_mutex_;

  // Threading and control
  std::thread camera_processing_thread_;
  std::thread processing_thread_;
  std::thread status_thread_;

  // Data queues
  tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> out_state_queue_;
  tbb::concurrent_bounded_queue<basalt::VioVisualizationData::Ptr> out_vis_queue_;

  // Odometry freeze watchdog
  std::atomic<int64_t> last_odom_ns_{0};
  int32_t odom_watchdog_timeout_ms_{5000};
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr keypoint_ratio_pub_;

  // VIO health trackers
  std::unique_ptr<OdometerHealthTrack> keypoint_health_;
  std::unique_ptr<OdometerHealthTrack> velocity_health_;
  std::unique_ptr<OdometerHealthTrack> acceleration_health_;

  // Runtime state flags
  std::atomic<bool> should_reset_{false};    // set by statusThread, cleared by runtimeThread
  std::atomic<bool> startup_success_{false}; // set by statusThread, cleared on reset

  // Thread functions
  void cameraProcessingThread();  // always running — feeds optical flow + IMU
  void runtimeThread();           // always running — VIO lifecycle orchestrator
  void odometerThread();          // per VIO lifecycle — publishes odometry, tracks health
  void statusThread();            // always running — vis output, health monitoring, status pub
  void initializeAsync();

  geometry_msgs::msg::TransformStamped lookupTransform(
      const std::string& target,
      const std::string& source,
      const rclcpp::Time& time_stamp);

  // Initialization timer
  rclcpp::TimerBase::SharedPtr init_timer_;
  bool is_initialized_ = false;

  // Thread shutdown control
  std::atomic<bool> should_stop_{false};
};

}  // namespace basalt
