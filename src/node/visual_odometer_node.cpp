/********************************************************
 * Basalt Visual Odometer ROS2 Node                *
 *                                                  *
 * Author:      Matthew Tavatgis                   *
 * Updated:     March 2026 (ROS2 Migration)        *
 *                                                  *
 * Brief:       Implements a ROS2 node that runs   *
 *              the basalt visual odometer and     *
 *              publishes results as ROS messages. *
 *                                                  *
 *******************************************************/

#include <basalt/node/visual_odometer_node.h>
#include <basalt/device/ros_camera.h>
#include <basalt/device/ros_imu.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include <basalt/optical_flow/optical_flow.h>
#include <basalt/calibration/calibration.hpp>
#include <basalt/camera/generic_camera.hpp>
#include <basalt/camera/pinhole_camera.hpp>
#include <basalt/camera/pinhole_radtan8_camera.hpp>
#include <basalt/io/dataset_io.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <std_msgs/msg/header.hpp>

namespace basalt {

// Helper function to load calibration from JSON file
Calibration<double> loadCalibrationFromJSON(const std::string& calib_path) {
  using json = nlohmann::json;

  Calibration<double> calib;

  try {
    std::ifstream file(calib_path);
    if (!file.is_open()) {
      throw std::runtime_error("Cannot open calibration file: " + calib_path);
    }

    json j;
    file >> j;
    file.close();

    // basalt_calibrate saves as {"value0": {...}}
    if (!j.contains("value0")) {
      throw std::runtime_error("Invalid calibration JSON: missing 'value0' key");
    }

    const auto& calib_data = j["value0"];

    // Load T_imu_cam transforms (IMU to camera extrinsics)
    if (calib_data.contains("T_imu_cam") && calib_data["T_imu_cam"].is_array()) {
      for (const auto& transform : calib_data["T_imu_cam"]) {
        Sophus::SE3d T_imu_cam;

        // Extract translation
        Eigen::Vector3d translation(
            transform["px"].get<double>(),
            transform["py"].get<double>(),
            transform["pz"].get<double>());

        // Extract quaternion (xyzw order)
        Eigen::Quaterniond quat(
            transform["qw"].get<double>(),  // w
            transform["qx"].get<double>(),  // x
            transform["qy"].get<double>(),  // y
            transform["qz"].get<double>()); // z

        T_imu_cam = Sophus::SE3d(quat, translation);
        calib.T_i_c.emplace_back(T_imu_cam);
      }
    } else {
      throw std::runtime_error("Invalid calibration JSON: missing 'T_imu_cam' array");
    }

    // Load camera intrinsics
    if (calib_data.contains("intrinsics") && calib_data["intrinsics"].is_array()) {
      for (const auto& intr_data : calib_data["intrinsics"]) {
        std::string cam_type = intr_data["camera_type"].get<std::string>();

        // Extract intrinsic parameters
        const auto& intr_params = intr_data["intrinsics"];
        double fx = intr_params["fx"].get<double>();
        double fy = intr_params["fy"].get<double>();
        double cx = intr_params["cx"].get<double>();
        double cy = intr_params["cy"].get<double>();

        GenericCamera<double> cam;

        if (cam_type == "pinhole-radtan8") {
          // Pinhole with radtan8 distortion (12 parameters)
          // Order: fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
          Eigen::Matrix<double, 12, 1> radtan8_params;
          radtan8_params << fx, fy, cx, cy,
                           intr_params["k1"].get<double>(),
                           intr_params["k2"].get<double>(),
                           intr_params["p1"].get<double>(),
                           intr_params["p2"].get<double>(),
                           intr_params["k3"].get<double>(),
                           intr_params["k4"].get<double>(),
                           intr_params["k5"].get<double>(),
                           intr_params["k6"].get<double>();
          PinholeRadtan8Camera<double> radtan8_cam(radtan8_params);
          cam.variant = radtan8_cam;
        } else if (cam_type == "pinhole") {
          // Standard pinhole model (4 parameters): fx, fy, cx, cy
          Eigen::Matrix<double, 4, 1> pinhole_params;
          pinhole_params << fx, fy, cx, cy;
          PinholeCamera<double> pinhole_cam(pinhole_params);
          cam.variant = pinhole_cam;
        } else {
          throw std::runtime_error("Unsupported camera type: " + cam_type);
        }

        calib.intrinsics.emplace_back(cam);
      }
    } else {
      throw std::runtime_error("Invalid calibration JSON: missing 'intrinsics' array");
    }

    // Load resolution if available
    if (calib_data.contains("resolution") && calib_data["resolution"].is_array()) {
      try {
        auto res_array = calib_data["resolution"].get<std::vector<std::array<int, 2>>>();
        for (const auto& res : res_array) {
          calib.resolution.emplace_back(res[0], res[1]);  // Each camera has width, height
        }
      } catch (const std::exception&) {
        // Skip resolution if parsing fails - it's optional
      }
    }

    // Load other calibration data if present (vignette, etc.)
    // For now, we skip these as they're optional

    return calib;
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("Failed to load calibration: ") + e.what());
  }
}

// Helper function to load calibration from YAML file
Calibration<double> loadCalibrationFromYAML(const std::string& calib_path) {
  Calibration<double> calib;

  try {
    YAML::Node root = YAML::LoadFile(calib_path);

    // Load T_imu_cam transforms (IMU to camera extrinsics)
    if (!root["T_imu_cam"]) {
      throw std::runtime_error("Missing 'T_imu_cam' in calibration YAML");
    }

    for (const auto& tf : root["T_imu_cam"]) {
      // Extract translation
      Eigen::Vector3d translation(
          tf["px"].as<double>(),
          tf["py"].as<double>(),
          tf["pz"].as<double>());

      // Extract quaternion (xyzw order)
      Eigen::Quaterniond quat(
          tf["qw"].as<double>(),  // w
          tf["qx"].as<double>(),  // x
          tf["qy"].as<double>(),  // y
          tf["qz"].as<double>()); // z

      Sophus::SE3d T_imu_cam = Sophus::SE3d(quat, translation);
      calib.T_i_c.emplace_back(T_imu_cam);
    }

    // Load camera intrinsics
    if (!root["intrinsics"]) {
      throw std::runtime_error("Missing 'intrinsics' in calibration YAML");
    }

    for (const auto& cam_node : root["intrinsics"]) {
      std::string cam_type = cam_node["camera_type"].as<std::string>();

      // Extract intrinsic parameters
      const auto& intr_params = cam_node["intrinsics"];
      double fx = intr_params["fx"].as<double>();
      double fy = intr_params["fy"].as<double>();
      double cx = intr_params["cx"].as<double>();
      double cy = intr_params["cy"].as<double>();

      GenericCamera<double> cam;

      if (cam_type == "pinhole-radtan8") {
        // Pinhole with radtan8 distortion (12 parameters)
        // Order: fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
        Eigen::Matrix<double, 12, 1> radtan8_params;
        radtan8_params << fx, fy, cx, cy,
                         intr_params["k1"].as<double>(),
                         intr_params["k2"].as<double>(),
                         intr_params["p1"].as<double>(),
                         intr_params["p2"].as<double>(),
                         intr_params["k3"].as<double>(),
                         intr_params["k4"].as<double>(),
                         intr_params["k5"].as<double>(),
                         intr_params["k6"].as<double>();
        PinholeRadtan8Camera<double> radtan8_cam(radtan8_params);
        cam.variant = radtan8_cam;
      } else if (cam_type == "pinhole") {
        // Standard pinhole model (4 parameters): fx, fy, cx, cy
        Eigen::Matrix<double, 4, 1> pinhole_params;
        pinhole_params << fx, fy, cx, cy;
        PinholeCamera<double> pinhole_cam(pinhole_params);
        cam.variant = pinhole_cam;
      } else {
        throw std::runtime_error("Unsupported camera type: " + cam_type);
      }

      calib.intrinsics.emplace_back(cam);
    }

    // Load resolution if available
    if (root["resolution"] && root["resolution"].IsSequence()) {
      for (const auto& res : root["resolution"]) {
        if (res.IsSequence() && res.size() >= 2) {
          calib.resolution.emplace_back(res[0].as<int>(), res[1].as<int>());
        }
      }
    }

    return calib;
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("Failed to load calibration YAML: ") + e.what());
  }
}

VisualOdometerNode::VisualOdometerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("basalt_vio_node", options), is_initialized_(false) {
  RCLCPP_INFO(get_logger(), "Creating basalt_vio_node...");

  // Declare parameters (these must be declared before getting them)
  declare_parameter<std::string>("calib_path", "");
  declare_parameter<std::string>("config_path", "");
  declare_parameter<std::string>("left_image_topic", "/oak/stereo/left/image");
  declare_parameter<std::string>("right_image_topic", "/oak/stereo/right/image");
  declare_parameter<std::string>("imu_topic", "");
  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<int>("thread_limit", 0);
  declare_parameter<bool>("publish_cloud", true);
  declare_parameter<bool>("publish_images", true);
  declare_parameter<int>("odom_watchdog_timeout_ms", 5000);

  // Schedule async initialization after node is fully constructed
  init_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&VisualOdometerNode::initializeAsync, this));

  RCLCPP_INFO(get_logger(), "Constructor complete - async initialization scheduled");
}

void VisualOdometerNode::initializeAsync() {
  // Fire only once
  if (is_initialized_) {
    return;
  }
  is_initialized_ = true;

  // Cancel the timer
  init_timer_.reset();

  RCLCPP_INFO(get_logger(), "Starting async initialization...");

  try {
    // Get parameters
    std::string calib_path = get_parameter("calib_path").as_string();
    std::string config_path = get_parameter("config_path").as_string();
    std::string left_topic = get_parameter("left_image_topic").as_string();
    std::string right_topic = get_parameter("right_image_topic").as_string();
    std::string imu_topic = get_parameter("imu_topic").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    thread_limit_ = get_parameter("thread_limit").as_int();
    publish_cloud_ = get_parameter("publish_cloud").as_bool();
    publish_images_ = get_parameter("publish_images").as_bool();
    odom_watchdog_timeout_ms_ = get_parameter("odom_watchdog_timeout_ms").as_int();

    // Initialize TF2 (now safe to call shared_from_this())
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());

    // Create publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::SystemDefaultsQoS());
    keypoint_ratio_pub_ = create_publisher<std_msgs::msg::Float32>(
        "basalt_vio/keypoint_ratio", rclcpp::SystemDefaultsQoS());
    if (publish_cloud_) {
      pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("keypoints", rclcpp::SystemDefaultsQoS());
    }
    if (publish_images_) {
      image_transport::ImageTransport it(shared_from_this());
      left_img_pub_  = it.advertise("basalt_vio/left/image_annotated", 1);
      right_img_pub_ = it.advertise("basalt_vio/right/image_annotated", 1);
      RCLCPP_INFO(get_logger(), "Image annotation publishers created");
    }

    // Create odometry freeze watchdog timer (Item 5)
    watchdog_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        int64_t last_ns = last_odom_ns_.load(std::memory_order_relaxed);
        if (last_ns == 0) return;  // VIO not yet started
        int64_t age_ms = (get_clock()->now().nanoseconds() - last_ns) / 1'000'000;
        if (age_ms > odom_watchdog_timeout_ms_) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                "VIO FREEZE DETECTED: no odometry for %ld ms", age_ms);
        }
    });

    // Initialize camera
    RCLCPP_INFO(get_logger(), "Initializing camera from topics: %s, %s", left_topic.c_str(), right_topic.c_str());
    camera_ = std::make_shared<RosCameraDevice>(shared_from_this(), left_topic, right_topic);
    camera_->start();

    // Initialize IMU
    bool use_imu = !imu_topic.empty();
    if (use_imu) {
      RCLCPP_INFO(get_logger(), "Initializing IMU from topic: %s", imu_topic.c_str());
      imu_ = std::make_shared<RosImuDevice>(shared_from_this(), imu_topic);
      imu_->start();
    }

    // Load calibration
    Calibration<double> calib;
    if (!calib_path.empty()) {
      try {
        // Detect extension: .yaml/.yml → YAML loader, otherwise → JSON loader
        auto ends_with = [](const std::string& s, const std::string& sfx) {
          return s.size() >= sfx.size() &&
                 s.compare(s.size() - sfx.size(), sfx.size(), sfx) == 0;
        };
        const bool is_yaml = ends_with(calib_path, ".yaml") || ends_with(calib_path, ".yml");
        calib = is_yaml ? loadCalibrationFromYAML(calib_path)
                        : loadCalibrationFromJSON(calib_path);
        RCLCPP_INFO(get_logger(), "Successfully loaded calibration from: %s", calib_path.c_str());
        RCLCPP_INFO(get_logger(), "Loaded %lu camera(s) and %lu IMU-camera transform(s)",
                    calib.intrinsics.size(), calib.T_i_c.size());
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to load calibration file: %s", e.what());
        RCLCPP_WARN(get_logger(), "Falling back to default/ROS CameraInfo calibration");
        if (use_imu) {
          RCLCPP_WARN(get_logger(), "IMU disabled - calibration file required for IMU fusion");
          use_imu = false;
          if (imu_) {
            imu_->stop();
            imu_.reset();
          }
        }
        calib = camera_->exportCalibration();
      }
    } else {
      if (use_imu) {
        RCLCPP_WARN(get_logger(), "Using IMU requires calibration file! Falling back to VO only...");
        use_imu = false;
        if (imu_) {
          imu_->stop();
          imu_.reset();
        }
      }
      RCLCPP_WARN(get_logger(), "No calibration file provided. Attempting to generate from ROS CameraInfo...");
      RCLCPP_WARN(get_logger(), "For best VIO accuracy, provide a basalt calibration file via calib_path parameter");
      calib = camera_->exportCalibration();
    }
    RCLCPP_INFO(get_logger(), "Calibration initialized successfully");

    // Get sensor frame ID
    RCLCPP_INFO(get_logger(), "Waiting for frame_id from camera...");
    sensor_frame_ = camera_->getFrameId();
    RCLCPP_INFO(get_logger(), "Frame ID received: %s", sensor_frame_.c_str());

    if (sensor_frame_.empty()) {
      RCLCPP_WARN(get_logger(), "No frame_id provided by sensor, assuming '%s'", base_frame_.c_str());
      sensor_frame_ = base_frame_;
    }

    // Load VIO config
    VioConfig vio_config;
    if (!config_path.empty()) {
      try {
        vio_config.load(config_path);
        RCLCPP_INFO(get_logger(), "Loaded VIO config from %s", config_path.c_str());
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Failed to load VIO config: %s, using defaults", e.what());
      }
    }

    // Initialize optical flow
    optical_flow_ = OpticalFlowFactory::getOpticalFlow(vio_config, calib);
    optical_flow_->input_queue.set_capacity(10);  // Q1 - bound optical flow input queue
    RCLCPP_INFO(get_logger(), "Optical flow initialized");

    // Initialize VIO estimator
    vio_estimator_ = VioEstimatorFactory::getVioEstimator(
        vio_config, calib, constants::g, use_imu, true);

    // Initialize VIO with bias estimates
    vio_estimator_->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    optical_flow_->output_queue = &vio_estimator_->vision_data_queue;
    vio_estimator_->out_state_queue = &out_state_queue_;
    if (publish_cloud_ || publish_images_) {
      vio_estimator_->out_vis_queue = &out_vis_queue_;
    }

    // Initialize bounded queues (Q2 - reduce to 30 = 3 sec at 10 Hz, was 300 = 30 sec)
    out_state_queue_.set_capacity(30);
    out_vis_queue_.set_capacity(30);

    // Start processing threads (Q5 - removed duplicate tbb_control_, runtimeThread has the correct one)
    camera_processing_thread_ = std::thread(&VisualOdometerNode::cameraProcessingThread, this);
    processing_thread_ = std::thread(&VisualOdometerNode::runtimeThread, this);
    if (publish_cloud_ || publish_images_) {
      status_thread_ = std::thread(&VisualOdometerNode::statusThread, this);
    }

    RCLCPP_INFO(get_logger(), "Async initialization complete, node started!");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Initialization failed: %s", e.what());
    is_initialized_ = false;
  }
}

VisualOdometerNode::~VisualOdometerNode() {
  RCLCPP_INFO(get_logger(), "Shutting down node...");
  // Signal threads to stop before joining (NASA §VIII)
  should_stop_ = true;

  if (imu_) {
    imu_->stop();
  }
  if (camera_) {
    camera_->stop();
  }
  if (camera_processing_thread_.joinable()) {
    camera_processing_thread_.join();
  }
  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
  if (status_thread_.joinable()) {
    status_thread_.join();
  }
  RCLCPP_INFO(get_logger(), "Terminated, cleaning up.");
}

void VisualOdometerNode::cameraProcessingThread() {
  uint64_t t_ns;
  std::vector<ManagedImage<uint16_t>> images;
  uint32_t frame_count = 0;
  uint32_t consecutive_misses = 0;

  RCLCPP_INFO(get_logger(), "Camera processing thread started");

  while (rclcpp::ok() && !should_stop_) {
    // Poll camera for images
    if (camera_->poll(t_ns, images)) {
      frame_count++;
      consecutive_misses = 0;  // Reset counter on successful frame
      if (frame_count % 10 == 0) {
        RCLCPP_INFO(get_logger(), "Received %u frames", frame_count);
      }

      // Create OpticalFlowInput with ImageData
      auto opt_flow_input = std::make_shared<OpticalFlowInput>();
      opt_flow_input->t_ns = t_ns;

      for (auto& img : images) {
        ImageData img_data;
        // Create managed image and move ownership
        auto managed_img = std::make_shared<ManagedImage<uint16_t>>();
        *managed_img = std::move(img);
        img_data.img = managed_img;
        img_data.exposure = 0.0;
        opt_flow_input->img_data.push_back(img_data);
      }
      images.clear();

      // Push to optical flow input queue (use try_push to prevent blocking camera thread)
      if (!optical_flow_->input_queue.try_push(opt_flow_input)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Optical flow input queue full — dropping frame at t=%lu ns", t_ns);
      }
    } else {
      consecutive_misses++;
      if (consecutive_misses >= 1000) {
        RCLCPP_WARN(get_logger(), "Camera: no frame for %u consecutive polls (~%u ms) — driver running?",
                    consecutive_misses, consecutive_misses);
        consecutive_misses = 0;
      }
    }

    // Poll IMU if available and forward to VIO estimator
    if (imu_) {
      uint64_t t_imu;
      Eigen::Vector3d accel, gyro;
      while (imu_->poll(t_imu, accel, gyro)) {
        auto imu_data = std::make_shared<ImuData<double>>();
        imu_data->t_ns = static_cast<int64_t>(t_imu);
        imu_data->accel = accel;
        imu_data->gyro = gyro;
        vio_estimator_->addIMUToQueue(imu_data);
      }
    }

    // Small sleep to prevent busy-waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  RCLCPP_INFO(get_logger(), "Camera processing thread exited (received %u total frames)", frame_count);
}

void VisualOdometerNode::runtimeThread() {
  basalt::PoseVelBiasState<double>::Ptr data_ptr;

  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.frame_id = odom_frame_;
  odom_tf.child_frame_id = base_frame_;

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  // Set number of threads for VIO to use
  std::unique_ptr<tbb::global_control> tbb_global_control;
  if (thread_limit_ > 0) {
    tbb_global_control = std::make_unique<tbb::global_control>(
        tbb::global_control::max_allowed_parallelism, thread_limit_);
  }

  while (rclcpp::ok() && !should_stop_) {
    // Get data from queue
    if (!out_state_queue_.try_pop(data_ptr)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (!data_ptr) {
      RCLCPP_ERROR(get_logger(), "End of output data queue. Exiting...");
      break;
    }

    // Get timestamp
    rclcpp::Time time_stamp(data_ptr->t_ns);

    // Convert VIO output to transforms
    Eigen::Vector3d trans = data_ptr->T_w_i.translation();
    Eigen::Quaterniond quat = data_ptr->T_w_i.unit_quaternion();

    // Validate state before publishing (NASA §VII: check floating-point for NaN/Inf)
    if (!trans.allFinite() || !quat.coeffs().allFinite() ||
        !data_ptr->vel_w_i.allFinite()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Non-finite VIO state at t=%ld ns — skipping publish", data_ptr->t_ns);
      continue;
    }

    tf2::Vector3 tf_trans(trans.x(), trans.y(), trans.z());
    tf2::Quaternion tf_quat(quat.x(), quat.y(), quat.z(), quat.w());

    tf2::Transform odom_to_base(tf_quat, tf_trans);

    // If sensor_frame is different from base_frame, apply TF transformation
    if (sensor_frame_ != base_frame_) {
      try {
        geometry_msgs::msg::TransformStamped camera_to_base_msg = lookupTransform(base_frame_, sensor_frame_, time_stamp);
        const auto& t = camera_to_base_msg.transform;
        tf2::Vector3 tf_trans_cb(t.translation.x, t.translation.y, t.translation.z);
        tf2::Quaternion tf_quat_cb(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w);
        tf2::Transform camera_to_base(tf_quat_cb, tf_trans_cb);
        odom_to_base = tf2::Transform(tf_quat, tf_trans) * camera_to_base.inverse();
      } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to get transform: %s", e.what());
        continue;
      }
    }

    // Convert tf2::Transform to geometry_msgs manually
    odom_tf.header.stamp = time_stamp;
    odom_tf.transform.translation.x = odom_to_base.getOrigin().x();
    odom_tf.transform.translation.y = odom_to_base.getOrigin().y();
    odom_tf.transform.translation.z = odom_to_base.getOrigin().z();
    tf2::Quaternion q = odom_to_base.getRotation();
    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_tf.transform.rotation.w = q.w();

    // Update odometry message
    odom_msg.header = odom_tf.header;
    odom_msg.twist.twist.linear.x = data_ptr->vel_w_i(0);
    odom_msg.twist.twist.linear.y = data_ptr->vel_w_i(1);
    odom_msg.twist.twist.linear.z = data_ptr->vel_w_i(2);

    odom_msg.pose.pose.orientation = odom_tf.transform.rotation;
    odom_msg.pose.pose.position.x = odom_tf.transform.translation.x;
    odom_msg.pose.pose.position.y = odom_tf.transform.translation.y;
    odom_msg.pose.pose.position.z = odom_tf.transform.translation.z;

    // Publish message and transform
    odom_pub_->publish(odom_msg);
    tf_broadcaster_->sendTransform(odom_tf);

    // Record odometry timestamp for watchdog (Item 5)
    last_odom_ns_.store(data_ptr->t_ns, std::memory_order_relaxed);
  }
}

void VisualOdometerNode::statusThread() {
  basalt::VioVisualizationData::Ptr data_ptr;

  sensor_msgs::msg::PointCloud2 pcl_msg;

  // Setup point cloud fields manually (ROS2 doesn't have PointCloud2Modifier)
  pcl_msg.fields.resize(3);
  pcl_msg.fields[0].name = "x";
  pcl_msg.fields[0].offset = 0;
  pcl_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pcl_msg.fields[0].count = 1;

  pcl_msg.fields[1].name = "y";
  pcl_msg.fields[1].offset = 4;
  pcl_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pcl_msg.fields[1].count = 1;

  pcl_msg.fields[2].name = "z";
  pcl_msg.fields[2].offset = 8;
  pcl_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pcl_msg.fields[2].count = 1;

  pcl_msg.header.frame_id = sensor_frame_;
  pcl_msg.height = 1;
  pcl_msg.point_step = 12;
  pcl_msg.is_dense = true;

  // Pre-allocate buffers for reuse across iterations (O1 - processing optimisation)
  std::vector<Eigen::Vector3d> valid_points;
  valid_points.reserve(1000);
  cv::Mat gray16, gray8, bgr;

  while (rclcpp::ok() && !should_stop_) {
    // Get data from queue
    if (!out_vis_queue_.try_pop(data_ptr)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    if (!data_ptr) {
      RCLCPP_ERROR(get_logger(), "End of output data queue. Exiting...");
      break;
    }

    // Get timestamp
    rclcpp::Time time_stamp(data_ptr->t_ns);

    // Get current pose transform
    Sophus::SE3d T_w_i;
    if (!data_ptr->states.empty()) {
      T_w_i = data_ptr->states.back();
    } else if (!data_ptr->frames.empty()) {
      T_w_i = data_ptr->frames.back();
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Pointcloud unavailable!");
      continue;
    }

    // Extract valid points and apply transform (reuse pre-allocated vector)
    valid_points.clear();
    for (const Eigen::Vector3d& point : data_ptr->points) {
      valid_points.push_back(T_w_i * point);
    }

    // Serialize to PointCloud2
    pcl_msg.header.stamp = time_stamp;
    pcl_msg.width = valid_points.size();
    pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
    pcl_msg.data.resize(pcl_msg.row_step);

    for (size_t i = 0; i < valid_points.size(); ++i) {
      float x = static_cast<float>(valid_points[i].x());
      float y = static_cast<float>(valid_points[i].y());
      float z = static_cast<float>(valid_points[i].z());

      std::memcpy(&pcl_msg.data[i * 12 + 0], &x, sizeof(float));
      std::memcpy(&pcl_msg.data[i * 12 + 4], &y, sizeof(float));
      std::memcpy(&pcl_msg.data[i * 12 + 8], &z, sizeof(float));
    }

    if (publish_cloud_) {
      pcl_pub_->publish(pcl_msg);
    }

    // Publish annotated images with keypoint overlays
    if (publish_images_ && data_ptr->opt_flow_res) {
      for (int cam = 0; cam < 2; cam++) {
        auto& img_data = data_ptr->opt_flow_res->input_images->img_data[cam];
        if (!img_data.img) continue;

        auto& raw = img_data.img;
        // Convert uint16 grayscale -> 8-bit -> 3-channel BGR (reuse pre-allocated mats)
        cv::Mat gray16(raw->h, raw->w, CV_16UC1, raw->ptr);
        gray16.convertTo(gray8, CV_8UC1, 1.0 / 256.0);
        cv::cvtColor(gray8, bgr, cv::COLOR_GRAY2BGR);

        // Layer 1: optical-flow tracked keypoints (yellow rings)
        if (cam < static_cast<int>(data_ptr->opt_flow_res->observations.size())) {
          for (const auto& [kp_id, transform] : data_ptr->opt_flow_res->observations[cam]) {
            Eigen::Vector2f px = transform.translation();
            cv::circle(bgr, cv::Point(static_cast<int>(px.x()), static_cast<int>(px.y())),
                       3, cv::Scalar(196, 0, 138), 1);
          }
        }

        // Layer 2: VIO-confirmed 3D landmark reprojections (green filled dots)
        if (cam < static_cast<int>(data_ptr->projections.size())) {
          for (const Eigen::Vector4d& proj : data_ptr->projections[cam]) {
            cv::circle(bgr,
                       cv::Point(static_cast<int>(proj[0]), static_cast<int>(proj[1])),
                       4, cv::Scalar(131, 0, 255), -1);
          }
        }

        // Publish image
        auto img_msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", bgr).toImageMsg();
        img_msg->header.stamp = time_stamp;
        img_msg->header.frame_id = (cam == 0) ? "oak_left_camera_optical_frame"
                                               : "oak_right_camera_optical_frame";
        if (cam == 0) {
          left_img_pub_.publish(*img_msg);
        } else {
          right_img_pub_.publish(*img_msg);
        }
      }
    }

    // Keypoint quality ratio (VIO-confirmed 3D landmarks / optically-tracked features)
    if (data_ptr->opt_flow_res && !data_ptr->opt_flow_res->observations.empty()) {
      size_t tracked   = data_ptr->opt_flow_res->observations[0].size();
      size_t confirmed = data_ptr->projections.empty() ? 0
                                                       : data_ptr->projections[0].size();
      if (tracked > 0) {
        std_msgs::msg::Float32 ratio_msg;
        ratio_msg.data = static_cast<float>(confirmed) / static_cast<float>(tracked);
        keypoint_ratio_pub_->publish(ratio_msg);
        if (ratio_msg.data < 0.2f) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "Low keypoint quality: %.2f (%zu/%zu confirmed/tracked)",
              ratio_msg.data, confirmed, tracked);
        }
      }
    }
  }
}

geometry_msgs::msg::TransformStamped VisualOdometerNode::lookupTransform(
    const std::string& target,
    const std::string& source,
    const rclcpp::Time& time_stamp) {
  try {
    // Convert rclcpp::Time to tf2::TimePoint using system_clock
    auto ns = time_stamp.nanoseconds();
    auto time_point = std::chrono::system_clock::time_point(std::chrono::nanoseconds(ns));
    return tf_buffer_->lookupTransform(target, source, time_point, std::chrono::milliseconds(100));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Transform lookup failed: %s", e.what());
    throw;
  }
}

}  // namespace basalt

// Executable entry point for standalone node
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<basalt::VisualOdometerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
