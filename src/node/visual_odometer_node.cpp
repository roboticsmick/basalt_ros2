/**
 * @file visual_odometer_node.cpp
 * @brief ROS2 node that runs basalt visual odometry and publishes results as ROS2 messages.
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
#include <basalt/serialization/headers_serialization.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <std_msgs/msg/header.hpp>

namespace basalt
{

  // Load calibration from JSON using cereal — reads complete Calibration<> struct in one call,
  // so no field can be silently skipped. Throws clearly if file is missing or malformed.
  Calibration<double> loadCalibrationFromJSON(const std::string &calib_path)
  {
    Calibration<double> calib;
    std::ifstream file(calib_path, std::ios::binary);
    if (!file.is_open())
      throw std::runtime_error("Cannot open calibration file: " + calib_path);
    try {
      cereal::JSONInputArchive archive(file);
      archive(calib);
    } catch (const std::exception &e) {
      throw std::runtime_error(std::string("Failed to parse calibration JSON: ") + e.what());
    }
    return calib;
  }

  // Helper function to load calibration from YAML file
  Calibration<double> loadCalibrationFromYAML(const std::string &calib_path)
  {
    Calibration<double> calib;

    try
    {
      YAML::Node root = YAML::LoadFile(calib_path);

      // Load T_imu_cam transforms (IMU to camera extrinsics)
      if (!root["T_imu_cam"])
      {
        throw std::runtime_error("Missing 'T_imu_cam' in calibration YAML");
      }

      for (const auto &tf : root["T_imu_cam"])
      {
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
      if (!root["intrinsics"])
      {
        throw std::runtime_error("Missing 'intrinsics' in calibration YAML");
      }

      for (const auto &cam_node : root["intrinsics"])
      {
        std::string cam_type = cam_node["camera_type"].as<std::string>();

        // Extract intrinsic parameters
        const auto &intr_params = cam_node["intrinsics"];
        double fx = intr_params["fx"].as<double>();
        double fy = intr_params["fy"].as<double>();
        double cx = intr_params["cx"].as<double>();
        double cy = intr_params["cy"].as<double>();

        GenericCamera<double> cam;

        if (cam_type == "pinhole-radtan8")
        {
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
        }
        else if (cam_type == "pinhole")
        {
          // Standard pinhole model (4 parameters): fx, fy, cx, cy
          Eigen::Matrix<double, 4, 1> pinhole_params;
          pinhole_params << fx, fy, cx, cy;
          PinholeCamera<double> pinhole_cam(pinhole_params);
          cam.variant = pinhole_cam;
        }
        else
        {
          throw std::runtime_error("Unsupported camera type: " + cam_type);
        }

        calib.intrinsics.emplace_back(cam);
      }

      // Load resolution if available
      if (root["resolution"] && root["resolution"].IsSequence())
      {
        for (const auto &res : root["resolution"])
        {
          if (res.IsSequence() && res.size() >= 2)
          {
            calib.resolution.emplace_back(res[0].as<int>(), res[1].as<int>());
          }
        }
      }

      // Load IMU parameters — warn on missing fields so users don't run with silent bad defaults
      const auto calib_log = rclcpp::get_logger("basalt_calibration");

      if (root["imu_update_rate"])
      {
        calib.imu_update_rate = root["imu_update_rate"].as<double>();
      }
      else
      {
        RCLCPP_WARN(calib_log,
                    "imu_update_rate missing from calibration YAML — defaulting to %.0f Hz. "
                    "This scales IMU noise weighting (sigma_d = sigma_c * sqrt(rate)). "
                    "Add imu_update_rate to your calibration file.",
                    calib.imu_update_rate);
      }

      auto load_vec3 = [&](const char *key, Eigen::Vector3d &out)
      {
        if (root[key] && root[key].IsSequence() && root[key].size() == 3)
          out = Eigen::Vector3d(root[key][0].as<double>(),
                                root[key][1].as<double>(),
                                root[key][2].as<double>());
        else
          RCLCPP_WARN(calib_log, "'%s' missing from calibration YAML — using default values.", key);
      };
      load_vec3("accel_noise_std", calib.accel_noise_std);
      load_vec3("gyro_noise_std", calib.gyro_noise_std);
      load_vec3("accel_bias_std", calib.accel_bias_std);
      load_vec3("gyro_bias_std", calib.gyro_bias_std);

      if (root["calib_accel_bias"] && root["calib_accel_bias"].IsSequence() && root["calib_accel_bias"].size() == 9)
      {
        for (int i = 0; i < 9; ++i)
          calib.calib_accel_bias.getParam()(i) = root["calib_accel_bias"][i].as<double>();
      }
      else
      {
        RCLCPP_WARN(calib_log,
                    "calib_accel_bias missing or wrong size in calibration YAML — "
                    "using zero bias. VIO will estimate accelerometer bias online from scratch.");
      }

      if (root["calib_gyro_bias"] && root["calib_gyro_bias"].IsSequence() && root["calib_gyro_bias"].size() == 12)
      {
        for (int i = 0; i < 12; ++i)
          calib.calib_gyro_bias.getParam()(i) = root["calib_gyro_bias"][i].as<double>();
      }
      else
      {
        RCLCPP_WARN(calib_log,
                    "calib_gyro_bias missing or wrong size in calibration YAML — "
                    "using zero bias. VIO will estimate gyroscope bias online from scratch.");
      }

      if (root["cam_time_offset_ns"])
        calib.cam_time_offset_ns = root["cam_time_offset_ns"].as<int64_t>();

      return calib;
    }
    catch (const std::exception &e)
    {
      throw std::runtime_error(std::string("Failed to load calibration YAML: ") + e.what());
    }
  }

  VisualOdometerNode::VisualOdometerNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("basalt_vio_node", options), is_initialized_(false)
  {
    RCLCPP_INFO(get_logger(), "Creating basalt_vio_node...");

    // Declare parameters (must be declared before getting them)
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
    declare_parameter<bool>("publish_transform", true);
    declare_parameter<int>("odom_watchdog_timeout_ms", 5000);

    // Schedule async initialization after node is fully constructed
    init_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&VisualOdometerNode::initializeAsync, this));

    RCLCPP_INFO(get_logger(), "Constructor complete - async initialization scheduled");
  }

  void VisualOdometerNode::initializeAsync()
  {
    // Fire only once
    if (is_initialized_)
    {
      return;
    }
    is_initialized_ = true;
    init_timer_.reset();

    RCLCPP_INFO(get_logger(), "Starting async initialization...");

    try
    {
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
      publish_transform_ = get_parameter("publish_transform").as_bool();
      odom_watchdog_timeout_ms_ = get_parameter("odom_watchdog_timeout_ms").as_int();
      use_imu_ = !imu_topic.empty();

      // Initialize TF2 (now safe to call shared_from_this())
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());

      // Create publishers
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::SystemDefaultsQoS());
      status_pub_ = create_publisher<std_msgs::msg::Bool>("basalt_vio/status", rclcpp::SystemDefaultsQoS());
      keypoint_ratio_pub_ = create_publisher<std_msgs::msg::Float32>(
          "basalt_vio/keypoint_ratio", rclcpp::SystemDefaultsQoS());
      keypoint_stats_pub_ = create_publisher<basalt_ros2::msg::KeypointStats>(
          "basalt_vio/keypoint_stats", rclcpp::SystemDefaultsQoS());
      if (publish_cloud_)
      {
        pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("keypoints", rclcpp::SystemDefaultsQoS());
      }
      if (publish_images_)
      {
        image_transport::ImageTransport it(shared_from_this());
        left_img_pub_ = it.advertise("basalt_vio/left/image_annotated", 1);
        right_img_pub_ = it.advertise("basalt_vio/right/image_annotated", 1);
        RCLCPP_INFO(get_logger(), "Image annotation publishers created");
      }

      // Odometry freeze watchdog
      watchdog_timer_ = create_wall_timer(std::chrono::seconds(1), [this]()
                                          {
        int64_t last_ns = last_odom_ns_.load(std::memory_order_relaxed);
        if (last_ns == 0) return;
        int64_t age_ms = (get_clock()->now().nanoseconds() - last_ns) / 1'000'000;
        if (age_ms > odom_watchdog_timeout_ms_) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                "VIO FREEZE DETECTED: no odometry for %ld ms — triggering reset", age_ms);
            if (startup_success_ && !should_reset_ && !should_stop_) {
                startup_success_ = false;
                should_reset_ = true;
            }
        } });

      // Initialize camera
      RCLCPP_INFO(get_logger(), "Initializing camera from topics: %s, %s",
                  left_topic.c_str(), right_topic.c_str());
      camera_ = std::make_shared<RosCameraDevice>(shared_from_this(), left_topic, right_topic);
      camera_->start();

      // Initialize IMU
      if (use_imu_)
      {
        RCLCPP_INFO(get_logger(), "Initializing IMU from topic: %s", imu_topic.c_str());
        imu_ = std::make_shared<RosImuDevice>(shared_from_this(), imu_topic);
        imu_->start();
      }

      // Load calibration
      if (!calib_path.empty())
      {
        try
        {
          auto ends_with = [](const std::string &s, const std::string &sfx)
          {
            return s.size() >= sfx.size() &&
                   s.compare(s.size() - sfx.size(), sfx.size(), sfx) == 0;
          };
          const bool is_yaml = ends_with(calib_path, ".yaml") || ends_with(calib_path, ".yml");
          calib_ = is_yaml ? loadCalibrationFromYAML(calib_path)
                           : loadCalibrationFromJSON(calib_path);
          // Stereo VIO uses cam0+cam1 only — trim extra cameras from combined calibrations
          if (calib_.intrinsics.size() > 2) {
            calib_.intrinsics.resize(2);
            calib_.T_i_c.resize(2);
            calib_.resolution.resize(2);
          }
          RCLCPP_INFO(get_logger(), "Loaded calibration from: %s (%zu cameras, %zu transforms)",
                      calib_path.c_str(), calib_.intrinsics.size(), calib_.T_i_c.size());
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(get_logger(), "Failed to load calibration: %s", e.what());
          RCLCPP_WARN(get_logger(), "Falling back to ROS CameraInfo calibration");
          if (use_imu_)
          {
            RCLCPP_WARN(get_logger(), "IMU disabled — calibration file required for IMU fusion");
            use_imu_ = false;
            if (imu_)
            {
              imu_->stop();
              imu_.reset();
            }
          }
          calib_ = camera_->exportCalibration();
        }
      }
      else
      {
        if (use_imu_)
        {
          RCLCPP_WARN(get_logger(), "IMU requires calibration file — falling back to VO only");
          use_imu_ = false;
          if (imu_)
          {
            imu_->stop();
            imu_.reset();
          }
        }
        RCLCPP_WARN(get_logger(), "No calibration file — generating from ROS CameraInfo");
        calib_ = camera_->exportCalibration();
      }
      RCLCPP_INFO(get_logger(), "Calibration ready");

      // Get sensor frame ID
      RCLCPP_INFO(get_logger(), "Waiting for frame_id from camera...");
      sensor_frame_ = camera_->getFrameId();
      if (sensor_frame_.empty())
      {
        RCLCPP_WARN(get_logger(), "No frame_id from sensor, assuming '%s'", base_frame_.c_str());
        sensor_frame_ = base_frame_;
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Sensor frame: %s", sensor_frame_.c_str());
      }

      // Load VIO config (stored as member for re-use across resets)
      if (!config_path.empty())
      {
        try
        {
          vio_config_.load(config_path);
          RCLCPP_INFO(get_logger(), "Loaded VIO config from %s", config_path.c_str());
        }
        catch (const std::exception &e)
        {
          RCLCPP_WARN(get_logger(), "Failed to load VIO config: %s — using defaults", e.what());
        }

        // Parse health tracking parameters from the same config file.
        // These keys are ignored by VioConfig::load() but read here separately.
        auto ends_with = [](const std::string &s, const std::string &sfx)
        {
          return s.size() >= sfx.size() &&
                 s.compare(s.size() - sfx.size(), sfx.size(), sfx) == 0;
        };
        const bool cfg_is_yaml = ends_with(config_path, ".yaml") || ends_with(config_path, ".yml");
        try
        {
          if (cfg_is_yaml)
          {
            YAML::Node cfg = YAML::LoadFile(config_path);
            if (cfg["vio_health_min_keypoints"])
              min_keypoints_ = cfg["vio_health_min_keypoints"].as<int>();
            if (cfg["vio_health_max_velocity"])
              max_velocity_ = cfg["vio_health_max_velocity"].as<double>();
            if (cfg["vio_health_max_acceleration"])
              max_acceleration_ = cfg["vio_health_max_acceleration"].as<double>();
            if (cfg["vio_health_startup_duration"])
              health_startup_duration_ = cfg["vio_health_startup_duration"].as<double>();
            if (cfg["vio_health_keypoint_timeout"])
              health_keypoint_timeout_ = cfg["vio_health_keypoint_timeout"].as<double>();
            if (cfg["vio_node_debug"])
              node_debug_ = cfg["vio_node_debug"].as<bool>();
          }
          else
          {
            // JSON: health params are inside "value0"
            std::ifstream f(config_path);
            nlohmann::json j;
            f >> j;
            const auto &v = j.contains("value0") ? j["value0"] : j;
            if (v.contains("vio_health_min_keypoints"))
              min_keypoints_ = v["vio_health_min_keypoints"].get<int>();
            if (v.contains("vio_health_max_velocity"))
              max_velocity_ = v["vio_health_max_velocity"].get<double>();
            if (v.contains("vio_health_max_acceleration"))
              max_acceleration_ = v["vio_health_max_acceleration"].get<double>();
            if (v.contains("vio_health_startup_duration"))
              health_startup_duration_ = v["vio_health_startup_duration"].get<double>();
            if (v.contains("vio_health_keypoint_timeout"))
              health_keypoint_timeout_ = v["vio_health_keypoint_timeout"].get<double>();
            if (v.contains("vio_node_debug"))
              node_debug_ = v["vio_node_debug"].get<bool>();
          }
          RCLCPP_INFO(get_logger(),
                      "Health params: min_kp=%d, max_v=%.2f m/s, max_a=%.2f m/s, startup=%.2fs, kp_timeout=%.2fs",
                      min_keypoints_, max_velocity_, max_acceleration_,
                      health_startup_duration_, health_keypoint_timeout_);
        }
        catch (const std::exception &e)
        {
          RCLCPP_WARN(get_logger(),
                      "Could not read health params from config: %s — using defaults", e.what());
        }
      }

      // Create health trackers
      keypoint_health_ = std::make_unique<OdometerHealthTrack>(health_keypoint_timeout_, health_startup_duration_);
      velocity_health_ = std::make_unique<OdometerHealthTrack>(0.0, health_startup_duration_);
      acceleration_health_ = std::make_unique<OdometerHealthTrack>(0.0, health_startup_duration_);

      // Initialize optical flow — stays running across VIO resets
      optical_flow_ = OpticalFlowFactory::getOpticalFlow(vio_config_, calib_);
      optical_flow_->input_queue.set_capacity(10); // Q1 - bound optical flow input queue
      RCLCPP_INFO(get_logger(), "Optical flow initialized");

      // Initialize bounded output queues (Q2 - 30 = 3 sec at 10 Hz)
      out_state_queue_.set_capacity(30);
      out_vis_queue_.set_capacity(30);

      // Start processing threads.
      // runtimeThread manages the VIO lifecycle (init → run → reset → repeat).
      // statusThread always runs for health monitoring regardless of vis flags.
      camera_processing_thread_ = std::thread(&VisualOdometerNode::cameraProcessingThread, this);
      processing_thread_ = std::thread(&VisualOdometerNode::runtimeThread, this);
      status_thread_ = std::thread(&VisualOdometerNode::statusThread, this);

      RCLCPP_INFO(get_logger(), "Async initialization complete, node started!");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Initialization failed: %s", e.what());
      is_initialized_ = false;
    }
  }

  VisualOdometerNode::~VisualOdometerNode()
  {
    RCLCPP_INFO(get_logger(), "Shutting down node...");
    should_stop_ = true;

    if (imu_)
      imu_->stop();
    if (camera_)
      camera_->stop();

    if (camera_processing_thread_.joinable())
      camera_processing_thread_.join();
    if (processing_thread_.joinable())
      processing_thread_.join();
    if (status_thread_.joinable())
      status_thread_.join();

    RCLCPP_INFO(get_logger(), "Terminated, cleaning up.");
  }

  // =============================================================================
  // Camera Processing Thread — always running
  // Feeds frames into optical flow and forwards IMU data to VIO.
  // =============================================================================
  void VisualOdometerNode::cameraProcessingThread()
  {
    uint64_t t_ns;
    std::vector<ManagedImage<uint16_t>> images;
    uint32_t frame_count = 0;
    uint32_t consecutive_misses = 0;

    RCLCPP_INFO(get_logger(), "Camera processing thread started");

    while (rclcpp::ok() && !should_stop_)
    {
      // Poll camera for images
      if (camera_->poll(t_ns, images))
      {
        frame_count++;
        consecutive_misses = 0;
        if (node_debug_)
        {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "frames=%u", frame_count);
        }

        // Create OpticalFlowInput and push to pipeline
        auto opt_flow_input = std::make_shared<OpticalFlowInput>();
        opt_flow_input->t_ns = t_ns;
        for (auto &img : images)
        {
          ImageData img_data;
          auto managed_img = std::make_shared<ManagedImage<uint16_t>>();
          *managed_img = std::move(img);
          img_data.img = managed_img;
          img_data.exposure = 0.0;
          opt_flow_input->img_data.push_back(img_data);
        }
        images.clear();

        // Non-blocking push — drop frame rather than stall camera thread
        if (!optical_flow_->input_queue.try_push(opt_flow_input))
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                               "Optical flow input queue full — dropping frame at t=%lu ns", t_ns);
        }
      }
      else
      {
        consecutive_misses++;
        if (consecutive_misses >= 1000)
        {
          RCLCPP_WARN(get_logger(),
                      "Camera: no frame for %u consecutive polls (~%u ms) — driver running?",
                      consecutive_misses, consecutive_misses);
          consecutive_misses = 0;
        }
      }

      // Forward IMU data to VIO — guarded by vio_mutex_ to prevent access during reset
      if (imu_)
      {
        uint64_t t_imu;
        Eigen::Vector3d accel, gyro;
        while (imu_->poll(t_imu, accel, gyro))
        {
          std::lock_guard<std::mutex> lock(vio_mutex_);
          if (!vio_estimator_)
            break; // VIO is mid-reset
          auto imu_data = std::make_shared<ImuData<double>>();
          imu_data->t_ns = static_cast<int64_t>(t_imu);
          imu_data->accel = accel;
          imu_data->gyro = gyro;
          vio_estimator_->addIMUToQueue(imu_data);
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    RCLCPP_INFO(get_logger(), "Camera processing thread exited (%u total frames)", frame_count);
  }

  // =============================================================================
  // Runtime Thread — always running (outer loop = VIO lifecycle)
  // Initialises VIO, spawns odometerThread, waits for reset signal,
  // tears down VIO, drains queues, and repeats.
  // Optical flow is NOT reset — it keeps running across VIO resets for
  // faster recovery (same design as basalt_ros).
  // =============================================================================
  void VisualOdometerNode::runtimeThread()
  {
    RCLCPP_INFO(get_logger(), "Runtime (VIO lifecycle) thread started");

    while (rclcpp::ok() && !should_stop_)
    {

      // --- Initialise VIO ---
      RCLCPP_INFO(get_logger(), "Initialising VIO estimator...");
      {
        std::lock_guard<std::mutex> lock(vio_mutex_);
        vio_estimator_ = VioEstimatorFactory::getVioEstimator(
            vio_config_, calib_, constants::g, use_imu_, false);  // float scalar (faster on ARM NEON)
        vio_estimator_->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

        // Connect optical flow output to VIO vision input
        optical_flow_->output_queue = &vio_estimator_->vision_data_queue;

        // Connect VIO output queues
        vio_estimator_->out_state_queue = &out_state_queue_;
        // Always connect vis queue — statusThread needs it for keypoint health tracking
        vio_estimator_->out_vis_queue = &out_vis_queue_;
      }
      RCLCPP_INFO(get_logger(), "VIO estimator ready");

      // Spawn odometerThread for this VIO lifecycle
      std::thread odometer_t(&VisualOdometerNode::odometerThread, this);

      // --- Wait for reset signal (from statusThread) or shutdown ---
      while (rclcpp::ok() && !should_stop_ && !should_reset_)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }

      if (should_stop_)
      {
        // Shutdown path — odometerThread will exit via should_stop_
        if (odometer_t.joinable())
          odometer_t.join();
        break;
      }

      // --- Tear down VIO ---
      RCLCPP_INFO(get_logger(), "Tearing down VIO for reset...");

      // Disconnect optical flow from VIO (null guard in optical flow prevents crash)
      optical_flow_->output_queue = nullptr;

      {
        std::lock_guard<std::mutex> lock(vio_mutex_);
        // Push terminators into VIO input queues so its internal threads clean up
        vio_estimator_->vision_data_queue.push(nullptr);
        if (use_imu_)
        {
          // Push a sentinel ImuData to unblock the IMU processing thread
          auto sentinel = std::make_shared<ImuData<double>>();
          vio_estimator_->imu_data_queue.push(sentinel);
        }
        vio_estimator_.reset();
      }

      // Wait for odometerThread to exit (it checks should_reset_)
      if (odometer_t.joinable())
        odometer_t.join();

      // Drain any stale items left in queues from the old VIO cycle
      {
        PoseVelBiasState<double>::Ptr tmp;
        while (out_state_queue_.try_pop(tmp))
        {
        }
      }
      {
        VioVisualizationData::Ptr tmp;
        while (out_vis_queue_.try_pop(tmp))
        {
        }
      }

      // Flush stale device buffers so new VIO starts with fresh data
      camera_->flushBuffers();
      if (imu_) imu_->flushBuffers();

      // Reset health state and flags for the new VIO cycle
      keypoint_health_->reset();
      velocity_health_->reset();
      acceleration_health_->reset();
      startup_success_ = false;
      should_reset_ = false;

      RCLCPP_INFO(get_logger(), "VIO reset complete, reinitialising...");
    }

    RCLCPP_INFO(get_logger(), "Runtime thread exited");
  }

  // =============================================================================
  // Odometer Thread — one per VIO lifecycle, spawned by runtimeThread
  // Processes VIO state estimates, publishes odometry, and tracks
  // velocity/acceleration health. Exits when should_reset_ or should_stop_.
  // Publishes identity/zero pose before startup_success_ to prevent
  // bad data reaching downstream navigation.
  // =============================================================================
  void VisualOdometerNode::odometerThread()
  {
    basalt::PoseVelBiasState<double>::Ptr data_ptr;

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.frame_id = odom_frame_;
    odom_tf.child_frame_id = base_frame_;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    // Set TBB thread limit for VIO solver
    std::unique_ptr<tbb::global_control> tbb_global_control;
    if (thread_limit_ > 0)
    {
      tbb_global_control = std::make_unique<tbb::global_control>(
          tbb::global_control::max_allowed_parallelism, thread_limit_);
    }

    // Previous velocity for acceleration health (base frame, norm is frame-invariant)
    Eigen::Vector3d v_b_prev = Eigen::Vector3d::Zero();

    RCLCPP_INFO(get_logger(), "Odometer thread started");

    while (rclcpp::ok() && !should_stop_ && !should_reset_)
    {
      if (!out_state_queue_.try_pop(data_ptr))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      if (!data_ptr) {
        // nullptr is either: (a) normal VIO shutdown — should_reset_ / should_stop_
        // already set, while-condition exits cleanly; or (b) numerical failure sentinel
        // pushed by optimize() — trigger reset now.
        if (!should_stop_ && !should_reset_) {
          RCLCPP_WARN(get_logger(), "VIO numerical failure detected — triggering reset");
          startup_success_ = false;
          should_reset_ = true;
        }
        continue;
      }

      // Validate state (check for NaN/Inf before publishing)
      if (!data_ptr->T_w_i.translation().allFinite() ||
          !data_ptr->T_w_i.unit_quaternion().coeffs().allFinite() ||
          !data_ptr->vel_w_i.allFinite())
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Non-finite VIO state at t=%ld ns — triggering reset", data_ptr->t_ns);
        if (!should_stop_ && !should_reset_)
        {
          startup_success_ = false;
          should_reset_ = true;
        }
        continue;
      }

      rclcpp::Time time_stamp(data_ptr->t_ns);
      auto now_tp = std::chrono::steady_clock::now();

      // Convert IMU pose to left camera pose using the extrinsic loaded from calib_path.
      // calib_.T_i_c[0] is the cam→IMU transform: p_imu = T_i_c * p_cam.
      // T_w_cam0 = T_w_i * T_i_c[0]  →  cam→imu→world  →  left camera pose in world frame.
      Sophus::SE3d T_w_b = data_ptr->T_w_i * calib_.T_i_c[0];

      // Rotate world-frame IMU velocity into base frame (REP-105: twist must be in child_frame_id)
      Eigen::Vector3d v_b = T_w_b.unit_quaternion().inverse() * data_ptr->vel_w_i;

      Eigen::Vector3d trans_wb = T_w_b.translation();
      Eigen::Quaterniond quat_wb = T_w_b.unit_quaternion();

      odom_tf.header.stamp = time_stamp;
      odom_tf.transform.translation.x = trans_wb.x();
      odom_tf.transform.translation.y = trans_wb.y();
      odom_tf.transform.translation.z = trans_wb.z();
      odom_tf.transform.rotation.x = quat_wb.x();
      odom_tf.transform.rotation.y = quat_wb.y();
      odom_tf.transform.rotation.z = quat_wb.z();
      odom_tf.transform.rotation.w = quat_wb.w();

      odom_msg.header = odom_tf.header;
      odom_msg.twist.twist.linear.x = v_b(0);
      odom_msg.twist.twist.linear.y = v_b(1);
      odom_msg.twist.twist.linear.z = v_b(2);
      odom_msg.pose.pose.orientation = odom_tf.transform.rotation;
      odom_msg.pose.pose.position.x = trans_wb.x();
      odom_msg.pose.pose.position.y = trans_wb.y();
      odom_msg.pose.pose.position.z = trans_wb.z();

      if (!startup_success_)
      {
        // Publish identity pose + zero twist — prevents garbage data reaching AUV nav stack
        odom_msg.pose = geometry_msgs::msg::PoseWithCovariance();
        odom_msg.pose.pose.orientation.w = 1.0; // valid unit quaternion
        odom_msg.twist = geometry_msgs::msg::TwistWithCovariance();
        // Do NOT broadcast TF until startup is confirmed
      }
      else
      {
        if (publish_transform_)
        {
          tf_broadcaster_->sendTransform(odom_tf);
        }
      }

      odom_pub_->publish(odom_msg);
      last_odom_ns_.store(data_ptr->t_ns, std::memory_order_relaxed);

      // Update velocity and acceleration health trackers
      double speed = v_b.norm();
      double accel_step = (v_b - v_b_prev).norm();
      last_speed_.store(speed, std::memory_order_relaxed);
      velocity_health_->update(speed <= max_velocity_, now_tp);
      acceleration_health_->update(accel_step <= max_acceleration_, now_tp);
      v_b_prev = v_b;
    }

    RCLCPP_INFO(get_logger(), "Odometer thread exited");
  }

  // =============================================================================
  // Status Thread — always running
  // Processes VIO visualisation data, publishes point cloud and annotated images,
  // tracks keypoint health, and monitors all three health trackers at 10 Hz.
  // Sets should_reset_ when runtime health fails; sets startup_success_ when
  // all startup criteria are met.
  // =============================================================================
  void VisualOdometerNode::statusThread()
  {
    basalt::VioVisualizationData::Ptr data_ptr;

    // Setup PointCloud2 message fields
    sensor_msgs::msg::PointCloud2 pcl_msg;
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

    // Pre-allocate reusable buffers (O1)
    std::vector<Eigen::Vector3d> valid_points;
    valid_points.reserve(1000);
    cv::Mat gray16, gray8, bgr;

    auto last_health_check = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "Status thread started");

    while (rclcpp::ok() && !should_stop_)
    {

      auto now_tp = std::chrono::steady_clock::now();

      // --- Health monitoring at 10 Hz (independent of vis data availability) ---
      if (std::chrono::duration_cast<std::chrono::milliseconds>(
              now_tp - last_health_check)
              .count() >= 100)
      {
        last_health_check = now_tp;

        if (startup_success_)
        {
          // Runtime: trigger VIO reset if any tracker fails
          if (!keypoint_health_->runtime_healthy())
          {
            RCLCPP_WARN(get_logger(), "Keypoint health failure — triggering VIO reset");
            startup_success_ = false;
            should_reset_ = true;
          }
          else if (!velocity_health_->runtime_healthy())
          {
            RCLCPP_WARN(get_logger(), "Velocity health failure — triggering VIO reset");
            startup_success_ = false;
            should_reset_ = true;
          }
          else if (!acceleration_health_->runtime_healthy())
          {
            RCLCPP_WARN(get_logger(), "Acceleration health failure — triggering VIO reset");
            startup_success_ = false;
            should_reset_ = true;
          }
        }
        else
        {
          // Startup: declare success when all three trackers are healthy
          if (!should_reset_ &&
              keypoint_health_->startup_healthy() &&
              velocity_health_->startup_healthy() &&
              acceleration_health_->startup_healthy())
          {
            RCLCPP_INFO(get_logger(), "VIO startup criteria met — publishing odometry");
            startup_success_ = true;
          }
          // Velocity and acceleration are physical safety bounds — fire even before startup
          else if (!should_reset_ &&
                   velocity_health_->is_initialised() &&
                   !velocity_health_->runtime_healthy())
          {
            RCLCPP_WARN(get_logger(), "Velocity safety limit exceeded before startup — triggering reset");
            should_reset_ = true;
          }
          else if (!should_reset_ &&
                   acceleration_health_->is_initialised() &&
                   !acceleration_health_->runtime_healthy())
          {
            RCLCPP_WARN(get_logger(), "Acceleration safety limit exceeded before startup — triggering reset");
            should_reset_ = true;
          }
        }

        // Publish estimate_valid status
        std_msgs::msg::Bool status_msg;
        status_msg.data = startup_success_.load();
        status_pub_->publish(status_msg);
      }

      // --- Process visualisation data ---
      if (!out_vis_queue_.try_pop(data_ptr))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      if (!data_ptr)
        continue;

      rclcpp::Time time_stamp(data_ptr->t_ns);

      // Get current pose transform for world-space point projection
      Sophus::SE3d T_w_i;
      if (!data_ptr->states.empty())
      {
        T_w_i = data_ptr->states.back();
      }
      else if (!data_ptr->frames.empty())
      {
        T_w_i = data_ptr->frames.back();
      }
      else
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Pointcloud unavailable!");
        continue;
      }

      // data_ptr->points are in world frame — transform to left camera frame
      // so pcl_msg frame_id = sensor_frame_ is correct.
      const Sophus::SE3d T_cam_w = (T_w_i * calib_.T_i_c[0]).inverse();
      valid_points.clear();
      for (const Eigen::Vector3d &point : data_ptr->points)
      {
        valid_points.push_back(T_cam_w * point);
      }

      // Update keypoint health (uses count of VIO-tracked 3D points)
      keypoint_health_->update(
          valid_points.size() >= static_cast<size_t>(min_keypoints_),
          std::chrono::steady_clock::now());

      // Publish point cloud
      if (publish_cloud_)
      {
        pcl_msg.header.stamp = time_stamp;
        pcl_msg.width = valid_points.size();
        pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
        pcl_msg.data.resize(pcl_msg.row_step);
        for (size_t i = 0; i < valid_points.size(); ++i)
        {
          float x = static_cast<float>(valid_points[i].x());
          float y = static_cast<float>(valid_points[i].y());
          float z = static_cast<float>(valid_points[i].z());
          std::memcpy(&pcl_msg.data[i * 12 + 0], &x, sizeof(float));
          std::memcpy(&pcl_msg.data[i * 12 + 4], &y, sizeof(float));
          std::memcpy(&pcl_msg.data[i * 12 + 8], &z, sizeof(float));
        }
        pcl_pub_->publish(pcl_msg);
      }

      // Publish annotated images with keypoint overlays
      if (publish_images_ && data_ptr->opt_flow_res)
      {
        for (int cam = 0; cam < 2; cam++)
        {
          auto &img_data = data_ptr->opt_flow_res->input_images->img_data[cam];
          if (!img_data.img)
            continue;

          auto &raw = img_data.img;
          cv::Mat gray16_local(raw->h, raw->w, CV_16UC1, raw->ptr);
          gray16_local.convertTo(gray8, CV_8UC1, 1.0 / 256.0);
          cv::cvtColor(gray8, bgr, cv::COLOR_GRAY2BGR);

          // Layer 1: optical-flow tracked keypoints (pink rings)
          if (cam < static_cast<int>(data_ptr->opt_flow_res->observations.size()))
          {
            for (const auto &[kp_id, transform] : data_ptr->opt_flow_res->observations[cam])
            {
              Eigen::Vector2f px = transform.translation();
              cv::circle(bgr, cv::Point(static_cast<int>(px.x()), static_cast<int>(px.y())),
                         3, cv::Scalar(196, 0, 138), 1);
            }
          }

          // Layer 2: VIO-confirmed 3D landmark reprojections (purple filled dots)
          if (cam < static_cast<int>(data_ptr->projections.size()))
          {
            for (const Eigen::Vector4d &proj : data_ptr->projections[cam])
            {
              cv::circle(bgr,
                         cv::Point(static_cast<int>(proj[0]), static_cast<int>(proj[1])),
                         4, cv::Scalar(131, 0, 255), -1);
            }
          }

          auto img_msg = cv_bridge::CvImage(
                             std_msgs::msg::Header(), "bgr8", bgr)
                             .toImageMsg();
          img_msg->header.stamp = time_stamp;
          img_msg->header.frame_id = (cam == 0) ? "oak_left_camera_optical_frame"
                                                : "oak_right_camera_optical_frame";
          if (cam == 0)
            left_img_pub_.publish(*img_msg);
          else
            right_img_pub_.publish(*img_msg);
        }
      }

      // Keypoint quality ratio (VIO-confirmed 3D landmarks / optically-tracked features)
      if (data_ptr->opt_flow_res && !data_ptr->opt_flow_res->observations.empty())
      {
        const auto &obs = data_ptr->opt_flow_res->observations;
        size_t tracked_l = obs[0].size();
        size_t tracked_r = obs.size() > 1 ? obs[1].size() : 0;
        size_t confirmed_l = data_ptr->projections.empty() ? 0 : data_ptr->projections[0].size();
        size_t confirmed_r = data_ptr->projections.size() > 1 ? data_ptr->projections[1].size() : 0;

        if (tracked_l > 0)
        {
          float ratio = static_cast<float>(confirmed_l) / static_cast<float>(tracked_l);

          std_msgs::msg::Float32 ratio_msg;
          ratio_msg.data = ratio;
          keypoint_ratio_pub_->publish(ratio_msg);

          basalt_ros2::msg::KeypointStats stats_msg;
          stats_msg.ratio          = ratio;
          stats_msg.tracked_left   = static_cast<uint32_t>(tracked_l);
          stats_msg.tracked_right  = static_cast<uint32_t>(tracked_r);
          stats_msg.confirmed_left = static_cast<uint32_t>(confirmed_l);
          stats_msg.confirmed_right = static_cast<uint32_t>(confirmed_r);
          keypoint_stats_pub_->publish(stats_msg);

          if (ratio < 0.2f)
          {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Low keypoint quality: %.2f (%zu/%zu confirmed/tracked)",
                                 ratio, confirmed_l, tracked_l);
          }

          if (node_debug_)
          {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "kp=%.2f (%zu/%zu L | %zu/%zu R) | speed=%.3fm/s"
                                 " | health: kp=%s vel=%s accel=%s",
                                 ratio,
                                 confirmed_l, tracked_l,
                                 confirmed_r, tracked_r,
                                 last_speed_.load(std::memory_order_relaxed),
                                 keypoint_health_->runtime_healthy() ? "OK" : "FAIL",
                                 velocity_health_->runtime_healthy() ? "OK" : "FAIL",
                                 acceleration_health_->runtime_healthy() ? "OK" : "FAIL");
          }
        }
      }
    }

    RCLCPP_INFO(get_logger(), "Status thread exited");
  }

  geometry_msgs::msg::TransformStamped VisualOdometerNode::lookupTransform(
      const std::string &target,
      const std::string &source,
      const rclcpp::Time &time_stamp)
  {
    try
    {
      auto ns = time_stamp.nanoseconds();
      auto time_point = std::chrono::system_clock::time_point(std::chrono::nanoseconds(ns));
      return tf_buffer_->lookupTransform(target, source, time_point, std::chrono::milliseconds(100));
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Transform lookup failed: %s", e.what());
      throw;
    }
  }

} // namespace basalt

// Executable entry point for standalone node
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<basalt::VisualOdometerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
