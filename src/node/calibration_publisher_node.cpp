#include <basalt/node/calibration_publisher_node.h>
#include <basalt/camera/generic_camera.hpp>
#include <basalt/camera/pinhole_camera.hpp>
#include <basalt/camera/pinhole_radtan8_camera.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

namespace basalt {
namespace {

// Helper: Load calibration from JSON file (anonymous namespace to avoid linker conflicts)
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

    if (!j.contains("value0")) {
      throw std::runtime_error("Invalid calibration JSON: missing 'value0' key");
    }

    const auto& calib_data = j["value0"];

    // Load T_imu_cam transforms
    if (calib_data.contains("T_imu_cam") && calib_data["T_imu_cam"].is_array()) {
      for (const auto& transform : calib_data["T_imu_cam"]) {
        Sophus::SE3d T_imu_cam;
        Eigen::Vector3d translation(transform["px"].get<double>(),
                                     transform["py"].get<double>(),
                                     transform["pz"].get<double>());
        Eigen::Quaterniond quat(transform["qw"].get<double>(),
                               transform["qx"].get<double>(),
                               transform["qy"].get<double>(),
                               transform["qz"].get<double>());
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
        const auto& intr_params = intr_data["intrinsics"];
        double fx = intr_params["fx"].get<double>();
        double fy = intr_params["fy"].get<double>();
        double cx = intr_params["cx"].get<double>();
        double cy = intr_params["cy"].get<double>();

        GenericCamera<double> cam;

        if (cam_type == "pinhole-radtan8") {
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

    // Load resolution
    if (calib_data.contains("resolution") && calib_data["resolution"].is_array()) {
      try {
        auto res_array = calib_data["resolution"].get<std::vector<std::array<int, 2>>>();
        for (const auto& res : res_array) {
          calib.resolution.emplace_back(res[0], res[1]);
        }
      } catch (const std::exception&) {
        // Skip if parsing fails
      }
    }

    return calib;
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("Failed to load calibration JSON: ") + e.what());
  }
}

// Helper: Load calibration from YAML file
Calibration<double> loadCalibrationFromYAML(const std::string& calib_path) {
  Calibration<double> calib;

  try {
    YAML::Node root = YAML::LoadFile(calib_path);

    if (!root["T_imu_cam"]) {
      throw std::runtime_error("Missing 'T_imu_cam' in calibration YAML");
    }

    for (const auto& tf : root["T_imu_cam"]) {
      Eigen::Vector3d translation(tf["px"].as<double>(),
                                   tf["py"].as<double>(),
                                   tf["pz"].as<double>());
      Eigen::Quaterniond quat(tf["qw"].as<double>(),
                             tf["qx"].as<double>(),
                             tf["qy"].as<double>(),
                             tf["qz"].as<double>());
      Sophus::SE3d T_imu_cam = Sophus::SE3d(quat, translation);
      calib.T_i_c.emplace_back(T_imu_cam);
    }

    if (!root["intrinsics"]) {
      throw std::runtime_error("Missing 'intrinsics' in calibration YAML");
    }

    for (const auto& cam_node : root["intrinsics"]) {
      std::string cam_type = cam_node["camera_type"].as<std::string>();
      const auto& intr_params = cam_node["intrinsics"];
      double fx = intr_params["fx"].as<double>();
      double fy = intr_params["fy"].as<double>();
      double cx = intr_params["cx"].as<double>();
      double cy = intr_params["cy"].as<double>();

      GenericCamera<double> cam;

      if (cam_type == "pinhole-radtan8") {
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
        Eigen::Matrix<double, 4, 1> pinhole_params;
        pinhole_params << fx, fy, cx, cy;
        PinholeCamera<double> pinhole_cam(pinhole_params);
        cam.variant = pinhole_cam;
      } else {
        throw std::runtime_error("Unsupported camera type: " + cam_type);
      }
      calib.intrinsics.emplace_back(cam);
    }

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

}  // anonymous namespace

CalibrationPublisherNode::CalibrationPublisherNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("calibration_publisher_node", options) {
  RCLCPP_INFO(get_logger(), "Creating calibration_publisher_node...");

  // Declare parameters
  declare_parameter<std::string>("calib_path", "");
  declare_parameter<std::string>("left_camera_info_topic", "/basalt/left/camera_info");
  declare_parameter<std::string>("right_camera_info_topic", "/basalt/right/camera_info");
  declare_parameter<int>("publish_interval_hz", 10);

  // Schedule async initialization
  init_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CalibrationPublisherNode::initializeAsync, this));

  RCLCPP_INFO(get_logger(), "Constructor complete - async initialization scheduled");
}

CalibrationPublisherNode::~CalibrationPublisherNode() {
  if (init_timer_) init_timer_.reset();
  if (publish_timer_) publish_timer_.reset();
}

void CalibrationPublisherNode::initializeAsync() {
  if (is_initialized_) {
    return;
  }
  is_initialized_ = true;
  init_timer_.reset();

  RCLCPP_INFO(get_logger(), "Starting async initialization...");

  try {
    std::string calib_path = get_parameter("calib_path").as_string();
    std::string left_topic = get_parameter("left_camera_info_topic").as_string();
    std::string right_topic = get_parameter("right_camera_info_topic").as_string();
    int hz = get_parameter("publish_interval_hz").as_int();

    if (calib_path.empty()) {
      throw std::runtime_error("calib_path parameter is empty");
    }

    // Load calibration
    auto ends_with = [](const std::string& s, const std::string& sfx) {
      return s.size() >= sfx.size() &&
             s.compare(s.size() - sfx.size(), sfx.size(), sfx) == 0;
    };
    const bool is_yaml = ends_with(calib_path, ".yaml") || ends_with(calib_path, ".yml");
    calib_ = is_yaml ? loadCalibrationFromYAML(calib_path)
                      : loadCalibrationFromJSON(calib_path);

    RCLCPP_INFO(get_logger(), "Successfully loaded calibration from: %s", calib_path.c_str());

    if (calib_.intrinsics.size() < 2) {
      throw std::runtime_error("Calibration must contain at least 2 cameras");
    }
    if (calib_.T_i_c.size() < 2) {
      throw std::runtime_error("Calibration must contain at least 2 IMU-camera transforms");
    }

    // Extract camera parameters
    cv::Mat K_left, D_left, K_right, D_right;
    cv::Size image_size;

    if (!calib_.resolution.empty()) {
      // calib_.resolution is vector of Eigen::Matrix<int, 2, 1> (width, height)
      image_size = cv::Size(calib_.resolution[0](0), calib_.resolution[0](1));
    } else {
      // Default fallback
      image_size = cv::Size(640, 400);
    }

    cameraToKD(calib_.intrinsics[0], K_left, D_left, image_size);
    cameraToKD(calib_.intrinsics[1], K_right, D_right, image_size);

    // Compute relative pose: right camera in left camera frame
    // T_left_imu = inv(T_imu_left), T_right_imu = T_imu_right
    // T_left_right = T_left_imu * T_imu_right
    Sophus::SE3d T_imu_left = calib_.T_i_c[0];
    Sophus::SE3d T_imu_right = calib_.T_i_c[1];
    Sophus::SE3d T_left_imu = T_imu_left.inverse();
    Sophus::SE3d T_left_right = T_left_imu * T_imu_right;

    // Extract R and t for cv::stereoRectify
    Eigen::Matrix3d R_mat = T_left_right.rotationMatrix();
    Eigen::Vector3d t_vec = T_left_right.translation();

    cv::Mat R(3, 3, CV_64F);
    cv::Mat t(3, 1, CV_64F);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R.at<double>(i, j) = R_mat(i, j);
      }
      t.at<double>(i, 0) = t_vec(i);
    }

    // Compute rectification transforms
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K_left, D_left, K_right, D_right, image_size,
                      R, t, R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, 0);

    RCLCPP_INFO(get_logger(), "Computed stereo rectification matrices");

    // Build CameraInfo messages
    left_info_msg_.header.frame_id = "oak_left_camera_optical_frame";
    left_info_msg_.height = image_size.height;
    left_info_msg_.width = image_size.width;
    left_info_msg_.distortion_model = "plumb_bob";
    left_info_msg_.d.assign(D_left.ptr<double>(), D_left.ptr<double>() + D_left.total());

    // K matrix
    std::copy(K_left.ptr<double>(), K_left.ptr<double>() + 9, left_info_msg_.k.begin());
    // R matrix (rectification rotation)
    std::copy(R1.ptr<double>(), R1.ptr<double>() + 9, left_info_msg_.r.begin());
    // P matrix (projection after rectification)
    std::copy(P1.ptr<double>(), P1.ptr<double>() + 12, left_info_msg_.p.begin());

    right_info_msg_.header.frame_id = "oak_right_camera_optical_frame";
    right_info_msg_.height = image_size.height;
    right_info_msg_.width = image_size.width;
    right_info_msg_.distortion_model = "plumb_bob";
    right_info_msg_.d.assign(D_right.ptr<double>(), D_right.ptr<double>() + D_right.total());

    std::copy(K_right.ptr<double>(), K_right.ptr<double>() + 9, right_info_msg_.k.begin());
    std::copy(R2.ptr<double>(), R2.ptr<double>() + 9, right_info_msg_.r.begin());
    std::copy(P2.ptr<double>(), P2.ptr<double>() + 12, right_info_msg_.p.begin());

    // Create publishers with SystemDefaultsQoS (RELIABLE+VOLATILE) for compatibility
    // with image_proc::RectifyNode (which subscribes with default VOLATILE durability)
    auto qos = rclcpp::SystemDefaultsQoS();
    left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(left_topic, qos);
    right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(right_topic, qos);

    RCLCPP_INFO(get_logger(), "Publishers created: %s, %s", left_topic.c_str(), right_topic.c_str());

    // Create publishing timer
    int period_ms = std::max(1, 1000 / hz);
    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&CalibrationPublisherNode::publishCameraInfo, this));

    RCLCPP_INFO(get_logger(), "Initialization complete - publishing CameraInfo at %d Hz", hz);

  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "Initialization failed: %s", e.what());
    rclcpp::shutdown();
  }
}

void CalibrationPublisherNode::publishCameraInfo() {
  auto now = get_clock()->now();
  left_info_msg_.header.stamp = now;
  right_info_msg_.header.stamp = now;
  left_info_pub_->publish(left_info_msg_);
  right_info_pub_->publish(right_info_msg_);
}

void CalibrationPublisherNode::cameraToKD(const GenericCamera<double>& cam,
                                           cv::Mat& K,
                                           cv::Mat& D,
                                           const cv::Size& image_size) {
  K = cv::Mat::eye(3, 3, CV_64F);
  D = cv::Mat::zeros(1, 8, CV_64F);

  // Use std::visit to extract parameters from the variant
  std::visit(
      [&](const auto& camera_model) {
        using CamType = std::decay_t<decltype(camera_model)>;

        if constexpr (std::is_same_v<CamType, PinholeRadtan8Camera<double>>) {
          // PinholeRadtan8Camera stores: [fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6]
          const auto& params = camera_model.getParam();
          K.at<double>(0, 0) = params[0];  // fx
          K.at<double>(1, 1) = params[1];  // fy
          K.at<double>(0, 2) = params[2];  // cx
          K.at<double>(1, 2) = params[3];  // cy

          // Distortion: k1, k2, p1, p2, k3, k4, k5, k6
          D.at<double>(0, 0) = params[4];  // k1
          D.at<double>(0, 1) = params[5];  // k2
          D.at<double>(0, 2) = params[6];  // p1
          D.at<double>(0, 3) = params[7];  // p2
          D.at<double>(0, 4) = params[8];  // k3
          D.at<double>(0, 5) = params[9];  // k4
          D.at<double>(0, 6) = params[10]; // k5
          D.at<double>(0, 7) = params[11]; // k6
        } else if constexpr (std::is_same_v<CamType, PinholeCamera<double>>) {
          // PinholeCamera stores: [fx, fy, cx, cy]
          const auto& params = camera_model.getParam();
          K.at<double>(0, 0) = params[0];  // fx
          K.at<double>(1, 1) = params[1];  // fy
          K.at<double>(0, 2) = params[2];  // cx
          K.at<double>(1, 2) = params[3];  // cy
          // D stays zero (no distortion)
        }
      },
      cam.variant);
}

}  // namespace basalt
