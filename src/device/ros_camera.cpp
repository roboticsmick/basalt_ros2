/**
 * @file ros_camera.cpp
 * @brief ROS2 subscriber to a stereo camera as a basalt-style device.
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

#include <basalt/device/ros_camera.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>

namespace basalt
{

  RosCameraDevice::RosCameraDevice(rclcpp::Node::SharedPtr node,
                                   const std::string &left_topic,
                                   const std::string &right_topic)
      : node_(node), left_topic_(left_topic), right_topic_(right_topic)
  {
    // Store topics for later subscription in start()
    // Do NOT create subscriptions here - the shared_ptr wrapper isn't ready yet
    RCLCPP_INFO(node_->get_logger(),
                "RosCameraDevice created (subscriptions will be set up in start()) with left=%s, right=%s",
                left_topic.c_str(), right_topic.c_str());
  }

  RosCameraDevice::~RosCameraDevice()
  {
    stop();
  }

  void RosCameraDevice::start()
  {
    if (is_running_)
    {
      RCLCPP_WARN(node_->get_logger(), "RosCameraDevice already running");
      return;
    }

    // Create subscriptions here when the shared_ptr is fully initialized
    it_ = std::make_unique<image_transport::ImageTransport>(node_);

    // Create message filter subscribers
    left_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        node_, left_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
    right_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        node_, right_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());

    // Create time synchronizer with queue size of 10
    sync_ = std::make_unique<message_filters::TimeSynchronizer<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(
        *left_sub_, *right_sub_, 10);

    // Register callback using std::bind
    sync_->registerCallback(std::bind(&RosCameraDevice::imageCallback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));

    // Create camera info subscription
    camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        "oak/left/camera_info",
        rclcpp::SensorDataQoS(),
        std::bind(&RosCameraDevice::cameraInfoCallback, this, std::placeholders::_1));

    is_running_ = true;
    RCLCPP_INFO(node_->get_logger(), "RosCameraDevice started - subscriptions active");
  }

  void RosCameraDevice::stop()
  {
    is_running_ = false;
    RCLCPP_INFO(node_->get_logger(), "RosCameraDevice stopped");
  }

  void RosCameraDevice::flushBuffers()
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    image_buffer_.clear();
  }

  std::string RosCameraDevice::getFrameId() const
  {
    return frame_id_;
  }

  basalt::Calibration<double> RosCameraDevice::exportCalibration()
  {
    // TODO: Extract calibration from camera_info messages
    // For now, return a default calibration
    RCLCPP_WARN(node_->get_logger(), "exportCalibration not implemented, using default");
    return basalt::Calibration<double>();
  }

  void RosCameraDevice::imageCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
  {
    // Convert ROS2 timestamp to nanoseconds
    uint64_t t_ns = rclcpp::Time(left_msg->header.stamp).nanoseconds();

    try
    {
      // Create ManagedImage objects with correct dimensions from input
      auto images = std::make_unique<std::vector<ManagedImage<uint16_t>>>();
      images->push_back(ManagedImage<uint16_t>(left_msg->width, left_msg->height));
      images->push_back(ManagedImage<uint16_t>(right_msg->width, right_msg->height));

      convertImageMsg(left_msg, (*images)[0]);
      convertImageMsg(right_msg, (*images)[1]);

      // Push to buffer
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      image_buffer_.push_back({t_ns, std::move(images)});

      // Keep buffer bounded (Q3 - reduced from 10 to 5 frames, ~0.8s vs ~1.7s latency)
      if (image_buffer_.size() > 5)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "Camera buffer full (%zu frames) — dropping oldest frame", image_buffer_.size());
        image_buffer_.pop_front();
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Error converting image: %s", e.what());
    }
  }

  bool RosCameraDevice::poll(uint64_t &t_ns,
                             std::vector<ManagedImage<uint16_t>> &img)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    if (image_buffer_.empty())
    {
      return false;
    }

    t_ns = image_buffer_.front().first;
    img = std::move(*image_buffer_.front().second);
    image_buffer_.pop_front();

    return true;
  }

  void RosCameraDevice::cameraInfoCallback(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg)
  {
    // Store camera info (focal length, principal point, distortion)
    // This will be used by exportCalibration() for the default calibration
    if (frame_id_.empty())
    {
      frame_id_ = msg->header.frame_id;
    }
    // TODO: Extract and store K, D, P matrices for calibration export
  }

  void RosCameraDevice::convertImageMsg(
      const sensor_msgs::msg::Image::ConstSharedPtr &msg,
      ManagedImage<uint16_t> &out)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    auto t_conv_start = std::chrono::steady_clock::now(); // O4 - timing instrumentation

    try
    {
      // Already correct format (reuse pre-allocated scratch mats - O3)
      if (msg->encoding == "mono16" || msg->encoding == "16UC1")
      {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        convert_scratch_gray16_ = cv_ptr->image;

        // Cast 8-bit to 16-bit
      }
      else if (msg->encoding == "mono8" || msg->encoding == "8UC1")
      {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC1);
        cv_ptr->image.convertTo(convert_scratch_gray16_, CV_16UC1, 256.0);

        // Convert BGR8 to grayscale, then cast to 16-bit
      }
      else if (msg->encoding == "bgr8")
      {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        cv::cvtColor(cv_ptr->image, convert_scratch_gray8_, cv::COLOR_BGR2GRAY);
        convert_scratch_gray8_.convertTo(convert_scratch_gray16_, CV_16UC1, 256.0);

        // Convert RGB8 to grayscale, then cast to 16-bit
      }
      else if (msg->encoding == "rgb8")
      {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        cv::cvtColor(cv_ptr->image, convert_scratch_gray8_, cv::COLOR_RGB2GRAY);
        convert_scratch_gray8_.convertTo(convert_scratch_gray16_, CV_16UC1, 256.0);

        // Convert RGBA8 to grayscale, then cast to 16-bit
      }
      else if (msg->encoding == "rgba8")
      {
        cv_ptr = cv_bridge::toCvShare(msg, "rgba8");
        cv::cvtColor(cv_ptr->image, convert_scratch_gray8_, cv::COLOR_RGBA2GRAY);
        convert_scratch_gray8_.convertTo(convert_scratch_gray16_, CV_16UC1, 256.0);

        // Convert BGRA8 to grayscale, then cast to 16-bit
      }
      else if (msg->encoding == "bgra8")
      {
        cv_ptr = cv_bridge::toCvShare(msg, "bgra8");
        cv::cvtColor(cv_ptr->image, convert_scratch_gray8_, cv::COLOR_BGRA2GRAY);
        convert_scratch_gray8_.convertTo(convert_scratch_gray16_, CV_16UC1, 256.0);

        // Unsupported encoding
      }
      else
      {
        throw std::runtime_error("Unsupported image encoding: " + msg->encoding);
      }

      // Validate output dimensions match input
      if (out.w != static_cast<size_t>(convert_scratch_gray16_.cols) ||
          out.h != static_cast<size_t>(convert_scratch_gray16_.rows))
      {
        throw std::runtime_error("Image dimensions mismatch");
      }

      // Copy image data to output
      std::memcpy(out.ptr, convert_scratch_gray16_.data, convert_scratch_gray16_.total() * convert_scratch_gray16_.elemSize());

      // Log conversion time (O4 - timing instrumentation)
      auto t_conv_us = std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::steady_clock::now() - t_conv_start)
                           .count();
      RCLCPP_DEBUG(node_->get_logger(), "Image conversion: %ld us (%s)", t_conv_us, msg->encoding.c_str());
    }
    catch (const cv_bridge::Exception &e)
    {
      throw std::runtime_error(std::string("cv_bridge exception: ") + e.what());
    }
  }

} // namespace basalt
