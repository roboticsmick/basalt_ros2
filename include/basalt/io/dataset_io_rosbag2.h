/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef DATASET_IO_ROSBAG2_H
#define DATASET_IO_ROSBAG2_H

#include <mutex>
#include <optional>

#include <basalt/io/dataset_io.h>
#include <basalt/io/rosbag2_storage_detector.h>
#include <basalt/utils/filesystem.h>

#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace basalt {

class Rosbag2VioDataset : public VioDataset {
  size_t num_cams;

  std::vector<int64_t> image_timestamps;

  // Store serialized messages for lazy image loading
  struct SerializedMessageData {
    std::shared_ptr<rclcpp::SerializedMessage> serialized_msg;
    std::string message_type;
  };

  // Vector of images for every timestamp
  // Assumes vector size is num_cams for every timestamp with null pointers for
  // missing frames
  std::unordered_map<int64_t,
                     std::vector<std::optional<SerializedMessageData>>>
      image_data_idx;

  Eigen::aligned_vector<AccelData> accel_data;
  Eigen::aligned_vector<GyroData> gyro_data;

  std::vector<int64_t> gt_timestamps;  // ordered gt timestamps
  Eigen::aligned_vector<Sophus::SE3d> gt_pose_data;

  int64_t mocap_to_imu_offset_ns;

  std::mutex m;  // Mutex for thread-safe lazy loading

 public:
  ~Rosbag2VioDataset() {}

  size_t get_num_cams() const { return num_cams; }

  std::vector<int64_t>& get_image_timestamps() { return image_timestamps; }

  const Eigen::aligned_vector<AccelData>& get_accel_data() const {
    return accel_data;
  }

  const Eigen::aligned_vector<GyroData>& get_gyro_data() const {
    return gyro_data;
  }

  const std::vector<int64_t>& get_gt_timestamps() const {
    return gt_timestamps;
  }

  const Eigen::aligned_vector<Sophus::SE3d>& get_gt_pose_data() const {
    return gt_pose_data;
  }

  int64_t get_mocap_to_imu_offset_ns() const { return mocap_to_imu_offset_ns; }

  std::vector<ImageData> get_image_data(int64_t t_ns);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  friend class Rosbag2IO;
};

class Rosbag2IO : public DatasetIoInterface {
 public:
  Rosbag2IO() {}

  void read(const std::string& path);

  void reset() { data.reset(); }

  VioDatasetPtr get_data() { return data; }

 private:
  std::shared_ptr<Rosbag2VioDataset> data;

  // Helper methods
  rosbag2_storage::StorageOptions createStorageOptions(
      const Rosbag2Info& bag_info);

  rosbag2_cpp::ConverterOptions createConverterOptions();

  void discoverTopics(rosbag2_cpp::readers::SequentialReader& reader,
                      std::set<std::string>& cam_topics,
                      std::string& imu_topic, std::string& mocap_topic,
                      std::string& point_topic);

  void indexMessages(rosbag2_cpp::readers::SequentialReader& reader,
                     const std::set<std::string>& cam_topics,
                     const std::string& imu_topic,
                     const std::string& mocap_topic,
                     const std::string& point_topic);

  // Template helper for deserializing ROS2 messages
  template <typename T>
  typename T::SharedPtr deserializeMessage(
      const rclcpp::SerializedMessage& serialized_msg);
};

}  // namespace basalt

#endif  // DATASET_IO_ROSBAG2_H
