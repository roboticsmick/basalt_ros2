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

#include <basalt/io/dataset_io_rosbag2.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <set>

namespace basalt {

void Rosbag2IO::read(const std::string& path) {
  if (!fs::exists(path)) {
    std::cerr << "No dataset found in " << path << std::endl;
    return;
  }

  // Detect bag format (MCAP, SQLite3, directory, single file)
  Rosbag2Info bag_info = Rosbag2Detector::detectBagFormat(path);

  if (bag_info.format == Rosbag2StorageFormat::UNKNOWN) {
    std::cerr << "Unknown ROS2 bag format at " << path << std::endl;
    std::cerr << "Supported formats: MCAP (.mcap), SQLite3 (.db3), or "
                 "directory with metadata.yaml"
              << std::endl;
    return;
  }

  std::cout << "Detected ROS2 bag format: "
            << Rosbag2Detector::getFormatName(bag_info.format) << " ("
            << bag_info.storage_id << ")" << std::endl;
  std::cout << "Storage files: " << bag_info.storage_files.size() << std::endl;

  // Initialize dataset
  data.reset(new Rosbag2VioDataset);

  // Configure storage options
  rosbag2_storage::StorageOptions storage_options =
      createStorageOptions(bag_info);
  rosbag2_cpp::ConverterOptions converter_options = createConverterOptions();

  // Open bag with sequential reader
  rosbag2_cpp::readers::SequentialReader reader;
  try {
    reader.open(storage_options, converter_options);
  } catch (const std::exception& e) {
    std::cerr << "Failed to open ROS2 bag: " << e.what() << std::endl;
    return;
  }

  // Discover topics
  std::set<std::string> cam_topics;
  std::string imu_topic, mocap_topic, point_topic;
  discoverTopics(reader, cam_topics, imu_topic, mocap_topic, point_topic);

  std::cout << "imu_topic: " << imu_topic << std::endl;
  std::cout << "mocap_topic: " << mocap_topic << std::endl;
  std::cout << "cam_topics: ";
  for (const std::string& s : cam_topics) std::cout << s << " ";
  std::cout << std::endl;

  data->num_cams = cam_topics.size();

  // Index all messages (single pass through bag)
  indexMessages(reader, cam_topics, imu_topic, mocap_topic, point_topic);

  std::cout << "Total images indexed: " << data->image_data_idx.size()
            << std::endl;
  std::cout << "Total IMU samples: " << data->accel_data.size() << std::endl;
  std::cout << "Total mocap poses: " << data->gt_timestamps.size()
            << std::endl;
}

rosbag2_storage::StorageOptions Rosbag2IO::createStorageOptions(
    const Rosbag2Info& bag_info) {
  rosbag2_storage::StorageOptions storage_options;

  // Set URI (path to bag)
  storage_options.uri = bag_info.bag_path;

  // Set storage plugin ID
  storage_options.storage_id = bag_info.storage_id;

  // Performance tuning
  storage_options.max_bagfile_size = 0;  // No split limit for reading
  storage_options.max_cache_size = 100 * 1024 * 1024;  // 100 MB cache

  return storage_options;
}

rosbag2_cpp::ConverterOptions Rosbag2IO::createConverterOptions() {
  rosbag2_cpp::ConverterOptions converter_options;

  // Use CDR (default ROS2 serialization)
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  return converter_options;
}

void Rosbag2IO::discoverTopics(rosbag2_cpp::readers::SequentialReader& reader,
                               std::set<std::string>& cam_topics,
                               std::string& imu_topic,
                               std::string& mocap_topic,
                               std::string& point_topic) {
  auto topics_and_types = reader.get_all_topics_and_types();

  for (const auto& topic_metadata : topics_and_types) {
    const std::string& topic = topic_metadata.name;
    const std::string& type = topic_metadata.type;

    // Camera topics
    if (type == "sensor_msgs/msg/Image") {
      cam_topics.insert(topic);
    }
    // IMU topic (exclude FCU topics)
    else if (type == "sensor_msgs/msg/Imu") {
      if (topic.rfind("/fcu", 0) != 0) {
        if (imu_topic.empty()) {
          imu_topic = topic;
        }
      }
    }
    // Mocap/Ground truth topics
    else if (type == "geometry_msgs/msg/TransformStamped" ||
             type == "geometry_msgs/msg/PoseStamped") {
      if (mocap_topic.empty()) {
        mocap_topic = topic;
      }
    }
    // Point tracking
    else if (type == "geometry_msgs/msg/PointStamped") {
      if (point_topic.empty()) {
        point_topic = topic;
      }
    }
  }
}

void Rosbag2IO::indexMessages(rosbag2_cpp::readers::SequentialReader& reader,
                              const std::set<std::string>& cam_topics,
                              const std::string& imu_topic,
                              const std::string& mocap_topic,
                              const std::string& point_topic) {
  // Create topic to camera ID mapping
  std::map<std::string, int> topic_to_id;
  int idx = 0;
  for (const std::string& topic : cam_topics) {
    topic_to_id[topic] = idx++;
  }

  int64_t min_time = std::numeric_limits<int64_t>::max();
  int64_t max_time = std::numeric_limits<int64_t>::min();

  std::vector<geometry_msgs::msg::TransformStamped::SharedPtr> mocap_msgs;
  std::vector<geometry_msgs::msg::PointStamped::SharedPtr> point_msgs;

  std::vector<int64_t> system_to_imu_offset_vec;
  std::vector<int64_t> system_to_mocap_offset_vec;

  std::set<int64_t> image_timestamps_set;

  int num_msgs = 0;

  // Iterate through all messages
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    const std::string& topic = bag_message->topic_name;

    // Process camera images (lazy load - store serialized message)
    if (cam_topics.find(topic) != cam_topics.end()) {
      // Deserialize header to get timestamp
      auto img_msg =
          deserializeMessage<sensor_msgs::msg::Image>(*bag_message->serialized_data);

      if (img_msg) {
        int64_t timestamp_ns =
            rclcpp::Time(img_msg->header.stamp).nanoseconds();

        // Store serialized message for lazy loading
        auto& img_vec = data->image_data_idx[timestamp_ns];
        if (img_vec.size() == 0) {
          img_vec.resize(data->num_cams);
        }

        img_vec[topic_to_id.at(topic)] =
            Rosbag2VioDataset::SerializedMessageData{
                std::make_shared<rclcpp::SerializedMessage>(
                    *bag_message->serialized_data),
                "sensor_msgs/msg/Image"};

        image_timestamps_set.insert(timestamp_ns);
        min_time = std::min(min_time, timestamp_ns);
        max_time = std::max(max_time, timestamp_ns);
      }
    }

    // Process IMU data (extract immediately)
    else if (topic == imu_topic) {
      auto imu_msg = deserializeMessage<sensor_msgs::msg::Imu>(
          *bag_message->serialized_data);

      if (imu_msg) {
        int64_t time = rclcpp::Time(imu_msg->header.stamp).nanoseconds();

        data->accel_data.emplace_back();
        data->accel_data.back().timestamp_ns = time;
        data->accel_data.back().data =
            Eigen::Vector3d(imu_msg->linear_acceleration.x,
                            imu_msg->linear_acceleration.y,
                            imu_msg->linear_acceleration.z);

        data->gyro_data.emplace_back();
        data->gyro_data.back().timestamp_ns = time;
        data->gyro_data.back().data = Eigen::Vector3d(
            imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
            imu_msg->angular_velocity.z);

        min_time = std::min(min_time, time);
        max_time = std::max(max_time, time);

        // Track time offset
        int64_t msg_arrival_time = bag_message->time_stamp;
        system_to_imu_offset_vec.push_back(time - msg_arrival_time);
      }
    }

    // Process mocap data
    else if (topic == mocap_topic) {
      // Try TransformStamped first
      auto transform_msg =
          deserializeMessage<geometry_msgs::msg::TransformStamped>(
              *bag_message->serialized_data);

      if (transform_msg) {
        mocap_msgs.push_back(transform_msg);
        int64_t time =
            rclcpp::Time(transform_msg->header.stamp).nanoseconds();
        int64_t msg_arrival_time = bag_message->time_stamp;
        system_to_mocap_offset_vec.push_back(time - msg_arrival_time);
      } else {
        // Try PoseStamped
        auto pose_msg = deserializeMessage<geometry_msgs::msg::PoseStamped>(
            *bag_message->serialized_data);

        if (pose_msg) {
          // Convert PoseStamped to TransformStamped
          auto transform_msg =
              std::make_shared<geometry_msgs::msg::TransformStamped>();
          transform_msg->header = pose_msg->header;
          transform_msg->transform.rotation = pose_msg->pose.orientation;
          transform_msg->transform.translation.x = pose_msg->pose.position.x;
          transform_msg->transform.translation.y = pose_msg->pose.position.y;
          transform_msg->transform.translation.z = pose_msg->pose.position.z;

          mocap_msgs.push_back(transform_msg);
          int64_t time = rclcpp::Time(pose_msg->header.stamp).nanoseconds();
          int64_t msg_arrival_time = bag_message->time_stamp;
          system_to_mocap_offset_vec.push_back(time - msg_arrival_time);
        }
      }
    }

    // Process point data
    else if (topic == point_topic) {
      auto point_msg = deserializeMessage<geometry_msgs::msg::PointStamped>(
          *bag_message->serialized_data);

      if (point_msg) {
        point_msgs.push_back(point_msg);
        int64_t time = rclcpp::Time(point_msg->header.stamp).nanoseconds();
        int64_t msg_arrival_time = bag_message->time_stamp;
        system_to_mocap_offset_vec.push_back(time - msg_arrival_time);
      }
    }

    num_msgs++;
  }

  // Convert image timestamps set to vector
  data->image_timestamps.assign(image_timestamps_set.begin(),
                                image_timestamps_set.end());

  // Calculate mocap-to-IMU offset (median)
  if (!system_to_mocap_offset_vec.empty() &&
      !system_to_imu_offset_vec.empty()) {
    int64_t system_to_imu_offset =
        system_to_imu_offset_vec[system_to_imu_offset_vec.size() / 2];
    int64_t system_to_mocap_offset =
        system_to_mocap_offset_vec[system_to_mocap_offset_vec.size() / 2];
    data->mocap_to_imu_offset_ns =
        system_to_imu_offset - system_to_mocap_offset;
  }

  // Process mocap poses
  data->gt_pose_data.clear();
  data->gt_timestamps.clear();

  if (!mocap_msgs.empty()) {
    for (size_t i = 0; i < mocap_msgs.size() - 1; i++) {
      auto mocap_msg = mocap_msgs[i];

      int64_t time = rclcpp::Time(mocap_msg->header.stamp).nanoseconds();

      Eigen::Quaterniond q(mocap_msg->transform.rotation.w,
                          mocap_msg->transform.rotation.x,
                          mocap_msg->transform.rotation.y,
                          mocap_msg->transform.rotation.z);

      Eigen::Vector3d t(mocap_msg->transform.translation.x,
                       mocap_msg->transform.translation.y,
                       mocap_msg->transform.translation.z);

      int64_t timestamp_ns = time + data->mocap_to_imu_offset_ns;
      data->gt_timestamps.emplace_back(timestamp_ns);
      data->gt_pose_data.emplace_back(q, t);
    }
  }

  // Process point data
  if (!point_msgs.empty()) {
    for (size_t i = 0; i < point_msgs.size() - 1; i++) {
      auto point_msg = point_msgs[i];

      int64_t time = rclcpp::Time(point_msg->header.stamp).nanoseconds();

      Eigen::Vector3d t(point_msg->point.x, point_msg->point.y,
                       point_msg->point.z);

      int64_t timestamp_ns = time;  // No offset for point data
      data->gt_timestamps.emplace_back(timestamp_ns);
      data->gt_pose_data.emplace_back(Sophus::SO3d(), t);
    }
  }

  std::cout << "Total messages processed: " << num_msgs << std::endl;
  std::cout << "Time range: " << min_time << " to " << max_time << std::endl;
  std::cout << "Mocap-to-IMU offset: " << data->mocap_to_imu_offset_ns << " ns"
            << std::endl;
}

template <typename T>
typename T::SharedPtr Rosbag2IO::deserializeMessage(
    const rclcpp::SerializedMessage& serialized_msg) {
  rclcpp::Serialization<T> serializer;
  auto msg = std::make_shared<T>();

  try {
    serializer.deserialize_message(&serialized_msg, msg.get());
  } catch (const std::exception& e) {
    std::cerr << "Failed to deserialize message: " << e.what() << std::endl;
    return nullptr;
  }

  return msg;
}

// Lazy image loading implementation
std::vector<ImageData> Rosbag2VioDataset::get_image_data(int64_t t_ns) {
  std::vector<ImageData> res(num_cams);

  auto it = image_data_idx.find(t_ns);
  if (it == image_data_idx.end()) {
    return res;
  }

  for (size_t i = 0; i < num_cams; i++) {
    if (!it->second[i].has_value()) continue;

    ImageData& id = res[i];

    // Deserialize image message
    std::lock_guard<std::mutex> lock(m);

    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
    serializer.deserialize_message(it->second[i]->serialized_msg.get(),
                                   img_msg.get());

    // Allocate image buffer
    id.img.reset(new ManagedImage<uint16_t>(img_msg->width, img_msg->height));

    // Extract exposure from frame_id if available
    if (!img_msg->header.frame_id.empty() &&
        std::isdigit(img_msg->header.frame_id[0])) {
      id.exposure = std::stol(img_msg->header.frame_id) * 1e-9;
    } else {
      id.exposure = -1;
    }

    // Convert image data based on encoding
    if (img_msg->encoding == "mono8") {
      const uint8_t* data_in = img_msg->data.data();
      uint16_t* data_out = id.img->ptr;

      for (size_t j = 0; j < img_msg->data.size(); j++) {
        int val = data_in[j];
        val = val << 8;  // Scale to 16-bit
        data_out[j] = val;
      }

    } else if (img_msg->encoding == "mono16") {
      std::memcpy(id.img->ptr, img_msg->data.data(), img_msg->data.size());
    } else {
      std::cerr << "Encoding " << img_msg->encoding << " is not supported."
                << std::endl;
      std::abort();
    }
  }

  return res;
}

}  // namespace basalt
