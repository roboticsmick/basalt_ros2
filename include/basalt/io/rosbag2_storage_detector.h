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

#ifndef ROSBAG2_STORAGE_DETECTOR_H
#define ROSBAG2_STORAGE_DETECTOR_H

#include <basalt/utils/filesystem.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace basalt {

enum class Rosbag2StorageFormat {
  MCAP,     // Modern MCAP format (default in ROS2)
  SQLITE3,  // SQLite3 format (legacy ROS2)
  UNKNOWN   // Unknown or unsupported format
};

struct Rosbag2Info {
  std::string bag_path;       // Path to bag (file or directory)
  std::string storage_id;     // "mcap" or "sqlite3"
  Rosbag2StorageFormat format;
  bool is_directory;   // true if metadata.yaml format
  bool is_single_file;  // true if standalone .mcap/.db3
  std::vector<std::string> storage_files;  // Actual data files

  Rosbag2Info()
      : format(Rosbag2StorageFormat::UNKNOWN),
        is_directory(false),
        is_single_file(false) {}
};

class Rosbag2Detector {
 public:
  /**
   * Detect ROS2 bag format from path
   *
   * Supports:
   * - Directory with metadata.yaml (standard ROS2 bag)
   * - Single .mcap file
   * - Single .db3 file
   *
   * @param path Path to bag file or directory
   * @return Rosbag2Info with detected format information
   */
  static Rosbag2Info detectBagFormat(const std::string& path) {
    Rosbag2Info info;
    info.bag_path = path;

    if (fs::is_directory(path)) {
      // Check for metadata.yaml (standard ROS2 bag directory)
      std::string metadata_path = (fs::path(path) / "metadata.yaml").string();
      if (fs::exists(metadata_path)) {
        info.is_directory = true;
        info.storage_id = parseStorageIdFromMetadata(metadata_path);

        // Find storage files in directory
        for (const auto& entry : fs::directory_iterator(path)) {
          std::string filename = entry.path().filename().string();

          // Check for MCAP files
          if (filename.find(".mcap") != std::string::npos) {
            info.storage_files.push_back(entry.path().string());
            if (info.format == Rosbag2StorageFormat::UNKNOWN) {
              info.format = Rosbag2StorageFormat::MCAP;
            }
          }
          // Check for SQLite3 files
          else if (filename.find(".db3") != std::string::npos) {
            info.storage_files.push_back(entry.path().string());
            if (info.format == Rosbag2StorageFormat::UNKNOWN) {
              info.format = Rosbag2StorageFormat::SQLITE3;
            }
          }
        }

        // If storage_id wasn't found in metadata, infer from files
        if (info.storage_id.empty()) {
          info.storage_id = (info.format == Rosbag2StorageFormat::MCAP)
                                ? "mcap"
                                : "sqlite3";
        }
      }
    } else if (fs::is_regular_file(path)) {
      // Single file format
      info.is_single_file = true;
      std::string ext = fs::path(path).extension().string();

      if (ext == ".mcap") {
        info.format = Rosbag2StorageFormat::MCAP;
        info.storage_id = "mcap";
        info.storage_files.push_back(path);
      } else if (ext == ".db3") {
        info.format = Rosbag2StorageFormat::SQLITE3;
        info.storage_id = "sqlite3";
        info.storage_files.push_back(path);
      }
    }

    return info;
  }

  /**
   * Get human-readable format name
   */
  static std::string getFormatName(Rosbag2StorageFormat format) {
    switch (format) {
      case Rosbag2StorageFormat::MCAP:
        return "MCAP";
      case Rosbag2StorageFormat::SQLITE3:
        return "SQLite3";
      default:
        return "Unknown";
    }
  }

 private:
  /**
   * Parse storage_identifier from metadata.yaml
   *
   * Simple YAML parsing to extract:
   * rosbag2_bagfile_information:
   *   storage_identifier: mcap
   */
  static std::string parseStorageIdFromMetadata(
      const std::string& metadata_path) {
    std::ifstream file(metadata_path);
    if (!file.is_open()) {
      std::cerr << "Failed to open metadata file: " << metadata_path
                << std::endl;
      return "";
    }

    std::string line;
    while (std::getline(file, line)) {
      // Look for storage_identifier line
      if (line.find("storage_identifier:") != std::string::npos) {
        size_t pos = line.find(":");
        if (pos != std::string::npos) {
          std::string value = line.substr(pos + 1);
          // Trim whitespace and quotes
          value.erase(0, value.find_first_not_of(" \t\"'"));
          value.erase(value.find_last_not_of(" \t\r\n\"'") + 1);
          return value;
        }
      }
    }

    return "";
  }
};

}  // namespace basalt

#endif  // ROSBAG2_STORAGE_DETECTOR_H
