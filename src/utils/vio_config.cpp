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

#include <basalt/utils/assert.h>
#include <basalt/utils/vio_config.h>

#include <fstream>

#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <magic_enum.hpp>
#include <yaml-cpp/yaml.h>

namespace basalt {

VioConfig::VioConfig() {
  optical_flow_type = "frame_to_frame";
  optical_flow_detection_grid_size = 50;
  optical_flow_max_recovered_dist2 = 0.09f;
  optical_flow_pattern = 51;
  optical_flow_max_iterations = 5;
  optical_flow_levels = 3;
  optical_flow_epipolar_error = 0.005;
  optical_flow_skip_frames = 1;

  vio_linearization_type = LinearizationType::ABS_QR;
  vio_sqrt_marg = true;

  vio_max_states = 3;
  vio_max_kfs = 7;
  vio_min_frames_after_kf = 5;
  vio_new_kf_keypoints_thresh = 0.7;

  vio_debug = false;
  vio_extended_logging = false;
  vio_obs_std_dev = 0.5;
  vio_obs_huber_thresh = 1.0;
  vio_min_triangulation_dist = 0.05;
  vio_max_iterations = 7;

  vio_enforce_realtime = false;

  vio_lm_lambda_initial = 1e-4;
  vio_lm_lambda_min = 1e-6;
  vio_lm_lambda_max = 1e2;

  vio_init_pose_weight = 1e8;
  vio_init_ba_weight = 1e1;
  vio_init_bg_weight = 1e2;

  vio_marg_lost_landmarks = true;

  vio_kf_marg_feature_ratio = 0.1;
}

void VioConfig::save(const std::string& filename) {
  std::ofstream os(filename);

  {
    cereal::JSONOutputArchive archive(os);
    archive(*this);
  }
  os.close();
}

void VioConfig::load(const std::string& filename) {
  // Detect extension: .yaml/.yml → YAML loader, otherwise → JSON loader
  auto ends_with = [](const std::string& s, const std::string& sfx) {
    return s.size() >= sfx.size() &&
           s.compare(s.size() - sfx.size(), sfx.size(), sfx) == 0;
  };
  if (ends_with(filename, ".yaml") || ends_with(filename, ".yml")) {
    loadFromYAML(filename);
    return;
  }
  std::ifstream is(filename);

  {
    cereal::JSONInputArchive archive(is);
    archive(*this);
  }
  is.close();
}
void VioConfig::loadFromYAML(const std::string& filename) {
  YAML::Node root = YAML::LoadFile(filename);

  // Optical flow parameters
  optical_flow_type = root["optical_flow_type"].as<std::string>(optical_flow_type);
  optical_flow_detection_grid_size =
      root["optical_flow_detection_grid_size"].as<int>(optical_flow_detection_grid_size);
  optical_flow_max_recovered_dist2 =
      root["optical_flow_max_recovered_dist2"].as<float>(optical_flow_max_recovered_dist2);
  optical_flow_pattern = root["optical_flow_pattern"].as<int>(optical_flow_pattern);
  optical_flow_max_iterations =
      root["optical_flow_max_iterations"].as<int>(optical_flow_max_iterations);
  optical_flow_levels = root["optical_flow_levels"].as<int>(optical_flow_levels);
  optical_flow_epipolar_error =
      root["optical_flow_epipolar_error"].as<float>(optical_flow_epipolar_error);
  optical_flow_skip_frames =
      root["optical_flow_skip_frames"].as<int>(optical_flow_skip_frames);

  // VIO solver type (string → enum via magic_enum)
  if (root["vio_linearization_type"]) {
    std::string name = root["vio_linearization_type"].as<std::string>();
    auto lin_enum = magic_enum::enum_cast<LinearizationType>(name);
    if (lin_enum.has_value()) {
      vio_linearization_type = lin_enum.value();
    } else {
      std::cerr << "Unknown vio_linearization_type: " << name
                << " — using default" << std::endl;
    }
  }

  vio_sqrt_marg = root["vio_sqrt_marg"].as<bool>(vio_sqrt_marg);
  vio_max_states = root["vio_max_states"].as<int>(vio_max_states);
  vio_max_kfs = root["vio_max_kfs"].as<int>(vio_max_kfs);
  vio_min_frames_after_kf =
      root["vio_min_frames_after_kf"].as<int>(vio_min_frames_after_kf);
  vio_new_kf_keypoints_thresh =
      root["vio_new_kf_keypoints_thresh"].as<float>(vio_new_kf_keypoints_thresh);
  vio_debug = root["vio_debug"].as<bool>(vio_debug);
  vio_extended_logging = root["vio_extended_logging"].as<bool>(vio_extended_logging);
  vio_max_iterations = root["vio_max_iterations"].as<int>(vio_max_iterations);
  vio_obs_std_dev = root["vio_obs_std_dev"].as<double>(vio_obs_std_dev);
  vio_obs_huber_thresh = root["vio_obs_huber_thresh"].as<double>(vio_obs_huber_thresh);
  vio_min_triangulation_dist =
      root["vio_min_triangulation_dist"].as<double>(vio_min_triangulation_dist);
  vio_enforce_realtime = root["vio_enforce_realtime"].as<bool>(vio_enforce_realtime);
  vio_lm_lambda_initial =
      root["vio_lm_lambda_initial"].as<double>(vio_lm_lambda_initial);
  vio_lm_lambda_min = root["vio_lm_lambda_min"].as<double>(vio_lm_lambda_min);
  vio_lm_lambda_max = root["vio_lm_lambda_max"].as<double>(vio_lm_lambda_max);
  vio_init_pose_weight =
      root["vio_init_pose_weight"].as<double>(vio_init_pose_weight);
  vio_init_ba_weight = root["vio_init_ba_weight"].as<double>(vio_init_ba_weight);
  vio_init_bg_weight = root["vio_init_bg_weight"].as<double>(vio_init_bg_weight);
  vio_marg_lost_landmarks =
      root["vio_marg_lost_landmarks"].as<bool>(vio_marg_lost_landmarks);
  vio_kf_marg_feature_ratio =
      root["vio_kf_marg_feature_ratio"].as<double>(vio_kf_marg_feature_ratio);
}

}  // namespace basalt

namespace cereal {

template <class Archive>
std::string save_minimal(const Archive& ar,
                         const basalt::LinearizationType& linearization_type) {
  UNUSED(ar);
  auto name = magic_enum::enum_name(linearization_type);
  return std::string(name);
}

template <class Archive>
void load_minimal(const Archive& ar,
                  basalt::LinearizationType& linearization_type,
                  const std::string& name) {
  UNUSED(ar);

  auto lin_enum = magic_enum::enum_cast<basalt::LinearizationType>(name);

  if (lin_enum.has_value()) {
    linearization_type = lin_enum.value();
  } else {
    std::cerr << "Could not find the LinearizationType for " << name
              << std::endl;
    std::abort();
  }
}

template <class Archive>
void serialize(Archive& ar, basalt::VioConfig& config) {
  ar(CEREAL_NVP(config.optical_flow_type));
  ar(CEREAL_NVP(config.optical_flow_detection_grid_size));
  ar(CEREAL_NVP(config.optical_flow_max_recovered_dist2));
  ar(CEREAL_NVP(config.optical_flow_pattern));
  ar(CEREAL_NVP(config.optical_flow_max_iterations));
  ar(CEREAL_NVP(config.optical_flow_epipolar_error));
  ar(CEREAL_NVP(config.optical_flow_levels));
  ar(CEREAL_NVP(config.optical_flow_skip_frames));

  ar(CEREAL_NVP(config.vio_linearization_type));
  ar(CEREAL_NVP(config.vio_sqrt_marg));
  ar(CEREAL_NVP(config.vio_max_states));
  ar(CEREAL_NVP(config.vio_max_kfs));
  ar(CEREAL_NVP(config.vio_min_frames_after_kf));
  ar(CEREAL_NVP(config.vio_new_kf_keypoints_thresh));
  ar(CEREAL_NVP(config.vio_debug));
  ar(CEREAL_NVP(config.vio_extended_logging));
  ar(CEREAL_NVP(config.vio_max_iterations));
  //  ar(CEREAL_NVP(config.vio_outlier_threshold));
  //  ar(CEREAL_NVP(config.vio_filter_iteration));

  ar(CEREAL_NVP(config.vio_obs_std_dev));
  ar(CEREAL_NVP(config.vio_obs_huber_thresh));
  ar(CEREAL_NVP(config.vio_min_triangulation_dist));

  ar(CEREAL_NVP(config.vio_enforce_realtime));

  //ar(CEREAL_NVP(config.vio_use_lm));
  ar(CEREAL_NVP(config.vio_lm_lambda_initial));
  ar(CEREAL_NVP(config.vio_lm_lambda_min));
  ar(CEREAL_NVP(config.vio_lm_lambda_max));

  //ar(CEREAL_NVP(config.vio_scale_jacobian));

  ar(CEREAL_NVP(config.vio_init_pose_weight));
  ar(CEREAL_NVP(config.vio_init_ba_weight));
  ar(CEREAL_NVP(config.vio_init_bg_weight));

  ar(CEREAL_NVP(config.vio_marg_lost_landmarks));
  ar(CEREAL_NVP(config.vio_kf_marg_feature_ratio));
}
}  // namespace cereal
