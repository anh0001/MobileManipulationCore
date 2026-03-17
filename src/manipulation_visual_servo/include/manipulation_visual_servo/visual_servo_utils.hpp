// Copyright 2026 MobileManipulationCore Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstddef>
#include <optional>
#include <string>

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace manipulation_visual_servo
{

struct DepthSample
{
  double depth_m{0.0};
  std::size_t valid_pixels{0};
  cv::Rect sampled_roi;
  int anchor_px{0};
  int anchor_py{0};
  double depth_iqr_m{0.0};
};

struct CenteringUpdate
{
  int streak{0};
  bool stable{false};
};

bool is_supported_depth_encoding(const std::string & encoding);

bool decode_depth_image(
  const sensor_msgs::msg::Image & msg,
  cv::Mat & depth_image,
  std::string * error_message = nullptr);

std::optional<DepthSample> sample_depth_at_roi_anchor(
  const cv::Mat & depth_image,
  const cv::Rect2d & tracked_roi,
  double anchor_x_norm,
  double anchor_y_norm,
  int depth_roi_half_size_px,
  std::size_t min_valid_depth_pixels,
  double max_iqr_m);

CenteringUpdate update_centering_streak(int current_streak, bool centered, int required_cycles);

bool depth_within_standoff(double depth_m, double grasp_standoff_m, double depth_tolerance_m);

bool depth_progress_stalled(
  double oldest_depth_m,
  double newest_depth_m,
  double min_progress_m);

double compute_depth_velocity_mps(
  double depth_m,
  double grasp_standoff_m,
  double lambda_z,
  double max_linear_velocity);

}  // namespace manipulation_visual_servo
