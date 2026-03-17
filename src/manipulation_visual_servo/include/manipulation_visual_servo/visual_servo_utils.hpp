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
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
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

struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  int width{0};
  int height{0};
};

struct BottleEstimate3D
{
  geometry_msgs::msg::Point centroid_camera;  // In camera optical frame
  double depth_m{0.0};
  std::size_t valid_pixels{0};
  double depth_iqr_m{0.0};
  cv::Rect body_roi;  // The body sub-region used for sampling
};

struct CenteringUpdate
{
  int streak{0};
  bool stable{false};
};

// Depth image decoding
bool is_supported_depth_encoding(const std::string & encoding);

bool decode_depth_image(
  const sensor_msgs::msg::Image & msg,
  cv::Mat & depth_image,
  std::string * error_message = nullptr);

// Extract the body sub-region of a detection ROI for depth sampling.
// The fractional parameters crop the detection box to the bottle body,
// avoiding the cap/neck (top) and base (bottom).
cv::Rect compute_body_roi(
  const cv::Rect2d & detection_roi,
  double body_top_frac,
  double body_bottom_frac,
  double body_left_frac,
  double body_right_frac,
  int image_width,
  int image_height);

// Sample depth from a specified ROI with median/IQR filtering.
// Returns nullopt if insufficient valid pixels or IQR exceeds threshold.
std::optional<DepthSample> sample_depth_in_roi(
  const cv::Mat & depth_image,
  const cv::Rect & roi,
  std::size_t min_valid_depth_pixels,
  double max_iqr_m);

// Estimate 3D bottle centroid from detection ROI + depth image.
// Crops the body region, samples depth, deprojects valid pixels using
// camera intrinsics, and returns the median 3D point in camera optical frame.
std::optional<BottleEstimate3D> estimate_bottle_3d(
  const cv::Mat & depth_image,
  const cv::Rect2d & detection_roi,
  const CameraIntrinsics & intrinsics,
  double body_top_frac,
  double body_bottom_frac,
  double body_left_frac,
  double body_right_frac,
  std::size_t min_valid_depth_pixels,
  double max_iqr_m);

// Synthesize grasp and pre-grasp poses from a bottle 3D position (in arm base frame)
// and a fixed grasp orientation template.
geometry_msgs::msg::Pose make_grasp_pose(
  const geometry_msgs::msg::Point & bottle_position_arm_base,
  double orient_x, double orient_y, double orient_z, double orient_w);

geometry_msgs::msg::Pose offset_pose_along_tool_z(
  const geometry_msgs::msg::Pose & pose,
  double offset_m);

geometry_msgs::msg::Pose offset_pose_along_axis(
  const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Vector3 & axis_arm_base,
  double offset_m);

geometry_msgs::msg::Pose make_pregrasp_pose(
  const geometry_msgs::msg::Pose & grasp_pose,
  double pregrasp_offset_m);

// Centering streak helper (retained from old pipeline)
CenteringUpdate update_centering_streak(int current_streak, bool centered, int required_cycles);

}  // namespace manipulation_visual_servo
