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

struct CartesianVector
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
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

double project_translation_onto_axis(
  const CartesianVector & translation,
  const CartesianVector & axis);

// Result of estimating a grasp on a (translucent) object standing on a table,
// computed in the depth camera's optical frame. Robust for objects whose own
// surface depth is unreliable (glass/plastic): the table plane is fit from
// valid depth in an annulus AROUND the object, then the object's footprint is
// found by intersecting the ray through the bbox bottom-center with that plane.
struct TableGraspEstimate
{
  CartesianVector footprint_cam;   // table point under the object (camera frame, meters)
  CartesianVector up_cam;          // unit table normal, oriented toward the camera
  std::size_t plane_points{0};     // number of inlier points used for the plane fit
  double plane_rms_m{0.0};         // RMS residual of the plane fit (meters)
  double footprint_depth_m{0.0};   // forward (camera +Z) distance to footprint
  bool valid{false};
};

// Fit the table plane from valid depth pixels in an annulus around `bbox`
// (between the bbox edge and `annulus_margin_frac` of the bbox size outward,
// excluding the bbox interior so the object's own bad depth is ignored), then
// ray-cast the bbox bottom-center pixel onto that plane to get the object
// footprint. `depth_image_mm` must be CV_16UC1 (millimeters, 0 = invalid).
TableGraspEstimate estimate_table_grasp(
  const cv::Mat & depth_image_mm,
  const cv::Rect2d & bbox,
  double fx, double fy, double cx, double cy,
  double annulus_margin_frac,
  double min_depth_m, double max_depth_m,
  std::size_t min_plane_points,
  double max_plane_rms_m);

}  // namespace manipulation_visual_servo
