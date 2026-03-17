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

#include "manipulation_visual_servo/visual_servo_utils.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <sensor_msgs/image_encodings.hpp>

namespace manipulation_visual_servo
{

namespace
{

template<typename T>
T clamp_value(T value, T lower, T upper)
{
  return std::max(lower, std::min(upper, value));
}

double percentile_from_sorted(const std::vector<uint16_t> & sorted_values, double fraction)
{
  if (sorted_values.empty()) {
    return 0.0;
  }
  const double position = clamp_value(fraction, 0.0, 1.0) *
    static_cast<double>(sorted_values.size() - 1);
  const auto lower_index = static_cast<std::size_t>(std::floor(position));
  const auto upper_index = static_cast<std::size_t>(std::ceil(position));
  const double lower_value = static_cast<double>(sorted_values[lower_index]);
  const double upper_value = static_cast<double>(sorted_values[upper_index]);
  const double weight = position - static_cast<double>(lower_index);
  return lower_value + (upper_value - lower_value) * weight;
}

}  // namespace

bool is_supported_depth_encoding(const std::string & encoding)
{
  return encoding == sensor_msgs::image_encodings::TYPE_16UC1 || encoding == "16UC1";
}

bool decode_depth_image(
  const sensor_msgs::msg::Image & msg,
  cv::Mat & depth_image,
  std::string * error_message)
{
  if (!is_supported_depth_encoding(msg.encoding)) {
    if (error_message != nullptr) {
      *error_message = "unsupported depth encoding '" + msg.encoding + "'";
    }
    return false;
  }

  const std::size_t expected_step = static_cast<std::size_t>(msg.width) * sizeof(uint16_t);
  if (msg.step < expected_step) {
    if (error_message != nullptr) {
      *error_message = "depth step is smaller than width * 2 bytes";
    }
    return false;
  }

  const std::size_t expected_bytes = static_cast<std::size_t>(msg.step) * msg.height;
  if (msg.data.size() < expected_bytes) {
    if (error_message != nullptr) {
      *error_message = "depth payload shorter than expected from step * height";
    }
    return false;
  }

  const cv::Mat view(
    static_cast<int>(msg.height),
    static_cast<int>(msg.width),
    CV_16UC1,
    const_cast<unsigned char *>(msg.data.data()),
    static_cast<std::size_t>(msg.step));
  depth_image = view.clone();
  return true;
}

cv::Rect compute_body_roi(
  const cv::Rect2d & detection_roi,
  double body_top_frac,
  double body_bottom_frac,
  double body_left_frac,
  double body_right_frac,
  int image_width,
  int image_height)
{
  const double top = detection_roi.y + detection_roi.height * clamp_value(body_top_frac, 0.0, 1.0);
  const double bottom =
    detection_roi.y + detection_roi.height * clamp_value(body_bottom_frac, 0.0, 1.0);
  const double left =
    detection_roi.x + detection_roi.width * clamp_value(body_left_frac, 0.0, 1.0);
  const double right =
    detection_roi.x + detection_roi.width * clamp_value(body_right_frac, 0.0, 1.0);

  const int x0 = clamp_value(static_cast<int>(std::round(left)), 0, image_width - 1);
  const int y0 = clamp_value(static_cast<int>(std::round(top)), 0, image_height - 1);
  const int x1 = clamp_value(static_cast<int>(std::round(right)), x0 + 1, image_width);
  const int y1 = clamp_value(static_cast<int>(std::round(bottom)), y0 + 1, image_height);

  return cv::Rect(x0, y0, x1 - x0, y1 - y0);
}

std::optional<DepthSample> sample_depth_in_roi(
  const cv::Mat & depth_image,
  const cv::Rect & roi,
  std::size_t min_valid_depth_pixels,
  double max_iqr_m)
{
  if (depth_image.empty() || depth_image.type() != CV_16UC1) {
    return std::nullopt;
  }

  const cv::Rect safe_roi = roi & cv::Rect(0, 0, depth_image.cols, depth_image.rows);
  if (safe_roi.width <= 0 || safe_roi.height <= 0) {
    return std::nullopt;
  }

  std::vector<uint16_t> valid_depth_mm;
  valid_depth_mm.reserve(static_cast<std::size_t>(safe_roi.area()));
  for (int row = safe_roi.y; row < safe_roi.y + safe_roi.height; ++row) {
    const auto * row_ptr = depth_image.ptr<uint16_t>(row);
    for (int col = safe_roi.x; col < safe_roi.x + safe_roi.width; ++col) {
      const uint16_t depth_mm = row_ptr[col];
      if (depth_mm > 0U) {
        valid_depth_mm.push_back(depth_mm);
      }
    }
  }

  if (valid_depth_mm.size() < min_valid_depth_pixels) {
    return std::nullopt;
  }

  std::sort(valid_depth_mm.begin(), valid_depth_mm.end());
  const double depth_iqr_m =
    (percentile_from_sorted(valid_depth_mm, 0.75) -
    percentile_from_sorted(valid_depth_mm, 0.25)) / 1000.0;
  if (max_iqr_m > 0.0 && depth_iqr_m > max_iqr_m) {
    return std::nullopt;
  }

  DepthSample sample;
  sample.depth_m = percentile_from_sorted(valid_depth_mm, 0.5) / 1000.0;
  sample.valid_pixels = valid_depth_mm.size();
  sample.sampled_roi = safe_roi;
  sample.anchor_px = safe_roi.x + safe_roi.width / 2;
  sample.anchor_py = safe_roi.y + safe_roi.height / 2;
  sample.depth_iqr_m = depth_iqr_m;
  return sample;
}

std::optional<BottleEstimate3D> estimate_bottle_3d(
  const cv::Mat & depth_image,
  const cv::Rect2d & detection_roi,
  const CameraIntrinsics & intrinsics,
  double body_top_frac,
  double body_bottom_frac,
  double body_left_frac,
  double body_right_frac,
  std::size_t min_valid_depth_pixels,
  double max_iqr_m)
{
  if (intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0) {
    return std::nullopt;
  }

  const cv::Rect body_roi = compute_body_roi(
    detection_roi, body_top_frac, body_bottom_frac, body_left_frac, body_right_frac,
    depth_image.cols, depth_image.rows);

  if (body_roi.width <= 0 || body_roi.height <= 0) {
    return std::nullopt;
  }

  // Collect valid depth pixels and their 3D deprojections
  struct PixelDepth
  {
    int u;
    int v;
    uint16_t depth_mm;
  };
  std::vector<PixelDepth> valid_pixels;
  valid_pixels.reserve(static_cast<std::size_t>(body_roi.area()));

  for (int row = body_roi.y; row < body_roi.y + body_roi.height; ++row) {
    const auto * row_ptr = depth_image.ptr<uint16_t>(row);
    for (int col = body_roi.x; col < body_roi.x + body_roi.width; ++col) {
      const uint16_t depth_mm = row_ptr[col];
      if (depth_mm > 0U) {
        valid_pixels.push_back({col, row, depth_mm});
      }
    }
  }

  if (valid_pixels.size() < min_valid_depth_pixels) {
    return std::nullopt;
  }

  // Sort by depth for IQR check
  std::vector<uint16_t> depth_values;
  depth_values.reserve(valid_pixels.size());
  for (const auto & px : valid_pixels) {
    depth_values.push_back(px.depth_mm);
  }
  std::sort(depth_values.begin(), depth_values.end());

  const double depth_iqr_m =
    (percentile_from_sorted(depth_values, 0.75) -
    percentile_from_sorted(depth_values, 0.25)) / 1000.0;
  if (max_iqr_m > 0.0 && depth_iqr_m > max_iqr_m) {
    return std::nullopt;
  }

  // IQR-based outlier rejection: keep pixels within [Q1 - 1.5*IQR, Q3 + 1.5*IQR]
  const double q1_mm = percentile_from_sorted(depth_values, 0.25);
  const double q3_mm = percentile_from_sorted(depth_values, 0.75);
  const double iqr_mm = q3_mm - q1_mm;
  const double lower_fence_mm = q1_mm - 1.5 * iqr_mm;
  const double upper_fence_mm = q3_mm + 1.5 * iqr_mm;

  // Deproject inlier pixels to 3D
  std::vector<double> xs, ys, zs;
  xs.reserve(valid_pixels.size());
  ys.reserve(valid_pixels.size());
  zs.reserve(valid_pixels.size());

  for (const auto & px : valid_pixels) {
    const double d_mm = static_cast<double>(px.depth_mm);
    if (d_mm < lower_fence_mm || d_mm > upper_fence_mm) {
      continue;
    }
    const double z = d_mm / 1000.0;
    const double x = (static_cast<double>(px.u) - intrinsics.cx) * z / intrinsics.fx;
    const double y = (static_cast<double>(px.v) - intrinsics.cy) * z / intrinsics.fy;
    xs.push_back(x);
    ys.push_back(y);
    zs.push_back(z);
  }

  if (xs.empty()) {
    return std::nullopt;
  }

  // Compute median of each coordinate for robust centroid
  auto median = [](std::vector<double> & vals) {
      std::sort(vals.begin(), vals.end());
      const std::size_t n = vals.size();
      if (n % 2 == 0) {
        return (vals[n / 2 - 1] + vals[n / 2]) / 2.0;
      }
      return vals[n / 2];
    };

  BottleEstimate3D estimate;
  estimate.centroid_camera.x = median(xs);
  estimate.centroid_camera.y = median(ys);
  estimate.centroid_camera.z = median(zs);
  estimate.depth_m = estimate.centroid_camera.z;
  estimate.valid_pixels = xs.size();
  estimate.depth_iqr_m = depth_iqr_m;
  estimate.body_roi = body_roi;
  return estimate;
}

geometry_msgs::msg::Pose make_grasp_pose(
  const geometry_msgs::msg::Point & bottle_position_arm_base,
  double orient_x, double orient_y, double orient_z, double orient_w)
{
  geometry_msgs::msg::Pose pose;
  pose.position = bottle_position_arm_base;
  pose.orientation.x = orient_x;
  pose.orientation.y = orient_y;
  pose.orientation.z = orient_z;
  pose.orientation.w = orient_w;

  // Normalize quaternion
  const double norm = std::sqrt(
    orient_x * orient_x + orient_y * orient_y +
    orient_z * orient_z + orient_w * orient_w);
  if (norm > 1e-12) {
    pose.orientation.x /= norm;
    pose.orientation.y /= norm;
    pose.orientation.z /= norm;
    pose.orientation.w /= norm;
  } else {
    pose.orientation.w = 1.0;
  }

  return pose;
}

geometry_msgs::msg::Pose make_pregrasp_pose(
  const geometry_msgs::msg::Pose & grasp_pose,
  double pregrasp_offset_m)
{
  // Retract along the tool approach axis (local Z) by pregrasp_offset_m.
  // The approach axis in the arm base frame is derived from the grasp orientation.
  // For quaternion q, the local Z axis is:
  //   z_axis = q * [0,0,1] * q_inv
  const double qx = grasp_pose.orientation.x;
  const double qy = grasp_pose.orientation.y;
  const double qz = grasp_pose.orientation.z;
  const double qw = grasp_pose.orientation.w;

  // Rotate [0, 0, 1] by quaternion
  const double az_x = 2.0 * (qx * qz + qw * qy);
  const double az_y = 2.0 * (qy * qz - qw * qx);
  const double az_z = 1.0 - 2.0 * (qx * qx + qy * qy);

  geometry_msgs::msg::Pose pregrasp = grasp_pose;
  pregrasp.position.x -= pregrasp_offset_m * az_x;
  pregrasp.position.y -= pregrasp_offset_m * az_y;
  pregrasp.position.z -= pregrasp_offset_m * az_z;
  return pregrasp;
}

CenteringUpdate update_centering_streak(int current_streak, bool centered, int required_cycles)
{
  CenteringUpdate update;
  update.streak = centered ? current_streak + 1 : 0;
  update.stable = update.streak >= std::max(1, required_cycles);
  return update;
}

}  // namespace manipulation_visual_servo
