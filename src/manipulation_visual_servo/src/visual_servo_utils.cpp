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

std::optional<DepthSample> sample_depth_at_roi_center(
  const cv::Mat & depth_image,
  const cv::Rect2d & tracked_roi,
  int depth_roi_half_size_px,
  std::size_t min_valid_depth_pixels)
{
  if (depth_image.empty() || depth_image.type() != CV_16UC1) {
    return std::nullopt;
  }

  const int cx = clamp_value(
    cvRound(tracked_roi.x + tracked_roi.width * 0.5), 0, depth_image.cols - 1);
  const int cy = clamp_value(
    cvRound(tracked_roi.y + tracked_roi.height * 0.5), 0, depth_image.rows - 1);
  const int half_size = std::max(0, depth_roi_half_size_px);

  const int x0 = clamp_value(cx - half_size, 0, depth_image.cols - 1);
  const int y0 = clamp_value(cy - half_size, 0, depth_image.rows - 1);
  const int x1 = clamp_value(cx + half_size + 1, x0 + 1, depth_image.cols);
  const int y1 = clamp_value(cy + half_size + 1, y0 + 1, depth_image.rows);
  const cv::Rect sample_roi(x0, y0, x1 - x0, y1 - y0);

  std::vector<uint16_t> valid_depth_mm;
  valid_depth_mm.reserve(static_cast<std::size_t>(sample_roi.area()));
  for (int row = sample_roi.y; row < sample_roi.y + sample_roi.height; ++row) {
    const auto * row_ptr = depth_image.ptr<uint16_t>(row);
    for (int col = sample_roi.x; col < sample_roi.x + sample_roi.width; ++col) {
      const uint16_t depth_mm = row_ptr[col];
      if (depth_mm > 0U) {
        valid_depth_mm.push_back(depth_mm);
      }
    }
  }

  if (valid_depth_mm.size() < min_valid_depth_pixels) {
    return std::nullopt;
  }

  const auto mid = valid_depth_mm.begin() + static_cast<std::ptrdiff_t>(valid_depth_mm.size() / 2);
  std::nth_element(valid_depth_mm.begin(), mid, valid_depth_mm.end());

  DepthSample sample;
  sample.depth_m = static_cast<double>(*mid) / 1000.0;
  sample.valid_pixels = valid_depth_mm.size();
  sample.sampled_roi = sample_roi;
  return sample;
}

CenteringUpdate update_centering_streak(int current_streak, bool centered, int required_cycles)
{
  CenteringUpdate update;
  update.streak = centered ? current_streak + 1 : 0;
  update.stable = update.streak >= std::max(1, required_cycles);
  return update;
}

bool depth_within_standoff(double depth_m, double grasp_standoff_m, double depth_tolerance_m)
{
  return std::abs(depth_m - grasp_standoff_m) <= std::max(0.0, depth_tolerance_m);
}

double compute_depth_velocity_mps(
  double depth_m,
  double grasp_standoff_m,
  double lambda_z,
  double max_linear_velocity)
{
  const double unclamped = lambda_z * (depth_m - grasp_standoff_m);
  const double limit = std::max(0.0, max_linear_velocity);
  if (limit <= 0.0) {
    return 0.0;
  }
  return clamp_value(unclamped, -limit, limit);
}

}  // namespace manipulation_visual_servo
