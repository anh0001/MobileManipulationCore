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

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <sensor_msgs/image_encodings.hpp>

#include "manipulation_visual_servo/visual_servo_utils.hpp"

namespace manipulation_visual_servo
{
namespace
{

sensor_msgs::msg::Image make_depth_image(
  const std::vector<uint16_t> & values,
  uint32_t width,
  uint32_t height,
  const std::string & encoding = sensor_msgs::image_encodings::TYPE_16UC1)
{
  sensor_msgs::msg::Image msg;
  msg.width = width;
  msg.height = height;
  msg.encoding = encoding;
  msg.step = width * sizeof(uint16_t);
  msg.data.resize(values.size() * sizeof(uint16_t));
  std::copy(
    reinterpret_cast<const uint8_t *>(values.data()),
    reinterpret_cast<const uint8_t *>(values.data()) + msg.data.size(),
    msg.data.begin());
  return msg;
}

// --- Depth decode tests ---

TEST(DepthDecodeTest, Accepts16UC1Image)
{
  const auto msg = make_depth_image({1000U, 1200U, 1400U, 1600U}, 2U, 2U);

  cv::Mat depth_image;
  std::string error_message;
  ASSERT_TRUE(decode_depth_image(msg, depth_image, &error_message)) << error_message;
  ASSERT_EQ(depth_image.type(), CV_16UC1);
  EXPECT_EQ(depth_image.at<uint16_t>(0, 0), 1000U);
  EXPECT_EQ(depth_image.at<uint16_t>(1, 1), 1600U);
}

TEST(DepthDecodeTest, RejectsUnsupportedEncoding)
{
  const auto msg = make_depth_image({1000U, 1100U, 1200U, 1300U}, 2U, 2U, "mono8");

  cv::Mat depth_image;
  std::string error_message;
  EXPECT_FALSE(decode_depth_image(msg, depth_image, &error_message));
  EXPECT_NE(error_message.find("unsupported depth encoding"), std::string::npos);
}

// --- Body ROI computation ---

TEST(BodyRoiTest, CropsDetectionToBodyRegion)
{
  // Detection ROI: (100, 50, 200x300) in a 640x480 image
  const cv::Rect2d det(100.0, 50.0, 200.0, 300.0);
  const auto roi = compute_body_roi(det, 0.30, 0.90, 0.20, 0.80, 640, 480);

  // Expected body region:
  // top = 50 + 300*0.30 = 140
  // bottom = 50 + 300*0.90 = 320
  // left = 100 + 200*0.20 = 140
  // right = 100 + 200*0.80 = 260
  EXPECT_EQ(roi.x, 140);
  EXPECT_EQ(roi.y, 140);
  EXPECT_EQ(roi.width, 120);   // 260 - 140
  EXPECT_EQ(roi.height, 180);  // 320 - 140
}

TEST(BodyRoiTest, ClipsToImageBounds)
{
  // Detection near image edge
  const cv::Rect2d det(600.0, 400.0, 100.0, 200.0);
  const auto roi = compute_body_roi(det, 0.0, 1.0, 0.0, 1.0, 640, 480);

  EXPECT_GE(roi.x, 0);
  EXPECT_GE(roi.y, 0);
  EXPECT_LE(roi.x + roi.width, 640);
  EXPECT_LE(roi.y + roi.height, 480);
}

// --- Depth sampling in ROI ---

TEST(DepthSampleRoiTest, SamplesMedianDepthAndIgnoresZeros)
{
  const cv::Mat depth_image = (cv::Mat_<uint16_t>(5, 5) <<
    0U, 0U, 0U, 0U, 0U,
    0U, 900U, 1000U, 1100U, 0U,
    0U, 1200U, 1300U, 1400U, 0U,
    0U, 1500U, 1600U, 1700U, 0U,
    0U, 0U, 0U, 0U, 0U);

  const cv::Rect roi(1, 1, 3, 3);
  const auto sample = sample_depth_in_roi(depth_image, roi, 5U, 0.5);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->valid_pixels, 9U);
  EXPECT_NEAR(sample->depth_m, 1.3, 1e-6);
}

TEST(DepthSampleRoiTest, RejectsInsufficientPixels)
{
  const cv::Mat depth_image = (cv::Mat_<uint16_t>(3, 3) <<
    0U, 0U, 0U,
    0U, 800U, 0U,
    0U, 0U, 0U);

  const cv::Rect roi(0, 0, 3, 3);
  const auto missing = sample_depth_in_roi(depth_image, roi, 2U, 0.5);
  EXPECT_FALSE(missing.has_value());

  const auto valid = sample_depth_in_roi(depth_image, roi, 1U, 0.5);
  ASSERT_TRUE(valid.has_value());
  EXPECT_NEAR(valid->depth_m, 0.8, 1e-6);
}

TEST(DepthSampleRoiTest, RejectsHighIqr)
{
  const cv::Mat depth_image = (cv::Mat_<uint16_t>(3, 3) <<
    1000U, 1000U, 2000U,
    1000U, 1000U, 2000U,
    1000U, 2000U, 2000U);

  const cv::Rect roi(0, 0, 3, 3);
  const auto rejected = sample_depth_in_roi(depth_image, roi, 5U, 0.20);
  EXPECT_FALSE(rejected.has_value());
}

// --- 3D centroid estimation ---

TEST(Estimate3DTest, ComputesCentroidFromValidDepth)
{
  // 5x5 depth image with a cluster of valid pixels in the center
  const cv::Mat depth_image = (cv::Mat_<uint16_t>(5, 5) <<
    0U, 0U, 0U, 0U, 0U,
    0U, 500U, 500U, 500U, 0U,
    0U, 500U, 500U, 500U, 0U,
    0U, 500U, 500U, 500U, 0U,
    0U, 0U, 0U, 0U, 0U);

  CameraIntrinsics intr;
  intr.fx = 1.0;
  intr.fy = 1.0;
  intr.cx = 2.0;  // image center
  intr.cy = 2.0;
  intr.width = 5;
  intr.height = 5;

  // Use full detection box, body fracs that keep center region
  const cv::Rect2d det(0.0, 0.0, 5.0, 5.0);
  const auto est = estimate_bottle_3d(
    depth_image, det, intr,
    0.2, 0.8, 0.2, 0.8,  // body fracs
    3U, 0.5);

  ASSERT_TRUE(est.has_value());
  // All valid pixels at depth 500mm = 0.5m, centered at (2,2)
  EXPECT_NEAR(est->depth_m, 0.5, 1e-6);
  EXPECT_NEAR(est->centroid_camera.z, 0.5, 1e-6);
  // At pixel (2,2) with cx=2,cy=2, fx=fy=1: x = (2-2)*0.5/1 = 0
  EXPECT_NEAR(est->centroid_camera.x, 0.0, 0.01);
  EXPECT_NEAR(est->centroid_camera.y, 0.0, 0.01);
}

TEST(Estimate3DTest, RejectsEmptyOrNoIntrinsics)
{
  const cv::Mat depth_image = cv::Mat::zeros(5, 5, CV_16UC1);
  CameraIntrinsics bad_intr{};

  const auto est = estimate_bottle_3d(
    depth_image, cv::Rect2d(0, 0, 5, 5), bad_intr,
    0.0, 1.0, 0.0, 1.0, 1U, 1.0);
  EXPECT_FALSE(est.has_value());
}

TEST(Estimate3DTest, RejectsNoisyDepthByIqr)
{
  // Wide spread of depths -> high IQR
  const cv::Mat depth_image = (cv::Mat_<uint16_t>(3, 3) <<
    100U, 500U, 900U,
    200U, 600U, 1000U,
    300U, 700U, 1100U);

  CameraIntrinsics intr;
  intr.fx = 1.0;
  intr.fy = 1.0;
  intr.cx = 1.0;
  intr.cy = 1.0;
  intr.width = 3;
  intr.height = 3;

  // Tight IQR threshold should reject
  const auto est = estimate_bottle_3d(
    depth_image, cv::Rect2d(0, 0, 3, 3), intr,
    0.0, 1.0, 0.0, 1.0, 3U, 0.05);
  EXPECT_FALSE(est.has_value());
}

// --- Grasp pose synthesis ---

TEST(GraspPoseTest, SetsPositionAndNormalizesOrientation)
{
  geometry_msgs::msg::Point pos;
  pos.x = 0.3;
  pos.y = 0.1;
  pos.z = 0.2;

  const auto pose = make_grasp_pose(pos, 0.0, 0.0, 0.0, 1.0);
  EXPECT_NEAR(pose.position.x, 0.3, 1e-6);
  EXPECT_NEAR(pose.position.y, 0.1, 1e-6);
  EXPECT_NEAR(pose.position.z, 0.2, 1e-6);
  // Should be unit quaternion
  const double norm = std::sqrt(
    pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w);
  EXPECT_NEAR(norm, 1.0, 1e-6);
}

TEST(GraspPoseTest, NormalizesNonUnitQuaternion)
{
  geometry_msgs::msg::Point pos;
  const auto pose = make_grasp_pose(pos, 2.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(pose.orientation.x, 1.0, 1e-6);
  EXPECT_NEAR(pose.orientation.y, 0.0, 1e-6);
  EXPECT_NEAR(pose.orientation.z, 0.0, 1e-6);
  EXPECT_NEAR(pose.orientation.w, 0.0, 1e-6);
}

TEST(OffsetPoseTest, RetractsTargetAlongToolAxis)
{
  geometry_msgs::msg::Point pos;
  pos.x = 0.3;
  pos.y = 0.0;
  pos.z = 0.2;

  const auto grasp = make_grasp_pose(pos, 1.0, 0.0, 0.0, 0.0);
  const auto offset = offset_pose_along_tool_z(grasp, 0.10);

  EXPECT_NEAR(offset.position.x, 0.3, 1e-6);
  EXPECT_NEAR(offset.position.y, 0.0, 1e-6);
  EXPECT_NEAR(offset.position.z, 0.30, 1e-6);
}

// --- Pre-grasp pose synthesis ---

TEST(PregraspPoseTest, RetractsAlongToolZAxis)
{
  geometry_msgs::msg::Point pos;
  pos.x = 0.3;
  pos.y = 0.0;
  pos.z = 0.2;

  // Grasp orientation: identity quaternion (tool Z = world Z)
  const auto grasp = make_grasp_pose(pos, 0.0, 0.0, 0.0, 1.0);
  const auto pregrasp = make_pregrasp_pose(grasp, 0.08);

  // Should retract 0.08 m along Z
  EXPECT_NEAR(pregrasp.position.x, 0.3, 1e-6);
  EXPECT_NEAR(pregrasp.position.y, 0.0, 1e-6);
  EXPECT_NEAR(pregrasp.position.z, 0.12, 1e-6);  // 0.2 - 0.08
}

TEST(PregraspPoseTest, RetractsAlongRotatedToolAxis)
{
  geometry_msgs::msg::Point pos;
  pos.x = 0.3;
  pos.y = 0.0;
  pos.z = 0.2;

  // Grasp orientation: 180 deg rotation about X (tool Z = -world Z, pointing down)
  const auto grasp = make_grasp_pose(pos, 1.0, 0.0, 0.0, 0.0);
  const auto pregrasp = make_pregrasp_pose(grasp, 0.08);

  // Tool Z axis for quat (1,0,0,0) is [0, 0, -1]
  // Retract along -[0,0,-1] = [0,0,+1]
  EXPECT_NEAR(pregrasp.position.x, 0.3, 1e-6);
  EXPECT_NEAR(pregrasp.position.y, 0.0, 1e-6);
  EXPECT_NEAR(pregrasp.position.z, 0.28, 1e-6);  // 0.2 + 0.08
}

TEST(OffsetPoseTest, RetractsAlongExplicitAxis)
{
  geometry_msgs::msg::Point pos;
  pos.x = 0.3;
  pos.y = 0.0;
  pos.z = 0.2;

  const auto grasp = make_grasp_pose(pos, 0.0, 0.70710678, 0.0, 0.70710678);
  geometry_msgs::msg::Vector3 axis;
  axis.x = 0.0;
  axis.y = 0.0;
  axis.z = -1.0;
  const auto offset = offset_pose_along_axis(grasp, axis, 0.10);

  EXPECT_NEAR(offset.position.x, 0.3, 1e-6);
  EXPECT_NEAR(offset.position.y, 0.0, 1e-6);
  EXPECT_NEAR(offset.position.z, 0.30, 1e-6);
}

// --- Centering streak ---

TEST(CenteringStreakTest, RequiresConsecutiveCycles)
{
  auto update = update_centering_streak(0, true, 3);
  EXPECT_EQ(update.streak, 1);
  EXPECT_FALSE(update.stable);

  update = update_centering_streak(update.streak, true, 3);
  EXPECT_EQ(update.streak, 2);
  EXPECT_FALSE(update.stable);

  update = update_centering_streak(update.streak, true, 3);
  EXPECT_EQ(update.streak, 3);
  EXPECT_TRUE(update.stable);

  update = update_centering_streak(update.streak, false, 3);
  EXPECT_EQ(update.streak, 0);
  EXPECT_FALSE(update.stable);
}

}  // namespace
}  // namespace manipulation_visual_servo
