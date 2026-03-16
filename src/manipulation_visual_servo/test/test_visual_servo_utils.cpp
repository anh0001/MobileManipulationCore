// Copyright 2026 MobileManipulationCore Contributors

#include <algorithm>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
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

TEST(DepthSampleTest, UsesMedianDepthAndIgnoresZeros)
{
  const cv::Mat depth_image = (cv::Mat_<uint16_t>(5, 5) <<
    0U, 0U, 0U, 0U, 0U,
    0U, 900U, 1000U, 1100U, 0U,
    0U, 1200U, 1300U, 1400U, 0U,
    0U, 1500U, 1600U, 1700U, 0U,
    0U, 0U, 0U, 0U, 0U);

  const auto sample = sample_depth_at_roi_center(depth_image, cv::Rect2d(1.0, 1.0, 2.0, 2.0), 1, 5U);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->valid_pixels, 9U);
  EXPECT_NEAR(sample->depth_m, 1.3, 1e-6);
}

TEST(DepthSampleTest, ClipsAtImageBorderAndRejectsInsufficientPixels)
{
  const cv::Mat depth_image = (cv::Mat_<uint16_t>(3, 3) <<
    0U, 0U, 0U,
    0U, 800U, 0U,
    0U, 0U, 0U);

  const auto missing = sample_depth_at_roi_center(depth_image, cv::Rect2d(0.0, 0.0, 1.0, 1.0), 2, 2U);
  EXPECT_FALSE(missing.has_value());

  const auto valid = sample_depth_at_roi_center(depth_image, cv::Rect2d(0.0, 0.0, 1.0, 1.0), 2, 1U);
  ASSERT_TRUE(valid.has_value());
  EXPECT_EQ(valid->sampled_roi.x, 0);
  EXPECT_EQ(valid->sampled_roi.y, 0);
  EXPECT_NEAR(valid->depth_m, 0.8, 1e-6);
}

TEST(StateHelperTest, CenteringRequiresStableCycles)
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

TEST(StateHelperTest, DepthWindowControlsApproachCompletionAndVelocity)
{
  EXPECT_TRUE(depth_within_standoff(0.160, 0.160, 0.015));
  EXPECT_TRUE(depth_within_standoff(0.172, 0.160, 0.015));
  EXPECT_FALSE(depth_within_standoff(0.190, 0.160, 0.015));

  EXPECT_NEAR(compute_depth_velocity_mps(0.200, 0.160, 1.0, 0.08), 0.04, 1e-6);
  EXPECT_NEAR(compute_depth_velocity_mps(0.100, 0.160, 1.0, 0.08), -0.06, 1e-6);
  EXPECT_NEAR(compute_depth_velocity_mps(0.500, 0.160, 1.0, 0.08), 0.08, 1e-6);
}

}  // namespace
}  // namespace manipulation_visual_servo
