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

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <manipulation_msgs/msg/policy_output.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace manipulation_visual_servo
{

enum class ServoState
{
  IDLE,
  ACQUIRE,
  TRACK,
  SERVO,
  LOST,
};

std::string state_to_string(ServoState state);

class VisualServoNode : public rclcpp::Node
{
public:
  explicit VisualServoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callbacks
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
  void detection_callback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg);
  void control_timer_callback();

  // State machine transitions
  void transition_to(ServoState new_state);
  void handle_idle();
  void handle_acquire();
  void handle_track();
  void handle_servo();
  void handle_lost();

  // Tracker helpers
  bool init_tracker(const cv::Mat & frame, const cv::Rect2d & roi);
  bool update_tracker(const cv::Mat & frame, cv::Rect2d & tracked_roi);

  // Control law
  geometry_msgs::msg::Twist compute_ibvs_twist(
    double feat_x, double feat_y,
    double feat_area,
    double desired_x, double desired_y,
    double desired_area);

  // Output
  void publish_policy_output(const geometry_msgs::msg::Twist & twist, float confidence);
  void publish_debug_overlay(const cv::Mat & frame, const cv::Rect2d & roi);
  void publish_state();

  // Twist -> EEF delta pose conversion
  geometry_msgs::msg::Pose twist_to_eef_delta(
    const geometry_msgs::msg::Twist & twist, double dt);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;

  // Publishers
  rclcpp::Publisher<manipulation_msgs::msg::PolicyOutput>::SharedPtr policy_output_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State
  ServoState state_{ServoState::IDLE};
  rclcpp::Time state_entry_time_;
  int ramp_step_{0};

  // Camera intrinsics
  bool camera_info_received_{false};
  int image_width_{0};
  int image_height_{0};

  // Latest image
  std::mutex image_mutex_;
  cv::Mat latest_frame_;
  rclcpp::Time latest_frame_stamp_;
  bool frame_available_{false};

  // Detection state
  std::mutex detection_mutex_;
  cv::Rect2d latest_detection_roi_;
  std::string latest_detection_class_;
  float latest_detection_confidence_{0.0f};
  bool detection_available_{false};
  rclcpp::Time latest_detection_stamp_;

  // Tracker state
  cv::Ptr<cv::Tracker> cv_tracker_;
  cv::Rect2d tracked_roi_;
  float tracking_confidence_{0.0f};
  bool tracker_initialized_{false};
  rclcpp::Time last_track_time_;

  // Desired feature values (image center, initial area)
  double desired_x_{0.0};
  double desired_y_{0.0};
  double desired_area_{0.0};

  // Parameters
  std::string rgb_topic_;
  std::string camera_info_topic_;
  std::string depth_topic_;
  std::string detection_topic_;
  std::string output_topic_;
  std::string reference_frame_;
  std::string camera_optical_frame_;
  std::string ee_frame_;
  std::string arm_base_frame_;
  std::string target_class_;

  bool use_depth_;
  double control_rate_hz_;
  double output_delta_horizon_sec_;
  double min_detection_confidence_;
  double min_tracking_confidence_;
  double lost_target_timeout_sec_;
  double acquire_timeout_sec_;
  double image_center_tolerance_px_;

  // Tracker params
  std::string tracker_type_;
  int klt_max_features_;
  double klt_quality_level_;
  double klt_min_distance_;
  int klt_window_size_;
  int klt_pyramid_levels_;

  // Control gains
  double lambda_xy_;
  double lambda_z_;
  double lambda_rz_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  int ramp_up_steps_;

  // Debug
  bool publish_overlay_;
  std::string overlay_topic_;
  bool publish_state_flag_;
  std::string state_topic_;
};

}  // namespace manipulation_visual_servo
