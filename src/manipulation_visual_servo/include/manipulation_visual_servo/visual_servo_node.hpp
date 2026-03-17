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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <manipulation_msgs/msg/policy_output.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "manipulation_visual_servo/visual_servo_utils.hpp"

namespace manipulation_visual_servo
{

enum class ServoState
{
  IDLE,
  ACQUIRE,
  ESTIMATE_BOTTLE_3D,
  OPEN_GRIPPER,
  PLAN_PREGRASP,
  EXEC_PREGRASP,
  FINAL_SERVO,
  CLOSE_GRIPPER,
  VERIFY_GRASP,
  LIFT_RETREAT,
  DONE,
  LOST,
};

std::string state_to_string(ServoState state);

class VisualServoNode : public rclcpp::Node
{
public:
  explicit VisualServoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Sensor callbacks
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
  void detection_callback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr & msg);

  // Control loop
  void control_timer_callback();
  void transition_to(ServoState new_state);

  // State handlers
  void handle_idle();
  void handle_acquire();
  void handle_estimate_bottle_3d();
  void handle_open_gripper();
  void handle_plan_pregrasp();
  void handle_exec_pregrasp();
  void handle_final_servo();
  void handle_close_gripper();
  void handle_verify_grasp();
  void handle_lift_retreat();
  void handle_done();
  void handle_lost();

  // Helpers
  bool fetch_latest_detection(cv::Rect2d & roi, float & confidence);
  bool get_current_ee_pose(geometry_msgs::msg::PoseStamped & pose);
  double get_gripper_width();

  // Transform bottle centroid from camera frame to arm base frame
  bool transform_point_to_arm_base(
    const geometry_msgs::msg::Point & point_camera,
    geometry_msgs::msg::Point & point_arm_base);

  // Compute image-space error from detection center to image center
  double compute_image_error(const cv::Rect2d & roi);

  // Compute 3D Cartesian residual between current EE and target
  double compute_position_residual(
    const geometry_msgs::msg::PoseStamped & current_ee,
    const geometry_msgs::msg::Pose & target);

  // Publishing
  void publish_move_group_target(const geometry_msgs::msg::Pose & target_pose);
  void publish_servo_twist(
    const geometry_msgs::msg::Point & position_error,
    double image_err_x, double image_err_y);
  void publish_gripper_command(double command);
  void publish_zero_motion();
  void publish_lift_command(double dz);
  void publish_debug_overlay(const cv::Mat & frame, const cv::Rect2d & roi);
  void publish_state();

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Publishers
  rclcpp::Publisher<manipulation_msgs::msg::PolicyOutput>::SharedPtr policy_output_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State machine
  ServoState state_{ServoState::IDLE};
  rclcpp::Time state_entry_time_;
  bool pregrasp_sent_{false};
  bool pregrasp_goal_active_{false};
  double pregrasp_best_residual_{0.0};
  rclcpp::Time pregrasp_best_residual_time_;
  int pregrasp_retry_count_{0};
  int convergence_streak_{0};
  int ramp_step_{0};

  // Camera
  CameraIntrinsics intrinsics_;
  bool camera_info_received_{false};

  // Latest sensor data (protected by mutexes)
  std::mutex image_mutex_;
  cv::Mat latest_frame_;
  rclcpp::Time latest_frame_stamp_;
  bool frame_available_{false};

  std::mutex depth_mutex_;
  cv::Mat latest_depth_frame_;
  rclcpp::Time latest_depth_stamp_;
  bool depth_available_{false};
  bool depth_encoding_warned_{false};

  std::mutex detection_mutex_;
  cv::Rect2d latest_detection_roi_;
  float latest_detection_confidence_{0.0F};
  bool detection_available_{false};
  rclcpp::Time latest_detection_stamp_;

  std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState::ConstSharedPtr latest_joint_state_;

  // Computed targets for current pick attempt
  std::optional<BottleEstimate3D> bottle_estimate_;
  geometry_msgs::msg::Pose grasp_pose_;
  geometry_msgs::msg::Pose pregrasp_pose_;
  cv::Rect2d cached_estimate_roi_;  // Cached detection ROI for depth retries

  // Lift/retreat tracking
  geometry_msgs::msg::PoseStamped ee_pose_at_lift_start_;
  bool lift_phase_done_{false};

  // Parameters — topics
  std::string rgb_topic_;
  std::string camera_info_topic_;
  std::string depth_topic_;
  std::string detection_topic_;
  std::string output_topic_;
  std::string joint_states_topic_;

  // Parameters — frames
  std::string reference_frame_;
  std::string camera_optical_frame_;
  std::string ee_frame_;
  std::string arm_base_frame_;

  // Parameters — general
  double control_rate_hz_;
  std::string target_class_;
  double min_detection_confidence_;

  // Parameters — timeouts
  double lost_target_timeout_sec_;
  double acquire_timeout_sec_;
  double estimate_timeout_sec_;
  double pregrasp_timeout_sec_;
  double final_servo_timeout_sec_;
  double verify_timeout_sec_;
  double lift_timeout_sec_;

  // Parameters — depth sampling
  double depth_roi_body_top_frac_;
  double depth_roi_body_bottom_frac_;
  double depth_roi_body_left_frac_;
  double depth_roi_body_right_frac_;
  int min_valid_depth_pixels_;
  double depth_sample_max_iqr_m_;
  double depth_stale_timeout_sec_;

  // Parameters — hybrid pick
  double pregrasp_offset_m_;
  double eef_link_to_grasp_offset_m_;
  double final_servo_distance_m_;
  double grasp_settle_sec_;
  double lift_distance_m_;
  double retreat_distance_m_;

  // Parameters — grasp orientation template
  double bottle_grasp_orient_x_;
  double bottle_grasp_orient_y_;
  double bottle_grasp_orient_z_;
  double bottle_grasp_orient_w_;

  // Parameters — convergence
  double final_position_tolerance_m_;
  double final_image_tolerance_px_;
  int final_convergence_cycles_;

  // Parameters — gripper
  double open_gripper_command_;
  double close_gripper_command_;
  double grasp_success_min_width_;
  double gripper_closed_position_;
  std::string gripper_joint_name_;

  // Parameters — servo control
  double servo_lambda_xy_;
  double servo_lambda_z_;
  double servo_max_linear_velocity_;
  double servo_max_angular_velocity_;
  int servo_ramp_up_steps_;

  // Parameters — debug
  bool publish_overlay_;
  std::string overlay_topic_;
  bool publish_state_flag_;
  std::string state_topic_;
};

}  // namespace manipulation_visual_servo
