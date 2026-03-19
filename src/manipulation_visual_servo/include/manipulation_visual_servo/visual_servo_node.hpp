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

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <manipulation_msgs/msg/policy_output.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
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
  TRACK,
  ALIGN_XY,
  OPEN_GRIPPER,
  APPROACH_DEPTH,
  CLOSE_GRIPPER,
  LIFT,
  DONE,
  LOST,
};

std::string state_to_string(ServoState state);

class VisualServoNode : public rclcpp::Node
{
public:
  explicit VisualServoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
  void joint_states_callback(const sensor_msgs::msg::JointState::ConstSharedPtr & msg);
  void detection_callback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg);
  void control_timer_callback();

  void transition_to(ServoState new_state, const std::string & reason = "");
  void handle_idle();
  void handle_acquire();
  void handle_track();
  void handle_align_xy();
  void handle_open_gripper();
  void handle_approach_depth();
  void handle_close_gripper();
  void handle_lift();
  void handle_done();
  void handle_lost();

  bool init_tracker(const cv::Mat & frame, const cv::Rect2d & roi);
  bool update_tracker(const cv::Mat & frame, cv::Rect2d & tracked_roi);
  bool fetch_latest_frame(cv::Mat & frame);
  bool acquire_from_detection(const cv::Mat & frame);
  bool update_tracking(const cv::Mat & frame);
  bool start_standoff_blind_push(double depth_m);
  void handle_standoff_blind_push(const cv::Mat * frame = nullptr);
  std::optional<size_t> find_joint_state_index(
    const sensor_msgs::msg::JointState & joint_state, const std::string & joint_name) const;
  std::optional<double> max_gripper_open_error();
  void reset_pick_progress();
  void reset_standoff_blind_push();
  void apply_ramp(geometry_msgs::msg::Twist & twist);
  void publish_zero_motion(float confidence = 1.0F);
  std::optional<CartesianVector> lookup_current_ee_position_in_reference();
  std::optional<CartesianVector> lookup_eef_positive_z_axis_in_reference();

  geometry_msgs::msg::Twist compute_alignment_twist(
    double feat_x, double feat_y, bool allow_ramp = true);
  geometry_msgs::msg::Twist compute_approach_twist(
    double feat_x, double feat_y, double depth_m, bool allow_ramp = true);
  geometry_msgs::msg::Pose twist_to_eef_delta(
    const geometry_msgs::msg::Twist & twist, double dt);
  geometry_msgs::msg::Twist transform_twist_to_reference(
    const geometry_msgs::msg::Twist & twist);

  void publish_policy_output(
    const geometry_msgs::msg::Twist & twist,
    float confidence,
    bool include_arm_target = true,
    bool gripper_active = false,
    double gripper_command = 0.0);
  void publish_reference_frame_delta(
    double dx, double dy, double dz, float confidence,
    bool gripper_active = false, double gripper_command = 0.0);
  void publish_debug_overlay(const cv::Mat & frame, const cv::Rect2d & roi);
  void publish_state();

  struct DepthHistoryEntry
  {
    rclcpp::Time stamp;
    double depth_m{0.0};
  };

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;

  rclcpp::Publisher<manipulation_msgs::msg::PolicyOutput>::SharedPtr policy_output_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  ServoState state_{ServoState::IDLE};
  rclcpp::Time state_entry_time_;
  int ramp_step_{0};

  bool camera_info_received_{false};
  int image_width_{0};
  int image_height_{0};

  std::mutex image_mutex_;
  cv::Mat latest_frame_;
  rclcpp::Time latest_frame_stamp_;
  rclcpp::Time last_rgb_receive_time_;
  bool frame_available_{false};
  rclcpp::Time last_processed_frame_stamp_;
  std::uint64_t latest_frame_generation_{0};
  std::uint64_t last_processed_frame_generation_{0};

  std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool joint_state_available_{false};

  std::mutex depth_mutex_;
  cv::Mat latest_depth_frame_;
  rclcpp::Time latest_depth_stamp_;
  bool depth_available_{false};
  bool depth_encoding_warned_{false};

  std::mutex detection_mutex_;
  cv::Rect2d latest_detection_roi_;
  std::string latest_detection_class_;
  float latest_detection_confidence_{0.0F};
  bool detection_available_{false};
  rclcpp::Time latest_detection_stamp_;

  cv::Ptr<cv::Tracker> cv_tracker_;
  cv::Rect2d tracked_roi_;
  float tracking_confidence_{0.0F};
  bool tracker_initialized_{false};
  rclcpp::Time last_track_time_;

  double desired_x_{0.0};
  double desired_y_{0.0};

  int centering_streak_{0};
  int close_depth_streak_{0};
  double accumulated_approach_distance_m_{0.0};
  double blind_approach_distance_m_{0.0};
  // True once depth reached standoff and we're blind-pushing the final distance.
  bool standoff_blind_active_{false};
  std::optional<CartesianVector> blind_push_start_position_;
  CartesianVector blind_push_axis_;
  rclcpp::Time blind_push_start_time_;
  double blind_push_timeout_sec_{0.0};
  double blind_push_start_accumulated_distance_m_{0.0};
  double accumulated_lift_distance_m_{0.0};
  std::optional<DepthSample> last_depth_sample_;
  std::deque<DepthHistoryEntry> depth_progress_history_;

  std::string rgb_topic_;
  std::string camera_info_topic_;
  std::string depth_topic_;
  std::string detection_topic_;
  std::string output_topic_;
  std::string joint_states_topic_;
  std::string reference_frame_;
  std::string camera_optical_frame_;
  std::string ee_frame_;
  std::string arm_base_frame_;
  std::string target_class_;
  std::string gripper_joint_name_;
  std::vector<std::string> gripper_joint_names_;
  double gripper_open_position_;
  std::vector<double> gripper_open_positions_;
  double gripper_open_position_tolerance_;

  bool use_depth_;
  double control_rate_hz_;
  double output_delta_horizon_sec_;
  double min_detection_confidence_;
  double min_tracking_confidence_;
  double lost_target_timeout_sec_;
  double acquire_timeout_sec_;
  double image_center_tolerance_px_;
  double grasp_standoff_m_;
  double grasp_depth_tolerance_m_;
  double depth_sample_anchor_x_;
  double depth_sample_anchor_y_;
  int depth_roi_half_size_px_;
  int min_valid_depth_pixels_;
  double depth_sample_max_iqr_m_;
  double depth_stale_timeout_sec_;
  int centering_stable_cycles_;
  int close_depth_stable_frames_;
  double grasp_settle_sec_;
  double lift_distance_m_;
  double max_approach_distance_m_;
  double approach_stall_window_sec_;
  double approach_min_progress_m_;
  double blind_approach_depth_threshold_m_;
  double blind_approach_velocity_fraction_;
  double blind_approach_max_distance_m_;
  double blind_approach_after_standoff_m_;
  double blind_push_timeout_config_sec_;
  double blind_push_close_tolerance_m_;
  double open_gripper_command_;
  double open_gripper_settle_sec_;
  double close_gripper_command_;

  std::string tracker_type_;
  int klt_max_features_;
  double klt_quality_level_;
  double klt_min_distance_;
  int klt_window_size_;
  int klt_pyramid_levels_;

  double lambda_xy_;
  double lambda_z_;
  double lambda_rz_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  int ramp_up_steps_;

  bool publish_overlay_;
  std::string overlay_topic_;
  bool publish_state_flag_;
  std::string state_topic_;
};

}  // namespace manipulation_visual_servo
