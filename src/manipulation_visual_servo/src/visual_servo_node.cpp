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

#include "manipulation_visual_servo/visual_servo_node.hpp"

#include <algorithm>
#include <cinttypes>
#include <chrono>
#include <cmath>
#include <optional>
#include <utility>

#include <control_msgs/msg/gripper_command.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/video/tracking.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "manipulation_visual_servo/visual_servo_utils.hpp"

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

std::string state_to_string(ServoState state)
{
  switch (state) {
    case ServoState::IDLE: return "IDLE";
    case ServoState::ACQUIRE: return "ACQUIRE";
    case ServoState::TRACK: return "TRACK";
    case ServoState::ALIGN_XY: return "ALIGN_XY";
    case ServoState::ESTIMATE_GRASP: return "ESTIMATE_GRASP";
    case ServoState::OPEN_GRIPPER: return "OPEN_GRIPPER";
    case ServoState::APPROACH_DEPTH: return "APPROACH_DEPTH";
    case ServoState::GUARDED_APPROACH: return "GUARDED_APPROACH";
    case ServoState::CLOSE_GRIPPER: return "CLOSE_GRIPPER";
    case ServoState::LIFT: return "LIFT";
    case ServoState::DONE: return "DONE";
    case ServoState::LOST: return "LOST";
    default: return "UNKNOWN";
  }
}

VisualServoNode::VisualServoNode(const rclcpp::NodeOptions & options)
: Node("visual_servo_node", options)
{
  this->declare_parameter("rgb_topic", "/piper/wrist_camera/piper_d405/color/image_rect_raw");
  this->declare_parameter(
    "camera_info_topic",
    "/piper/wrist_camera/piper_d405/color/camera_info");
  this->declare_parameter("depth_topic", "/piper/wrist_camera/piper_d405/depth/image_rect_raw");
  this->declare_parameter("mask_topic", "/manipulation/target_mask");
  this->declare_parameter("grasp_use_mask", true);
  this->declare_parameter("detection_topic", "/manipulation/target_detections");
  this->declare_parameter("output_topic", "/manipulation/policy_output");
  this->declare_parameter("joint_states_topic", "/joint_states");
  this->declare_parameter("use_depth", true);
  this->declare_parameter("control_rate_hz", 20.0);
  this->declare_parameter("output_delta_horizon_sec", 0.0);
  this->declare_parameter("target_class", "");
  this->declare_parameter("min_detection_confidence", 0.4);
  this->declare_parameter("min_tracking_confidence", 0.5);
  this->declare_parameter("lost_target_timeout_sec", 0.3);
  this->declare_parameter("acquire_timeout_sec", 5.0);
  this->declare_parameter("image_center_tolerance_px", 8.0);
  this->declare_parameter("reference_frame", "piper_base_link");
  this->declare_parameter("camera_optical_frame", "piper_camera_optical_frame");
  this->declare_parameter("ee_frame", "piper_tcp");
  this->declare_parameter("arm_base_frame", "piper_base_link");
  this->declare_parameter("grasp_standoff_m", 0.115);
  this->declare_parameter("grasp_depth_tolerance_m", 0.015);
  this->declare_parameter("depth_sample_anchor_x", 0.50);
  this->declare_parameter("depth_sample_anchor_y", 0.68);
  this->declare_parameter("depth_roi_half_size_px", 8);
  this->declare_parameter("min_valid_depth_pixels", 12);
  this->declare_parameter("depth_sample_max_iqr_m", 0.015);
  this->declare_parameter("depth_stale_timeout_sec", 0.25);
  this->declare_parameter("centering_stable_cycles", 3);
  this->declare_parameter("close_depth_stable_frames", 3);
  this->declare_parameter("grasp_settle_sec", 0.75);
  this->declare_parameter("lift_distance_m", 0.08);
  this->declare_parameter("max_approach_distance_m", 0.50);
  this->declare_parameter("approach_stall_window_sec", 1.0);
  this->declare_parameter("approach_min_progress_m", 0.01);
  this->declare_parameter("blind_approach_depth_threshold_m", 0.30);
  this->declare_parameter("blind_approach_velocity_fraction", 0.5);
  this->declare_parameter("blind_approach_max_distance_m", 0.20);
  this->declare_parameter("blind_approach_after_standoff_m", 0.03);
  this->declare_parameter("blind_push_timeout_sec", 7.2);
  this->declare_parameter("blind_push_close_tolerance_m", 0.006);
  this->declare_parameter("blind_push_offset_x", 0.0);
  this->declare_parameter("blind_push_offset_y", 0.0);
  this->declare_parameter("open_gripper_settle_sec", 3.0);
  this->declare_parameter("gripper_cmd_action", "/piper_gripper_controller/gripper_cmd");
  this->declare_parameter("gripper_joint_name", "piper_joint7");
  this->declare_parameter<std::vector<std::string>>(
    "gripper_joint_names", std::vector<std::string>{});
  this->declare_parameter("gripper_open_position", 0.065);
  this->declare_parameter("gripper_closed_position", 0.0);
  this->declare_parameter<std::vector<double>>(
    "gripper_open_positions", std::vector<double>{});
  this->declare_parameter("gripper_open_position_tolerance", 0.02);
  this->declare_parameter("gripper_max_effort", 5.0);

  // Look-then-move (table-plane) grasp parameters.
  this->declare_parameter("use_table_grasp", true);
  this->declare_parameter("grasp_top_down", true);
  this->declare_parameter("grasp_enabled", true);
  this->declare_parameter("grasp_auto_loop", false);
  this->declare_parameter("grasp_use_move_group", true);
  this->declare_parameter("grasp_plane_annulus_frac", 0.3);
  this->declare_parameter("grasp_plane_min_depth_m", 0.12);
  this->declare_parameter("grasp_plane_max_depth_m", 0.60);
  this->declare_parameter("grasp_plane_min_points", 60);
  this->declare_parameter("grasp_plane_max_rms_m", 0.03);
  this->declare_parameter("grasp_height_above_table_m", 0.055);
  this->declare_parameter("grasp_object_radius_m", 0.03);
  this->declare_parameter("neck_grasp_offset_m", 0.025);  // neck grasp: aim this far below the measured object top
  this->declare_parameter("grasp_band_width_margin_m", 0.012);  // jaw clearance for band width check
  this->declare_parameter("pregrasp_standoff_m", 0.12);
  this->declare_parameter("guarded_approach_speed_mps", 0.02);
  this->declare_parameter("guarded_reach_tolerance_m", 0.01);
  this->declare_parameter("grasp_estimate_settle_cycles", 10);
  this->declare_parameter("grasp_estimate_max_attempts", 40);
  this->declare_parameter("grasp_max_reach_m", 0.55);
  this->declare_parameter("grasp_offset_x", -0.035);  // calib: estimate overshoots forward
  this->declare_parameter("grasp_offset_y", -0.018);  // calib from teach demo
  this->declare_parameter("grasp_offset_z", 0.0);

  this->declare_parameter("tracker_type", "mil");
  this->declare_parameter("klt_max_features", 200);
  this->declare_parameter("klt_quality_level", 0.01);
  this->declare_parameter("klt_min_distance", 5.0);
  this->declare_parameter("klt_window_size", 10);
  this->declare_parameter("klt_pyramid_levels", 3);

  this->declare_parameter("control.lambda_xy", 0.3);
  this->declare_parameter("control.lambda_z", 0.1);
  this->declare_parameter("control.lambda_rz", 0.1);
  this->declare_parameter("control.max_linear_velocity", 0.08);
  this->declare_parameter("control.max_angular_velocity", 0.30);
  this->declare_parameter("control.ramp_up_steps", 5);

  this->declare_parameter("debug.publish_overlay", true);
  this->declare_parameter("debug.overlay_topic", "/visual_servo/debug_image");
  this->declare_parameter("debug.publish_state", true);
  this->declare_parameter("debug.state_topic", "/visual_servo/state");

  rgb_topic_ = this->get_parameter("rgb_topic").as_string();
  camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
  depth_topic_ = this->get_parameter("depth_topic").as_string();
  mask_topic_ = this->get_parameter("mask_topic").as_string();
  grasp_use_mask_ = this->get_parameter("grasp_use_mask").as_bool();
  detection_topic_ = this->get_parameter("detection_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();
  use_depth_ = this->get_parameter("use_depth").as_bool();
  control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
  output_delta_horizon_sec_ = this->get_parameter("output_delta_horizon_sec").as_double();
  target_class_ = this->get_parameter("target_class").as_string();
  min_detection_confidence_ = this->get_parameter("min_detection_confidence").as_double();
  min_tracking_confidence_ = this->get_parameter("min_tracking_confidence").as_double();
  lost_target_timeout_sec_ = this->get_parameter("lost_target_timeout_sec").as_double();
  acquire_timeout_sec_ = this->get_parameter("acquire_timeout_sec").as_double();
  image_center_tolerance_px_ = this->get_parameter("image_center_tolerance_px").as_double();
  reference_frame_ = this->get_parameter("reference_frame").as_string();
  camera_optical_frame_ = this->get_parameter("camera_optical_frame").as_string();
  ee_frame_ = this->get_parameter("ee_frame").as_string();
  arm_base_frame_ = this->get_parameter("arm_base_frame").as_string();
  grasp_standoff_m_ = this->get_parameter("grasp_standoff_m").as_double();
  grasp_depth_tolerance_m_ = this->get_parameter("grasp_depth_tolerance_m").as_double();
  depth_sample_anchor_x_ = this->get_parameter("depth_sample_anchor_x").as_double();
  depth_sample_anchor_y_ = this->get_parameter("depth_sample_anchor_y").as_double();
  depth_roi_half_size_px_ = this->get_parameter("depth_roi_half_size_px").as_int();
  min_valid_depth_pixels_ = this->get_parameter("min_valid_depth_pixels").as_int();
  depth_sample_max_iqr_m_ = this->get_parameter("depth_sample_max_iqr_m").as_double();
  depth_stale_timeout_sec_ = this->get_parameter("depth_stale_timeout_sec").as_double();
  centering_stable_cycles_ = this->get_parameter("centering_stable_cycles").as_int();
  close_depth_stable_frames_ = this->get_parameter("close_depth_stable_frames").as_int();
  grasp_settle_sec_ = this->get_parameter("grasp_settle_sec").as_double();
  lift_distance_m_ = this->get_parameter("lift_distance_m").as_double();
  max_approach_distance_m_ = this->get_parameter("max_approach_distance_m").as_double();
  approach_stall_window_sec_ = this->get_parameter("approach_stall_window_sec").as_double();
  approach_min_progress_m_ = this->get_parameter("approach_min_progress_m").as_double();
  blind_approach_depth_threshold_m_ =
    this->get_parameter("blind_approach_depth_threshold_m").as_double();
  blind_approach_velocity_fraction_ =
    this->get_parameter("blind_approach_velocity_fraction").as_double();
  blind_approach_max_distance_m_ =
    this->get_parameter("blind_approach_max_distance_m").as_double();
  blind_approach_after_standoff_m_ =
    this->get_parameter("blind_approach_after_standoff_m").as_double();
  blind_push_timeout_config_sec_ =
    this->get_parameter("blind_push_timeout_sec").as_double();
  blind_push_close_tolerance_m_ =
    this->get_parameter("blind_push_close_tolerance_m").as_double();
  blind_push_offset_x_ = this->get_parameter("blind_push_offset_x").as_double();
  blind_push_offset_y_ = this->get_parameter("blind_push_offset_y").as_double();
  open_gripper_settle_sec_ = this->get_parameter("open_gripper_settle_sec").as_double();
  gripper_cmd_action_ = this->get_parameter("gripper_cmd_action").as_string();
  gripper_joint_name_ = this->get_parameter("gripper_joint_name").as_string();
  gripper_joint_names_ = this->get_parameter("gripper_joint_names").as_string_array();
  gripper_open_position_ = this->get_parameter("gripper_open_position").as_double();
  gripper_closed_position_ = this->get_parameter("gripper_closed_position").as_double();
  gripper_open_positions_ = this->get_parameter("gripper_open_positions").as_double_array();
  gripper_open_position_tolerance_ =
    this->get_parameter("gripper_open_position_tolerance").as_double();
  gripper_max_effort_ = this->get_parameter("gripper_max_effort").as_double();

  use_table_grasp_ = this->get_parameter("use_table_grasp").as_bool();
  grasp_top_down_ = this->get_parameter("grasp_top_down").as_bool();
  grasp_enabled_ = this->get_parameter("grasp_enabled").as_bool();
  grasp_auto_loop_ = this->get_parameter("grasp_auto_loop").as_bool();
  grasp_use_move_group_ = this->get_parameter("grasp_use_move_group").as_bool();
  grasp_plane_annulus_frac_ = this->get_parameter("grasp_plane_annulus_frac").as_double();
  grasp_plane_min_depth_m_ = this->get_parameter("grasp_plane_min_depth_m").as_double();
  grasp_plane_max_depth_m_ = this->get_parameter("grasp_plane_max_depth_m").as_double();
  grasp_plane_min_points_ = static_cast<int>(this->get_parameter("grasp_plane_min_points").as_int());
  grasp_plane_max_rms_m_ = this->get_parameter("grasp_plane_max_rms_m").as_double();
  grasp_height_above_table_m_ = this->get_parameter("grasp_height_above_table_m").as_double();
  grasp_object_radius_m_ = this->get_parameter("grasp_object_radius_m").as_double();
  neck_grasp_offset_m_ = this->get_parameter("neck_grasp_offset_m").as_double();
  grasp_band_width_margin_m_ = this->get_parameter("grasp_band_width_margin_m").as_double();
  pregrasp_standoff_m_ = this->get_parameter("pregrasp_standoff_m").as_double();
  guarded_approach_speed_mps_ = this->get_parameter("guarded_approach_speed_mps").as_double();
  guarded_reach_tolerance_m_ = this->get_parameter("guarded_reach_tolerance_m").as_double();
  grasp_estimate_settle_cycles_ =
    static_cast<int>(this->get_parameter("grasp_estimate_settle_cycles").as_int());
  grasp_estimate_max_attempts_ =
    static_cast<int>(this->get_parameter("grasp_estimate_max_attempts").as_int());
  grasp_max_reach_m_ = this->get_parameter("grasp_max_reach_m").as_double();
  grasp_offset_x_ = this->get_parameter("grasp_offset_x").as_double();
  grasp_offset_y_ = this->get_parameter("grasp_offset_y").as_double();
  grasp_offset_z_ = this->get_parameter("grasp_offset_z").as_double();

  if (gripper_joint_names_.empty() && !gripper_joint_name_.empty()) {
    gripper_joint_names_.push_back(gripper_joint_name_);
  }
  if (gripper_open_positions_.empty()) {
    gripper_open_positions_.push_back(gripper_open_position_);
  }
  if (gripper_joint_names_.size() != gripper_open_positions_.size()) {
    RCLCPP_WARN(
      this->get_logger(),
      "[INIT][GRIPPER] joint/open target size mismatch (names=%zu, open=%zu); "
      "using primary gripper joint only",
      gripper_joint_names_.size(), gripper_open_positions_.size());
    gripper_joint_names_ = {gripper_joint_name_};
    gripper_open_positions_ = {gripper_open_position_};
  }

  tracker_type_ = this->get_parameter("tracker_type").as_string();
  klt_max_features_ = this->get_parameter("klt_max_features").as_int();
  klt_quality_level_ = this->get_parameter("klt_quality_level").as_double();
  klt_min_distance_ = this->get_parameter("klt_min_distance").as_double();
  klt_window_size_ = this->get_parameter("klt_window_size").as_int();
  klt_pyramid_levels_ = this->get_parameter("klt_pyramid_levels").as_int();

  lambda_xy_ = this->get_parameter("control.lambda_xy").as_double();
  lambda_z_ = this->get_parameter("control.lambda_z").as_double();
  lambda_rz_ = this->get_parameter("control.lambda_rz").as_double();
  max_linear_velocity_ = this->get_parameter("control.max_linear_velocity").as_double();
  max_angular_velocity_ = this->get_parameter("control.max_angular_velocity").as_double();
  ramp_up_steps_ = this->get_parameter("control.ramp_up_steps").as_int();

  publish_overlay_ = this->get_parameter("debug.publish_overlay").as_bool();
  overlay_topic_ = this->get_parameter("debug.overlay_topic").as_string();
  publish_state_flag_ = this->get_parameter("debug.publish_state").as_bool();
  state_topic_ = this->get_parameter("debug.state_topic").as_string();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Sensor callbacks use a reentrant group so RGB, depth, joint-state and
  // detection messages are never blocked by the control timer.  The control
  // timer has its own mutually-exclusive group to guarantee only one tick
  // executes at a time.
  sensor_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
  timer_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rgb_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sensor_sub_opts;
  sensor_sub_opts.callback_group = sensor_cb_group_;

  // RGB on its own group + a shallow queue (latest-frame semantics for servoing)
  // so the high-rate joint_states/depth callbacks cannot starve it.
  rclcpp::SubscriptionOptions rgb_sub_opts;
  rgb_sub_opts.callback_group = rgb_cb_group_;
  rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    rgb_topic_, rclcpp::SensorDataQoS().keep_last(2),
    std::bind(&VisualServoNode::image_callback, this, std::placeholders::_1),
    rgb_sub_opts);

  if (use_depth_) {
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VisualServoNode::depth_callback, this, std::placeholders::_1),
      sensor_sub_opts);
  }

  if (grasp_use_mask_) {
    mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      mask_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VisualServoNode::mask_callback, this, std::placeholders::_1),
      sensor_sub_opts);
  }

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::SensorDataQoS(),
    std::bind(&VisualServoNode::camera_info_callback, this, std::placeholders::_1),
    sensor_sub_opts);

  rclcpp::SubscriptionOptions sensor_sub_opts_reliable;
  sensor_sub_opts_reliable.callback_group = sensor_cb_group_;

  joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, 10,
    std::bind(&VisualServoNode::joint_states_callback, this, std::placeholders::_1),
    sensor_sub_opts_reliable);

  detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    detection_topic_, 10,
    std::bind(&VisualServoNode::detection_callback, this, std::placeholders::_1),
    sensor_sub_opts_reliable);

  gripper_cmd_client_ = rclcpp_action::create_client<GripperCommand>(this, gripper_cmd_action_);

  policy_output_pub_ = this->create_publisher<manipulation_msgs::msg::PolicyOutput>(
    output_topic_, 10);

  if (publish_state_flag_) {
    state_pub_ = this->create_publisher<std_msgs::msg::String>(state_topic_, 10);
  }

  if (publish_overlay_) {
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(overlay_topic_, 10);
  }

  auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&VisualServoNode::control_timer_callback, this),
    timer_cb_group_);

  state_entry_time_ = this->now();
  last_track_time_ = this->now();
  last_processed_frame_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  last_rgb_receive_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  last_depth_receive_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  latest_frame_generation_ = 0;
  last_processed_frame_generation_ = 0;

  RCLCPP_INFO(
    this->get_logger(),
    "[INIT] Visual servo ready | rgb=%s depth=%s detections=%s output=%s rate=%.1fHz",
    rgb_topic_.c_str(), depth_topic_.c_str(), detection_topic_.c_str(), output_topic_.c_str(),
    control_rate_hz_);
  RCLCPP_INFO(
    this->get_logger(),
    "[INIT] Frames | ref=%s camera=%s ee=%s",
    reference_frame_.c_str(), camera_optical_frame_.c_str(), ee_frame_.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "[INIT] Tracker=%s use_depth=%s target_class=%s delta_horizon=%.3fs",
    tracker_type_.c_str(), use_depth_ ? "true" : "false",
    target_class_.empty() ? "<any>" : target_class_.c_str(),
    output_delta_horizon_sec_ >
    0.0 ? output_delta_horizon_sec_ : 1.0 / std::max(1.0, control_rate_hz_));
  RCLCPP_INFO(
    this->get_logger(),
    "[INIT] Pick | use_depth=%s standoff=%.3f tol=%.3f lift=%.3f "
    "max_approach=%.3f blind_after_standoff=%.3f blind_push_offset=(%.4f, %.4f)",
    use_depth_ ? "true" : "false", grasp_standoff_m_, grasp_depth_tolerance_m_,
    lift_distance_m_, max_approach_distance_m_, blind_approach_after_standoff_m_,
    blind_push_offset_x_, blind_push_offset_y_);
  RCLCPP_INFO(
    this->get_logger(),
    "[INIT] Depth | anchor=(%.2f, %.2f) half_size=%d min_valid=%d max_iqr=%.3f "
    "close_stable=%d stall_window=%.2fs min_progress=%.3f",
    depth_sample_anchor_x_, depth_sample_anchor_y_, depth_roi_half_size_px_,
    min_valid_depth_pixels_, depth_sample_max_iqr_m_, close_depth_stable_frames_,
    approach_stall_window_sec_, approach_min_progress_m_);
  RCLCPP_INFO(
    this->get_logger(),
    "[INIT] Gripper action | action=%s open=%.3f close=%.3f open_tol=%.3f "
    "open_settle=%.2fs close_settle=%.2fs",
    gripper_cmd_action_.c_str(), gripper_open_position_, gripper_closed_position_,
    gripper_open_position_tolerance_, open_gripper_settle_sec_, grasp_settle_sec_);
}

void VisualServoNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // Keep this callback CHEAP: just stash the raw message and bump the
  // generation. The cvtColor/clone happens lazily in fetch_latest_frame() at the
  // 20 Hz control rate when a frame is actually consumed, so the 30 Hz image
  // stream cannot back up and get its best-effort samples dropped under load.
  std::lock_guard<std::mutex> lock(image_mutex_);
  latest_image_msg_ = msg;
  latest_frame_stamp_ = msg->header.stamp;
  last_rgb_receive_time_ = this->now();
  ++latest_frame_generation_;
  frame_available_ = true;
}

void VisualServoNode::depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv::Mat depth_image;
  std::string error_message;
  if (!decode_depth_image(*msg, depth_image, &error_message)) {
    if (!depth_encoding_warned_) {
      RCLCPP_WARN(
        this->get_logger(),
        "[DEPTH] frame rejected from %s: %s", depth_topic_.c_str(), error_message.c_str());
      depth_encoding_warned_ = true;
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "[DEPTH] frame rejected from %s: %s", depth_topic_.c_str(), error_message.c_str());
    }
    return;
  }

  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    latest_depth_frame_ = depth_image;
    latest_depth_stamp_ = msg->header.stamp;
    last_depth_receive_time_ = this->now();
    depth_available_ = true;
  }
  depth_encoding_warned_ = false;
}

void VisualServoNode::mask_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // MobileSAM target mask, mono8 (1 byte/pixel), already at the source/depth
  // resolution. Decode directly without cv_bridge (single-channel raw bytes).
  if ((msg->encoding != "mono8" && msg->encoding != "8UC1") ||
    msg->height == 0 || msg->width == 0)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "[MASK] frame rejected: encoding=%s %ux%u", msg->encoding.c_str(),
      msg->width, msg->height);
    return;
  }
  const cv::Mat view(
    static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC1,
    const_cast<uint8_t *>(msg->data.data()), msg->step);
  std::lock_guard<std::mutex> lock(mask_mutex_);
  latest_mask_frame_ = view.clone();
  last_mask_receive_time_ = this->now();
  mask_available_ = true;
}

void VisualServoNode::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  if (camera_info_received_) {
    return;
  }

  if (msg->k[0] > 0.0 && msg->k[4] > 0.0) {
    const double px = msg->k[0];
    const double py = msg->k[4];
    const double u0 = msg->k[2];
    const double v0 = msg->k[5];
    image_width_ = static_cast<int>(msg->width);
    image_height_ = static_cast<int>(msg->height);
    fx_ = px;
    fy_ = py;
    cx_ = u0;
    cy_ = v0;
    camera_info_received_ = true;
    desired_x_ = u0;
    desired_y_ = v0;

    RCLCPP_INFO(
      this->get_logger(),
      "[CAMERA] intrinsics fx=%.1f fy=%.1f cx=%.1f cy=%.1f size=%dx%d",
      px, py, u0, v0, image_width_, image_height_);
  }
}

void VisualServoNode::joint_states_callback(
  const sensor_msgs::msg::JointState::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  latest_joint_state_ = *msg;
  joint_state_available_ = true;
}

void VisualServoNode::detection_callback(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg)
{
  if (msg->detections.empty()) {
    return;
  }

  const vision_msgs::msg::Detection2D * best = nullptr;
  float best_score = 0.0F;

  for (const auto & det : msg->detections) {
    float score = 0.0F;
    std::string class_id;
    if (!det.results.empty()) {
      score = static_cast<float>(det.results[0].hypothesis.score);
      class_id = det.results[0].hypothesis.class_id;
    }

    if (score < min_detection_confidence_) {
      continue;
    }
    if (!target_class_.empty() && class_id != target_class_) {
      continue;
    }
    if (score > best_score) {
      best_score = score;
      best = &det;
    }
  }

  if (!best) {
    return;
  }

  const double cx = best->bbox.center.position.x;
  const double cy = best->bbox.center.position.y;
  const double w = best->bbox.size_x;
  const double h = best->bbox.size_y;

  std::lock_guard<std::mutex> lock(detection_mutex_);
  latest_detection_roi_ = cv::Rect2d(cx - w / 2.0, cy - h / 2.0, w, h);
  latest_detection_confidence_ = best_score;
  latest_detection_class_ = best->results.empty() ? "" : best->results[0].hypothesis.class_id;
  detection_available_ = true;
  latest_detection_stamp_ = msg->header.stamp;
}

void VisualServoNode::control_timer_callback()
{
  switch (state_) {
    case ServoState::IDLE:
      handle_idle();
      break;
    case ServoState::ACQUIRE:
      handle_acquire();
      break;
    case ServoState::TRACK:
      handle_track();
      break;
    case ServoState::ALIGN_XY:
      handle_align_xy();
      break;
    case ServoState::ESTIMATE_GRASP:
      handle_estimate_grasp();
      break;
    case ServoState::OPEN_GRIPPER:
      handle_open_gripper();
      break;
    case ServoState::APPROACH_DEPTH:
      handle_approach_depth();
      break;
    case ServoState::GUARDED_APPROACH:
      handle_guarded_approach();
      break;
    case ServoState::CLOSE_GRIPPER:
      handle_close_gripper();
      break;
    case ServoState::LIFT:
      handle_lift();
      break;
    case ServoState::DONE:
      handle_done();
      break;
    case ServoState::LOST:
      handle_lost();
      break;
  }

  if (publish_state_flag_) {
    publish_state();
  }
}

void VisualServoNode::transition_to(ServoState new_state, const std::string & reason)
{
  if (new_state == state_) {
    return;
  }

  // Cancel an in-flight gripper goal ONLY when aborting/resetting. The piper
  // gripper releases its hold when its action goal is cancelled, so cancelling
  // on OPEN_GRIPPER->approach would let it close mid-approach, and cancelling on
  // CLOSE_GRIPPER->LIFT would drop the grasped object. New gripper goals preempt
  // the old one in ensure_gripper_goal_started(), so no cancel is needed there.
  if (new_state == ServoState::LOST || new_state == ServoState::IDLE) {
    cancel_gripper_goal("aborting to " + state_to_string(new_state));
  }

  if (reason.empty()) {
    RCLCPP_INFO(
      this->get_logger(), "[STATE] %s -> %s",
      state_to_string(state_).c_str(), state_to_string(new_state).c_str());
  } else {
    RCLCPP_INFO(
      this->get_logger(), "[STATE] %s -> %s | %s",
      state_to_string(state_).c_str(), state_to_string(new_state).c_str(), reason.c_str());
  }
  state_ = new_state;
  state_entry_time_ = this->now();
  ramp_step_ = 0;

  if (new_state == ServoState::ACQUIRE) {
    reset_pick_progress();
  } else if (new_state == ServoState::ALIGN_XY) {
    centering_streak_ = 0;
  } else if (new_state == ServoState::APPROACH_DEPTH) {
    close_depth_streak_ = 0;
    accumulated_approach_distance_m_ = 0.0;
    reset_standoff_blind_push();
    depth_progress_history_.clear();
  } else if (new_state == ServoState::LIFT) {
    accumulated_lift_distance_m_ = 0.0;
  }
}

bool VisualServoNode::fetch_latest_frame(cv::Mat & frame)
{
  // Take the latest raw message under the lock (cheap shared_ptr copy), then
  // convert OUTSIDE the lock so the producer (image_callback) is never blocked.
  sensor_msgs::msg::Image::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!frame_available_ || latest_image_msg_ == nullptr) {
      return false;
    }
    // Some camera drivers reuse header stamps, so use an internal generation
    // counter instead of timestamp equality.
    if (latest_frame_generation_ == last_processed_frame_generation_) {
      return false;
    }
    msg = latest_image_msg_;
    last_processed_frame_stamp_ = latest_frame_stamp_;
    last_processed_frame_generation_ = latest_frame_generation_;
  }
  try {
    const cv::Mat view(
      static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
      const_cast<unsigned char *>(msg->data.data()), static_cast<std::size_t>(msg->step));
    if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(view, frame, cv::COLOR_RGB2BGR);
    } else if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
      frame = view.clone();
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Unsupported image encoding: %s (expected bgr8/rgb8)", msg->encoding.c_str());
      return false;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Image conversion failed: %s", e.what());
    return false;
  }
  return true;
}

bool VisualServoNode::fallback_to_detection_tracking(const char * context, bool allow_stale_roi)
{
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    if (detection_available_) {
      tracked_roi_ = latest_detection_roi_;
      tracking_confidence_ = latest_detection_confidence_;
      detection_available_ = false;
      last_track_time_ = this->now();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[%s] fresh RGB stalled on %s; using latest detection ROI instead",
        context, rgb_topic_.c_str());
      return true;
    }
  }

  if (!allow_stale_roi) {
    return false;
  }

  if (tracked_roi_.width <= 0.0 || tracked_roi_.height <= 0.0) {
    return false;
  }

  const double since_track = (this->now() - last_track_time_).seconds();
  if (since_track > lost_target_timeout_sec_) {
    return false;
  }

  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "[%s] fresh RGB stalled on %s; reusing last tracked ROI for %.2fs",
    context, rgb_topic_.c_str(), since_track);
  return true;
}

void VisualServoNode::reset_pick_progress()
{
  centering_streak_ = 0;
  close_depth_streak_ = 0;
  accumulated_approach_distance_m_ = 0.0;
  reset_standoff_blind_push();
  accumulated_lift_distance_m_ = 0.0;
  last_depth_sample_.reset();
  depth_progress_history_.clear();
  grasp_target_ref_.reset();
  pregrasp_target_ref_.reset();
  guarded_at_pregrasp_ = false;
  estimate_attempts_ = 0;
}

void VisualServoNode::reset_standoff_blind_push()
{
  blind_approach_distance_m_ = 0.0;
  standoff_blind_active_ = false;
  blind_push_start_position_.reset();
  blind_push_axis_ = CartesianVector{};
  blind_push_start_time_ = this->now();
  blind_push_timeout_sec_ = 0.0;
  blind_push_start_accumulated_distance_m_ = 0.0;
}

void VisualServoNode::reset_gripper_action_state()
{
  std::lock_guard<std::mutex> lock(gripper_action_mutex_);
  ++gripper_goal_generation_;
  gripper_goal_handle_.reset();
  gripper_goal_state_ = ServoState::IDLE;
  gripper_goal_started_ = false;
  gripper_goal_completed_ = false;
  gripper_goal_succeeded_ = false;
  gripper_goal_stalled_ = false;
  gripper_goal_reached_goal_ = false;
  gripper_goal_error_.clear();
}

void VisualServoNode::cancel_gripper_goal(const std::string & reason)
{
  GripperGoalHandle::SharedPtr goal_handle;
  ServoState goal_state = ServoState::IDLE;
  {
    std::lock_guard<std::mutex> lock(gripper_action_mutex_);
    goal_handle = gripper_goal_handle_;
    goal_state = gripper_goal_state_;
    ++gripper_goal_generation_;
    gripper_goal_handle_.reset();
    gripper_goal_state_ = ServoState::IDLE;
    gripper_goal_started_ = false;
    gripper_goal_completed_ = false;
    gripper_goal_succeeded_ = false;
    gripper_goal_stalled_ = false;
    gripper_goal_reached_goal_ = false;
    gripper_goal_error_.clear();
  }

  if (goal_handle) {
    if (reason.empty()) {
      RCLCPP_INFO(
        this->get_logger(),
        "[GRIPPER][%s] canceling in-flight goal",
        state_to_string(goal_state).c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "[GRIPPER][%s] canceling in-flight goal: %s",
        state_to_string(goal_state).c_str(), reason.c_str());
    }
    gripper_cmd_client_->async_cancel_goal(goal_handle);
  }
}

void VisualServoNode::ensure_gripper_goal_started(ServoState command_state)
{
  {
    std::lock_guard<std::mutex> lock(gripper_action_mutex_);
    if (gripper_goal_started_ && gripper_goal_state_ == command_state) {
      return;
    }
  }

  const double target_position =
    command_state == ServoState::OPEN_GRIPPER ? gripper_open_position_ : gripper_closed_position_;
  const char * label = command_state == ServoState::OPEN_GRIPPER ? "OPEN" : "CLOSE";

  cancel_gripper_goal("starting direct action command");

  if (!gripper_cmd_client_->wait_for_action_server(std::chrono::seconds(0))) {
    std::lock_guard<std::mutex> lock(gripper_action_mutex_);
    gripper_goal_state_ = command_state;
    gripper_goal_started_ = true;
    gripper_goal_completed_ = true;
    gripper_goal_succeeded_ = false;
    gripper_goal_stalled_ = false;
    gripper_goal_reached_goal_ = false;
    gripper_goal_error_ = "action server unavailable";
    RCLCPP_WARN(
      this->get_logger(),
      "[GRIPPER][%s] action server %s not available",
      label, gripper_cmd_action_.c_str());
    return;
  }

  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(gripper_action_mutex_);
    generation = ++gripper_goal_generation_;
    gripper_goal_state_ = command_state;
    gripper_goal_started_ = true;
    gripper_goal_completed_ = false;
    gripper_goal_succeeded_ = false;
    gripper_goal_stalled_ = false;
    gripper_goal_reached_goal_ = false;
    gripper_goal_error_.clear();
    gripper_goal_handle_.reset();
  }

  GripperCommand::Goal goal;
  goal.command = control_msgs::msg::GripperCommand();
  goal.command.position = target_position;
  // Nonzero effort is required or the piper gripper does not actuate (the joint
  // feedback still reports the commanded position, masking the no-move).
  goal.command.max_effort = gripper_max_effort_;

  rclcpp_action::Client<GripperCommand>::SendGoalOptions options;
  options.goal_response_callback =
    [this, command_state, generation](
    const GripperGoalHandle::SharedPtr & goal_handle) {
      handle_gripper_goal_response(command_state, generation, goal_handle);
    };
  options.result_callback =
    [this, command_state, generation](const GripperGoalHandle::WrappedResult & result) {
      handle_gripper_goal_result(command_state, generation, result);
    };

  RCLCPP_INFO(
    this->get_logger(),
    "[GRIPPER][%s] sending direct GripperCommand to %.4f via %s",
    label, target_position, gripper_cmd_action_.c_str());

  try {
    gripper_cmd_client_->async_send_goal(goal, options);
  } catch (const std::exception & ex) {
    std::lock_guard<std::mutex> lock(gripper_action_mutex_);
    if (generation != gripper_goal_generation_ || gripper_goal_state_ != command_state) {
      return;
    }
    gripper_goal_completed_ = true;
    gripper_goal_succeeded_ = false;
    gripper_goal_error_ = std::string("failed to send goal: ") + ex.what();
    RCLCPP_WARN(
      this->get_logger(),
      "[GRIPPER][%s] failed to send GripperCommand goal: %s",
      label, ex.what());
  }
}

void VisualServoNode::handle_gripper_goal_response(
  ServoState command_state, uint64_t generation, const GripperGoalHandle::SharedPtr & goal_handle)
{
  const char * label = command_state == ServoState::OPEN_GRIPPER ? "OPEN" : "CLOSE";
  std::lock_guard<std::mutex> lock(gripper_action_mutex_);
  if (generation != gripper_goal_generation_ || gripper_goal_state_ != command_state) {
    return;
  }

  if (!goal_handle) {
    gripper_goal_completed_ = true;
    gripper_goal_succeeded_ = false;
    gripper_goal_error_ = "goal rejected";
    RCLCPP_WARN(this->get_logger(), "[GRIPPER][%s] GripperCommand goal rejected", label);
    return;
  }

  gripper_goal_handle_ = goal_handle;
  RCLCPP_INFO(
    this->get_logger(),
    "[GRIPPER][%s] action accepted by %s",
    label, gripper_cmd_action_.c_str());
}

void VisualServoNode::handle_gripper_goal_result(
  ServoState command_state, uint64_t generation, const GripperGoalHandle::WrappedResult & result)
{
  const char * label = command_state == ServoState::OPEN_GRIPPER ? "OPEN" : "CLOSE";
  {
    std::lock_guard<std::mutex> lock(gripper_action_mutex_);
    if (generation != gripper_goal_generation_ || gripper_goal_state_ != command_state) {
      return;
    }

    gripper_goal_handle_.reset();
    gripper_goal_completed_ = true;
    gripper_goal_succeeded_ = false;
    gripper_goal_stalled_ = false;
    gripper_goal_reached_goal_ = false;
    gripper_goal_error_.clear();

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      gripper_goal_succeeded_ = true;
      if (result.result) {
        gripper_goal_stalled_ = result.result->stalled;
        gripper_goal_reached_goal_ = result.result->reached_goal;
      }
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
      gripper_goal_error_ = "goal aborted";
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      gripper_goal_error_ = "goal canceled";
    } else {
      gripper_goal_error_ = "goal ended with unknown result";
    }
  }

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    if (result.result && result.result->stalled && !result.result->reached_goal) {
      RCLCPP_WARN(
        this->get_logger(),
        "[GRIPPER][%s] hardware stopped at %.4f before the commanded target; "
        "treating this as success",
        label, result.result->position);
    } else if (result.result) {
      RCLCPP_INFO(
        this->get_logger(),
        "[GRIPPER][%s] reached commanded position %.4f",
        label, result.result->position);
    } else {
      RCLCPP_INFO(this->get_logger(), "[GRIPPER][%s] action completed", label);
    }
    return;
  }

  RCLCPP_WARN(
    this->get_logger(),
    "[GRIPPER][%s] action failed: %s",
    label,
    result.code == rclcpp_action::ResultCode::ABORTED ? "goal aborted" :
    result.code == rclcpp_action::ResultCode::CANCELED ? "goal canceled" :
    "unknown result");
}

std::optional<CartesianVector> VisualServoNode::lookup_current_ee_position_in_reference()
{
  const std::string target_frame = reference_frame_.empty() ? arm_base_frame_ : reference_frame_;
  try {
    const auto transform = tf_buffer_->lookupTransform(target_frame, ee_frame_, tf2::TimePointZero);
    return CartesianVector{
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z};
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[TF] lookup %s -> %s failed during blind push: %s",
      ee_frame_.c_str(), target_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

std::optional<CartesianVector> VisualServoNode::lookup_eef_positive_z_axis_in_reference()
{
  const std::string target_frame = reference_frame_.empty() ? arm_base_frame_ : reference_frame_;
  if (ee_frame_.empty() || target_frame.empty()) {
    return std::nullopt;
  }
  if (ee_frame_ == target_frame) {
    return CartesianVector{0.0, 0.0, 1.0};
  }

  try {
    const auto transform = tf_buffer_->lookupTransform(
      target_frame, ee_frame_, tf2::TimePointZero);
    const auto & q = transform.transform.rotation;
    const double qx = q.x;
    const double qy = q.y;
    const double qz = q.z;
    const double qw = q.w;
    const double vx = 0.0;
    const double vy = 0.0;
    const double vz = 1.0;
    const double t2 = qw * vx + qy * vz - qz * vy;
    const double t3 = qw * vy + qz * vx - qx * vz;
    const double t4 = qw * vz + qx * vy - qy * vx;
    const double t5 = -qx * vx - qy * vy - qz * vz;
    CartesianVector axis;
    axis.x = t2 * qw - t5 * qx - t3 * qz + t4 * qy;
    axis.y = t3 * qw - t5 * qy - t4 * qx + t2 * qz;
    axis.z = t4 * qw - t5 * qz - t2 * qy + t3 * qx;

    const double axis_norm = std::sqrt(
      axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (axis_norm < 1e-9) {
      return std::nullopt;
    }
    axis.x /= axis_norm;
    axis.y /= axis_norm;
    axis.z /= axis_norm;
    return axis;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[TF] lookup %s -> %s failed while latching blind axis: %s",
      ee_frame_.c_str(), target_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

bool VisualServoNode::start_standoff_blind_push(double depth_m)
{
  const double blind_speed_mps = blind_approach_velocity_fraction_ * max_linear_velocity_;
  if (blind_approach_after_standoff_m_ <= 0.0 || blind_speed_mps <= 1e-6) {
    return false;
  }

  reset_standoff_blind_push();

  const auto start_position = lookup_current_ee_position_in_reference();
  const auto blind_axis = lookup_eef_positive_z_axis_in_reference();
  if (!start_position.has_value() || !blind_axis.has_value()) {
    return false;
  }

  blind_push_start_position_ = start_position;
  blind_push_axis_ = blind_axis.value();
  blind_push_start_time_ = this->now();
  blind_push_timeout_sec_ = std::max(0.1, blind_push_timeout_config_sec_);
  blind_push_start_accumulated_distance_m_ = accumulated_approach_distance_m_;
  blind_approach_distance_m_ = 0.0;
  standoff_blind_active_ = true;

  RCLCPP_INFO(
    this->get_logger(),
    "[APPROACH] depth reached standoff (%.3fm); blind push (eef +Z) "
    "start=(%.4f, %.4f, %.4f) axis=(%.3f, %.3f, %.3f) "
    "dist=%.3fm vel=%.3fm/s timeout=%.2fs",
    depth_m,
    blind_push_start_position_->x, blind_push_start_position_->y, blind_push_start_position_->z,
    blind_push_axis_.x, blind_push_axis_.y, blind_push_axis_.z,
    blind_approach_after_standoff_m_, blind_speed_mps, blind_push_timeout_sec_);
  return true;
}

void VisualServoNode::handle_standoff_blind_push(const cv::Mat * frame)
{
  const double blind_speed_mps = blind_approach_velocity_fraction_ * max_linear_velocity_;
  if (!blind_push_start_position_.has_value() || blind_speed_mps <= 1e-6) {
    publish_zero_motion(tracking_confidence_);
    transition_to(ServoState::LOST, "standoff blind push state invalid");
    if (publish_overlay_ && frame != nullptr) {
      publish_debug_overlay(*frame, tracked_roi_);
    }
    return;
  }

  const auto current_position = lookup_current_ee_position_in_reference();
  if (!current_position.has_value()) {
    publish_zero_motion(tracking_confidence_);
    transition_to(ServoState::LOST, "blind push TF unavailable");
    if (publish_overlay_ && frame != nullptr) {
      publish_debug_overlay(*frame, tracked_roi_);
    }
    return;
  }

  const CartesianVector translation{
    current_position->x - blind_push_start_position_->x,
    current_position->y - blind_push_start_position_->y,
    current_position->z - blind_push_start_position_->z};
  blind_approach_distance_m_ = std::max(
    0.0, project_translation_onto_axis(translation, blind_push_axis_));
  accumulated_approach_distance_m_ =
    blind_push_start_accumulated_distance_m_ + blind_approach_distance_m_;

  if (blind_approach_distance_m_ > blind_approach_max_distance_m_) {
    RCLCPP_WARN(
      this->get_logger(),
      "[APPROACH] aborting pick: blind actual distance %.3fm exceeded limit %.3fm",
      blind_approach_distance_m_, blind_approach_max_distance_m_);
    publish_zero_motion(tracking_confidence_);
    transition_to(ServoState::LOST, "blind push distance limit exceeded");
    if (publish_overlay_ && frame != nullptr) {
      publish_debug_overlay(*frame, tracked_roi_);
    }
    return;
  }

  if (accumulated_approach_distance_m_ > max_approach_distance_m_) {
    RCLCPP_WARN(
      this->get_logger(),
      "[APPROACH] aborting pick: total distance %.3fm exceeded limit %.3fm",
      accumulated_approach_distance_m_, max_approach_distance_m_);
    publish_zero_motion(tracking_confidence_);
    transition_to(ServoState::LOST, "approach distance limit exceeded during blind push");
    if (publish_overlay_ && frame != nullptr) {
      publish_debug_overlay(*frame, tracked_roi_);
    }
    return;
  }

  if (blind_approach_distance_m_ >= blind_approach_after_standoff_m_) {
    publish_zero_motion(tracking_confidence_);
    transition_to(ServoState::CLOSE_GRIPPER, "standoff blind push completed");
    if (publish_overlay_ && frame != nullptr) {
      publish_debug_overlay(*frame, tracked_roi_);
    }
    return;
  }

  const double remaining = std::max(
    0.0,
    blind_approach_after_standoff_m_ - blind_approach_distance_m_);
  const double elapsed_sec = (this->now() - blind_push_start_time_).seconds();
  if (elapsed_sec >= blind_push_timeout_sec_) {
    if (remaining <= std::max(0.0, blind_push_close_tolerance_m_)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[APPROACH] blind push timed out after %.2fs but remaining %.3fm is within "
        "close tolerance %.3fm; closing gripper",
        elapsed_sec, remaining, blind_push_close_tolerance_m_);
      publish_zero_motion(tracking_confidence_);
      transition_to(ServoState::CLOSE_GRIPPER, "blind push timeout within close tolerance");
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "[APPROACH] aborting pick: blind push timed out after %.2fs "
        "(actual=%.3fm target=%.3fm remaining=%.3fm tol=%.3fm)",
        elapsed_sec, blind_approach_distance_m_, blind_approach_after_standoff_m_,
        remaining, blind_push_close_tolerance_m_);
      publish_zero_motion(tracking_confidence_);
      transition_to(ServoState::LOST, "blind push timed out");
    }
    if (publish_overlay_ && frame != nullptr) {
      publish_debug_overlay(*frame, tracked_roi_);
    }
    return;
  }

  const double cycle_dt = 1.0 / std::max(1.0, control_rate_hz_);
  const double delta_horizon_sec = output_delta_horizon_sec_ > 0.0 ?
    output_delta_horizon_sec_ : cycle_dt;
  const double velocity = std::min(blind_speed_mps, remaining / cycle_dt);
  const double step_distance = velocity * delta_horizon_sec;

  // Compute a lateral axis perpendicular to the push direction in the reference
  // frame horizontal plane.  blind_push_offset_x applies along this lateral axis
  // (positive = robot left / +Y when the arm faces +X) and blind_push_offset_y
  // applies along the reference frame Z-up direction.
  const double ax = blind_push_axis_.x;
  const double ay = blind_push_axis_.y;
  const double horiz_len = std::sqrt(ax * ax + ay * ay);
  // lateral = push_axis rotated 90° CCW in XY: (-ay, ax, 0), normalized
  const double lat_x = (horiz_len > 1e-6) ? (-ay / horiz_len) : 0.0;
  const double lat_y = (horiz_len > 1e-6) ? ( ax / horiz_len) : 0.0;

  const double progress_fraction =
    step_distance / std::max(1e-6, blind_approach_after_standoff_m_);
  const double off_lateral = blind_push_offset_x_ * progress_fraction;
  const double off_vertical = blind_push_offset_y_ * progress_fraction;

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "[APPROACH] standoff blind push (eef +Z): actual=%.3f/%.3f remaining=%.3f "
    "elapsed=%.2f/%.2f axis=(%.3f, %.3f, %.3f) lateral=(%.3f, %.3f) "
    "offset=(%.4f, %.4f)",
    blind_approach_distance_m_, blind_approach_after_standoff_m_, remaining,
    elapsed_sec, blind_push_timeout_sec_,
    blind_push_axis_.x, blind_push_axis_.y, blind_push_axis_.z,
    lat_x, lat_y,
    blind_push_offset_x_, blind_push_offset_y_);

  publish_reference_frame_delta(
    blind_push_axis_.x * step_distance + lat_x * off_lateral,
    blind_push_axis_.y * step_distance + lat_y * off_lateral,
    blind_push_axis_.z * step_distance + off_vertical,
    tracking_confidence_);
  if (publish_overlay_ && frame != nullptr) {
    publish_debug_overlay(*frame, tracked_roi_);
  }
}

bool VisualServoNode::acquire_from_detection(const cv::Mat & frame)
{
  cv::Rect2d det_roi;
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    if (!detection_available_) {
      return false;
    }
    det_roi = latest_detection_roi_;
    detection_available_ = false;
  }

  det_roi.x = std::max(0.0, det_roi.x);
  det_roi.y = std::max(0.0, det_roi.y);
  det_roi.width = std::min(det_roi.width, static_cast<double>(frame.cols) - det_roi.x);
  det_roi.height = std::min(det_roi.height, static_cast<double>(frame.rows) - det_roi.y);

  if (det_roi.width < 10.0 || det_roi.height < 10.0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[ACQUIRE] detection ROI too small: %.0fx%.0f", det_roi.width, det_roi.height);
    return false;
  }

  if (!init_tracker(frame, det_roi)) {
    return false;
  }

  tracked_roi_ = det_roi;
  last_track_time_ = this->now();
  return true;
}

bool VisualServoNode::update_tracking(const cv::Mat & frame)
{
  // Fresh detections act as the source of truth at ~1 Hz; MIL propagates the
  // ROI between detections so the servo loop stays locked to the bottle body.
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    if (detection_available_) {
      cv::Rect2d det_roi = latest_detection_roi_;
      detection_available_ = false;

      det_roi.x = std::max(0.0, det_roi.x);
      det_roi.y = std::max(0.0, det_roi.y);
      det_roi.width = std::min(det_roi.width, static_cast<double>(frame.cols) - det_roi.x);
      det_roi.height = std::min(det_roi.height, static_cast<double>(frame.rows) - det_roi.y);

      if (det_roi.width >= 10.0 && det_roi.height >= 10.0) {
        // Re-init the tracker so it tracks from the new detection position.
        init_tracker(frame, det_roi);
        tracked_roi_ = det_roi;
        last_track_time_ = this->now();
        tracking_confidence_ = 1.0F;
        return true;
      }
    }
  }

  const double since_track = (this->now() - last_track_time_).seconds();
  if (since_track > lost_target_timeout_sec_) {
    transition_to(ServoState::LOST, "tracker timed out");
    return false;
  }

  cv::Rect2d updated_roi;
  if (!update_tracker(frame, updated_roi)) {
    // MIL tracker lost the target (e.g. scene shifted during RGB stall).
    // Re-init from the last known ROI on the current frame before giving up.
    if (tracked_roi_.width >= 10.0 && tracked_roi_.height >= 10.0 &&
      since_track <= lost_target_timeout_sec_)
    {
      init_tracker(frame, tracked_roi_);
      last_track_time_ = this->now();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[TRACK] MIL tracker failed; re-initialized from last ROI");
      return true;
    }
    transition_to(ServoState::LOST, "tracker update failed");
    return false;
  }

  tracked_roi_ = updated_roi;
  last_track_time_ = this->now();
  return true;
}

void VisualServoNode::handle_idle()
{
  if (!camera_info_received_) {
    return;
  }

  // Re-read runtime-tunable params so an external autotuner's `ros2 param set`
  // takes effect on the next attempt without relaunching.
  grasp_enabled_ = this->get_parameter("grasp_enabled").as_bool();
  if (!grasp_enabled_) {
    return;
  }
  grasp_offset_x_ = this->get_parameter("grasp_offset_x").as_double();
  grasp_offset_y_ = this->get_parameter("grasp_offset_y").as_double();
  grasp_offset_z_ = this->get_parameter("grasp_offset_z").as_double();
  grasp_height_above_table_m_ = this->get_parameter("grasp_height_above_table_m").as_double();
  neck_grasp_offset_m_ = this->get_parameter("neck_grasp_offset_m").as_double();
  // Plane-fit tunables — live so the threshold can be tightened without relaunch.
  grasp_plane_max_rms_m_ = this->get_parameter("grasp_plane_max_rms_m").as_double();
  grasp_plane_annulus_frac_ = this->get_parameter("grasp_plane_annulus_frac").as_double();
  grasp_plane_min_points_ = static_cast<int>(this->get_parameter("grasp_plane_min_points").as_int());

  std::lock_guard<std::mutex> lock(detection_mutex_);
  if (detection_available_) {
    transition_to(ServoState::ACQUIRE, "detection available");
  }
}

void VisualServoNode::handle_acquire()
{
  const double elapsed = (this->now() - state_entry_time_).seconds();
  if (elapsed > acquire_timeout_sec_) {
    RCLCPP_WARN(this->get_logger(), "[ACQUIRE] timed out after %.1fs", elapsed);
    transition_to(ServoState::IDLE, "acquire timeout");
    return;
  }

  cv::Mat frame;
  if (!fetch_latest_frame(frame)) {
    return;
  }

  if (acquire_from_detection(frame)) {
    transition_to(ServoState::TRACK, "tracker initialized from detection");
  }
}

void VisualServoNode::handle_track()
{
  cv::Mat frame;
  const ServoState post_track = (use_table_grasp_ && grasp_use_move_group_) ?
    ServoState::ESTIMATE_GRASP : ServoState::ALIGN_XY;
  if (!fetch_latest_frame(frame)) {
    if (fallback_to_detection_tracking("TRACK")) {
      transition_to(post_track, "target acquired from detection fallback");
      return;
    }
    std::uint64_t latest_generation = 0;
    std::uint64_t processed_generation = 0;
    rclcpp::Time latest_stamp(0, 0, this->get_clock()->get_clock_type());
    rclcpp::Time last_rgb_receive(0, 0, this->get_clock()->get_clock_type());
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      latest_generation = latest_frame_generation_;
      processed_generation = last_processed_frame_generation_;
      latest_stamp = latest_frame_stamp_;
      last_rgb_receive = last_rgb_receive_time_;
    }
    const double since_rgb_sec =
      last_rgb_receive.nanoseconds() > 0 ? (this->now() - last_rgb_receive).seconds() : -1.0;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[TRACK] waiting for fresh RGB frame: latest_gen=%" PRIu64 " processed_gen=%" PRIu64 " "
      "latest_stamp=%.3f since_rgb=%.3fs",
      static_cast<uint64_t>(latest_generation),
      static_cast<uint64_t>(processed_generation),
      latest_stamp.seconds(), since_rgb_sec);
    return;
  }

  if (update_tracking(frame)) {
    transition_to(post_track, "target acquired");
  }
}

void VisualServoNode::handle_align_xy()
{
  cv::Mat frame;
  const bool has_fresh_frame = fetch_latest_frame(frame);
  if (has_fresh_frame) {
    if (!update_tracking(frame)) {
      return;
    }
  } else if (!fallback_to_detection_tracking("ALIGN")) {
    return;
  }

  const double feat_x = tracked_roi_.x + tracked_roi_.width * 0.5;
  const double feat_y = tracked_roi_.y + tracked_roi_.height * 0.5;
  const double err_x = feat_x - desired_x_;
  const double err_y = feat_y - desired_y_;
  const double pixel_error = std::sqrt(err_x * err_x + err_y * err_y);
  const bool centered = pixel_error <= image_center_tolerance_px_;

  const auto centering_update = update_centering_streak(
    centering_streak_, centered, centering_stable_cycles_);
  centering_streak_ = centering_update.streak;

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 500,
    "[ALIGN] err_px=%.1f dx=%.1f dy=%.1f feat=(%.1f,%.1f) desired=(%.1f,%.1f) "
    "centered=%s streak=%d/%d tol=%.1f",
    pixel_error, err_x, err_y, feat_x, feat_y, desired_x_, desired_y_,
    centered ? "yes" : "no", centering_streak_, centering_stable_cycles_,
    image_center_tolerance_px_);

  const auto twist = compute_alignment_twist(feat_x, feat_y);
  publish_policy_output(twist, tracking_confidence_);

  if (publish_overlay_ && has_fresh_frame) {
    publish_debug_overlay(frame, tracked_roi_);
  }

  if (centering_update.stable) {
    if (use_table_grasp_) {
      // Look-then-move: estimate the 3D grasp pose at this safe standoff before
      // committing any forward motion (never servo into the D405 blind zone).
      transition_to(ServoState::ESTIMATE_GRASP, "target centered; estimating grasp");
    } else {
      transition_to(ServoState::OPEN_GRIPPER, "target centered");
    }
  }
}

void VisualServoNode::handle_open_gripper()
{
  cv::Mat frame;
  if (fetch_latest_frame(frame)) {
    if (!update_tracking(frame)) {
      return;
    }
    if (publish_overlay_) {
      publish_debug_overlay(frame, tracked_roi_);
    }
  }

  ensure_gripper_goal_started(ServoState::OPEN_GRIPPER);

  const double elapsed = (this->now() - state_entry_time_).seconds();
  const bool settle_elapsed = elapsed >= open_gripper_settle_sec_;
  const auto open_error = max_gripper_open_error();
  const bool gripper_fully_open =
    open_error.has_value() && *open_error <= gripper_open_position_tolerance_;
  bool action_completed = false;
  bool action_succeeded = false;
  bool action_stalled = false;
  bool action_reached_goal = false;
  std::string action_error;
  {
    std::lock_guard<std::mutex> lock(gripper_action_mutex_);
    action_completed = gripper_goal_completed_;
    action_succeeded = gripper_goal_succeeded_;
    action_stalled = gripper_goal_stalled_;
    action_reached_goal = gripper_goal_reached_goal_;
    action_error = gripper_goal_error_;
  }

  if (open_error.has_value()) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[GRIPPER][OPEN] target=%.4f feedback=%s max_err=%.3f tol=%.3f action=%s "
      "stalled=%s reached_goal=%s settle=%.2f/%.2f",
      gripper_open_position_,
      gripper_fully_open ? "ready" : "waiting",
      *open_error, gripper_open_position_tolerance_,
      action_succeeded ? "succeeded" : (action_completed ? action_error.c_str() : "pending"),
      action_stalled ? "true" : "false",
      action_reached_goal ? "true" : "false",
      elapsed, open_gripper_settle_sec_);
  } else {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[GRIPPER][OPEN] target=%.4f feedback=waiting joint_states=%s action=%s "
      "settle=%.2f/%.2f",
      gripper_open_position_,
      joint_states_topic_.c_str(),
      action_succeeded ? "succeeded" : (action_completed ? action_error.c_str() : "pending"),
      elapsed, open_gripper_settle_sec_);
  }

  publish_policy_output(geometry_msgs::msg::Twist(), tracking_confidence_, false, false, 0.0);

  if (gripper_fully_open || action_succeeded) {
    if (gripper_fully_open) {
      RCLCPP_INFO(
        this->get_logger(),
        "[GRIPPER][OPEN] joint feedback confirms the gripper is open "
        "(max_err=%.4f <= tol=%.4f)",
        *open_error, gripper_open_position_tolerance_);
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "[GRIPPER][OPEN] open action completed; starting depth approach");
    }
    const bool table_ready = use_table_grasp_ && grasp_target_ref_.has_value();
    transition_to(
      table_ready ? ServoState::GUARDED_APPROACH : ServoState::APPROACH_DEPTH,
      gripper_fully_open ?
      "gripper fully opened; starting approach" :
      "open gripper action completed; starting approach");
    return;
  }

  if (settle_elapsed) {
    if (action_completed && !action_succeeded) {
      RCLCPP_WARN(
        this->get_logger(),
        "[GRIPPER][OPEN] settle elapsed after action error (%s); continuing to depth approach",
        action_error.c_str());
    } else if (open_error.has_value()) {
      RCLCPP_WARN(
        this->get_logger(),
        "[GRIPPER][OPEN] settle elapsed without confirmed open feedback "
        "(max_err=%.3f tol=%.3f); continuing to depth approach",
        *open_error, gripper_open_position_tolerance_);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "[GRIPPER][OPEN] settle elapsed without gripper feedback on %s; "
        "continuing to depth approach",
        joint_states_topic_.c_str());
    }
    const bool table_ready = use_table_grasp_ && grasp_target_ref_.has_value();
    transition_to(
      table_ready ? ServoState::GUARDED_APPROACH : ServoState::APPROACH_DEPTH,
      "open command settled; starting approach");
  }
}

std::optional<CartesianVector> VisualServoNode::transform_point_to_reference(
  const CartesianVector & point_cam)
{
  const std::string target = reference_frame_.empty() ? arm_base_frame_ : reference_frame_;
  if (camera_optical_frame_.empty() || target.empty()) {
    return std::nullopt;
  }
  geometry_msgs::msg::PointStamped in;
  in.header.frame_id = camera_optical_frame_;
  in.point.x = point_cam.x;
  in.point.y = point_cam.y;
  in.point.z = point_cam.z;
  try {
    const auto tf = tf_buffer_->lookupTransform(
      target, camera_optical_frame_, tf2::TimePointZero);
    geometry_msgs::msg::PointStamped out;
    tf2::doTransform(in, out, tf);
    return CartesianVector{out.point.x, out.point.y, out.point.z};
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[TF] point %s -> %s failed: %s",
      camera_optical_frame_.c_str(), target.c_str(), ex.what());
    return std::nullopt;
  }
}

bool VisualServoNode::estimate_grasp_pose_in_reference()
{
  cv::Mat depth;
  cv::Rect2d roi;
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    if (!depth_available_ || latest_depth_frame_.empty()) {
      return false;
    }
    depth = latest_depth_frame_;
  }
  cv::Mat object_mask;
  if (grasp_use_mask_) {
    std::lock_guard<std::mutex> lock(mask_mutex_);
    if (mask_available_ && !latest_mask_frame_.empty() &&
      latest_mask_frame_.rows == depth.rows && latest_mask_frame_.cols == depth.cols)
    {
      object_mask = latest_mask_frame_;  // CV_8UC1, nonzero = object
    }
  }
  roi = tracked_roi_;
  if (roi.width <= 1.0 || roi.height <= 1.0 || !camera_info_received_) {
    return false;
  }

  const auto est = estimate_table_grasp(
    depth, roi, fx_, fy_, cx_, cy_,
    grasp_plane_annulus_frac_,
    grasp_plane_min_depth_m_, grasp_plane_max_depth_m_,
    static_cast<std::size_t>(std::max(1, grasp_plane_min_points_)),
    grasp_plane_max_rms_m_,
    object_mask, gripper_open_position_);
  if (!est.valid) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[ESTIMATE] table-plane fit failed: got %zu pts (need %d) rms=%.4f (need<=%.3f)",
      est.plane_points, grasp_plane_min_points_, est.plane_rms_m, grasp_plane_max_rms_m_);
    return false;
  }

  // A top-down grasp must aim at the narrow neck, which needs the object's
  // measured top height. If the estimate fell back to the bbox-ray (too few
  // object surface points -> object_height == 0), reject it and retry for a
  // frame with real object points, rather than blindly grasping at the body
  // floor height (which closes on the ~gripper-wide body and misses).
  if (grasp_top_down_ && est.object_height_m <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[ESTIMATE] no object height (sparse object points); retrying for a cleaner frame");
    return false;
  }

  // Grasp height: prefer the mask-derived band selector (grasp_height_m) — it
  // picks a height where the object's minor span fits the jaw, generalising a
  // narrow bottle neck and a solid loaf. Fall back to the legacy neck offset
  // (object_top - neck_offset) when no mask/band is available. Never below the
  // body floor.
  double grasp_h = grasp_height_above_table_m_;
  bool used_band = false;
  if (grasp_top_down_ && est.mask_used && est.grasp_height_m > 0.0) {
    // The band selector already chose the graspable height. Clamp only to a
    // small collision clearance above the surface — NOT the body-grasp floor,
    // which would push a flat object's grasp up into the air above it.
    grasp_h = std::max(0.010, est.grasp_height_m);
    used_band = true;
    // Reject if even the best band is wider than the jaw can open.
    if (est.object_width_m > 0.0 &&
      est.object_width_m > gripper_open_position_ - grasp_band_width_margin_m_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[ESTIMATE] object too wide to grasp: minor width=%.3f > jaw=%.3f-%.3f",
        est.object_width_m, gripper_open_position_, grasp_band_width_margin_m_);
      return false;
    }
  } else if (grasp_top_down_ && est.object_height_m > 0.0) {
    grasp_h = std::max(grasp_height_above_table_m_,
      est.object_height_m - neck_grasp_offset_m_);
  }
  RCLCPP_INFO(
    this->get_logger(),
    "[ESTIMATE] object_height=%.3f object_pts=%zu grasp_height=%.3f "
    "band=%s width=%.3f mask=%s",
    est.object_height_m, est.object_points, grasp_h,
    used_band ? "yes" : "no", est.object_width_m, est.mask_used ? "yes" : "no");

  // Grasp point in the camera frame: footprint lifted off the table along the
  // table normal by the chosen grasp height.
  const CartesianVector grasp_cam{
    est.footprint_cam.x + est.up_cam.x * grasp_h,
    est.footprint_cam.y + est.up_cam.y * grasp_h,
    est.footprint_cam.z + est.up_cam.z * grasp_h};

  const auto grasp_ref_opt = transform_point_to_reference(grasp_cam);
  const auto ee_opt = lookup_current_ee_position_in_reference();
  if (!grasp_ref_opt.has_value() || !ee_opt.has_value()) {
    return false;
  }
  CartesianVector grasp_ref = grasp_ref_opt.value();

  // Apply the fixed hand-eye calibration offset (base frame).
  grasp_ref.x += grasp_offset_x_;
  grasp_ref.y += grasp_offset_y_;
  grasp_ref.z += grasp_offset_z_;

  // Reach guard: refuse to command a target the arm cannot reach (horizontal
  // distance from the base origin), so we never strain into a singularity.
  const double reach = std::sqrt(grasp_ref.x * grasp_ref.x + grasp_ref.y * grasp_ref.y);
  if (reach > grasp_max_reach_m_) {
    RCLCPP_WARN(
      this->get_logger(),
      "[ESTIMATE] grasp out of reach: horiz=%.3f > max=%.3f "
      "(grasp x=%.3f y=%.3f) — object too far; reposition closer",
      reach, grasp_max_reach_m_, grasp_ref.x, grasp_ref.y);
    return false;
  }

  CartesianVector pregrasp_ref;
  if (grasp_top_down_) {
    // Top-down grasp: an upright object's XY at any height equals its base
    // footprint XY, so grasp directly over the footprint and descend straight
    // down. Pre-grasp sits a standoff distance ABOVE the grasp point.
    grasp_approach_dir_ref_ = CartesianVector{0.0, 0.0, -1.0};
    pregrasp_ref = CartesianVector{
      grasp_ref.x, grasp_ref.y, grasp_ref.z + pregrasp_standoff_m_};
  } else {
    // Side grasp: approach horizontally; the bbox-bottom ray hits the near face
    // so push one radius further to the object center, and back off the
    // pre-grasp horizontally.
    const CartesianVector ee = ee_opt.value();
    double ax = grasp_ref.x - ee.x;
    double ay = grasp_ref.y - ee.y;
    const double ahyp = std::sqrt(ax * ax + ay * ay);
    if (ahyp < 1e-6) {
      return false;
    }
    ax /= ahyp;
    ay /= ahyp;
    grasp_approach_dir_ref_ = CartesianVector{ax, ay, 0.0};
    grasp_ref.x += ax * grasp_object_radius_m_;
    grasp_ref.y += ay * grasp_object_radius_m_;
    pregrasp_ref = CartesianVector{
      grasp_ref.x - ax * pregrasp_standoff_m_,
      grasp_ref.y - ay * pregrasp_standoff_m_,
      grasp_ref.z};
  }

  grasp_target_ref_ = grasp_ref;
  pregrasp_target_ref_ = pregrasp_ref;
  guarded_at_pregrasp_ = false;

  RCLCPP_INFO(
    this->get_logger(),
    "[ESTIMATE] mode=%s grasp=(%.3f, %.3f, %.3f) pregrasp=(%.3f, %.3f, %.3f) "
    "plane_pts=%zu rms=%.4f footprint_depth=%.3f",
    grasp_top_down_ ? "top_down" : "side",
    grasp_ref.x, grasp_ref.y, grasp_ref.z,
    pregrasp_ref.x, pregrasp_ref.y, pregrasp_ref.z,
    est.plane_points, est.plane_rms_m, est.footprint_depth_m);
  return true;
}

void VisualServoNode::handle_estimate_grasp()
{
  // Hold the arm still and keep the tracker fresh while estimating.
  cv::Mat frame;
  if (fetch_latest_frame(frame)) {
    update_tracking(frame);
    if (publish_overlay_) {
      publish_debug_overlay(frame, tracked_roi_);
    }
  }
  publish_zero_motion(tracking_confidence_);

  // Let the arm settle (no vibration) before sampling depth.
  const double elapsed = (this->now() - state_entry_time_).seconds();
  const double settle_sec =
    static_cast<double>(grasp_estimate_settle_cycles_) / std::max(1.0, control_rate_hz_);
  if (elapsed < settle_sec) {
    return;
  }

  if (estimate_grasp_pose_in_reference()) {
    estimate_attempts_ = 0;
    transition_to(ServoState::OPEN_GRIPPER, "grasp pose estimated");
    return;
  }

  if (++estimate_attempts_ >= grasp_estimate_max_attempts_) {
    estimate_attempts_ = 0;
    RCLCPP_WARN(
      this->get_logger(),
      "[ESTIMATE] giving up after %d attempts; releasing target",
      grasp_estimate_max_attempts_);
    transition_to(ServoState::LOST, "grasp estimate failed");
  }
}

void VisualServoNode::handle_guarded_approach()
{
  if (!grasp_target_ref_.has_value() || !pregrasp_target_ref_.has_value()) {
    transition_to(ServoState::LOST, "guarded approach without target");
    return;
  }

  // Safety timeout: never push indefinitely.
  const double elapsed = (this->now() - state_entry_time_).seconds();
  if (elapsed > 30.0) {
    publish_zero_motion(tracking_confidence_);
    transition_to(ServoState::LOST, "guarded approach timeout");
    return;
  }

  const auto ee_opt = lookup_current_ee_position_in_reference();
  if (!ee_opt.has_value()) {
    publish_zero_motion(tracking_confidence_);
    return;
  }
  const CartesianVector ee = ee_opt.value();
  const CartesianVector target =
    guarded_at_pregrasp_ ? grasp_target_ref_.value() : pregrasp_target_ref_.value();

  const double dx = target.x - ee.x;
  const double dy = target.y - ee.y;
  const double dz = target.z - ee.z;
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "[GUARDED] phase=%s ee=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) dist=%.3f tol=%.3f",
    guarded_at_pregrasp_ ? "grasp" : "pregrasp",
    ee.x, ee.y, ee.z, target.x, target.y, target.z, dist, guarded_reach_tolerance_m_);

  if (dist <= guarded_reach_tolerance_m_) {
    if (!guarded_at_pregrasp_) {
      guarded_at_pregrasp_ = true;
      state_entry_time_ = this->now();  // reset timeout for the descent leg
      RCLCPP_INFO(this->get_logger(), "[GUARDED] reached pre-grasp; descending to grasp");
      publish_zero_motion(tracking_confidence_);
      return;
    }
    publish_zero_motion(tracking_confidence_);
    transition_to(ServoState::CLOSE_GRIPPER, "reached grasp pose");
    return;
  }

  double scale;
  if (grasp_use_move_group_) {
    // Publish the FULL delta: the adapter (move_group + delta mode) plans
    // current+delta and executes a non-singular trajectory. Re-publishing each
    // cycle is harmless — the adapter ignores new goals while one is active.
    scale = 1.0;
  } else {
    // Servo: the adapter converts an EEF-delta to a velocity by dividing by its
    // command horizon, so the delta must be speed*horizon (not speed*cycle_dt).
    const double horizon = std::max(0.1, output_delta_horizon_sec_);
    const double step = std::min(dist, guarded_approach_speed_mps_ * horizon);
    scale = step / dist;
  }
  publish_reference_frame_delta(dx * scale, dy * scale, dz * scale, tracking_confidence_);
}

void VisualServoNode::handle_approach_depth()
{
  if (standoff_blind_active_) {
    handle_standoff_blind_push();
    return;
  }

  cv::Mat frame;
  const bool has_fresh_frame = fetch_latest_frame(frame);
  if (has_fresh_frame) {
    if (!update_tracking(frame)) {
      return;
    }
  } else if (!fallback_to_detection_tracking("APPROACH")) {
    return;
  }

  const double feat_x = tracked_roi_.x + tracked_roi_.width * 0.5;
  const double feat_y = tracked_roi_.y + tracked_roi_.height * 0.5;

  std::optional<DepthSample> depth_sample;
  rclcpp::Time depth_sample_stamp(0, 0, this->get_clock()->get_clock_type());
  bool depth_available_snapshot = false;
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    depth_available_snapshot = depth_available_;
    if (use_depth_ && depth_available_) {
      const double depth_age_sec = std::abs((this->now() - last_depth_receive_time_).seconds());
      if (depth_age_sec <= depth_stale_timeout_sec_) {
        depth_sample = sample_depth_at_roi_anchor(
          latest_depth_frame_, tracked_roi_, depth_sample_anchor_x_, depth_sample_anchor_y_,
          depth_roi_half_size_px_, static_cast<std::size_t>(std::max(1, min_valid_depth_pixels_)),
          depth_sample_max_iqr_m_);
        if (depth_sample.has_value()) {
          depth_sample_stamp = latest_depth_stamp_;
        }
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[DEPTH] stale by %.3fs; holding forward motion", depth_age_sec);
      }
    }
  }

  if (!use_depth_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "[DEPTH] use_depth=false; depth-guided approach disabled");
  }

  if (!standoff_blind_active_ && depth_sample.has_value()) {
    last_depth_sample_ = depth_sample;
    const bool depth_in_band = depth_within_standoff(
      depth_sample->depth_m, grasp_standoff_m_, grasp_depth_tolerance_m_);
    const auto close_update = update_centering_streak(
      close_depth_streak_, depth_in_band, close_depth_stable_frames_);
    close_depth_streak_ = close_update.streak;
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 500,
      "[APPROACH] depth=%.3fm iqr=%.3f valid_px=%zu standoff=%.3f "
      "tol=%.3f anchor=(%d,%d) roi=(%d,%d,%dx%d) close_streak=%d/%d",
      depth_sample->depth_m, depth_sample->depth_iqr_m, depth_sample->valid_pixels,
      grasp_standoff_m_, grasp_depth_tolerance_m_,
      depth_sample->anchor_px, depth_sample->anchor_py,
      depth_sample->sampled_roi.x, depth_sample->sampled_roi.y,
      depth_sample->sampled_roi.width, depth_sample->sampled_roi.height,
      close_depth_streak_, close_depth_stable_frames_);

    if (depth_in_band && close_update.stable) {
      if (blind_approach_after_standoff_m_ > 0.0) {
        if (!start_standoff_blind_push(depth_sample->depth_m)) {
          publish_zero_motion(tracking_confidence_);
          transition_to(ServoState::LOST, "failed to initialize blind push");
          if (publish_overlay_ && has_fresh_frame) {
            publish_debug_overlay(frame, tracked_roi_);
          }
          return;
        }
        handle_standoff_blind_push(&frame);
        return;
      } else {
        // No blind push configured — close immediately
        publish_zero_motion(tracking_confidence_);
        transition_to(ServoState::CLOSE_GRIPPER, "depth reached grasp band");
        if (publish_overlay_ && has_fresh_frame) {
          publish_debug_overlay(frame, tracked_roi_);
        }
        return;
      }
    }

    if (!standoff_blind_active_ && depth_in_band) {
      // The gripper was already opened in OPEN_GRIPPER. Do not keep re-sending the
      // same open command while waiting for depth stability, or the adapter will
      // periodically issue fresh trajectory goals during approach.
      publish_policy_output(geometry_msgs::msg::Twist(), tracking_confidence_);
      if (publish_overlay_ && has_fresh_frame) {
        publish_debug_overlay(frame, tracked_roi_);
      }
      return;
    }
  } else if (!standoff_blind_active_) {
    // Only reset close streak — preserve last_depth_sample_ for blind approach
    // fallback and depth_progress_history_ for stall detection continuity.
    close_depth_streak_ = 0;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 500,
      "[DEPTH] sample failed: depth_available=%s "
      "roi=(%.0f,%.0f,%.0fx%.0f) anchor=(%.2f,%.2f) min_valid_px=%d "
      "half_size=%d max_iqr=%.3f",
      depth_available_snapshot ? "true" : "false",
      tracked_roi_.x, tracked_roi_.y, tracked_roi_.width, tracked_roi_.height,
      depth_sample_anchor_x_, depth_sample_anchor_y_,
      min_valid_depth_pixels_, depth_roi_half_size_px_, depth_sample_max_iqr_m_);
  }

  geometry_msgs::msg::Twist twist = compute_alignment_twist(feat_x, feat_y);
  if (!standoff_blind_active_ && depth_sample.has_value()) {
    twist = compute_approach_twist(feat_x, feat_y, depth_sample->depth_m);

    if (depth_sample_stamp.nanoseconds() > 0) {
      if (!depth_progress_history_.empty() &&
        depth_progress_history_.back().stamp == depth_sample_stamp)
      {
        depth_progress_history_.back().depth_m = depth_sample->depth_m;
      } else {
        depth_progress_history_.push_back({depth_sample_stamp, depth_sample->depth_m});
      }

      const double stall_window_sec = std::max(0.0, approach_stall_window_sec_);
      if (stall_window_sec > 0.0) {
        const auto window = rclcpp::Duration::from_seconds(stall_window_sec);
        while (depth_progress_history_.size() > 1 &&
          (depth_sample_stamp - depth_progress_history_.front().stamp) > window)
        {
          depth_progress_history_.pop_front();
        }

        if (twist.linear.z > 1e-4 && !depth_progress_history_.empty() &&
          (depth_sample_stamp - depth_progress_history_.front().stamp).seconds() >=
          stall_window_sec &&
          depth_progress_stalled(
            depth_progress_history_.front().depth_m, depth_sample->depth_m,
            approach_min_progress_m_))
        {
          const double observed_progress =
            depth_progress_history_.front().depth_m - depth_sample->depth_m;
          RCLCPP_WARN(
            this->get_logger(),
            "[APPROACH] aborting pick: depth progress stalled (%.3fm over %.2fs, min %.3fm)",
            observed_progress, stall_window_sec, approach_min_progress_m_);
          publish_zero_motion(tracking_confidence_);
          transition_to(ServoState::LOST, "depth progress stalled");
          return;
        }
      }
    }

    // Accumulate actual distance per control cycle (not per output horizon).
    // twist.linear.z is a velocity in m/s; each cycle is 1/control_rate_hz seconds.
    const double cycle_dt = 1.0 / std::max(1.0, control_rate_hz_);
    accumulated_approach_distance_m_ += std::max(0.0, twist.linear.z * cycle_dt);
    if (accumulated_approach_distance_m_ > max_approach_distance_m_) {
      RCLCPP_WARN(
        this->get_logger(),
        "[APPROACH] aborting pick: distance %.3fm exceeded limit %.3fm",
        accumulated_approach_distance_m_, max_approach_distance_m_);
      publish_zero_motion(tracking_confidence_);
      transition_to(ServoState::LOST, "approach distance limit exceeded");
      return;
    }
  }

  // If depth has never been available during this approach phase and the stall
  // window has elapsed, synthesize a starting depth so blind approach can begin.
  if (!last_depth_sample_.has_value() && !depth_sample.has_value() &&
    !standoff_blind_active_)
  {
    const double approach_elapsed =
      (this->now() - state_entry_time_).seconds();
    if (approach_elapsed >= approach_stall_window_sec_) {
      DepthSample synthetic;
      synthetic.depth_m = blind_approach_depth_threshold_m_ - 0.001;
      synthetic.valid_pixels = 0;
      synthetic.depth_iqr_m = 0.0;
      last_depth_sample_ = synthetic;
      RCLCPP_WARN(
        this->get_logger(),
        "[APPROACH] no depth samples after %.1fs; starting blind approach "
        "from assumed depth %.3fm",
        approach_elapsed, synthetic.depth_m);
    }
  }

  const bool blind_approach_eligible = last_depth_sample_.has_value() &&
    last_depth_sample_->depth_m<blind_approach_depth_threshold_m_ &&
      last_depth_sample_->depth_m> grasp_standoff_m_;
  if (!depth_sample.has_value() && blind_approach_eligible) {
    // Blind forward approach: depth is unavailable but last reading was close.
    // Continue forward at reduced velocity, dead-reckoning the remaining distance.
    const double blind_vel = blind_approach_velocity_fraction_ * max_linear_velocity_;
    twist.linear.z = blind_vel;

    const double cycle_dt = 1.0 / std::max(1.0, control_rate_hz_);
    accumulated_approach_distance_m_ += blind_vel * cycle_dt;
    blind_approach_distance_m_ += blind_vel * cycle_dt;

    if (blind_approach_distance_m_ > blind_approach_max_distance_m_) {
      RCLCPP_WARN(
        this->get_logger(),
        "[APPROACH] aborting pick: blind distance %.3fm exceeded limit %.3fm",
        blind_approach_distance_m_, blind_approach_max_distance_m_);
      publish_zero_motion(tracking_confidence_);
      transition_to(ServoState::LOST, "blind approach distance limit exceeded");
      return;
    }

    const double est_depth = last_depth_sample_->depth_m - blind_approach_distance_m_;
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[APPROACH] blind last_depth=%.3f blind_dist=%.3f est_depth=%.3f "
      "standoff=%.3f tol=%.3f close_streak=%d/%d",
      last_depth_sample_->depth_m, blind_approach_distance_m_, est_depth,
      grasp_standoff_m_, grasp_depth_tolerance_m_,
      close_depth_streak_, close_depth_stable_frames_);

    if (est_depth <= grasp_standoff_m_ + grasp_depth_tolerance_m_) {
      close_depth_streak_++;
      if (close_depth_streak_ >= close_depth_stable_frames_) {
        publish_zero_motion(tracking_confidence_);
        transition_to(ServoState::CLOSE_GRIPPER, "blind approach estimate reached grasp band");
        if (publish_overlay_) {
          publish_debug_overlay(frame, tracked_roi_);
        }
        return;
      }
    }
  }

  // Keep moving with the gripper state latched from OPEN_GRIPPER. Avoid reasserting
  // the same open command every control cycle during approach.
  publish_policy_output(twist, tracking_confidence_);
  if (publish_overlay_ && has_fresh_frame) {
    publish_debug_overlay(frame, tracked_roi_);
  }
}

void VisualServoNode::handle_close_gripper()
{
  ensure_gripper_goal_started(ServoState::CLOSE_GRIPPER);

  bool action_completed = false;
  bool action_succeeded = false;
  bool action_stalled = false;
  bool action_reached_goal = false;
  std::string action_error;
  {
    std::lock_guard<std::mutex> lock(gripper_action_mutex_);
    action_completed = gripper_goal_completed_;
    action_succeeded = gripper_goal_succeeded_;
    action_stalled = gripper_goal_stalled_;
    action_reached_goal = gripper_goal_reached_goal_;
    action_error = gripper_goal_error_;
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "[GRIPPER][CLOSE] target=%.4f action=%s stalled=%s reached_goal=%s settle=%.2f/%.2f",
    gripper_closed_position_,
    action_succeeded ? "succeeded" : (action_completed ? action_error.c_str() : "pending"),
    action_stalled ? "true" : "false",
    action_reached_goal ? "true" : "false",
    (this->now() - state_entry_time_).seconds(), grasp_settle_sec_);

  publish_policy_output(geometry_msgs::msg::Twist(), tracking_confidence_, false, false, 0.0);

  // Wait grasp_settle_sec before lifting even if the gripper action reports
  // success immediately (the piper gripper action returns on command accept,
  // not on physical close) so the fingers actually grip the object first.
  const double close_elapsed = (this->now() - state_entry_time_).seconds();
  if (action_succeeded && close_elapsed >= grasp_settle_sec_) {
    RCLCPP_INFO(
      this->get_logger(),
      "[GRIPPER][CLOSE] close complete + settled; lifting the grasped object");
    transition_to(ServoState::LIFT, "close gripper action completed; lifting");
    return;
  }

  if ((this->now() - state_entry_time_).seconds() >= grasp_settle_sec_) {
    if (action_completed && !action_succeeded) {
      RCLCPP_WARN(
        this->get_logger(),
        "[GRIPPER][CLOSE] settle elapsed after action error (%s); continuing to lift",
        action_error.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "[GRIPPER][CLOSE] settle elapsed before close action completed; continuing to lift");
    }
    transition_to(ServoState::LIFT, "close command settled; lifting");
  }
}

void VisualServoNode::handle_lift()
{
  const double cycle_dt = 1.0 / std::max(1.0, control_rate_hz_);
  const double delta_horizon_sec = output_delta_horizon_sec_ > 0.0 ?
    output_delta_horizon_sec_ : cycle_dt;
  const double remaining = std::max(0.0, lift_distance_m_ - accumulated_lift_distance_m_);

  if (remaining <= 1e-6) {
    publish_zero_motion(tracking_confidence_);
    transition_to(ServoState::DONE, "lift completed");
    return;
  }

  // Send a delta scaled to the adapter horizon, but only accumulate the
  // per-cycle portion so the lift distance tracks real progress.
  const double velocity = std::min(max_linear_velocity_, remaining / cycle_dt);
  const double dz = velocity * delta_horizon_sec;
  publish_reference_frame_delta(0.0, 0.0, dz, tracking_confidence_, false, 0.0);
  accumulated_lift_distance_m_ += velocity * cycle_dt;
}

void VisualServoNode::handle_done()
{
  publish_policy_output(geometry_msgs::msg::Twist(), tracking_confidence_, false, false, 0.0);
  // Autonomous tuning: after holding briefly in DONE, fall back to IDLE so the
  // next grasp_enabled=true starts a fresh attempt (gated in handle_idle).
  grasp_auto_loop_ = this->get_parameter("grasp_auto_loop").as_bool();
  if (grasp_auto_loop_) {
    const double elapsed = (this->now() - state_entry_time_).seconds();
    if (elapsed >= std::max(0.5, grasp_settle_sec_)) {
      transition_to(ServoState::IDLE, "auto-loop: ready for next attempt");
    }
  }
}

void VisualServoNode::handle_lost()
{
  tracker_initialized_ = false;
  centering_streak_ = 0;
  close_depth_streak_ = 0;
  last_depth_sample_.reset();
  depth_progress_history_.clear();
  publish_zero_motion(0.0F);

  std::lock_guard<std::mutex> lock(detection_mutex_);
  if (detection_available_) {
    transition_to(ServoState::ACQUIRE, "new detection available");
  }
}

std::optional<size_t> VisualServoNode::find_joint_state_index(
  const sensor_msgs::msg::JointState & joint_state, const std::string & joint_name) const
{
  const auto is_gripper_alias = [](const std::string & name) {
      return name == "gripper" || name == "joint7" || name == "piper_joint7";
    };

  for (size_t i = 0; i < joint_state.name.size(); ++i) {
    if (joint_state.name[i] == joint_name) {
      return i;
    }
  }

  if (!is_gripper_alias(joint_name)) {
    return std::nullopt;
  }

  for (size_t i = 0; i < joint_state.name.size(); ++i) {
    if (is_gripper_alias(joint_state.name[i])) {
      return i;
    }
  }

  return std::nullopt;
}

std::optional<double> VisualServoNode::max_gripper_open_error()
{
  if (gripper_joint_names_.empty() || gripper_open_positions_.empty()) {
    return std::nullopt;
  }

  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  if (!joint_state_available_) {
    return std::nullopt;
  }

  const size_t position_count =
    std::min(latest_joint_state_.name.size(), latest_joint_state_.position.size());
  if (position_count == 0) {
    return std::nullopt;
  }

  double max_error = 0.0;
  for (size_t i = 0; i < gripper_joint_names_.size(); ++i) {
    const auto idx = find_joint_state_index(latest_joint_state_, gripper_joint_names_[i]);
    if (!idx.has_value() || *idx >= position_count) {
      return std::nullopt;
    }
    const double actual = latest_joint_state_.position[*idx];
    max_error = std::max(max_error, std::abs(actual - gripper_open_positions_[i]));
  }

  return max_error;
}

bool VisualServoNode::init_tracker(const cv::Mat & frame, const cv::Rect2d & roi)
{
  if (tracker_type_ != "mil" && tracker_type_ != "MIL") {
    RCLCPP_WARN_ONCE(
      this->get_logger(),
      "[TRACKER] tracker_type='%s' unsupported in this build; using MIL",
      tracker_type_.c_str());
  }
  cv_tracker_ = cv::TrackerMIL::create();

  const cv::Rect roi_int(
    cvRound(roi.x),
    cvRound(roi.y),
    cvRound(roi.width),
    cvRound(roi.height));
  if (roi_int.width <= 0 || roi_int.height <= 0) {
    RCLCPP_WARN(
      this->get_logger(), "[TRACKER] init skipped: invalid ROI [%.1f, %.1f, %.1f, %.1f]",
      roi.x, roi.y, roi.width, roi.height);
    tracker_initialized_ = false;
    return false;
  }

  try {
    cv_tracker_->init(frame, roi_int);
    tracker_initialized_ = true;
    tracking_confidence_ = 1.0F;
    return true;
  } catch (const cv::Exception & e) {
    RCLCPP_WARN(this->get_logger(), "[TRACKER] init failed: %s", e.what());
    tracker_initialized_ = false;
    return false;
  }
}

bool VisualServoNode::update_tracker(const cv::Mat & frame, cv::Rect2d & tracked_roi)
{
  if (!tracker_initialized_ || !cv_tracker_) {
    return false;
  }

  try {
    cv::Rect tracked_roi_int(
      cvRound(tracked_roi_.x),
      cvRound(tracked_roi_.y),
      cvRound(tracked_roi_.width),
      cvRound(tracked_roi_.height));
    const bool ok = cv_tracker_->update(frame, tracked_roi_int);
    if (ok) {
      tracked_roi = cv::Rect2d(
        static_cast<double>(tracked_roi_int.x),
        static_cast<double>(tracked_roi_int.y),
        static_cast<double>(tracked_roi_int.width),
        static_cast<double>(tracked_roi_int.height));
      if (tracked_roi.x >= 0 && tracked_roi.y >= 0 &&
        tracked_roi.x + tracked_roi.width <= frame.cols &&
        tracked_roi.y + tracked_roi.height <= frame.rows &&
        tracked_roi.width > 5 && tracked_roi.height > 5)
      {
        tracking_confidence_ = 0.8F;
        return true;
      }
    }
    tracking_confidence_ = 0.0F;
    return false;
  } catch (const cv::Exception & e) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[TRACKER] update failed: %s", e.what());
    tracking_confidence_ = 0.0F;
    return false;
  }
}

void VisualServoNode::apply_ramp(geometry_msgs::msg::Twist & twist)
{
  if (ramp_step_ < ramp_up_steps_ && ramp_up_steps_ > 0) {
    const double ramp_factor = static_cast<double>(ramp_step_ + 1) /
      static_cast<double>(ramp_up_steps_);
    twist.linear.x *= ramp_factor;
    twist.linear.y *= ramp_factor;
    twist.linear.z *= ramp_factor;
    twist.angular.x *= ramp_factor;
    twist.angular.y *= ramp_factor;
    twist.angular.z *= ramp_factor;
    ++ramp_step_;
  }
}

geometry_msgs::msg::Twist VisualServoNode::compute_alignment_twist(
  double feat_x, double feat_y, bool allow_ramp)
{
  geometry_msgs::msg::Twist twist;

  double norm_err_x = 0.0;
  double norm_err_y = 0.0;
  if (image_width_ > 0 && image_height_ > 0) {
    norm_err_x = (feat_x - desired_x_) / static_cast<double>(image_width_);
    norm_err_y = (feat_y - desired_y_) / static_cast<double>(image_height_);
  }

  twist.linear.x =
    clamp_value(lambda_xy_ * norm_err_x, -max_linear_velocity_, max_linear_velocity_);
  twist.linear.y =
    clamp_value(lambda_xy_ * norm_err_y, -max_linear_velocity_, max_linear_velocity_);
  if (allow_ramp) {
    apply_ramp(twist);
  }
  return twist;
}

geometry_msgs::msg::Twist VisualServoNode::compute_approach_twist(
  double feat_x, double feat_y, double depth_m, bool allow_ramp)
{
  auto twist = compute_alignment_twist(feat_x, feat_y, false);
  twist.linear.z = compute_depth_velocity_mps(
    depth_m, grasp_standoff_m_, lambda_z_, max_linear_velocity_);
  if (allow_ramp) {
    apply_ramp(twist);
  }
  return twist;
}

geometry_msgs::msg::Pose VisualServoNode::twist_to_eef_delta(
  const geometry_msgs::msg::Twist & twist, double dt)
{
  geometry_msgs::msg::Pose delta;
  delta.position.x = twist.linear.x * dt;
  delta.position.y = twist.linear.y * dt;
  delta.position.z = twist.linear.z * dt;

  const double half_ax = twist.angular.x * dt * 0.5;
  const double half_ay = twist.angular.y * dt * 0.5;
  const double half_az = twist.angular.z * dt * 0.5;
  delta.orientation.x = half_ax;
  delta.orientation.y = half_ay;
  delta.orientation.z = half_az;
  delta.orientation.w = 1.0;

  const double norm = std::sqrt(
    delta.orientation.x * delta.orientation.x +
    delta.orientation.y * delta.orientation.y +
    delta.orientation.z * delta.orientation.z +
    delta.orientation.w * delta.orientation.w);
  if (norm > 1e-12) {
    delta.orientation.x /= norm;
    delta.orientation.y /= norm;
    delta.orientation.z /= norm;
    delta.orientation.w /= norm;
  }

  return delta;
}

geometry_msgs::msg::Twist VisualServoNode::transform_twist_to_reference(
  const geometry_msgs::msg::Twist & twist)
{
  geometry_msgs::msg::Twist output_twist = twist;

  if (camera_optical_frame_.empty() || reference_frame_.empty() ||
    camera_optical_frame_ == reference_frame_)
  {
    return output_twist;
  }

  try {
    auto transform = tf_buffer_->lookupTransform(
      reference_frame_, camera_optical_frame_, tf2::TimePointZero);
    const auto & q = transform.transform.rotation;
    auto rotate_vec = [&q](double vx, double vy, double vz, double & ox, double & oy, double & oz) {
        const double qx = q.x;
        const double qy = q.y;
        const double qz = q.z;
        const double qw = q.w;
        const double t2 = qw * vx + qy * vz - qz * vy;
        const double t3 = qw * vy + qz * vx - qx * vz;
        const double t4 = qw * vz + qx * vy - qy * vx;
        const double t5 = -qx * vx - qy * vy - qz * vz;
        ox = t2 * qw - t5 * qx - t3 * qz + t4 * qy;
        oy = t3 * qw - t5 * qy - t4 * qx + t2 * qz;
        oz = t4 * qw - t5 * qz - t2 * qy + t3 * qx;
      };

    rotate_vec(
      twist.linear.x, twist.linear.y, twist.linear.z,
      output_twist.linear.x, output_twist.linear.y, output_twist.linear.z);
    rotate_vec(
      twist.angular.x, twist.angular.y, twist.angular.z,
      output_twist.angular.x, output_twist.angular.y, output_twist.angular.z);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "[TF] lookup %s -> %s failed: %s, publishing in camera frame",
      camera_optical_frame_.c_str(), reference_frame_.c_str(), ex.what());
  }

  return output_twist;
}

void VisualServoNode::publish_policy_output(
  const geometry_msgs::msg::Twist & twist,
  float confidence,
  bool include_arm_target,
  bool gripper_active,
  double gripper_command)
{
  manipulation_msgs::msg::PolicyOutput msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = reference_frame_;
  msg.reference_frame = reference_frame_;
  msg.confidence = confidence;

  if (include_arm_target) {
    const double delta_horizon_sec = output_delta_horizon_sec_ > 0.0 ?
      output_delta_horizon_sec_ : 1.0 / std::max(1.0, control_rate_hz_);
    const auto output_twist = transform_twist_to_reference(twist);
    msg.has_eef_target = true;
    msg.eef_target_pose = twist_to_eef_delta(output_twist, delta_horizon_sec);
  } else {
    msg.has_eef_target = false;
  }

  msg.has_joint_deltas = false;
  msg.gripper_active = gripper_active;
  msg.gripper_command = clamp_value(gripper_command, 0.0, 1.0);
  msg.has_base_hint = false;

  policy_output_pub_->publish(msg);
}

void VisualServoNode::publish_reference_frame_delta(
  double dx, double dy, double dz, float confidence, bool gripper_active, double gripper_command)
{
  manipulation_msgs::msg::PolicyOutput msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = reference_frame_;
  msg.reference_frame = reference_frame_;
  msg.confidence = confidence;
  msg.has_eef_target = true;
  msg.eef_target_pose.position.x = dx;
  msg.eef_target_pose.position.y = dy;
  msg.eef_target_pose.position.z = dz;
  msg.eef_target_pose.orientation.w = 1.0;
  msg.has_joint_deltas = false;
  msg.gripper_active = gripper_active;
  msg.gripper_command = clamp_value(gripper_command, 0.0, 1.0);
  msg.has_base_hint = false;
  policy_output_pub_->publish(msg);
}

void VisualServoNode::publish_zero_motion(float confidence)
{
  publish_reference_frame_delta(0.0, 0.0, 0.0, confidence);
}

void VisualServoNode::publish_debug_overlay(const cv::Mat & frame, const cv::Rect2d & roi)
{
  if (!debug_image_pub_) {
    return;
  }

  cv::Mat overlay = frame.clone();
  cv::rectangle(
    overlay,
    cv::Point(static_cast<int>(roi.x), static_cast<int>(roi.y)),
    cv::Point(static_cast<int>(roi.x + roi.width), static_cast<int>(roi.y + roi.height)),
    cv::Scalar(0, 255, 0), 2);

  const cv::Point center(
    static_cast<int>(roi.x + roi.width * 0.5),
    static_cast<int>(roi.y + roi.height * 0.5));
  const cv::Point desired(static_cast<int>(desired_x_), static_cast<int>(desired_y_));
  cv::circle(overlay, center, 5, cv::Scalar(0, 0, 255), -1);
  cv::drawMarker(overlay, desired, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 20, 2);
  cv::line(overlay, center, desired, cv::Scalar(0, 255, 255), 1);

  if (last_depth_sample_.has_value()) {
    cv::rectangle(
      overlay,
      last_depth_sample_->sampled_roi,
      cv::Scalar(255, 255, 0), 1);
    cv::drawMarker(
      overlay,
      cv::Point(last_depth_sample_->anchor_px, last_depth_sample_->anchor_py),
      cv::Scalar(255, 0, 255), cv::MARKER_CROSS, 14, 2);
  }

  const double pixel_error = std::sqrt(
    std::pow(static_cast<double>(center.x - desired.x), 2.0) +
    std::pow(static_cast<double>(center.y - desired.y), 2.0));
  std::string state_text = "State: " + state_to_string(state_);
  std::string err_text = "Err: " + std::to_string(static_cast<int>(pixel_error)) + " px";
  cv::putText(
    overlay, state_text, cv::Point(10, 30),
    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
  cv::putText(
    overlay, err_text, cv::Point(10, 60),
    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

  if (last_depth_sample_.has_value()) {
    const std::string depth_text = "Depth: " + std::to_string(last_depth_sample_->depth_m) + " m";
    const std::string sample_text =
      "Anchor: (" + std::to_string(last_depth_sample_->anchor_px) + "," +
      std::to_string(last_depth_sample_->anchor_py) + ") valid=" +
      std::to_string(last_depth_sample_->valid_pixels) + " iqr=" +
      std::to_string(last_depth_sample_->depth_iqr_m) + " close=" +
      std::to_string(close_depth_streak_) + "/" +
      std::to_string(close_depth_stable_frames_);
    cv::putText(
      overlay, depth_text, cv::Point(10, 90),
      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(
      overlay, sample_text, cv::Point(10, 120),
      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
  }

  sensor_msgs::msg::Image msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = camera_optical_frame_;
  msg.height = static_cast<uint32_t>(overlay.rows);
  msg.width = static_cast<uint32_t>(overlay.cols);
  msg.encoding = sensor_msgs::image_encodings::BGR8;
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(overlay.step);
  msg.data.assign(overlay.datastart, overlay.dataend);
  debug_image_pub_->publish(msg);
}

void VisualServoNode::publish_state()
{
  if (!state_pub_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = state_to_string(state_);
  state_pub_->publish(msg);
}

}  // namespace manipulation_visual_servo

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulation_visual_servo::VisualServoNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
