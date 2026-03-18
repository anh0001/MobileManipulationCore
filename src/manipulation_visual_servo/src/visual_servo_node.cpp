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
#include <cmath>
#include <optional>

#include <opencv2/imgproc.hpp>
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

double deg_to_rad(double degrees)
{
  return degrees * 3.14159265358979323846 / 180.0;
}

bool nearly_equal(double lhs, double rhs, double eps = 1e-6)
{
  return std::abs(lhs - rhs) <= eps;
}

}  // namespace

std::string state_to_string(ServoState state)
{
  switch (state) {
    case ServoState::IDLE: return "IDLE";
    case ServoState::ACQUIRE: return "ACQUIRE";
    case ServoState::ESTIMATE_BOTTLE_3D: return "ESTIMATE_BOTTLE_3D";
    case ServoState::OPEN_GRIPPER: return "OPEN_GRIPPER";
    case ServoState::PLAN_PREGRASP: return "PLAN_PREGRASP";
    case ServoState::EXEC_PREGRASP: return "EXEC_PREGRASP";
    case ServoState::FINAL_SERVO: return "FINAL_SERVO";
    case ServoState::CLOSE_GRIPPER: return "CLOSE_GRIPPER";
    case ServoState::VERIFY_GRASP: return "VERIFY_GRASP";
    case ServoState::LIFT_RETREAT: return "LIFT_RETREAT";
    case ServoState::DONE: return "DONE";
    case ServoState::LOST: return "LOST";
    default: return "UNKNOWN";
  }
}

VisualServoNode::VisualServoNode(const rclcpp::NodeOptions & options)
: Node("visual_servo_node", options)
{
  // Declare parameters — topics
  this->declare_parameter("rgb_topic", "/piper/wrist_camera/piper_d405/color/image_rect_raw");
  this->declare_parameter(
    "camera_info_topic",
    "/piper/wrist_camera/piper_d405/color/camera_info");
  this->declare_parameter("depth_topic", "/piper/wrist_camera/piper_d405/depth/image_rect_raw");
  this->declare_parameter("detection_topic", "/manipulation/target_detections");
  this->declare_parameter("output_topic", "/manipulation/policy_output");
  this->declare_parameter("joint_states_topic", "/joint_states");

  // Declare parameters — frames
  this->declare_parameter("reference_frame", "piper_base_link");
  this->declare_parameter("camera_optical_frame", "piper_camera_optical_frame");
  this->declare_parameter("ee_frame", "piper_tcp");
  this->declare_parameter("arm_base_frame", "piper_base_link");

  // Declare parameters — general
  this->declare_parameter("control_rate_hz", 20.0);
  this->declare_parameter("target_class", "");
  this->declare_parameter("min_detection_confidence", 0.4);

  // Declare parameters — timeouts
  this->declare_parameter("lost_target_timeout_sec", 2.0);
  this->declare_parameter("acquire_timeout_sec", 5.0);
  this->declare_parameter("estimate_timeout_sec", 3.0);
  this->declare_parameter("pregrasp_timeout_sec", 15.0);
  this->declare_parameter("final_servo_timeout_sec", 10.0);
  this->declare_parameter("verify_timeout_sec", 2.0);
  this->declare_parameter("lift_timeout_sec", 10.0);

  // Declare parameters — depth sampling
  this->declare_parameter("depth_roi_body_top_frac", 0.30);
  this->declare_parameter("depth_roi_body_bottom_frac", 0.90);
  this->declare_parameter("depth_roi_body_left_frac", 0.20);
  this->declare_parameter("depth_roi_body_right_frac", 0.80);
  this->declare_parameter("min_valid_depth_pixels", 10);
  this->declare_parameter("depth_sample_max_iqr_m", 0.03);
  this->declare_parameter("depth_stale_timeout_sec", 1.0);

  // Declare parameters — hybrid pick
  this->declare_parameter("pregrasp_offset_m", 0.08);
  this->declare_parameter("eef_link_to_grasp_offset_m", 0.0);
  this->declare_parameter("grasp_approach_axis_x", 0.0);
  this->declare_parameter("grasp_approach_axis_y", 0.0);
  this->declare_parameter("grasp_approach_axis_z", -1.0);
  this->declare_parameter("final_servo_distance_m", 0.04);
  this->declare_parameter("grasp_settle_sec", 1.0);
  this->declare_parameter("lift_distance_m", 0.08);
  this->declare_parameter("retreat_distance_m", 0.05);

  // Declare parameters — grasp orientation template
  this->declare_parameter("object_grasp_roll_deg", 180.0);
  this->declare_parameter("object_grasp_pitch_deg", 0.0);
  this->declare_parameter("object_grasp_yaw_deg", 0.0);
  this->declare_parameter("object_grasp_orientation_x", 1.0);
  this->declare_parameter("object_grasp_orientation_y", 0.0);
  this->declare_parameter("object_grasp_orientation_z", 0.0);
  this->declare_parameter("object_grasp_orientation_w", 0.0);

  // Declare parameters — convergence
  this->declare_parameter("final_position_tolerance_m", 0.008);
  this->declare_parameter("final_image_tolerance_px", 12.0);
  this->declare_parameter("final_convergence_cycles", 3);

  // Declare parameters — workspace guard in arm_base_frame
  this->declare_parameter("workspace_x_min", -0.1);
  this->declare_parameter("workspace_x_max", 0.9);
  this->declare_parameter("workspace_y_min", -0.6);
  this->declare_parameter("workspace_y_max", 0.6);
  this->declare_parameter("workspace_z_min", -0.2);
  this->declare_parameter("workspace_z_max", 1.0);

  // Declare parameters — gripper
  this->declare_parameter("open_gripper_command", 1.0);
  this->declare_parameter("close_gripper_command", 0.0);
  this->declare_parameter("grasp_success_min_width", 0.003);
  this->declare_parameter("gripper_closed_position", 0.0);
  this->declare_parameter("gripper_joint_name", "piper_joint7");

  // Declare parameters — servo control
  this->declare_parameter("servo_control.lambda_xy", 2.0);
  this->declare_parameter("servo_control.lambda_z", 2.0);
  this->declare_parameter("servo_control.max_linear_velocity", 0.05);
  this->declare_parameter("servo_control.max_angular_velocity", 0.20);
  this->declare_parameter("servo_control.ramp_up_steps", 3);

  // Declare parameters — debug
  this->declare_parameter("debug.publish_overlay", true);
  this->declare_parameter("debug.overlay_topic", "/visual_servo/debug_image");
  this->declare_parameter("debug.publish_state", true);
  this->declare_parameter("debug.state_topic", "/visual_servo/state");

  // Read parameters — topics
  rgb_topic_ = this->get_parameter("rgb_topic").as_string();
  camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
  depth_topic_ = this->get_parameter("depth_topic").as_string();
  detection_topic_ = this->get_parameter("detection_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();

  // Read parameters — frames
  reference_frame_ = this->get_parameter("reference_frame").as_string();
  camera_optical_frame_ = this->get_parameter("camera_optical_frame").as_string();
  ee_frame_ = this->get_parameter("ee_frame").as_string();
  arm_base_frame_ = this->get_parameter("arm_base_frame").as_string();

  // Read parameters — general
  control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
  target_class_ = this->get_parameter("target_class").as_string();
  min_detection_confidence_ = this->get_parameter("min_detection_confidence").as_double();

  // Read parameters — timeouts
  lost_target_timeout_sec_ = this->get_parameter("lost_target_timeout_sec").as_double();
  acquire_timeout_sec_ = this->get_parameter("acquire_timeout_sec").as_double();
  estimate_timeout_sec_ = this->get_parameter("estimate_timeout_sec").as_double();
  pregrasp_timeout_sec_ = this->get_parameter("pregrasp_timeout_sec").as_double();
  final_servo_timeout_sec_ = this->get_parameter("final_servo_timeout_sec").as_double();
  verify_timeout_sec_ = this->get_parameter("verify_timeout_sec").as_double();
  lift_timeout_sec_ = this->get_parameter("lift_timeout_sec").as_double();

  // Read parameters — depth sampling
  depth_roi_body_top_frac_ = this->get_parameter("depth_roi_body_top_frac").as_double();
  depth_roi_body_bottom_frac_ = this->get_parameter("depth_roi_body_bottom_frac").as_double();
  depth_roi_body_left_frac_ = this->get_parameter("depth_roi_body_left_frac").as_double();
  depth_roi_body_right_frac_ = this->get_parameter("depth_roi_body_right_frac").as_double();
  min_valid_depth_pixels_ = this->get_parameter("min_valid_depth_pixels").as_int();
  depth_sample_max_iqr_m_ = this->get_parameter("depth_sample_max_iqr_m").as_double();
  depth_stale_timeout_sec_ = this->get_parameter("depth_stale_timeout_sec").as_double();

  // Read parameters — hybrid pick
  pregrasp_offset_m_ = this->get_parameter("pregrasp_offset_m").as_double();
  eef_link_to_grasp_offset_m_ = this->get_parameter("eef_link_to_grasp_offset_m").as_double();
  grasp_approach_axis_x_ = this->get_parameter("grasp_approach_axis_x").as_double();
  grasp_approach_axis_y_ = this->get_parameter("grasp_approach_axis_y").as_double();
  grasp_approach_axis_z_ = this->get_parameter("grasp_approach_axis_z").as_double();
  final_servo_distance_m_ = this->get_parameter("final_servo_distance_m").as_double();
  grasp_settle_sec_ = this->get_parameter("grasp_settle_sec").as_double();
  lift_distance_m_ = this->get_parameter("lift_distance_m").as_double();
  retreat_distance_m_ = this->get_parameter("retreat_distance_m").as_double();

  // Read parameters — grasp orientation
  const double object_grasp_roll_deg = this->get_parameter("object_grasp_roll_deg").as_double();
  const double object_grasp_pitch_deg = this->get_parameter("object_grasp_pitch_deg").as_double();
  const double object_grasp_yaw_deg = this->get_parameter("object_grasp_yaw_deg").as_double();
  const double legacy_grasp_qx = this->get_parameter("object_grasp_orientation_x").as_double();
  const double legacy_grasp_qy = this->get_parameter("object_grasp_orientation_y").as_double();
  const double legacy_grasp_qz = this->get_parameter("object_grasp_orientation_z").as_double();
  const double legacy_grasp_qw = this->get_parameter("object_grasp_orientation_w").as_double();

  const bool custom_rpy =
    !nearly_equal(object_grasp_roll_deg, 180.0) ||
    !nearly_equal(object_grasp_pitch_deg, 0.0) ||
    !nearly_equal(object_grasp_yaw_deg, 0.0);
  const bool custom_legacy_quat =
    !nearly_equal(legacy_grasp_qx, 1.0) ||
    !nearly_equal(legacy_grasp_qy, 0.0) ||
    !nearly_equal(legacy_grasp_qz, 0.0) ||
    !nearly_equal(legacy_grasp_qw, 0.0);

  tf2::Quaternion object_grasp_quat;
  if (custom_rpy || !custom_legacy_quat) {
    object_grasp_quat.setRPY(
      deg_to_rad(object_grasp_roll_deg),
      deg_to_rad(object_grasp_pitch_deg),
      deg_to_rad(object_grasp_yaw_deg));
    object_grasp_quat.normalize();
  } else {
    object_grasp_quat.setValue(
      legacy_grasp_qx, legacy_grasp_qy, legacy_grasp_qz, legacy_grasp_qw);
    if (object_grasp_quat.length2() < 1e-12) {
      object_grasp_quat.setRPY(
        deg_to_rad(object_grasp_roll_deg),
        deg_to_rad(object_grasp_pitch_deg),
        deg_to_rad(object_grasp_yaw_deg));
      object_grasp_quat.normalize();
      RCLCPP_WARN(
        this->get_logger(),
        "Deprecated quaternion grasp parameters were invalid; "
        "falling back to Euler grasp parameters.");
    } else {
      object_grasp_quat.normalize();
      RCLCPP_WARN(
        this->get_logger(),
        "Using deprecated quaternion grasp parameters. "
        "Prefer object_grasp_roll_deg / pitch_deg / yaw_deg.");
    }
  }

  object_grasp_orient_x_ = object_grasp_quat.x();
  object_grasp_orient_y_ = object_grasp_quat.y();
  object_grasp_orient_z_ = object_grasp_quat.z();
  object_grasp_orient_w_ = object_grasp_quat.w();

  // Read parameters — convergence
  final_position_tolerance_m_ = this->get_parameter("final_position_tolerance_m").as_double();
  final_image_tolerance_px_ = this->get_parameter("final_image_tolerance_px").as_double();
  final_convergence_cycles_ = this->get_parameter("final_convergence_cycles").as_int();

  // Read parameters — workspace guard in arm_base_frame
  workspace_x_min_ = this->get_parameter("workspace_x_min").as_double();
  workspace_x_max_ = this->get_parameter("workspace_x_max").as_double();
  workspace_y_min_ = this->get_parameter("workspace_y_min").as_double();
  workspace_y_max_ = this->get_parameter("workspace_y_max").as_double();
  workspace_z_min_ = this->get_parameter("workspace_z_min").as_double();
  workspace_z_max_ = this->get_parameter("workspace_z_max").as_double();

  // Read parameters — gripper
  open_gripper_command_ = this->get_parameter("open_gripper_command").as_double();
  close_gripper_command_ = this->get_parameter("close_gripper_command").as_double();
  grasp_success_min_width_ = this->get_parameter("grasp_success_min_width").as_double();
  gripper_closed_position_ = this->get_parameter("gripper_closed_position").as_double();
  gripper_joint_name_ = this->get_parameter("gripper_joint_name").as_string();

  // Read parameters — servo control
  servo_lambda_xy_ = this->get_parameter("servo_control.lambda_xy").as_double();
  servo_lambda_z_ = this->get_parameter("servo_control.lambda_z").as_double();
  servo_max_linear_velocity_ = this->get_parameter("servo_control.max_linear_velocity").as_double();
  servo_max_angular_velocity_ =
    this->get_parameter("servo_control.max_angular_velocity").as_double();
  servo_ramp_up_steps_ = this->get_parameter("servo_control.ramp_up_steps").as_int();

  // Read parameters — debug
  publish_overlay_ = this->get_parameter("debug.publish_overlay").as_bool();
  overlay_topic_ = this->get_parameter("debug.overlay_topic").as_string();
  publish_state_flag_ = this->get_parameter("debug.publish_state").as_bool();
  state_topic_ = this->get_parameter("debug.state_topic").as_string();

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create subscriptions
  rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    rgb_topic_, rclcpp::SensorDataQoS(),
    std::bind(&VisualServoNode::image_callback, this, std::placeholders::_1));

  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    depth_topic_, rclcpp::SensorDataQoS(),
    std::bind(&VisualServoNode::depth_callback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::SensorDataQoS(),
    std::bind(&VisualServoNode::camera_info_callback, this, std::placeholders::_1));

  detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    detection_topic_, 10,
    std::bind(&VisualServoNode::detection_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, 10,
    std::bind(&VisualServoNode::joint_state_callback, this, std::placeholders::_1));

  // Create publishers
  policy_output_pub_ = this->create_publisher<manipulation_msgs::msg::PolicyOutput>(
    output_topic_, 10);

  if (publish_state_flag_) {
    state_pub_ = this->create_publisher<std_msgs::msg::String>(state_topic_, 10);
  }

  if (publish_overlay_) {
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(overlay_topic_, 10);
  }

  // Create control timer
  auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&VisualServoNode::control_timer_callback, this));

  state_entry_time_ = this->now();

  RCLCPP_INFO(
    this->get_logger(),
    "HybridPickNode initialized: rgb=%s depth=%s detections=%s output=%s rate=%.1f Hz",
    rgb_topic_.c_str(), depth_topic_.c_str(), detection_topic_.c_str(), output_topic_.c_str(),
    control_rate_hz_);
  RCLCPP_INFO(
    this->get_logger(),
    "Pick config: eef_to_grasp=%.3f pregrasp_offset=%.3f "
    "approach_axis=[%.3f,%.3f,%.3f] final_servo_dist=%.3f lift=%.3f retreat=%.3f",
    eef_link_to_grasp_offset_m_, pregrasp_offset_m_,
    grasp_approach_axis_x_, grasp_approach_axis_y_, grasp_approach_axis_z_,
    final_servo_distance_m_, lift_distance_m_, retreat_distance_m_);
  RCLCPP_INFO(
    this->get_logger(),
    "Grasp RPY deg: [%.1f, %.1f, %.1f] quat=[%.3f, %.3f, %.3f, %.3f] "
    "pos_tol=%.4f img_tol=%.1f conv_cycles=%d",
    object_grasp_roll_deg, object_grasp_pitch_deg, object_grasp_yaw_deg,
    object_grasp_orient_x_, object_grasp_orient_y_,
    object_grasp_orient_z_, object_grasp_orient_w_,
    final_position_tolerance_m_, final_image_tolerance_px_, final_convergence_cycles_);
  RCLCPP_INFO(
    this->get_logger(),
    "Workspace guard (%s): x=[%.3f,%.3f] y=[%.3f,%.3f] z=[%.3f,%.3f]",
    arm_base_frame_.c_str(),
    workspace_x_min_, workspace_x_max_,
    workspace_y_min_, workspace_y_max_,
    workspace_z_min_, workspace_z_max_);
}

// --- Sensor callbacks ---

void VisualServoNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    cv::Mat converted;
    if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
      const cv::Mat view(
        static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
        const_cast<unsigned char *>(msg->data.data()),
        static_cast<std::size_t>(msg->step));
      converted = view.clone();
    } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      const cv::Mat view(
        static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
        const_cast<unsigned char *>(msg->data.data()),
        static_cast<std::size_t>(msg->step));
      cv::cvtColor(view, converted, cv::COLOR_RGB2BGR);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Unsupported image encoding: %s", msg->encoding.c_str());
      return;
    }
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_frame_ = converted;
    latest_frame_stamp_ = msg->header.stamp;
    frame_available_ = true;
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Image conversion failed: %s", e.what());
  }
}

void VisualServoNode::depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv::Mat depth_image;
  std::string error_message;
  if (!decode_depth_image(*msg, depth_image, &error_message)) {
    if (!depth_encoding_warned_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Depth frame rejected: %s", error_message.c_str());
      depth_encoding_warned_ = true;
    }
    return;
  }
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    latest_depth_frame_ = depth_image;
    latest_depth_stamp_ = msg->header.stamp;
    depth_available_ = true;
  }
  depth_encoding_warned_ = false;
}

void VisualServoNode::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  if (camera_info_received_) {
    return;
  }
  if (msg->k[0] > 0.0 && msg->k[4] > 0.0) {
    intrinsics_.fx = msg->k[0];
    intrinsics_.fy = msg->k[4];
    intrinsics_.cx = msg->k[2];
    intrinsics_.cy = msg->k[5];
    intrinsics_.width = static_cast<int>(msg->width);
    intrinsics_.height = static_cast<int>(msg->height);
    camera_info_received_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Camera intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f [%dx%d]",
      intrinsics_.fx, intrinsics_.fy, intrinsics_.cx, intrinsics_.cy,
      intrinsics_.width, intrinsics_.height);
  }
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
  detection_available_ = true;
  latest_detection_stamp_ = msg->header.stamp;
}

void VisualServoNode::joint_state_callback(
  const sensor_msgs::msg::JointState::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  latest_joint_state_ = msg;
}

// --- Control loop ---

void VisualServoNode::control_timer_callback()
{
  switch (state_) {
    case ServoState::IDLE: handle_idle(); break;
    case ServoState::ACQUIRE: handle_acquire(); break;
    case ServoState::ESTIMATE_BOTTLE_3D: handle_estimate_bottle_3d(); break;
    case ServoState::OPEN_GRIPPER: handle_open_gripper(); break;
    case ServoState::PLAN_PREGRASP: handle_plan_pregrasp(); break;
    case ServoState::EXEC_PREGRASP: handle_exec_pregrasp(); break;
    case ServoState::FINAL_SERVO: handle_final_servo(); break;
    case ServoState::CLOSE_GRIPPER: handle_close_gripper(); break;
    case ServoState::VERIFY_GRASP: handle_verify_grasp(); break;
    case ServoState::LIFT_RETREAT: handle_lift_retreat(); break;
    case ServoState::DONE: handle_done(); break;
    case ServoState::LOST: handle_lost(); break;
  }

  if (publish_state_flag_) {
    publish_state();
  }
}

void VisualServoNode::transition_to(ServoState new_state)
{
  if (new_state == state_) {
    return;
  }
  RCLCPP_INFO(
    this->get_logger(), "State transition: %s -> %s",
    state_to_string(state_).c_str(), state_to_string(new_state).c_str());
  state_ = new_state;
  state_entry_time_ = this->now();
  ramp_step_ = 0;

  if (new_state == ServoState::ACQUIRE) {
    bottle_estimate_.reset();
    convergence_streak_ = 0;
    pregrasp_sent_ = false;
    pregrasp_retry_count_ = 0;
    cached_estimate_roi_ = cv::Rect2d();
    lift_phase_done_ = false;
  } else if (new_state == ServoState::FINAL_SERVO) {
    convergence_streak_ = 0;
  } else if (new_state == ServoState::LIFT_RETREAT) {
    lift_phase_done_ = false;
    get_current_ee_pose(ee_pose_at_lift_start_);
  }
}

// --- Helpers ---

bool VisualServoNode::fetch_latest_detection(cv::Rect2d & roi, float & confidence)
{
  std::lock_guard<std::mutex> lock(detection_mutex_);
  if (!detection_available_) {
    return false;
  }
  roi = latest_detection_roi_;
  confidence = latest_detection_confidence_;
  detection_available_ = false;
  return true;
}

bool VisualServoNode::get_current_ee_pose(geometry_msgs::msg::PoseStamped & pose)
{
  try {
    auto tf = tf_buffer_->lookupTransform(
      arm_base_frame_, ee_frame_, tf2::TimePointZero);
    pose.header.frame_id = arm_base_frame_;
    pose.header.stamp = tf.header.stamp;
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "EE TF lookup failed: %s", ex.what());
    return false;
  }
}

double VisualServoNode::get_gripper_width()
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  if (!latest_joint_state_) {
    return -1.0;
  }
  for (std::size_t i = 0; i < latest_joint_state_->name.size(); ++i) {
    if (latest_joint_state_->name[i] == gripper_joint_name_ &&
      i < latest_joint_state_->position.size())
    {
      return latest_joint_state_->position[i];
    }
  }
  return -1.0;
}

bool VisualServoNode::transform_point_to_arm_base(
  const geometry_msgs::msg::Point & point_camera,
  geometry_msgs::msg::Point & point_arm_base)
{
  try {
    geometry_msgs::msg::PointStamped in;
    in.header.frame_id = camera_optical_frame_;
    in.header.stamp = rclcpp::Time(0);
    in.point = point_camera;

    geometry_msgs::msg::PointStamped out;
    tf_buffer_->transform(in, out, arm_base_frame_);
    point_arm_base = out.point;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Camera->arm_base TF failed: %s", ex.what());
    return false;
  }
}

double VisualServoNode::compute_image_error(const cv::Rect2d & roi)
{
  if (!camera_info_received_) {
    return 1e6;
  }
  const double feat_x = roi.x + roi.width * 0.5;
  const double feat_y = roi.y + roi.height * 0.5;
  const double err_x = feat_x - intrinsics_.cx;
  const double err_y = feat_y - intrinsics_.cy;
  return std::sqrt(err_x * err_x + err_y * err_y);
}

double VisualServoNode::compute_position_residual(
  const geometry_msgs::msg::PoseStamped & current_ee,
  const geometry_msgs::msg::Pose & target)
{
  const double dx = current_ee.pose.position.x - target.position.x;
  const double dy = current_ee.pose.position.y - target.position.y;
  const double dz = current_ee.pose.position.z - target.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool VisualServoNode::is_pose_position_within_workspace(
  const geometry_msgs::msg::Pose & pose) const
{
  return
    pose.position.x >= workspace_x_min_ && pose.position.x <= workspace_x_max_ &&
    pose.position.y >= workspace_y_min_ && pose.position.y <= workspace_y_max_ &&
    pose.position.z >= workspace_z_min_ && pose.position.z <= workspace_z_max_;
}

// --- State handlers ---

void VisualServoNode::handle_idle()
{
  if (!camera_info_received_) {
    return;
  }
  std::lock_guard<std::mutex> lock(detection_mutex_);
  if (detection_available_) {
    transition_to(ServoState::ACQUIRE);
  }
}

void VisualServoNode::handle_acquire()
{
  const double elapsed = (this->now() - state_entry_time_).seconds();
  if (elapsed > acquire_timeout_sec_) {
    RCLCPP_WARN(this->get_logger(), "Acquire timed out after %.1f sec", elapsed);
    transition_to(ServoState::IDLE);
    return;
  }

  cv::Rect2d roi;
  float confidence;
  if (fetch_latest_detection(roi, confidence)) {
    if (roi.width >= 10.0 && roi.height >= 10.0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Target acquired: roi=(%.0f,%.0f,%.0fx%.0f) conf=%.2f",
        roi.x, roi.y, roi.width, roi.height, confidence);
      transition_to(ServoState::ESTIMATE_BOTTLE_3D);
    }
  }
}

void VisualServoNode::handle_estimate_bottle_3d()
{
  const double elapsed = (this->now() - state_entry_time_).seconds();
  if (elapsed > estimate_timeout_sec_) {
    RCLCPP_WARN(this->get_logger(), "3D estimation timed out after %.1f sec", elapsed);
    transition_to(ServoState::LOST);
    return;
  }

  // Get latest detection ROI — keep a cached copy so that depth retries
  // do not have to wait for the next (slow, ~1 Hz) detection message.
  cv::Rect2d det_roi;
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    if (detection_available_) {
      cached_estimate_roi_ = latest_detection_roi_;
      detection_available_ = false;
    }
  }
  if (cached_estimate_roi_.width <= 0.0 || cached_estimate_roi_.height <= 0.0) {
    return;  // No detection yet
  }
  det_roi = cached_estimate_roi_;

  // Get latest depth
  cv::Mat depth;
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    if (!depth_available_) {
      return;
    }
    const double age = std::abs((this->now() - latest_depth_stamp_).seconds());
    if (age > depth_stale_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Depth stale by %.3f sec in ESTIMATE_BOTTLE_3D", age);
      return;
    }
    depth = latest_depth_frame_.clone();
  }

  // Estimate 3D centroid in camera frame
  auto estimate = estimate_bottle_3d(
    depth, det_roi, intrinsics_,
    depth_roi_body_top_frac_, depth_roi_body_bottom_frac_,
    depth_roi_body_left_frac_, depth_roi_body_right_frac_,
    static_cast<std::size_t>(std::max(1, min_valid_depth_pixels_)),
    depth_sample_max_iqr_m_);

  if (!estimate.has_value()) {
    const cv::Rect body_roi = compute_body_roi(
      det_roi, depth_roi_body_top_frac_, depth_roi_body_bottom_frac_,
      depth_roi_body_left_frac_, depth_roi_body_right_frac_,
      depth.cols, depth.rows);
    // Count valid depth pixels for diagnostics
    int valid_px = 0;
    for (int row = body_roi.y; row < body_roi.y + body_roi.height; ++row) {
      const auto * row_ptr = depth.ptr<uint16_t>(row);
      for (int col = body_roi.x; col < body_roi.x + body_roi.width; ++col) {
        if (row_ptr[col] > 0U) {++valid_px;}
      }
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "3D estimation failed: body_roi=(%d,%d,%dx%d) valid_depth_px=%d/%d (need %d) "
      "depth_size=%dx%d det_roi=(%.0f,%.0f,%.0fx%.0f)",
      body_roi.x, body_roi.y, body_roi.width, body_roi.height,
      valid_px, body_roi.area(), min_valid_depth_pixels_,
      depth.cols, depth.rows,
      det_roi.x, det_roi.y, det_roi.width, det_roi.height);
    return;
  }

  // Transform centroid to arm base frame
  geometry_msgs::msg::Point bottle_in_arm_base;
  if (!transform_point_to_arm_base(estimate->centroid_camera, bottle_in_arm_base)) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Bottle 3D estimate: camera=(%.3f,%.3f,%.3f) arm_base=(%.3f,%.3f,%.3f) "
    "valid_px=%zu iqr=%.4f",
    estimate->centroid_camera.x, estimate->centroid_camera.y, estimate->centroid_camera.z,
    bottle_in_arm_base.x, bottle_in_arm_base.y, bottle_in_arm_base.z,
    estimate->valid_pixels, estimate->depth_iqr_m);

  // Synthesize grasp and pre-grasp poses
  const auto bottle_center_pose = make_grasp_pose(
    bottle_in_arm_base,
    object_grasp_orient_x_, object_grasp_orient_y_,
    object_grasp_orient_z_, object_grasp_orient_w_);
  geometry_msgs::msg::Vector3 grasp_approach_axis;
  grasp_approach_axis.x = grasp_approach_axis_x_;
  grasp_approach_axis.y = grasp_approach_axis_y_;
  grasp_approach_axis.z = grasp_approach_axis_z_;

  grasp_pose_ = offset_pose_along_axis(
    bottle_center_pose, grasp_approach_axis, eef_link_to_grasp_offset_m_);
  pregrasp_pose_ = offset_pose_along_axis(
    grasp_pose_, grasp_approach_axis, pregrasp_offset_m_);

  if (!is_pose_position_within_workspace(pregrasp_pose_)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Rejecting 3D estimate outside workspace (%s): bottle=[%.3f,%.3f,%.3f] "
      "pregrasp=[%.3f,%.3f,%.3f] bounds x=[%.2f,%.2f] y=[%.2f,%.2f] z=[%.2f,%.2f]",
      arm_base_frame_.c_str(),
      bottle_center_pose.position.x, bottle_center_pose.position.y, bottle_center_pose.position.z,
      pregrasp_pose_.position.x, pregrasp_pose_.position.y, pregrasp_pose_.position.z,
      workspace_x_min_, workspace_x_max_,
      workspace_y_min_, workspace_y_max_,
      workspace_z_min_, workspace_z_max_);
    transition_to(ServoState::LOST);
    return;
  }

  bottle_estimate_ = estimate;

  RCLCPP_INFO(
    this->get_logger(),
    "Bottle center: [%.3f,%.3f,%.3f] EEF grasp target: [%.3f,%.3f,%.3f] "
    "Pre-grasp: [%.3f,%.3f,%.3f]",
    bottle_center_pose.position.x, bottle_center_pose.position.y, bottle_center_pose.position.z,
    grasp_pose_.position.x, grasp_pose_.position.y, grasp_pose_.position.z,
    pregrasp_pose_.position.x, pregrasp_pose_.position.y, pregrasp_pose_.position.z);

  transition_to(ServoState::OPEN_GRIPPER);
}

void VisualServoNode::handle_open_gripper()
{
  const double elapsed = (this->now() - state_entry_time_).seconds();

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "[OPEN_GRIPPER] cmd=%.2f settle=%.2f/%.2f",
    open_gripper_command_, elapsed, grasp_settle_sec_);

  publish_gripper_command(open_gripper_command_);

  if (elapsed >= grasp_settle_sec_) {
    transition_to(ServoState::PLAN_PREGRASP);
  }
}

void VisualServoNode::handle_plan_pregrasp()
{
  // Send the pre-grasp pose once via MoveGroup
  if (!pregrasp_sent_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Sending pre-grasp pose via MoveGroup: [%.3f,%.3f,%.3f]",
      pregrasp_pose_.position.x, pregrasp_pose_.position.y, pregrasp_pose_.position.z);
    publish_move_group_target(pregrasp_pose_);
    pregrasp_sent_ = true;
    return;
  }

  // Transition immediately to EXEC_PREGRASP — the adapter handles the async execution
  transition_to(ServoState::EXEC_PREGRASP);
}

void VisualServoNode::handle_exec_pregrasp()
{
  const double elapsed = (this->now() - state_entry_time_).seconds();
  if (elapsed > pregrasp_timeout_sec_) {
    RCLCPP_WARN(this->get_logger(), "Pre-grasp execution timed out after %.1f sec", elapsed);
    transition_to(ServoState::LOST);
    return;
  }

  // Check if EE has reached near the pre-grasp position
  geometry_msgs::msg::PoseStamped current_ee;
  if (!get_current_ee_pose(current_ee)) {
    return;
  }

  const double residual = compute_position_residual(current_ee, pregrasp_pose_);
  const double dist_to_grasp = compute_position_residual(current_ee, grasp_pose_);

  const double pregrasp_handoff_distance = std::max(final_servo_distance_m_, 0.005);
  if (residual <= pregrasp_handoff_distance || dist_to_grasp <= final_servo_distance_m_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Pre-grasp handoff reached: pregrasp_residual=%.4f grasp_residual=%.4f "
      "handoff=%.4f -> switching to FINAL_SERVO",
      residual, dist_to_grasp, pregrasp_handoff_distance);
    transition_to(ServoState::FINAL_SERVO);
    return;
  }

  // Stall detection: if the residual hasn't decreased meaningfully, the MoveIt
  // goal likely failed (e.g. planning failure / ABORTED).  Detect this early
  // instead of waiting for the full timeout.
  constexpr double kStallThreshold = 0.002;   // metres
  constexpr double kStallWindowSec = 3.0;     // seconds without progress
  if (elapsed < 0.5) {
    // Seed the baseline on first evaluation
    pregrasp_best_residual_ = residual;
    pregrasp_best_residual_time_ = this->now();
  } else {
    if (residual < pregrasp_best_residual_ - kStallThreshold) {
      pregrasp_best_residual_ = residual;
      pregrasp_best_residual_time_ = this->now();
    }
    const double stall_elapsed =
      (this->now() - pregrasp_best_residual_time_).seconds();
    if (stall_elapsed > kStallWindowSec) {
      RCLCPP_WARN(
        this->get_logger(),
        "Pre-grasp stall detected: residual=%.4f unchanged for %.1f sec "
        "(MoveIt goal may have failed). Retrying.",
        residual, stall_elapsed);
      // Re-send the MoveGroup goal once, then reset the stall timer
      publish_move_group_target(pregrasp_pose_);
      pregrasp_best_residual_ = residual;
      pregrasp_best_residual_time_ = this->now();
      pregrasp_retry_count_++;
      if (pregrasp_retry_count_ > 1) {
        RCLCPP_WARN(
          this->get_logger(),
          "Pre-grasp failed after %d retries, giving up.",
          pregrasp_retry_count_);
        transition_to(ServoState::LOST);
        return;
      }
    }
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1000,
    "[EXEC_PREGRASP] residual=%.4f (threshold=%.4f) ee=[%.3f,%.3f,%.3f] "
    "target=[%.3f,%.3f,%.3f] t=%.1fs%s",
    residual, final_servo_distance_m_,
    current_ee.pose.position.x, current_ee.pose.position.y, current_ee.pose.position.z,
    pregrasp_pose_.position.x, pregrasp_pose_.position.y, pregrasp_pose_.position.z,
    elapsed,
    (residual < final_servo_distance_m_) ? " [CLOSE - switching soon]" : "");
}

void VisualServoNode::handle_final_servo()
{
  const double elapsed = (this->now() - state_entry_time_).seconds();
  if (elapsed > final_servo_timeout_sec_) {
    RCLCPP_WARN(this->get_logger(), "Final servo timed out after %.1f sec", elapsed);
    transition_to(ServoState::LOST);
    return;
  }

  // Get current EE pose
  geometry_msgs::msg::PoseStamped current_ee;
  if (!get_current_ee_pose(current_ee)) {
    return;
  }

  // Compute 3D position error to grasp pose
  geometry_msgs::msg::Point pos_error;
  pos_error.x = grasp_pose_.position.x - current_ee.pose.position.x;
  pos_error.y = grasp_pose_.position.y - current_ee.pose.position.y;
  pos_error.z = grasp_pose_.position.z - current_ee.pose.position.z;
  const double pos_residual = std::sqrt(
    pos_error.x * pos_error.x + pos_error.y * pos_error.y + pos_error.z * pos_error.z);

  // Get latest detection for image-space error
  double image_err = 1e6;
  double img_err_x = 0.0;
  double img_err_y = 0.0;
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    if (detection_available_) {
      image_err = compute_image_error(latest_detection_roi_);
      img_err_x = (latest_detection_roi_.x + latest_detection_roi_.width * 0.5) - intrinsics_.cx;
      img_err_y = (latest_detection_roi_.y + latest_detection_roi_.height * 0.5) - intrinsics_.cy;
      // Don't consume — keep available for overlay
    }
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1000,
    "[FINAL_SERVO] pos_err=%.4f/%.4f dx=%.4f dy=%.4f dz=%.4f | "
    "img_err=%.1f/%.1f | streak=%d/%d | t=%.1fs",
    pos_residual, final_position_tolerance_m_,
    pos_error.x, pos_error.y, pos_error.z,
    image_err, final_image_tolerance_px_,
    convergence_streak_, final_convergence_cycles_,
    elapsed);

  // Check convergence
  const bool pos_ok = pos_residual <= final_position_tolerance_m_;
  const bool img_ok = image_err <= final_image_tolerance_px_;
  const auto conv_update = update_centering_streak(
    convergence_streak_, pos_ok && img_ok, final_convergence_cycles_);
  convergence_streak_ = conv_update.streak;

  if (conv_update.stable) {
    publish_zero_motion();
    transition_to(ServoState::CLOSE_GRIPPER);
    return;
  }

  // Send servo twist toward grasp pose
  publish_servo_twist(pos_error, img_err_x, img_err_y);

  // Debug overlay
  if (publish_overlay_) {
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      if (frame_available_) {
        frame = latest_frame_.clone();
      }
    }
    if (!frame.empty()) {
      std::lock_guard<std::mutex> lock(detection_mutex_);
      if (detection_available_) {
        publish_debug_overlay(frame, latest_detection_roi_);
      }
    }
  }
}

void VisualServoNode::handle_close_gripper()
{
  const double elapsed = (this->now() - state_entry_time_).seconds();

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "[CLOSE_GRIPPER] cmd=%.2f settle=%.2f/%.2f",
    close_gripper_command_, elapsed, grasp_settle_sec_);

  publish_gripper_command(close_gripper_command_);

  if (elapsed >= grasp_settle_sec_) {
    transition_to(ServoState::VERIFY_GRASP);
  }
}

void VisualServoNode::handle_verify_grasp()
{
  const double elapsed = (this->now() - state_entry_time_).seconds();

  const double width = get_gripper_width();

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "[VERIFY_GRASP] gripper_width=%.4f threshold=%.4f elapsed=%.2f",
    width, gripper_closed_position_ + grasp_success_min_width_, elapsed);

  if (width < 0.0) {
    // No joint state yet — wait
    if (elapsed > verify_timeout_sec_) {
      RCLCPP_WARN(this->get_logger(), "Verify timed out: no gripper joint state");
      transition_to(ServoState::LOST);
    }
    return;
  }

  // Success: gripper is open wider than closed + min_width (object between fingers)
  if (width > gripper_closed_position_ + grasp_success_min_width_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Grasp verified: width=%.4f > threshold=%.4f",
      width, gripper_closed_position_ + grasp_success_min_width_);
    transition_to(ServoState::LIFT_RETREAT);
    return;
  }

  // Failure: gripper closed near-fully with no object
  if (elapsed > verify_timeout_sec_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Grasp failed: width=%.4f <= threshold=%.4f — reopening and retreating",
      width, gripper_closed_position_ + grasp_success_min_width_);
    // Reopen gripper
    publish_gripper_command(open_gripper_command_);
    transition_to(ServoState::LOST);
  }
}

void VisualServoNode::handle_lift_retreat()
{
  const double elapsed = (this->now() - state_entry_time_).seconds();
  if (elapsed > lift_timeout_sec_) {
    RCLCPP_WARN(this->get_logger(), "Lift/retreat timed out after %.1f sec", elapsed);
    transition_to(ServoState::DONE);
    return;
  }

  geometry_msgs::msg::PoseStamped current_ee;
  if (!get_current_ee_pose(current_ee)) {
    return;
  }

  if (!lift_phase_done_) {
    // Lift: check if EE Z has increased by lift_distance_m from start
    const double lifted =
      current_ee.pose.position.z - ee_pose_at_lift_start_.pose.position.z;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[LIFT_RETREAT] lift: actual=%.4f target=%.4f",
      lifted, lift_distance_m_);

    if (lifted >= lift_distance_m_) {
      lift_phase_done_ = true;
      RCLCPP_INFO(this->get_logger(), "Lift complete (%.4f m), starting retreat", lifted);
    } else {
      // Command upward delta
      const double dz = std::min(
        lift_distance_m_ - lifted,
        servo_max_linear_velocity_ / std::max(1.0, control_rate_hz_));
      publish_lift_command(dz);
    }
  } else {
    // Retreat: move backward (negative X in arm base frame) by retreat_distance_m
    const double retreated =
      ee_pose_at_lift_start_.pose.position.x - current_ee.pose.position.x;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[LIFT_RETREAT] retreat: actual=%.4f target=%.4f",
      retreated, retreat_distance_m_);

    if (retreated >= retreat_distance_m_ || retreat_distance_m_ <= 0.001) {
      RCLCPP_INFO(this->get_logger(), "Retreat complete (%.4f m)", retreated);
      transition_to(ServoState::DONE);
    } else {
      // Command backward delta
      const double dx = -std::min(
        retreat_distance_m_ - retreated,
        servo_max_linear_velocity_ / std::max(1.0, control_rate_hz_));

      manipulation_msgs::msg::PolicyOutput msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = reference_frame_;
      msg.reference_frame = reference_frame_;
      msg.confidence = 1.0;
      msg.has_eef_target = true;
      msg.eef_target_pose.position.x = dx;
      msg.eef_target_pose.position.y = 0.0;
      msg.eef_target_pose.position.z = 0.0;
      msg.eef_target_pose.orientation.w = 1.0;
      msg.arm_command_mode = "moveit_servo";
      msg.gripper_active = true;
      msg.gripper_command = close_gripper_command_;
      policy_output_pub_->publish(msg);
    }
  }
}

void VisualServoNode::handle_done()
{
  publish_zero_motion();
}

void VisualServoNode::handle_lost()
{
  bottle_estimate_.reset();
  convergence_streak_ = 0;
  pregrasp_sent_ = false;
  publish_zero_motion();

  // Try to re-acquire from fresh detection
  std::lock_guard<std::mutex> lock(detection_mutex_);
  if (detection_available_) {
    transition_to(ServoState::ACQUIRE);
  }
}

// --- Publishing ---

void VisualServoNode::publish_move_group_target(const geometry_msgs::msg::Pose & target_pose)
{
  // The adapter runs with eef_target_is_delta=true, so we must send the
  // delta from the current EE pose to the desired target, not the absolute pose.
  geometry_msgs::msg::PoseStamped current_ee;
  geometry_msgs::msg::Pose delta_pose;
  if (get_current_ee_pose(current_ee)) {
    delta_pose.position.x = target_pose.position.x - current_ee.pose.position.x;
    delta_pose.position.y = target_pose.position.y - current_ee.pose.position.y;
    delta_pose.position.z = target_pose.position.z - current_ee.pose.position.z;

    // Compute delta rotation: q_delta = q_target * q_current^-1
    tf2::Quaternion q_current, q_target;
    tf2::fromMsg(current_ee.pose.orientation, q_current);
    tf2::fromMsg(target_pose.orientation, q_target);
    tf2::Quaternion q_delta = q_target * q_current.inverse();
    q_delta.normalize();
    delta_pose.orientation = tf2::toMsg(q_delta);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Cannot compute delta for MoveGroup target: TF lookup failed");
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[MOVE_GROUP] target=[%.3f,%.3f,%.3f] current_ee=[%.3f,%.3f,%.3f] delta=[%.3f,%.3f,%.3f]",
    target_pose.position.x, target_pose.position.y, target_pose.position.z,
    current_ee.pose.position.x, current_ee.pose.position.y, current_ee.pose.position.z,
    delta_pose.position.x, delta_pose.position.y, delta_pose.position.z);

  manipulation_msgs::msg::PolicyOutput msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = reference_frame_;
  msg.reference_frame = reference_frame_;
  msg.confidence = 1.0;
  msg.has_eef_target = true;
  msg.eef_target_pose = delta_pose;
  msg.has_joint_deltas = false;
  msg.gripper_active = true;
  msg.gripper_command = open_gripper_command_;
  msg.has_base_hint = false;
  msg.arm_command_mode = "move_group_position_only";
  policy_output_pub_->publish(msg);
}

void VisualServoNode::publish_servo_twist(
  const geometry_msgs::msg::Point & position_error,
  double /*image_err_x*/, double /*image_err_y*/)
{
  // Proportional control on 3D position error (arm_base_frame).
  // Image error parameters reserved for future lateral correction refinement.
  const double dt = 1.0 / std::max(1.0, control_rate_hz_);

  double vx = clamp_value(
    servo_lambda_z_ * position_error.x, -servo_max_linear_velocity_, servo_max_linear_velocity_);
  double vy = clamp_value(
    servo_lambda_z_ * position_error.y, -servo_max_linear_velocity_, servo_max_linear_velocity_);
  double vz = clamp_value(
    servo_lambda_z_ * position_error.z, -servo_max_linear_velocity_, servo_max_linear_velocity_);

  // Apply ramp
  if (ramp_step_ < servo_ramp_up_steps_ && servo_ramp_up_steps_ > 0) {
    const double ramp = static_cast<double>(ramp_step_ + 1) /
      static_cast<double>(servo_ramp_up_steps_);
    vx *= ramp;
    vy *= ramp;
    vz *= ramp;
    ++ramp_step_;
  }

  // Convert velocity to delta pose for the adapter
  manipulation_msgs::msg::PolicyOutput msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = reference_frame_;
  msg.reference_frame = reference_frame_;
  msg.confidence = 1.0;
  msg.has_eef_target = true;
  msg.eef_target_pose.position.x = vx * dt;
  msg.eef_target_pose.position.y = vy * dt;
  msg.eef_target_pose.position.z = vz * dt;
  msg.eef_target_pose.orientation.w = 1.0;
  msg.has_joint_deltas = false;
  msg.gripper_active = true;
  msg.gripper_command = open_gripper_command_;
  msg.has_base_hint = false;
  msg.arm_command_mode = "moveit_servo";
  policy_output_pub_->publish(msg);
}

void VisualServoNode::publish_gripper_command(double command)
{
  manipulation_msgs::msg::PolicyOutput msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = reference_frame_;
  msg.reference_frame = reference_frame_;
  msg.confidence = 1.0;
  msg.has_eef_target = false;
  msg.has_joint_deltas = false;
  msg.gripper_active = true;
  msg.gripper_command = clamp_value(command, 0.0, 1.0);
  msg.has_base_hint = false;
  policy_output_pub_->publish(msg);
}

void VisualServoNode::publish_zero_motion()
{
  manipulation_msgs::msg::PolicyOutput msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = reference_frame_;
  msg.reference_frame = reference_frame_;
  msg.confidence = 1.0;
  msg.has_eef_target = true;
  msg.eef_target_pose.orientation.w = 1.0;
  msg.has_joint_deltas = false;
  msg.gripper_active = false;
  msg.has_base_hint = false;
  msg.arm_command_mode = "moveit_servo";
  policy_output_pub_->publish(msg);
}

void VisualServoNode::publish_lift_command(double dz)
{
  manipulation_msgs::msg::PolicyOutput msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = reference_frame_;
  msg.reference_frame = reference_frame_;
  msg.confidence = 1.0;
  msg.has_eef_target = true;
  msg.eef_target_pose.position.x = 0.0;
  msg.eef_target_pose.position.y = 0.0;
  msg.eef_target_pose.position.z = dz;
  msg.eef_target_pose.orientation.w = 1.0;
  msg.has_joint_deltas = false;
  msg.gripper_active = true;
  msg.gripper_command = close_gripper_command_;
  msg.has_base_hint = false;
  msg.arm_command_mode = "moveit_servo";
  policy_output_pub_->publish(msg);
}

void VisualServoNode::publish_debug_overlay(const cv::Mat & frame, const cv::Rect2d & roi)
{
  if (!debug_image_pub_) {
    return;
  }

  cv::Mat overlay = frame.clone();

  // Detection ROI
  cv::rectangle(
    overlay,
    cv::Point(static_cast<int>(roi.x), static_cast<int>(roi.y)),
    cv::Point(static_cast<int>(roi.x + roi.width), static_cast<int>(roi.y + roi.height)),
    cv::Scalar(0, 255, 0), 2);

  // Feature center and image center
  const cv::Point center(
    static_cast<int>(roi.x + roi.width * 0.5),
    static_cast<int>(roi.y + roi.height * 0.5));
  const cv::Point desired(static_cast<int>(intrinsics_.cx), static_cast<int>(intrinsics_.cy));
  cv::circle(overlay, center, 5, cv::Scalar(0, 0, 255), -1);
  cv::drawMarker(overlay, desired, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 20, 2);
  cv::line(overlay, center, desired, cv::Scalar(0, 255, 255), 1);

  // Body ROI if estimate exists
  if (bottle_estimate_.has_value()) {
    cv::rectangle(overlay, bottle_estimate_->body_roi, cv::Scalar(255, 255, 0), 1);
  }

  // State text
  const double pixel_error = compute_image_error(roi);
  const std::string state_text = "State: " + state_to_string(state_);
  const std::string err_text = "ImgErr: " + std::to_string(static_cast<int>(pixel_error)) + " px";
  cv::putText(
    overlay, state_text, cv::Point(10, 30),
    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
  cv::putText(
    overlay, err_text, cv::Point(10, 60),
    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

  if (bottle_estimate_.has_value()) {
    const std::string depth_text =
      "Depth: " + std::to_string(bottle_estimate_->depth_m) + " m";
    cv::putText(
      overlay, depth_text, cv::Point(10, 90),
      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
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
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
