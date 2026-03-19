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
#include <utility>

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
    case ServoState::OPEN_GRIPPER: return "OPEN_GRIPPER";
    case ServoState::APPROACH_DEPTH: return "APPROACH_DEPTH";
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
  this->declare_parameter("open_gripper_command", 1.0);
  this->declare_parameter("open_gripper_settle_sec", 3.0);
  this->declare_parameter("close_gripper_command", 0.0);
  this->declare_parameter("gripper_joint_name", "piper_joint7");
  this->declare_parameter<std::vector<std::string>>(
    "gripper_joint_names", std::vector<std::string>{});
  this->declare_parameter("gripper_open_position", 0.75);
  this->declare_parameter<std::vector<double>>(
    "gripper_open_positions", std::vector<double>{});
  this->declare_parameter("gripper_open_position_tolerance", 0.02);

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
  open_gripper_command_ = this->get_parameter("open_gripper_command").as_double();
  open_gripper_settle_sec_ = this->get_parameter("open_gripper_settle_sec").as_double();
  close_gripper_command_ = this->get_parameter("close_gripper_command").as_double();
  gripper_joint_name_ = this->get_parameter("gripper_joint_name").as_string();
  gripper_joint_names_ = this->get_parameter("gripper_joint_names").as_string_array();
  gripper_open_position_ = this->get_parameter("gripper_open_position").as_double();
  gripper_open_positions_ = this->get_parameter("gripper_open_positions").as_double_array();
  gripper_open_position_tolerance_ =
    this->get_parameter("gripper_open_position_tolerance").as_double();

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

  rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    rgb_topic_, rclcpp::SensorDataQoS(),
    std::bind(&VisualServoNode::image_callback, this, std::placeholders::_1));

  if (use_depth_) {
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VisualServoNode::depth_callback, this, std::placeholders::_1));
  }

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::SensorDataQoS(),
    std::bind(&VisualServoNode::camera_info_callback, this, std::placeholders::_1));

  joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, 10,
    std::bind(&VisualServoNode::joint_states_callback, this, std::placeholders::_1));

  detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    detection_topic_, 10,
    std::bind(&VisualServoNode::detection_callback, this, std::placeholders::_1));

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
    std::bind(&VisualServoNode::control_timer_callback, this));

  state_entry_time_ = this->now();
  last_track_time_ = this->now();
  last_processed_frame_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

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
    "[INIT] Pick | use_depth=%s standoff=%.3f tol=%.3f lift=%.3f max_approach=%.3f",
    use_depth_ ? "true" : "false", grasp_standoff_m_, grasp_depth_tolerance_m_,
    lift_distance_m_, max_approach_distance_m_);
  RCLCPP_INFO(
    this->get_logger(),
    "[INIT] Depth | anchor=(%.2f, %.2f) half_size=%d min_valid=%d max_iqr=%.3f "
    "close_stable=%d stall_window=%.2fs min_progress=%.3f",
    depth_sample_anchor_x_, depth_sample_anchor_y_, depth_roi_half_size_px_,
    min_valid_depth_pixels_, depth_sample_max_iqr_m_, close_depth_stable_frames_,
    approach_stall_window_sec_, approach_min_progress_m_);
  RCLCPP_INFO(
    this->get_logger(),
    "[INIT] Gripper verify | joint_states=%s joints=%zu open=%.3f open_tol=%.3f "
    "open_settle=%.2fs",
    joint_states_topic_.c_str(), gripper_joint_names_.size(), gripper_open_position_,
    gripper_open_position_tolerance_, open_gripper_settle_sec_);
}

void VisualServoNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    cv::Mat converted;
    if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
      const cv::Mat view(
        static_cast<int>(msg->height),
        static_cast<int>(msg->width),
        CV_8UC3,
        const_cast<unsigned char *>(msg->data.data()),
        static_cast<std::size_t>(msg->step));
      converted = view.clone();
    } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      const cv::Mat view(
        static_cast<int>(msg->height),
        static_cast<int>(msg->width),
        CV_8UC3,
        const_cast<unsigned char *>(msg->data.data()),
        static_cast<std::size_t>(msg->step));
      cv::cvtColor(view, converted, cv::COLOR_RGB2BGR);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Unsupported image encoding: %s (expected bgr8/rgb8)", msg->encoding.c_str());
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
    const double px = msg->k[0];
    const double py = msg->k[4];
    const double u0 = msg->k[2];
    const double v0 = msg->k[5];
    image_width_ = static_cast<int>(msg->width);
    image_height_ = static_cast<int>(msg->height);
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
    case ServoState::OPEN_GRIPPER:
      handle_open_gripper();
      break;
    case ServoState::APPROACH_DEPTH:
      handle_approach_depth();
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
    blind_approach_distance_m_ = 0.0;
    depth_progress_history_.clear();
  } else if (new_state == ServoState::LIFT) {
    accumulated_lift_distance_m_ = 0.0;
  }
}

bool VisualServoNode::fetch_latest_frame(cv::Mat & frame)
{
  std::lock_guard<std::mutex> lock(image_mutex_);
  if (!frame_available_) {
    return false;
  }
  // Skip if this is the same frame we already processed (prevents
  // re-running the tracker on stale data and accumulating duplicate deltas).
  if (latest_frame_stamp_ == last_processed_frame_stamp_) {
    return false;
  }
  frame = latest_frame_.clone();
  last_processed_frame_stamp_ = latest_frame_stamp_;
  return true;
}

void VisualServoNode::reset_pick_progress()
{
  centering_streak_ = 0;
  close_depth_streak_ = 0;
  accumulated_approach_distance_m_ = 0.0;
  blind_approach_distance_m_ = 0.0;
  accumulated_lift_distance_m_ = 0.0;
  last_depth_sample_.reset();
  depth_progress_history_.clear();
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
  if (!fetch_latest_frame(frame)) {
    return;
  }

  if (update_tracking(frame)) {
    transition_to(ServoState::ALIGN_XY, "target acquired");
  }
}

void VisualServoNode::handle_align_xy()
{
  cv::Mat frame;
  if (!fetch_latest_frame(frame)) {
    return;
  }

  if (!update_tracking(frame)) {
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

  if (publish_overlay_) {
    publish_debug_overlay(frame, tracked_roi_);
  }

  if (centering_update.stable) {
    transition_to(ServoState::OPEN_GRIPPER, "target centered");
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

  const double elapsed = (this->now() - state_entry_time_).seconds();
  const bool settle_elapsed = elapsed >= open_gripper_settle_sec_;
  const auto open_error = max_gripper_open_error();
  const bool gripper_fully_open =
    open_error.has_value() && *open_error <= gripper_open_position_tolerance_;

  if (open_error.has_value()) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[GRIPPER][OPEN] command=%.2f feedback=%s max_err=%.3f tol=%.3f settle=%.2f/%.2f",
      open_gripper_command_,
      gripper_fully_open ? "ready" : "waiting",
      *open_error, gripper_open_position_tolerance_, elapsed, open_gripper_settle_sec_);
  } else {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[GRIPPER][OPEN] command=%.2f feedback=waiting joint_states=%s settle=%.2f/%.2f",
      open_gripper_command_, joint_states_topic_.c_str(), elapsed, open_gripper_settle_sec_);
  }

  // Do NOT include an arm target (has_eef_target=false) so the adapter does not
  // feed MoveIt Servo, which would cause the servo-to-piper bridge to publish
  // a competing JointState that resets the gripper to its current position.
  publish_policy_output(
    geometry_msgs::msg::Twist(), tracking_confidence_, false, true, open_gripper_command_);

  if (gripper_fully_open) {
    transition_to(
      ServoState::APPROACH_DEPTH,
      "gripper fully opened; starting depth approach");
    return;
  }

  if (settle_elapsed) {
    if (open_error.has_value()) {
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
    transition_to(
      ServoState::APPROACH_DEPTH,
      "open command settled; starting depth approach");
  }
}

void VisualServoNode::handle_approach_depth()
{
  cv::Mat frame;
  if (!fetch_latest_frame(frame)) {
    return;
  }

  if (!update_tracking(frame)) {
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
      const double depth_age_sec = std::abs((this->now() - latest_depth_stamp_).seconds());
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

  if (depth_sample.has_value()) {
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
      publish_zero_motion(tracking_confidence_);
      transition_to(ServoState::CLOSE_GRIPPER, "depth reached grasp band");
      if (publish_overlay_) {
        publish_debug_overlay(frame, tracked_roi_);
      }
      return;
    }

    if (depth_in_band) {
      // The gripper was already opened in OPEN_GRIPPER. Do not keep re-sending the
      // same open command while waiting for depth stability, or the adapter will
      // periodically issue fresh trajectory goals during approach.
      publish_policy_output(geometry_msgs::msg::Twist(), tracking_confidence_);
      if (publish_overlay_) {
        publish_debug_overlay(frame, tracked_roi_);
      }
      return;
    }
  } else {
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
  if (depth_sample.has_value()) {
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
  if (publish_overlay_) {
    publish_debug_overlay(frame, tracked_roi_);
  }
}

void VisualServoNode::handle_close_gripper()
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "[GRIPPER][CLOSE] command=%.2f settle=%.2f/%.2f",
    close_gripper_command_,
    (this->now() - state_entry_time_).seconds(), grasp_settle_sec_);

  publish_policy_output(
    geometry_msgs::msg::Twist(), tracking_confidence_, true, true, close_gripper_command_);

  if ((this->now() - state_entry_time_).seconds() >= grasp_settle_sec_) {
    transition_to(ServoState::LIFT, "gripper closed; lifting");
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
  publish_reference_frame_delta(0.0, 0.0, dz, tracking_confidence_, true, close_gripper_command_);
  accumulated_lift_distance_m_ += velocity * cycle_dt;
}

void VisualServoNode::handle_done()
{
  publish_zero_motion(tracking_confidence_);
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
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
