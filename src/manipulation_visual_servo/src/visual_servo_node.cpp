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

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/video/tracking.hpp>

namespace manipulation_visual_servo
{

std::string state_to_string(ServoState state)
{
  switch (state) {
    case ServoState::IDLE: return "IDLE";
    case ServoState::ACQUIRE: return "ACQUIRE";
    case ServoState::TRACK: return "TRACK";
    case ServoState::SERVO: return "SERVO";
    case ServoState::LOST: return "LOST";
    default: return "UNKNOWN";
  }
}

VisualServoNode::VisualServoNode(const rclcpp::NodeOptions & options)
: Node("visual_servo_node", options)
{
  // Declare parameters with defaults from visual_servo_params.yaml
  this->declare_parameter("rgb_topic",
    "/piper/wrist_camera/piper_d405/color/image_rect_raw");
  this->declare_parameter("camera_info_topic",
    "/piper/wrist_camera/piper_d405/color/camera_info");
  this->declare_parameter("depth_topic",
    "/piper/wrist_camera/piper_d405/depth/image_rect_raw");
  this->declare_parameter("detection_topic", "/manipulation/target_detections");
  this->declare_parameter("output_topic", "/manipulation/policy_output");
  this->declare_parameter("use_depth", false);
  this->declare_parameter("control_rate_hz", 20.0);
  this->declare_parameter("target_class", "");
  this->declare_parameter("min_detection_confidence", 0.4);
  this->declare_parameter("min_tracking_confidence", 0.5);
  this->declare_parameter("lost_target_timeout_sec", 0.3);
  this->declare_parameter("acquire_timeout_sec", 5.0);
  this->declare_parameter("image_center_tolerance_px", 8.0);
  this->declare_parameter("reference_frame", "piper_base_link");
  this->declare_parameter("camera_optical_frame", "piper_camera_optical_frame");
  this->declare_parameter("ee_frame", "piper_link6");
  this->declare_parameter("arm_base_frame", "piper_base_link");

  // Tracker parameters
  this->declare_parameter("tracker_type", "klt");
  this->declare_parameter("klt_max_features", 200);
  this->declare_parameter("klt_quality_level", 0.01);
  this->declare_parameter("klt_min_distance", 5.0);
  this->declare_parameter("klt_window_size", 10);
  this->declare_parameter("klt_pyramid_levels", 3);

  // Control gains
  this->declare_parameter("control.lambda_xy", 0.3);
  this->declare_parameter("control.lambda_z", 0.1);
  this->declare_parameter("control.lambda_rz", 0.1);
  this->declare_parameter("control.max_linear_velocity", 0.08);
  this->declare_parameter("control.max_angular_velocity", 0.30);
  this->declare_parameter("control.ramp_up_steps", 5);

  // Debug parameters
  this->declare_parameter("debug.publish_overlay", true);
  this->declare_parameter("debug.overlay_topic", "/visual_servo/debug_image");
  this->declare_parameter("debug.publish_state", true);
  this->declare_parameter("debug.state_topic", "/visual_servo/state");

  // Read parameters
  rgb_topic_ = this->get_parameter("rgb_topic").as_string();
  camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
  depth_topic_ = this->get_parameter("depth_topic").as_string();
  detection_topic_ = this->get_parameter("detection_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  use_depth_ = this->get_parameter("use_depth").as_bool();
  control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
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

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribers
  rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    rgb_topic_, rclcpp::SensorDataQoS(),
    std::bind(&VisualServoNode::image_callback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::SensorDataQoS(),
    std::bind(&VisualServoNode::camera_info_callback, this, std::placeholders::_1));

  detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    detection_topic_, 10,
    std::bind(&VisualServoNode::detection_callback, this, std::placeholders::_1));

  // Publishers
  policy_output_pub_ = this->create_publisher<manipulation_msgs::msg::PolicyOutput>(
    output_topic_, 10);

  if (publish_state_flag_) {
    state_pub_ = this->create_publisher<std_msgs::msg::String>(state_topic_, 10);
  }

  if (publish_overlay_) {
    debug_image_pub_ = image_transport::create_publisher(this, overlay_topic_);
  }

  // Control timer
  auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&VisualServoNode::control_timer_callback, this));

  state_entry_time_ = this->now();
  last_track_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
    "VisualServoNode initialized: rgb=%s, detections=%s, output=%s, rate=%.1f Hz",
    rgb_topic_.c_str(), detection_topic_.c_str(), output_topic_.c_str(), control_rate_hz_);
  RCLCPP_INFO(this->get_logger(),
    "Control gains: lambda_xy=%.3f, lambda_z=%.3f, max_lin=%.3f, max_ang=%.3f",
    lambda_xy_, lambda_z_, max_linear_velocity_, max_angular_velocity_);
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void VisualServoNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_frame_ = cv_ptr->image.clone();
    latest_frame_stamp_ = msg->header.stamp;
    frame_available_ = true;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "cv_bridge conversion failed: %s", e.what());
  }
}

void VisualServoNode::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  if (camera_info_received_) {
    return;
  }

  if (msg->k[0] > 0.0 && msg->k[4] > 0.0) {
    double px = msg->k[0];  // fx
    double py = msg->k[4];  // fy
    double u0 = msg->k[2];  // cx
    double v0 = msg->k[5];  // cy
    cam_params_.initPersProjWithoutDistortion(px, py, u0, v0);
    image_width_ = static_cast<int>(msg->width);
    image_height_ = static_cast<int>(msg->height);
    camera_info_received_ = true;

    // Set desired feature point to image center
    desired_x_ = u0;
    desired_y_ = v0;

    RCLCPP_INFO(this->get_logger(),
      "Camera intrinsics received: fx=%.1f fy=%.1f cx=%.1f cy=%.1f [%dx%d]",
      px, py, u0, v0, image_width_, image_height_);
  }
}

void VisualServoNode::detection_callback(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg)
{
  if (msg->detections.empty()) {
    return;
  }

  // Find best matching detection
  const vision_msgs::msg::Detection2D * best = nullptr;
  float best_score = 0.0f;

  for (const auto & det : msg->detections) {
    float score = 0.0f;
    std::string class_id;

    if (!det.results.empty()) {
      score = static_cast<float>(det.results[0].hypothesis.score);
      class_id = det.results[0].hypothesis.class_id;
    }

    if (score < min_detection_confidence_) {
      continue;
    }

    // Filter by target class if specified
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

  // Convert detection bbox to cv::Rect2d
  double cx = best->bbox.center.position.x;
  double cy = best->bbox.center.position.y;
  double w = best->bbox.size_x;
  double h = best->bbox.size_y;

  std::lock_guard<std::mutex> lock(detection_mutex_);
  latest_detection_roi_ = cv::Rect2d(cx - w / 2.0, cy - h / 2.0, w, h);
  latest_detection_confidence_ = best_score;
  latest_detection_class_ = best->results.empty() ? "" : best->results[0].hypothesis.class_id;
  detection_available_ = true;
  latest_detection_stamp_ = msg->header.stamp;
}

// ---------------------------------------------------------------------------
// Control timer
// ---------------------------------------------------------------------------

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
    case ServoState::SERVO:
      handle_servo();
      break;
    case ServoState::LOST:
      handle_lost();
      break;
  }

  if (publish_state_flag_) {
    publish_state();
  }
}

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

void VisualServoNode::transition_to(ServoState new_state)
{
  if (new_state == state_) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s",
    state_to_string(state_).c_str(), state_to_string(new_state).c_str());
  state_ = new_state;
  state_entry_time_ = this->now();
  ramp_step_ = 0;
}

void VisualServoNode::handle_idle()
{
  // Wait for camera_info before doing anything
  if (!camera_info_received_) {
    return;
  }

  // Check if we have a detection to acquire
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    if (detection_available_) {
      transition_to(ServoState::ACQUIRE);
    }
  }
}

void VisualServoNode::handle_acquire()
{
  // Check for timeout
  double elapsed = (this->now() - state_entry_time_).seconds();
  if (elapsed > acquire_timeout_sec_) {
    RCLCPP_WARN(this->get_logger(), "Acquire timed out after %.1f sec", elapsed);
    transition_to(ServoState::IDLE);
    return;
  }

  cv::Mat frame;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!frame_available_) {
      return;
    }
    frame = latest_frame_.clone();
  }

  cv::Rect2d det_roi;
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    if (!detection_available_) {
      return;
    }
    det_roi = latest_detection_roi_;
    detection_available_ = false;
  }

  // Clamp ROI to image bounds
  det_roi.x = std::max(0.0, det_roi.x);
  det_roi.y = std::max(0.0, det_roi.y);
  det_roi.width = std::min(det_roi.width, static_cast<double>(frame.cols) - det_roi.x);
  det_roi.height = std::min(det_roi.height, static_cast<double>(frame.rows) - det_roi.y);

  if (det_roi.width < 10.0 || det_roi.height < 10.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Detection ROI too small: %.0fx%.0f", det_roi.width, det_roi.height);
    return;
  }

  // Initialize tracker on detection ROI
  if (init_tracker(frame, det_roi)) {
    tracked_roi_ = det_roi;
    // Set desired area from initial detection
    desired_area_ = det_roi.width * det_roi.height;
    last_track_time_ = this->now();
    transition_to(ServoState::TRACK);
  }
}

void VisualServoNode::handle_track()
{
  cv::Mat frame;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!frame_available_) {
      return;
    }
    frame = latest_frame_.clone();
  }

  cv::Rect2d new_roi;
  bool ok = update_tracker(frame, new_roi);

  if (!ok || tracking_confidence_ < min_tracking_confidence_) {
    double since_track = (this->now() - last_track_time_).seconds();
    if (since_track > lost_target_timeout_sec_) {
      transition_to(ServoState::LOST);
    }
    return;
  }

  tracked_roi_ = new_roi;
  last_track_time_ = this->now();

  // Once tracking is stable, transition to servo
  transition_to(ServoState::SERVO);
}

void VisualServoNode::handle_servo()
{
  cv::Mat frame;
  rclcpp::Time frame_stamp;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!frame_available_) {
      return;
    }
    frame = latest_frame_.clone();
    frame_stamp = latest_frame_stamp_;
  }

  // Update tracker
  cv::Rect2d new_roi;
  bool ok = update_tracker(frame, new_roi);

  if (!ok || tracking_confidence_ < min_tracking_confidence_) {
    double since_track = (this->now() - last_track_time_).seconds();
    if (since_track > lost_target_timeout_sec_) {
      transition_to(ServoState::LOST);
    }
    return;
  }

  tracked_roi_ = new_roi;
  last_track_time_ = this->now();

  // Check for new detections to re-seed tracker if available
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    if (detection_available_) {
      // Re-seed tracker from new detection for drift correction
      cv::Rect2d det_roi = latest_detection_roi_;
      detection_available_ = false;

      // Only re-seed if detection is reasonably close to tracked position
      double dx = std::abs((det_roi.x + det_roi.width / 2.0) -
        (tracked_roi_.x + tracked_roi_.width / 2.0));
      double dy = std::abs((det_roi.y + det_roi.height / 2.0) -
        (tracked_roi_.y + tracked_roi_.height / 2.0));
      double diag = std::sqrt(tracked_roi_.width * tracked_roi_.width +
        tracked_roi_.height * tracked_roi_.height);

      if (dx < diag && dy < diag) {
        init_tracker(frame, det_roi);
        tracked_roi_ = det_roi;
      }
    }
  }

  // Compute current feature values (centroid + area)
  double feat_x = tracked_roi_.x + tracked_roi_.width / 2.0;
  double feat_y = tracked_roi_.y + tracked_roi_.height / 2.0;
  double feat_area = tracked_roi_.width * tracked_roi_.height;

  // Check convergence
  double err_x = feat_x - desired_x_;
  double err_y = feat_y - desired_y_;
  double pixel_error = std::sqrt(err_x * err_x + err_y * err_y);

  if (pixel_error < image_center_tolerance_px_ &&
    std::abs(feat_area - desired_area_) / desired_area_ < 0.05)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Servo converged: pixel_error=%.1f px", pixel_error);
  }

  // Compute IBVS twist command
  auto twist = compute_ibvs_twist(feat_x, feat_y, feat_area,
    desired_x_, desired_y_, desired_area_);

  // Apply ramp-up
  if (ramp_step_ < ramp_up_steps_ && ramp_up_steps_ > 0) {
    double ramp_factor = static_cast<double>(ramp_step_ + 1) / ramp_up_steps_;
    twist.linear.x *= ramp_factor;
    twist.linear.y *= ramp_factor;
    twist.linear.z *= ramp_factor;
    twist.angular.x *= ramp_factor;
    twist.angular.y *= ramp_factor;
    twist.angular.z *= ramp_factor;
    ramp_step_++;
  }

  // Publish command via PolicyOutput
  publish_policy_output(twist, tracking_confidence_);

  // Debug overlay
  if (publish_overlay_) {
    publish_debug_overlay(frame, tracked_roi_);
  }
}

void VisualServoNode::handle_lost()
{
  // Clear tracker
  tracker_initialized_ = false;

  // Check for new detections to re-acquire
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    if (detection_available_) {
      transition_to(ServoState::ACQUIRE);
      return;
    }
  }

  // After some time in LOST, go back to IDLE
  double elapsed = (this->now() - state_entry_time_).seconds();
  if (elapsed > acquire_timeout_sec_) {
    RCLCPP_WARN(this->get_logger(), "Lost target, returning to IDLE");
    transition_to(ServoState::IDLE);
  }
}

// ---------------------------------------------------------------------------
// Tracker
// ---------------------------------------------------------------------------

bool VisualServoNode::init_tracker(const cv::Mat & frame, const cv::Rect2d & roi)
{
  // Use CSRT tracker for good accuracy within ROI
  cv_tracker_ = cv::TrackerCSRT::create();

  try {
    cv_tracker_->init(frame, roi);
    tracker_initialized_ = true;
    tracking_confidence_ = 1.0f;
    RCLCPP_DEBUG(this->get_logger(), "Tracker initialized on ROI [%.0f, %.0f, %.0f, %.0f]",
      roi.x, roi.y, roi.width, roi.height);
    return true;
  } catch (const cv::Exception & e) {
    RCLCPP_WARN(this->get_logger(), "Tracker init failed: %s", e.what());
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
    bool ok = cv_tracker_->update(frame, tracked_roi);
    if (ok) {
      // Validate ROI is within image bounds
      if (tracked_roi.x >= 0 && tracked_roi.y >= 0 &&
        tracked_roi.x + tracked_roi.width <= frame.cols &&
        tracked_roi.y + tracked_roi.height <= frame.rows &&
        tracked_roi.width > 5 && tracked_roi.height > 5)
      {
        tracking_confidence_ = 0.8f;  // CSRT doesn't provide confidence, use fixed value
        return true;
      }
    }
    tracking_confidence_ = 0.0f;
    return false;
  } catch (const cv::Exception & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Tracker update failed: %s", e.what());
    tracking_confidence_ = 0.0f;
    return false;
  }
}

// ---------------------------------------------------------------------------
// IBVS control law
// ---------------------------------------------------------------------------

geometry_msgs::msg::Twist VisualServoNode::compute_ibvs_twist(
  double feat_x, double feat_y, double feat_area,
  double des_x, double des_y, double des_area)
{
  geometry_msgs::msg::Twist twist;

  // Image-space error (pixels)
  double err_x = feat_x - des_x;
  double err_y = feat_y - des_y;

  // Normalized errors: map pixel error to velocity using image dimensions
  double norm_err_x = 0.0;
  double norm_err_y = 0.0;
  if (image_width_ > 0 && image_height_ > 0) {
    norm_err_x = err_x / static_cast<double>(image_width_);
    norm_err_y = err_y / static_cast<double>(image_height_);
  }

  // Scale/depth error from area ratio
  double area_ratio = 1.0;
  if (des_area > 0.0) {
    area_ratio = feat_area / des_area;
  }
  double err_z = area_ratio - 1.0;  // positive = too close, negative = too far

  // Camera frame twist (optical convention: z forward, x right, y down)
  // Lateral: move camera to center the target
  //   - Error in image x -> camera y velocity (with sign flip for servo)
  //   - Error in image y -> camera z velocity
  // Forward/back: move camera to match desired distance
  //   - Area error -> camera x velocity (z in optical frame)

  // In camera optical frame:
  //   x = right, y = down, z = forward (into scene)
  // To center target:
  //   target right of center (err_x > 0) -> move camera right -> vy > 0
  //   target below center (err_y > 0) -> move camera down -> vz > 0
  //   target too close (err_z > 0) -> move camera back -> vx < 0

  twist.linear.y = -lambda_xy_ * norm_err_x;   // lateral
  twist.linear.z = -lambda_xy_ * norm_err_y;   // vertical
  twist.linear.x = -lambda_z_ * err_z;          // forward/back

  // Clamp velocities
  auto clamp = [](double val, double lim) {
      return std::max(-lim, std::min(lim, val));
    };

  twist.linear.x = clamp(twist.linear.x, max_linear_velocity_);
  twist.linear.y = clamp(twist.linear.y, max_linear_velocity_);
  twist.linear.z = clamp(twist.linear.z, max_linear_velocity_);
  twist.angular.x = clamp(twist.angular.x, max_angular_velocity_);
  twist.angular.y = clamp(twist.angular.y, max_angular_velocity_);
  twist.angular.z = clamp(twist.angular.z, max_angular_velocity_);

  return twist;
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------

geometry_msgs::msg::Pose VisualServoNode::twist_to_eef_delta(
  const geometry_msgs::msg::Twist & twist, double dt)
{
  geometry_msgs::msg::Pose delta;

  // Convert twist to small displacement
  delta.position.x = twist.linear.x * dt;
  delta.position.y = twist.linear.y * dt;
  delta.position.z = twist.linear.z * dt;

  // Small-angle approximation for orientation delta
  double half_ax = twist.angular.x * dt * 0.5;
  double half_ay = twist.angular.y * dt * 0.5;
  double half_az = twist.angular.z * dt * 0.5;

  delta.orientation.x = half_ax;
  delta.orientation.y = half_ay;
  delta.orientation.z = half_az;
  delta.orientation.w = 1.0;

  // Normalize quaternion
  double norm = std::sqrt(
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

void VisualServoNode::publish_policy_output(
  const geometry_msgs::msg::Twist & twist,
  float confidence)
{
  // Transform twist from camera optical frame to reference frame (arm base) via TF
  geometry_msgs::msg::Twist output_twist = twist;

  if (!camera_optical_frame_.empty() && !reference_frame_.empty() &&
    camera_optical_frame_ != reference_frame_)
  {
    try {
      auto transform = tf_buffer_->lookupTransform(
        reference_frame_, camera_optical_frame_,
        tf2::TimePointZero);

      // Rotate the twist linear/angular vectors by the transform rotation
      auto & q = transform.transform.rotation;

      // Apply rotation to linear velocity
      auto rotate_vec = [&q](double vx, double vy, double vz,
          double & ox, double & oy, double & oz) {
          // Quaternion rotation: v' = q * v * q_inv
          double qx = q.x, qy = q.y, qz = q.z, qw = q.w;
          // v as quaternion: (vx, vy, vz, 0)
          double t2 = qw * vx + qy * vz - qz * vy;
          double t3 = qw * vy + qz * vx - qx * vz;
          double t4 = qw * vz + qx * vy - qy * vx;
          double t5 = -qx * vx - qy * vy - qz * vz;
          ox = t2 * qw - t5 * qx - t3 * qz + t4 * qy;
          oy = t3 * qw - t5 * qy - t4 * qx + t2 * qz;
          oz = t4 * qw - t5 * qz - t2 * qy + t3 * qx;
        };

      double lx, ly, lz;
      rotate_vec(twist.linear.x, twist.linear.y, twist.linear.z, lx, ly, lz);
      output_twist.linear.x = lx;
      output_twist.linear.y = ly;
      output_twist.linear.z = lz;

      double ax, ay, az;
      rotate_vec(twist.angular.x, twist.angular.y, twist.angular.z, ax, ay, az);
      output_twist.angular.x = ax;
      output_twist.angular.y = ay;
      output_twist.angular.z = az;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF lookup %s -> %s failed: %s, publishing in camera frame",
        camera_optical_frame_.c_str(), reference_frame_.c_str(), ex.what());
    }
  }

  // Convert to EEF delta pose for PolicyOutput
  double dt = 1.0 / control_rate_hz_;
  auto eef_delta = twist_to_eef_delta(output_twist, dt);

  manipulation_msgs::msg::PolicyOutput msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = reference_frame_;

  msg.has_eef_target = true;
  msg.eef_target_pose = eef_delta;
  msg.reference_frame = reference_frame_;
  msg.confidence = confidence;

  msg.has_joint_deltas = false;
  msg.gripper_active = false;
  msg.has_base_hint = false;

  policy_output_pub_->publish(msg);
}

void VisualServoNode::publish_debug_overlay(
  const cv::Mat & frame, const cv::Rect2d & roi)
{
  cv::Mat overlay = frame.clone();

  // Draw tracked ROI
  cv::rectangle(overlay,
    cv::Point(static_cast<int>(roi.x), static_cast<int>(roi.y)),
    cv::Point(static_cast<int>(roi.x + roi.width),
    static_cast<int>(roi.y + roi.height)),
    cv::Scalar(0, 255, 0), 2);

  // Draw centroid
  cv::Point center(
    static_cast<int>(roi.x + roi.width / 2.0),
    static_cast<int>(roi.y + roi.height / 2.0));
  cv::circle(overlay, center, 5, cv::Scalar(0, 0, 255), -1);

  // Draw desired point (image center)
  cv::Point desired(static_cast<int>(desired_x_), static_cast<int>(desired_y_));
  cv::drawMarker(overlay, desired, cv::Scalar(255, 0, 0),
    cv::MARKER_CROSS, 20, 2);

  // Draw error line
  cv::line(overlay, center, desired, cv::Scalar(0, 255, 255), 1);

  // State text
  std::string state_text = "State: " + state_to_string(state_);
  cv::putText(overlay, state_text, cv::Point(10, 30),
    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

  // Pixel error
  double err = std::sqrt(
    std::pow(center.x - desired.x, 2) + std::pow(center.y - desired.y, 2));
  std::string err_text = "Err: " + std::to_string(static_cast<int>(err)) + " px";
  cv::putText(overlay, err_text, cv::Point(10, 60),
    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

  // Publish
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", overlay).toImageMsg();
  msg->header.stamp = this->now();
  debug_image_pub_.publish(msg);
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

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulation_visual_servo::VisualServoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
