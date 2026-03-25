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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <manipulation_msgs/msg/policy_output.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/planning_options.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace
{

std::string describeNormalizedGripperCommand(double command)
{
  if (command >= 0.95) {
    return "open";
  }
  if (command <= 0.05) {
    return "close";
  }
  return "partial";
}

constexpr double kDefaultGripperMoveGroupTimeoutSec = 15.0;

}  // namespace

/**
 * @brief Adapter node that maps policy outputs to hardware commands
 *
 * This node takes high-level policy outputs and converts them to:
 * - Base velocity commands (/cmd_vel)
 * - Arm joint trajectories (via FollowJointTrajectory action)
 * - Gripper commands
 *
 * It incorporates TF-aware planning, safety checks, and coordination
 * between the mobile base and manipulator arm.
 */
class AdapterNode : public rclcpp::Node
{
public:
  AdapterNode()
  : Node("adapter_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("ee_frame", "piper_tcp");
    this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    this->declare_parameter<std::string>("navigate_to_pose_action", "/navigate_to_pose");
    this->declare_parameter<std::string>(
      "follow_joint_trajectory_action", "/arm_controller/follow_joint_trajectory");
    this->declare_parameter<bool>("use_moveit", true);
    this->declare_parameter<std::string>("move_group_action", "/move_action");
    this->declare_parameter<std::string>("move_group_name", "arm");
    this->declare_parameter<std::string>("move_group_eef_link", "piper_tcp");
    this->declare_parameter<std::string>("gripper_move_group_name", "piper_gripper");
    this->declare_parameter<double>("gripper_move_group_timeout_sec", 15.0);
    this->declare_parameter<double>("moveit_action_wait_sec", 1.0);
    this->declare_parameter<double>("moveit_planning_time", 2.0);
    this->declare_parameter<int>("moveit_planning_attempts", 3);
    this->declare_parameter<double>("moveit_velocity_scaling", 0.5);
    this->declare_parameter<double>("moveit_accel_scaling", 0.5);
    this->declare_parameter<double>("moveit_position_tolerance", 0.01);
    this->declare_parameter<double>("moveit_orientation_tolerance", 0.1);
    this->declare_parameter<bool>("move_to_ready_on_startup", false);
    this->declare_parameter<std::vector<std::string>>(
      "ready_pose_joint_names",
      std::vector<std::string>{
      "piper_joint1", "piper_joint2", "piper_joint3",
      "piper_joint4", "piper_joint5", "piper_joint6"});
    this->declare_parameter<std::vector<double>>(
      "ready_pose_joint_positions",
      std::vector<double>{0.0, 1.2, -0.2, 0.0, -0.8, 0.0});
    this->declare_parameter<double>("ready_pose_start_delay_sec", 1.0);
    this->declare_parameter<double>("ready_pose_retry_period_sec", 1.0);
    this->declare_parameter<int>("ready_pose_max_attempts", 3);
    this->declare_parameter<std::vector<std::string>>(
      "arm_joint_names",
      std::vector<std::string>{});
    this->declare_parameter<double>("arm_command_duration_sec", 1.5);
    this->declare_parameter<std::string>("gripper_joint_name", "piper_joint7");
    this->declare_parameter<std::vector<std::string>>(
      "gripper_joint_names",
      std::vector<std::string>{});
    this->declare_parameter<double>("gripper_open_position", 0.065);
    this->declare_parameter<double>("gripper_closed_position", 0.0);
    this->declare_parameter<std::vector<double>>(
      "gripper_open_positions",
      std::vector<double>{});
    this->declare_parameter<std::vector<double>>(
      "gripper_closed_positions",
      std::vector<double>{});
    this->declare_parameter<double>("gripper_command_epsilon", 0.01);
    this->declare_parameter<double>("max_base_velocity", 0.5);
    this->declare_parameter<double>("max_arm_velocity", 1.0);
    this->declare_parameter<double>("safety_timeout_sec", 2.0);
    this->declare_parameter<std::string>("workspace_frame_id", "piper_base_link");
    this->declare_parameter<double>("workspace_x_min", 0.1);
    this->declare_parameter<double>("workspace_x_max", 0.8);
    this->declare_parameter<double>("workspace_y_min", -0.5);
    this->declare_parameter<double>("workspace_y_max", 0.5);
    this->declare_parameter<double>("workspace_z_min", 0.0);
    this->declare_parameter<double>("workspace_z_max", 1.0);
    this->declare_parameter<bool>("eef_target_is_delta", false);
    this->declare_parameter<std::string>("arm_base_frame", "piper_base_link");
    this->declare_parameter<std::string>("arm_execution_mode", "moveit_servo");
    this->declare_parameter<std::string>("servo_cartesian_topic", "/servo_node/delta_twist_cmds");
    this->declare_parameter<double>("servo_publish_rate_hz", 30.0);
    this->declare_parameter<double>("servo_command_horizon_sec", 2.0);
    this->declare_parameter<double>("servo_max_linear_velocity", 0.10);
    this->declare_parameter<double>("servo_max_angular_velocity", 0.35);
    this->declare_parameter<bool>("pause_base_during_servo", true);
    this->declare_parameter<std::string>("servo_start_service", "/servo_node/start_servo");
    this->declare_parameter<bool>("wait_for_servo_ready", true);
    this->declare_parameter<double>("servo_ready_timeout_sec", 20.0);
    this->declare_parameter<std::string>("visual_servo_state_topic", "/visual_servo/state");
    this->declare_parameter<bool>("return_to_ready_after_visual_servo", true);

    // Get parameters
    base_frame_ = this->get_parameter("base_frame").as_string();
    ee_frame_ = this->get_parameter("ee_frame").as_string();
    joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();
    navigate_to_pose_action_ = this->get_parameter("navigate_to_pose_action").as_string();
    follow_joint_trajectory_action_ =
      this->get_parameter("follow_joint_trajectory_action").as_string();
    use_moveit_ = this->get_parameter("use_moveit").as_bool();
    move_group_action_ = this->get_parameter("move_group_action").as_string();
    move_group_name_ = this->get_parameter("move_group_name").as_string();
    move_group_eef_link_ = this->get_parameter("move_group_eef_link").as_string();
    gripper_move_group_name_ = this->get_parameter("gripper_move_group_name").as_string();
    gripper_move_group_timeout_sec_ =
      this->get_parameter("gripper_move_group_timeout_sec").as_double();
    moveit_action_wait_sec_ = this->get_parameter("moveit_action_wait_sec").as_double();
    moveit_planning_time_ = this->get_parameter("moveit_planning_time").as_double();
    moveit_planning_attempts_ = this->get_parameter("moveit_planning_attempts").as_int();
    moveit_velocity_scaling_ = this->get_parameter("moveit_velocity_scaling").as_double();
    moveit_accel_scaling_ = this->get_parameter("moveit_accel_scaling").as_double();
    moveit_position_tolerance_ = this->get_parameter("moveit_position_tolerance").as_double();
    moveit_orientation_tolerance_ =
      this->get_parameter("moveit_orientation_tolerance").as_double();
    move_to_ready_on_startup_ = this->get_parameter("move_to_ready_on_startup").as_bool();
    ready_pose_joint_names_ = this->get_parameter("ready_pose_joint_names").as_string_array();
    ready_pose_joint_positions_ =
      this->get_parameter("ready_pose_joint_positions").as_double_array();
    ready_pose_start_delay_sec_ = this->get_parameter("ready_pose_start_delay_sec").as_double();
    ready_pose_retry_period_sec_ = this->get_parameter("ready_pose_retry_period_sec").as_double();
    ready_pose_max_attempts_ = this->get_parameter("ready_pose_max_attempts").as_int();
    arm_joint_names_ = this->get_parameter("arm_joint_names").as_string_array();
    arm_command_duration_sec_ = this->get_parameter("arm_command_duration_sec").as_double();
    gripper_joint_name_ = this->get_parameter("gripper_joint_name").as_string();
    gripper_joint_names_ = this->get_parameter("gripper_joint_names").as_string_array();
    gripper_open_position_ = this->get_parameter("gripper_open_position").as_double();
    gripper_closed_position_ = this->get_parameter("gripper_closed_position").as_double();
    gripper_open_positions_ = this->get_parameter("gripper_open_positions").as_double_array();
    gripper_closed_positions_ = this->get_parameter("gripper_closed_positions").as_double_array();
    gripper_command_epsilon_ = this->get_parameter("gripper_command_epsilon").as_double();
    max_base_velocity_ = this->get_parameter("max_base_velocity").as_double();
    max_arm_velocity_ = this->get_parameter("max_arm_velocity").as_double();
    safety_timeout_sec_ = this->get_parameter("safety_timeout_sec").as_double();
    workspace_frame_id_ = this->get_parameter("workspace_frame_id").as_string();
    workspace_x_min_ = this->get_parameter("workspace_x_min").as_double();
    workspace_x_max_ = this->get_parameter("workspace_x_max").as_double();
    workspace_y_min_ = this->get_parameter("workspace_y_min").as_double();
    workspace_y_max_ = this->get_parameter("workspace_y_max").as_double();
    workspace_z_min_ = this->get_parameter("workspace_z_min").as_double();
    workspace_z_max_ = this->get_parameter("workspace_z_max").as_double();
    eef_target_is_delta_ = this->get_parameter("eef_target_is_delta").as_bool();
    arm_base_frame_ = this->get_parameter("arm_base_frame").as_string();
    arm_execution_mode_ = this->get_parameter("arm_execution_mode").as_string();
    servo_cartesian_topic_ = this->get_parameter("servo_cartesian_topic").as_string();
    servo_publish_rate_hz_ = this->get_parameter("servo_publish_rate_hz").as_double();
    servo_command_horizon_sec_ = this->get_parameter("servo_command_horizon_sec").as_double();
    servo_max_linear_velocity_ = this->get_parameter("servo_max_linear_velocity").as_double();
    servo_max_angular_velocity_ = this->get_parameter("servo_max_angular_velocity").as_double();
    pause_base_during_servo_ = this->get_parameter("pause_base_during_servo").as_bool();
    servo_start_service_ = this->get_parameter("servo_start_service").as_string();
    wait_for_servo_ready_ = this->get_parameter("wait_for_servo_ready").as_bool();
    servo_ready_timeout_sec_ = this->get_parameter("servo_ready_timeout_sec").as_double();
    visual_servo_state_topic_ = this->get_parameter("visual_servo_state_topic").as_string();
    return_to_ready_after_visual_servo_ =
      this->get_parameter("return_to_ready_after_visual_servo").as_bool();

    std::transform(
      arm_execution_mode_.begin(), arm_execution_mode_.end(), arm_execution_mode_.begin(),
      [](unsigned char ch) {return static_cast<char>(std::tolower(ch));});
    if (arm_execution_mode_ != "moveit_servo" && arm_execution_mode_ != "move_group") {
      RCLCPP_WARN(
        this->get_logger(),
        "Unsupported arm_execution_mode='%s'; falling back to move_group",
        arm_execution_mode_.c_str());
      arm_execution_mode_ = "move_group";
    }

    if (servo_publish_rate_hz_ <= 0.0) {
      servo_publish_rate_hz_ = 30.0;
    }
    if (servo_command_horizon_sec_ <= 0.0) {
      servo_command_horizon_sec_ = 2.0;
    }
    if (servo_max_linear_velocity_ <= 0.0) {
      servo_max_linear_velocity_ = 0.10;
    }
    if (servo_max_angular_velocity_ <= 0.0) {
      servo_max_angular_velocity_ = 0.35;
    }
    if (servo_ready_timeout_sec_ < 0.0) {
      servo_ready_timeout_sec_ = 0.0;
    }
    if (gripper_move_group_timeout_sec_ <= 0.0) {
      gripper_move_group_timeout_sec_ = kDefaultGripperMoveGroupTimeoutSec;
    }
    if (ready_pose_start_delay_sec_ < 0.0) {
      ready_pose_start_delay_sec_ = 0.0;
    }
    if (ready_pose_retry_period_sec_ <= 0.0) {
      ready_pose_retry_period_sec_ = 1.0;
    }
    if (ready_pose_max_attempts_ < 1) {
      ready_pose_max_attempts_ = 1;
    }
    if (move_to_ready_on_startup_ &&
      ready_pose_joint_names_.size() != ready_pose_joint_positions_.size())
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "ready_pose_joint_names (%zu) and ready_pose_joint_positions (%zu) length mismatch. "
        "Disabling startup ready pose.",
        ready_pose_joint_names_.size(), ready_pose_joint_positions_.size());
      move_to_ready_on_startup_ = false;
    }
    if (move_to_ready_on_startup_ && ready_pose_joint_names_.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Startup ready pose enabled but no ready_pose_joint_names provided. "
        "Disabling startup ready pose.");
      move_to_ready_on_startup_ = false;
    }

    // Initialize TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create subscription to policy output
    policy_sub_ = this->create_subscription<manipulation_msgs::msg::PolicyOutput>(
      "/manipulation/policy_output", 10,
      std::bind(&AdapterNode::policyCallback, this, std::placeholders::_1));

    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, 10,
      std::bind(&AdapterNode::jointStatesCallback, this, std::placeholders::_1));
    visual_servo_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      visual_servo_state_topic_, 10,
      std::bind(&AdapterNode::visualServoStateCallback, this, std::placeholders::_1));

    // Create publishers for hardware commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    servo_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      servo_cartesian_topic_, 10);

    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, navigate_to_pose_action_);
    arm_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this, follow_joint_trajectory_action_);
    if (use_moveit_ && (!isServoMode() || move_to_ready_on_startup_)) {
      move_group_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
        this, move_group_action_);
    }
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>(servo_start_service_);

    // Create safety timer
    safety_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(safety_timeout_sec_ * 1000)),
      std::bind(&AdapterNode::safetyCheck, this));

    if (isServoMode()) {
      servo_publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / servo_publish_rate_hz_)),
        std::bind(&AdapterNode::publishServoCommand, this));
    }
    if (move_to_ready_on_startup_) {
      if (!use_moveit_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Startup ready pose requires MoveIt action support. Disabling startup ready pose.");
        move_to_ready_on_startup_ = false;
      } else {
        startup_ready_pose_completed_.store(false);
        ready_pose_start_time_steady_ =
          std::chrono::steady_clock::now() + std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(ready_pose_start_delay_sec_));
        auto retry_period = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(ready_pose_retry_period_sec_));
        if (retry_period < std::chrono::milliseconds(1)) {
          retry_period = std::chrono::milliseconds(1);
        }
        ready_pose_timer_ = this->create_wall_timer(
          retry_period, std::bind(&AdapterNode::startupReadyPoseTimerCallback, this));
      }
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[INIT] Adapter ready | base=%s arm_base=%s ee=%s workspace=%s",
      base_frame_.c_str(), arm_base_frame_.c_str(), ee_frame_.c_str(), workspace_frame_id_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "[INIT] Control | mode=%s moveit=%s delta_targets=%s",
      arm_execution_mode_.c_str(), use_moveit_ ? "true" : "false",
      eef_target_is_delta_ ? "true" : "false");
    if (use_moveit_) {
      RCLCPP_INFO(
        this->get_logger(),
        "[INIT] MoveIt | action=%s arm_group=%s gripper_group=%s eef_link=%s",
        move_group_action_.c_str(), move_group_name_.c_str(),
        gripper_move_group_name_.c_str(), move_group_eef_link_.c_str());
    }
    if (use_moveit_ && move_group_eef_link_ != ee_frame_) {
      RCLCPP_WARN(
        this->get_logger(),
        "[INIT] ee_frame='%s' differs from move_group_eef_link='%s'. "
        "Use this only when TF and MoveIt are intentionally targeting different tool frames.",
        ee_frame_.c_str(), move_group_eef_link_.c_str());
    }
    RCLCPP_INFO(
      this->get_logger(), "[INIT] Startup ready pose=%s",
      move_to_ready_on_startup_ ? "true" : "false");
    if (move_to_ready_on_startup_) {
      RCLCPP_INFO(
        this->get_logger(),
        "[INIT] Ready pose | joints=%zu start_delay=%.2fs retry=%.2fs max_attempts=%d",
        ready_pose_joint_names_.size(), ready_pose_start_delay_sec_,
        ready_pose_retry_period_sec_, ready_pose_max_attempts_);
    }
    if (isServoMode()) {
      RCLCPP_INFO(
        this->get_logger(),
        "[INIT] Servo | topic=%s horizon=%.3fs rate=%.1fHz "
        "pause_base=%s wait_ready=%s timeout=%.1fs",
        servo_cartesian_topic_.c_str(), servo_command_horizon_sec_, servo_publish_rate_hz_,
        pause_base_during_servo_ ? "true" : "false",
        wait_for_servo_ready_ ? "true" : "false", servo_ready_timeout_sec_);
    }
    RCLCPP_INFO(
      this->get_logger(),
      "[INIT] Visual servo ready return=%s state_topic=%s",
      return_to_ready_after_visual_servo_ ? "true" : "false",
      visual_servo_state_topic_.c_str());

    // Initialize safety timestamp to avoid large "no policy output" elapsed time at startup.
    last_policy_time_ = this->now();
  }

private:
  struct GripperCommandDecision
  {
    bool skip_arm_this_cycle = false;
  };

  bool isGripperJointName(const std::string & joint_name) const
  {
    if (joint_name == gripper_joint_name_) {
      return true;
    }
    return std::find(
      gripper_joint_names_.begin(), gripper_joint_names_.end(),
      joint_name) != gripper_joint_names_.end();
  }

  bool buildGripperJointTargets(
    double normalized_command,
    std::vector<std::string> & joint_names,
    std::vector<double> & target_positions) const
  {
    if (!gripper_joint_names_.empty()) {
      if (
        gripper_open_positions_.size() != gripper_joint_names_.size() ||
        gripper_closed_positions_.size() != gripper_joint_names_.size())
      {
        RCLCPP_WARN(
          this->get_logger(),
          "[GRIPPER][CONFIG] joint names/positions size mismatch (names=%zu, open=%zu, closed=%zu)",
          gripper_joint_names_.size(),
          gripper_open_positions_.size(),
          gripper_closed_positions_.size());
        return false;
      }
      joint_names = gripper_joint_names_;
      target_positions.reserve(joint_names.size());
      for (size_t i = 0; i < joint_names.size(); ++i) {
        target_positions.push_back(
          gripper_closed_positions_[i] +
          normalized_command * (gripper_open_positions_[i] - gripper_closed_positions_[i]));
      }
      return true;
    }

    joint_names = {gripper_joint_name_};
    target_positions = {
      gripper_closed_position_ +
      normalized_command * (gripper_open_position_ - gripper_closed_position_)};
    return true;
  }

  void releaseServoSuppressionForGripper()
  {
    servo_suppressed_for_gripper_.store(false);
  }

  void suppressServoForGripperTransition()
  {
    stopServoCommand();
    servo_suppressed_for_gripper_.store(true);
  }

  bool isServoSuppressedForGripper() const
  {
    return servo_suppressed_for_gripper_.load();
  }

  bool isCurrentGripperMoveGroupGoal(uint64_t generation) const
  {
    return gripper_goal_generation_.load() == generation;
  }

  void resetGripperMoveGroupHandle()
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    gripper_moveit_goal_handle_.reset();
  }

  void checkGripperMoveGroupTimeout()
  {
    if (!gripper_moveit_goal_active_.load()) {
      return;
    }

    const double elapsed_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - gripper_goal_start_steady_).count();
    if (elapsed_sec < gripper_move_group_timeout_sec_) {
      return;
    }

    gripper_goal_generation_.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (move_group_client_ && gripper_moveit_goal_handle_) {
        move_group_client_->async_cancel_goal(gripper_moveit_goal_handle_);
      }
      gripper_moveit_goal_handle_.reset();
    }
    gripper_moveit_goal_active_.store(false);
    latched_gripper_command_.reset();
    releaseServoSuppressionForGripper();
    RCLCPP_WARN(
      this->get_logger(),
      "[GRIPPER] MoveGroup goal timed out after %.1fs; released servo suppression",
      elapsed_sec);
  }

  bool sendGripperMoveGroupGoal(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & joint_positions,
    double normalized_command)
  {
    if (!use_moveit_) {
      RCLCPP_WARN_ONCE(
        this->get_logger(),
        "[GRIPPER] MoveIt is disabled; cannot execute gripper goals via MoveGroup.");
      return false;
    }
    if (gripper_move_group_name_.empty()) {
      RCLCPP_WARN(this->get_logger(), "[GRIPPER] gripper_move_group_name is empty");
      return false;
    }
    if (!ensureMoveGroupClientReady()) {
      return false;
    }

    const uint64_t generation = gripper_goal_generation_.fetch_add(1) + 1;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (move_group_client_ && gripper_moveit_goal_handle_) {
        move_group_client_->async_cancel_goal(gripper_moveit_goal_handle_);
      }
      gripper_moveit_goal_handle_.reset();
    }

    moveit_msgs::action::MoveGroup::Goal goal;
    goal.request.group_name = gripper_move_group_name_;
    goal.request.num_planning_attempts = moveit_planning_attempts_;
    goal.request.allowed_planning_time = moveit_planning_time_;
    goal.request.max_velocity_scaling_factor = moveit_velocity_scaling_;
    goal.request.max_acceleration_scaling_factor = moveit_accel_scaling_;
    goal.request.goal_constraints.push_back(
      buildJointGoalConstraints(joint_names, joint_positions, "gripper_goal"));
    goal.request.start_state.is_diff = true;

    goal.planning_options.plan_only = false;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = false;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    auto send_goal_options =
      rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this, generation](
      rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr goal_handle) {
        if (!isCurrentGripperMoveGroupGoal(generation)) {
          if (goal_handle && move_group_client_) {
            move_group_client_->async_cancel_goal(goal_handle);
          }
          return;
        }

        if (!goal_handle) {
          RCLCPP_WARN(this->get_logger(), "[GRIPPER] MoveGroup goal was rejected");
          gripper_moveit_goal_active_.store(false);
          latched_gripper_command_.reset();
          releaseServoSuppressionForGripper();
          resetGripperMoveGroupHandle();
          return;
        }

        std::lock_guard<std::mutex> lock(goal_mutex_);
        gripper_moveit_goal_handle_ = goal_handle;
      };

    send_goal_options.result_callback =
      [this, generation](const auto & result) {
        if (!isCurrentGripperMoveGroupGoal(generation)) {
          return;
        }

        gripper_moveit_goal_active_.store(false);
        releaseServoSuppressionForGripper();
        resetGripperMoveGroupHandle();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          latched_gripper_command_.reset();
          RCLCPP_WARN(
            this->get_logger(),
            "[GRIPPER] MoveGroup goal failed with code %d",
            static_cast<int>(result.code));
        } else {
          RCLCPP_INFO(this->get_logger(), "[GRIPPER] MoveGroup goal succeeded");
        }
      };

    latched_gripper_command_ = normalized_command;
    gripper_moveit_goal_active_.store(true);
    gripper_goal_start_steady_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      this->get_logger(),
      "[GRIPPER] sending %s command=%.2f via MoveGroup group=%s joints=%zu",
      describeNormalizedGripperCommand(normalized_command).c_str(),
      normalized_command, gripper_move_group_name_.c_str(), joint_names.size());
    move_group_client_->async_send_goal(goal, send_goal_options);
    return true;
  }

  void policyCallback(const manipulation_msgs::msg::PolicyOutput::SharedPtr msg)
  {
    last_policy_time_ = this->now();
    checkGripperMoveGroupTimeout();

    if (move_to_ready_on_startup_ && !startup_ready_pose_completed_.load()) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[READY] dropping policy output while startup ready pose is pending");
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Received policy output");

    bool skip_arm_this_cycle = false;
    if (msg->gripper_active) {
      const auto gripper_decision = processGripperCommand(msg->gripper_command);
      skip_arm_this_cycle = gripper_decision.skip_arm_this_cycle;
    }

    bool arm_command_sent = false;
    if (!skip_arm_this_cycle && msg->has_eef_target) {
      if (msg->has_joint_deltas) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Policy output has both EEF target and joint deltas; prioritizing EEF target");
      }
      arm_command_sent = processEndEffectorTarget(msg);
    }

    if (!skip_arm_this_cycle && !arm_command_sent && msg->has_joint_deltas) {
      arm_command_sent = processJointDeltas(msg->joint_deltas);
    }

    if (!msg->gripper_active && !msg->has_eef_target && !msg->has_joint_deltas) {
      stopServoCommand();
    }

    // Process base hint if available
    if (msg->has_base_hint) {
      if (pause_base_during_servo_ && isServoCommandActive()) {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Skipping base velocity hint while servo arm command is active");
        return;
      }
      processBaseCommand(msg->base_velocity_hint);
    }
  }

  bool processEndEffectorTarget(const manipulation_msgs::msg::PolicyOutput::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped target_out;
    if (!resolveEndEffectorTarget(msg, target_out)) {
      return false;
    }

    if (!isWithinWorkspace(target_out)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[WORKSPACE] rejecting EEF target frame=%s xyz=(%.4f, %.4f, %.4f) outside %s "
        "bounds x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f]",
        target_out.header.frame_id.c_str(),
        target_out.pose.position.x, target_out.pose.position.y, target_out.pose.position.z,
        workspace_frame_id_.c_str(),
        workspace_x_min_, workspace_x_max_, workspace_y_min_, workspace_y_max_,
        workspace_z_min_, workspace_z_max_);
      return false;
    }

    if (!gripper_moveit_goal_active_.load()) {
      releaseServoSuppressionForGripper();
    }

    const std::string source_frame =
      msg->reference_frame.empty() ? base_frame_ : msg->reference_frame;
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[EEF][%s] source=%s target=%s xyz=(%.4f, %.4f, %.4f)",
      isServoMode() ? "SERVO" : "MOVEIT",
      source_frame.c_str(), target_out.header.frame_id.c_str(),
      target_out.pose.position.x, target_out.pose.position.y, target_out.pose.position.z);

    if (isServoMode()) {
      return queueServoCommand(target_out);
    }

    if (!use_moveit_) {
      RCLCPP_WARN_ONCE(
        this->get_logger(),
        "[MOVEIT] disabled; cannot execute EEF pose targets without IK.");
      return false;
    }

    return sendMoveGroupGoal(target_out);
  }

  bool processJointDeltas(const std::vector<double> & joint_deltas)
  {
    if (!last_joint_state_) {
      RCLCPP_WARN(this->get_logger(), "Joint deltas received but no joint state available yet");
      return false;
    }

    std::unordered_map<std::string, double> joint_map;
    joint_map.reserve(last_joint_state_->name.size());
    for (size_t i = 0; i < last_joint_state_->name.size(); ++i) {
      if (i < last_joint_state_->position.size()) {
        joint_map[last_joint_state_->name[i]] = last_joint_state_->position[i];
      }
    }

    std::vector<std::string> joint_names = arm_joint_names_;
    std::vector<double> current_positions;
    current_positions.reserve(joint_names.size());

    if (!joint_names.empty()) {
      std::vector<std::string> filtered_joint_names;
      filtered_joint_names.reserve(joint_names.size());
      for (const auto & name : joint_names) {
        if (isGripperJointName(name)) {
          continue;
        }
        auto it = joint_map.find(name);
        if (it == joint_map.end()) {
          RCLCPP_WARN(this->get_logger(), "Joint '%s' not found in joint states", name.c_str());
          return false;
        }
        filtered_joint_names.push_back(name);
        current_positions.push_back(it->second);
      }
      joint_names = filtered_joint_names;
    } else {
      joint_names.clear();
      current_positions.clear();
      for (size_t i = 0; i < last_joint_state_->name.size(); ++i) {
        if (i >= last_joint_state_->position.size()) {
          continue;
        }
        if (isGripperJointName(last_joint_state_->name[i])) {
          continue;
        }
        joint_names.push_back(last_joint_state_->name[i]);
        current_positions.push_back(last_joint_state_->position[i]);
      }
    }

    if (joint_names.empty()) {
      RCLCPP_WARN(this->get_logger(), "No non-gripper arm joints available for joint deltas");
      return false;
    }

    if (joint_deltas.size() != joint_names.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Joint delta length mismatch (expected %zu, got %zu)",
        joint_names.size(), joint_deltas.size());
      return false;
    }

    std::vector<double> target_positions;
    target_positions.reserve(joint_deltas.size());
    for (size_t i = 0; i < joint_deltas.size(); ++i) {
      target_positions.push_back(current_positions[i] + joint_deltas[i]);
    }

    return sendJointTrajectoryGoal(
      arm_client_, joint_names, target_positions, arm_command_duration_sec_, "arm",
      &arm_goal_active_, &arm_goal_handle_);
  }

  GripperCommandDecision processGripperCommand(double command)
  {
    GripperCommandDecision decision;
    if (command < 0.0 || command > 1.0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "[GRIPPER] received out-of-range command=%.4f; expected normalized [0, 1]. "
        "Clamping before mapping to gripper joint position.",
        command);
    }

    const double clamped = std::clamp(command, 0.0, 1.0);
    if (
      latched_gripper_command_.has_value() &&
      std::abs(latched_gripper_command_.value() - clamped) < gripper_command_epsilon_)
    {
      return decision;
    }

    std::vector<std::string> joint_names;
    std::vector<double> target_positions;
    if (!buildGripperJointTargets(clamped, joint_names, target_positions)) {
      return decision;
    }

    decision.skip_arm_this_cycle = true;
    suppressServoForGripperTransition();
    if (!sendGripperMoveGroupGoal(joint_names, target_positions, clamped)) {
      if (!gripper_moveit_goal_active_.load()) {
        releaseServoSuppressionForGripper();
        latched_gripper_command_.reset();
      }
      RCLCPP_WARN(
        this->get_logger(),
        "[GRIPPER] failed to send %s command %.2f via MoveGroup",
        describeNormalizedGripperCommand(clamped).c_str(), clamped);
    }
    return decision;
  }

  void processBaseCommand(const geometry_msgs::msg::Twist & twist)
  {
    // Apply safety limits
    geometry_msgs::msg::Twist safe_twist = twist;

    // Clamp linear velocity
    double linear_mag = std::sqrt(
      twist.linear.x * twist.linear.x +
      twist.linear.y * twist.linear.y);
    if (linear_mag > max_base_velocity_) {
      double scale = max_base_velocity_ / linear_mag;
      safe_twist.linear.x *= scale;
      safe_twist.linear.y *= scale;
    }

    // Publish base command
    cmd_vel_pub_->publish(safe_twist);
    RCLCPP_DEBUG(
      this->get_logger(), "Base command: [%.2f, %.2f, %.2f]",
      safe_twist.linear.x, safe_twist.linear.y, safe_twist.angular.z);
  }

  bool isServoMode() const
  {
    return arm_execution_mode_ == "moveit_servo";
  }

  bool isServoCommandActive()
  {
    if (!servo_command_active_.load()) {
      return false;
    }

    std::lock_guard<std::mutex> lock(servo_mutex_);
    return servo_command_active_.load() && this->now() <= servo_command_until_;
  }

  static bool isServoAlreadyRunningMessage(const std::string & message)
  {
    std::string lowered = message;
    std::transform(
      lowered.begin(), lowered.end(), lowered.begin(),
      [](unsigned char ch) {return static_cast<char>(std::tolower(ch));});
    const bool has_already = lowered.find("already") != std::string::npos;
    const bool has_running = lowered.find("running") != std::string::npos;
    const bool has_started = lowered.find("started") != std::string::npos;
    return has_already && (has_running || has_started);
  }

  bool isServoReadyForCommands()
  {
    if (!isServoMode() || !wait_for_servo_ready_ || servo_started_.load()) {
      return true;
    }

    bool expected = false;
    if (servo_wait_started_.compare_exchange_strong(expected, true)) {
      servo_wait_start_steady_ = std::chrono::steady_clock::now();
    }

    const double elapsed_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - servo_wait_start_steady_).count();
    if (servo_ready_timeout_sec_ > 0.0 && elapsed_sec > servo_ready_timeout_sec_) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "[SERVO] timed out waiting for readiness (%.1fs > %.1fs). "
        "Dropping servo command until ready.",
        elapsed_sec, servo_ready_timeout_sec_);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "[SERVO] waiting for readiness before sending commands (elapsed %.1fs).",
        elapsed_sec);
    }
    return false;
  }

  void requestServoStart()
  {
    if (!isServoMode() || servo_started_.load() || servo_start_requested_.load()) {
      return;
    }

    if (!servo_start_client_) {
      servo_start_client_ = this->create_client<std_srvs::srv::Trigger>(servo_start_service_);
    }

    // Keep policy callback non-blocking; just skip this cycle if service graph
    // hasn't reported readiness yet.
    if (!servo_start_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "[SERVO] start service '%s' not available yet; commands will queue locally",
        servo_start_service_.c_str());
      return;
    }

    servo_start_requested_.store(true);
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    servo_start_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        servo_start_requested_.store(false);
        try {
          auto response = future.get();
          const bool started = response && (
            response->success || isServoAlreadyRunningMessage(response->message));
          if (started) {
            if (!servo_started_.load()) {
              RCLCPP_INFO(
                this->get_logger(), "[SERVO] ready via %s: %s",
                servo_start_service_.c_str(), response->message.c_str());
            }
            servo_started_.store(true);
            servo_wait_started_.store(false);
          } else {
            RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 5000,
              "[SERVO] failed to start via '%s'",
              servo_start_service_.c_str());
          }
        } catch (const std::exception & ex) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "[SERVO] start call failed: %s", ex.what());
        }
      });
  }

  bool queueServoCommand(const geometry_msgs::msg::PoseStamped & target_in_arm_base)
  {
    if (isServoSuppressedForGripper()) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[SERVO] dropping arm command while gripper MoveGroup goal is active");
      return false;
    }

    requestServoStart();
    if (!isServoReadyForCommands()) {
      return false;
    }

    geometry_msgs::msg::TransformStamped current_ee_tf;
    try {
      current_ee_tf = tf_buffer_->lookupTransform(
        arm_base_frame_, ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "[TF][SERVO] lookup current EEF '%s' in arm base '%s' failed: %s",
        ee_frame_.c_str(),
        arm_base_frame_.c_str(), ex.what());
      return false;
    }

    const double horizon_sec = std::max(1e-3, servo_command_horizon_sec_);

    geometry_msgs::msg::Twist target_twist;
    target_twist.linear.x =
      (target_in_arm_base.pose.position.x - current_ee_tf.transform.translation.x) / horizon_sec;
    target_twist.linear.y =
      (target_in_arm_base.pose.position.y - current_ee_tf.transform.translation.y) / horizon_sec;
    target_twist.linear.z =
      (target_in_arm_base.pose.position.z - current_ee_tf.transform.translation.z) / horizon_sec;

    tf2::Quaternion q_current;
    tf2::fromMsg(current_ee_tf.transform.rotation, q_current);
    if (q_current.length2() < 1e-12) {
      q_current.setValue(0.0, 0.0, 0.0, 1.0);
    } else {
      q_current.normalize();
    }

    tf2::Quaternion q_target;
    tf2::fromMsg(target_in_arm_base.pose.orientation, q_target);
    if (q_target.length2() < 1e-12) {
      q_target.setValue(0.0, 0.0, 0.0, 1.0);
    } else {
      q_target.normalize();
    }

    tf2::Quaternion q_delta = q_target * q_current.inverse();
    q_delta.normalize();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q_delta).getRPY(roll, pitch, yaw);

    target_twist.angular.x = roll / horizon_sec;
    target_twist.angular.y = pitch / horizon_sec;
    target_twist.angular.z = yaw / horizon_sec;

    auto clampVectorMagnitude = [](double & x, double & y, double & z, double max_magnitude) {
        if (max_magnitude <= 0.0) {
          return;
        }
        const double magnitude = std::sqrt(x * x + y * y + z * z);
        if (magnitude <= max_magnitude || magnitude < 1e-12) {
          return;
        }
        const double scale = max_magnitude / magnitude;
        x *= scale;
        y *= scale;
        z *= scale;
      };

    clampVectorMagnitude(
      target_twist.linear.x, target_twist.linear.y, target_twist.linear.z,
      servo_max_linear_velocity_);
    clampVectorMagnitude(
      target_twist.angular.x, target_twist.angular.y, target_twist.angular.z,
      servo_max_angular_velocity_);

    {
      std::lock_guard<std::mutex> lock(servo_mutex_);
      latest_servo_twist_ = target_twist;
      servo_command_until_ = this->now() + rclcpp::Duration::from_seconds(horizon_sec);
      servo_command_active_.store(true);
      servo_zero_sent_.store(false);
    }

    const double linear_speed = std::sqrt(
      target_twist.linear.x * target_twist.linear.x +
      target_twist.linear.y * target_twist.linear.y +
      target_twist.linear.z * target_twist.linear.z);
    const double angular_speed = std::sqrt(
      target_twist.angular.x * target_twist.angular.x +
      target_twist.angular.y * target_twist.angular.y +
      target_twist.angular.z * target_twist.angular.z);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[SERVO] current=(%.4f, %.4f, %.4f) target=(%.4f, %.4f, %.4f) "
      "cmd_lin=(%.3f, %.3f, %.3f) |v|=%.3f cmd_ang=(%.3f, %.3f, %.3f) |w|=%.3f horizon=%.2fs",
      current_ee_tf.transform.translation.x,
      current_ee_tf.transform.translation.y,
      current_ee_tf.transform.translation.z,
      target_in_arm_base.pose.position.x,
      target_in_arm_base.pose.position.y,
      target_in_arm_base.pose.position.z,
      target_twist.linear.x, target_twist.linear.y, target_twist.linear.z, linear_speed,
      target_twist.angular.x, target_twist.angular.y, target_twist.angular.z, angular_speed,
      horizon_sec);

    return true;
  }

  void publishServoCommand()
  {
    if (!isServoMode()) {
      return;
    }
    if (isServoSuppressedForGripper()) {
      return;
    }

    // Proactively request Servo start so readiness logs appear even before the
    // first policy output is received.
    if (wait_for_servo_ready_ && !servo_started_.load()) {
      requestServoStart();
    }

    geometry_msgs::msg::Twist twist_to_publish;
    bool should_publish = false;

    {
      std::lock_guard<std::mutex> lock(servo_mutex_);
      if (servo_command_active_.load()) {
        if (this->now() <= servo_command_until_) {
          twist_to_publish = latest_servo_twist_;
          should_publish = true;
        } else {
          servo_command_active_.store(false);
          if (!servo_zero_sent_.load()) {
            twist_to_publish = geometry_msgs::msg::Twist();
            servo_zero_sent_.store(true);
            should_publish = true;
          }
        }
      }
    }

    if (!should_publish) {
      return;
    }

    geometry_msgs::msg::TwistStamped twist_stamped;
    twist_stamped.header.stamp = this->now();
    twist_stamped.header.frame_id = arm_base_frame_;
    twist_stamped.twist = twist_to_publish;
    servo_twist_pub_->publish(twist_stamped);
  }

  void publishServoZeroTwist()
  {
    if (!isServoMode()) {
      return;
    }
    geometry_msgs::msg::TwistStamped twist_stamped;
    twist_stamped.header.stamp = this->now();
    twist_stamped.header.frame_id = arm_base_frame_;
    servo_twist_pub_->publish(twist_stamped);
  }

  void stopServoCommand()
  {
    if (!isServoMode()) {
      return;
    }

    bool should_publish_zero = false;
    {
      std::lock_guard<std::mutex> lock(servo_mutex_);
      if (servo_command_active_.load() || !servo_zero_sent_.load()) {
        should_publish_zero = true;
      }
      servo_command_active_.store(false);
      servo_zero_sent_.store(true);
    }

    if (should_publish_zero) {
      publishServoZeroTwist();
    }
  }

  moveit_msgs::msg::Constraints buildPoseGoalConstraints(
    const geometry_msgs::msg::PoseStamped & target) const
  {
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::PositionConstraint position_constraint;
    position_constraint.header = target.header;
    position_constraint.link_name = move_group_eef_link_;
    position_constraint.weight = 1.0;

    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    sphere.dimensions.resize(1);
    sphere.dimensions[0] = std::max(1e-4, moveit_position_tolerance_);
    position_constraint.constraint_region.primitives.push_back(sphere);

    geometry_msgs::msg::Pose region_pose;
    region_pose.position = target.pose.position;
    region_pose.orientation.w = 1.0;
    position_constraint.constraint_region.primitive_poses.push_back(region_pose);
    constraints.position_constraints.push_back(position_constraint);

    // Only add orientation constraint when the target has a meaningful orientation
    // (non-identity quaternion). Policy outputs typically have position-only targets;
    // adding a tight orientation constraint causes OMPL to fail finding valid IK states.
    tf2::Quaternion orientation;
    tf2::fromMsg(target.pose.orientation, orientation);
    bool has_orientation = orientation.length2() > 1e-12 &&
      std::abs(orientation.w() - 1.0) > 1e-3;

    if (has_orientation) {
      moveit_msgs::msg::OrientationConstraint orientation_constraint;
      orientation_constraint.header = target.header;
      orientation_constraint.link_name = move_group_eef_link_;
      orientation.normalize();
      orientation_constraint.orientation = tf2::toMsg(orientation);
      double orientation_tol = std::max(1e-4, moveit_orientation_tolerance_);
      orientation_constraint.absolute_x_axis_tolerance = orientation_tol;
      orientation_constraint.absolute_y_axis_tolerance = orientation_tol;
      orientation_constraint.absolute_z_axis_tolerance = orientation_tol;
      orientation_constraint.weight = 1.0;
      constraints.orientation_constraints.push_back(orientation_constraint);
    }

    return constraints;
  }

  moveit_msgs::msg::Constraints buildJointGoalConstraints(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & joint_positions,
    const std::string & constraint_name = "") const
  {
    moveit_msgs::msg::Constraints constraints;
    constraints.name = constraint_name;
    const double joint_tolerance = std::max(1e-4, moveit_position_tolerance_);

    for (size_t i = 0; i < joint_names.size(); ++i) {
      moveit_msgs::msg::JointConstraint joint_constraint;
      joint_constraint.joint_name = joint_names[i];
      joint_constraint.position = joint_positions[i];
      joint_constraint.tolerance_above = joint_tolerance;
      joint_constraint.tolerance_below = joint_tolerance;
      joint_constraint.weight = 1.0;
      constraints.joint_constraints.push_back(joint_constraint);
    }

    return constraints;
  }

  void startupReadyPoseTimerCallback()
  {
    if (!move_to_ready_on_startup_ || startup_ready_pose_completed_.load()) {
      if (ready_pose_timer_) {
        ready_pose_timer_->cancel();
      }
      return;
    }

    if (std::chrono::steady_clock::now() < ready_pose_start_time_steady_) {
      return;
    }

    if (startup_ready_pose_goal_active_.load()) {
      return;
    }

    if (startup_ready_pose_attempts_ >= ready_pose_max_attempts_) {
      startup_ready_pose_completed_.store(true);
      if (ready_pose_timer_) {
        ready_pose_timer_->cancel();
      }
      RCLCPP_ERROR(
        this->get_logger(),
        "[READY] failed after %d attempts. "
        "Continuing without startup ready move.",
        ready_pose_max_attempts_);
      return;
    }

    (void)moveToReadyPose();
  }

  void visualServoStateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string new_state = msg->data;
    if (new_state != "DONE") {
      if (new_state != last_visual_servo_state_) {
        post_visual_servo_ready_pose_completed_.store(false);
        post_visual_servo_ready_pose_goal_active_.store(false);
        post_visual_servo_ready_pose_attempts_ = 0;
      }
      last_visual_servo_state_ = new_state;
      return;
    }

    last_visual_servo_state_ = new_state;
    if (!return_to_ready_after_visual_servo_ || !use_moveit_ || ready_pose_joint_names_.empty()) {
      return;
    }
    if (
      post_visual_servo_ready_pose_completed_.load() ||
      post_visual_servo_ready_pose_goal_active_.load())
    {
      return;
    }
    if (post_visual_servo_ready_pose_attempts_ >= ready_pose_max_attempts_) {
      post_visual_servo_ready_pose_completed_.store(true);
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "[READY][POST] failed after %d attempts. Staying in the current pose.",
        ready_pose_max_attempts_);
      return;
    }

    const auto retry_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(ready_pose_retry_period_sec_));
    const auto now = std::chrono::steady_clock::now();
    if (now < last_post_visual_servo_ready_pose_attempt_steady_ + retry_period) {
      return;
    }

    last_post_visual_servo_ready_pose_attempt_steady_ = now;
    (void)moveToPostVisualServoReadyPose();
  }

  bool moveToReadyPose()
  {
    if (!move_to_ready_on_startup_ || startup_ready_pose_completed_.load()) {
      return false;
    }

    if (!ensureMoveGroupClientReady()) {
      return false;
    }

    if (moveit_goal_active_.load()) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[READY] MoveIt goal already active; deferring startup ready pose");
      return false;
    }

    const int attempt_number = startup_ready_pose_attempts_ + 1;
    startup_ready_pose_attempts_ = attempt_number;

    moveit_msgs::action::MoveGroup::Goal goal;
    goal.request.group_name = move_group_name_;
    goal.request.num_planning_attempts = moveit_planning_attempts_;
    goal.request.allowed_planning_time = moveit_planning_time_;
    goal.request.max_velocity_scaling_factor = moveit_velocity_scaling_;
    goal.request.max_acceleration_scaling_factor = moveit_accel_scaling_;
    goal.request.goal_constraints.push_back(
      buildJointGoalConstraints(
        ready_pose_joint_names_, ready_pose_joint_positions_, "startup_ready_pose"));
    goal.request.start_state.is_diff = true;

    goal.planning_options.plan_only = false;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = false;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    auto send_goal_options =
      rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this, attempt_number](
      rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(
            this->get_logger(),
            "[READY] goal rejected on attempt %d/%d",
            attempt_number, ready_pose_max_attempts_);
          moveit_goal_active_.store(false);
          startup_ready_pose_goal_active_.store(false);
          std::lock_guard<std::mutex> lock(goal_mutex_);
          moveit_goal_handle_.reset();
          return;
        }
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          moveit_goal_handle_ = goal_handle;
        }
        moveit_goal_active_.store(true);
        startup_ready_pose_goal_active_.store(true);
      };

    send_goal_options.result_callback =
      [this, attempt_number](const auto & result) {
        moveit_goal_active_.store(false);
        startup_ready_pose_goal_active_.store(false);
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          moveit_goal_handle_.reset();
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          startup_ready_pose_completed_.store(true);
          if (ready_pose_timer_) {
            ready_pose_timer_->cancel();
          }
          RCLCPP_INFO(this->get_logger(), "[READY] startup ready pose reached");
          return;
        }

        RCLCPP_WARN(
          this->get_logger(),
          "[READY] failed with code %d on attempt %d/%d",
          static_cast<int>(result.code), attempt_number, ready_pose_max_attempts_);
        if (attempt_number >= ready_pose_max_attempts_) {
          startup_ready_pose_completed_.store(true);
          if (ready_pose_timer_) {
            ready_pose_timer_->cancel();
          }
          RCLCPP_ERROR(
            this->get_logger(),
            "[READY] failed after %d attempts. "
            "Continuing without startup ready move.",
            ready_pose_max_attempts_);
        }
      };

    RCLCPP_INFO(
      this->get_logger(),
      "[READY] sending startup ready pose via MoveIt (%d/%d)",
      attempt_number, ready_pose_max_attempts_);
    moveit_goal_active_.store(true);
    startup_ready_pose_goal_active_.store(true);
    move_group_client_->async_send_goal(goal, send_goal_options);
    return true;
  }

  bool moveToPostVisualServoReadyPose()
  {
    if (!return_to_ready_after_visual_servo_ || !use_moveit_ || ready_pose_joint_names_.empty()) {
      return false;
    }

    if (!ensureMoveGroupClientReady()) {
      return false;
    }

    if (moveit_goal_active_.load()) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[READY][POST] MoveIt goal already active; deferring return to ready pose");
      return false;
    }

    stopServoCommand();

    const int attempt_number = post_visual_servo_ready_pose_attempts_ + 1;
    post_visual_servo_ready_pose_attempts_ = attempt_number;

    moveit_msgs::action::MoveGroup::Goal goal;
    goal.request.group_name = move_group_name_;
    goal.request.num_planning_attempts = moveit_planning_attempts_;
    goal.request.allowed_planning_time = moveit_planning_time_;
    goal.request.max_velocity_scaling_factor = moveit_velocity_scaling_;
    goal.request.max_acceleration_scaling_factor = moveit_accel_scaling_;
    goal.request.goal_constraints.push_back(
      buildJointGoalConstraints(
        ready_pose_joint_names_, ready_pose_joint_positions_, "post_visual_servo_ready_pose"));
    goal.request.start_state.is_diff = true;

    goal.planning_options.plan_only = false;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = false;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    auto send_goal_options =
      rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this, attempt_number](
      rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr goal_handle) {
        if (!goal_handle) {
          moveit_goal_active_.store(false);
          post_visual_servo_ready_pose_goal_active_.store(false);
          std::lock_guard<std::mutex> lock(goal_mutex_);
          moveit_goal_handle_.reset();
          RCLCPP_WARN(
            this->get_logger(),
            "[READY][POST] goal rejected on attempt %d/%d",
            attempt_number, ready_pose_max_attempts_);
          return;
        }
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          moveit_goal_handle_ = goal_handle;
        }
        moveit_goal_active_.store(true);
        post_visual_servo_ready_pose_goal_active_.store(true);
      };

    send_goal_options.result_callback =
      [this, attempt_number](const auto & result) {
        moveit_goal_active_.store(false);
        post_visual_servo_ready_pose_goal_active_.store(false);
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          moveit_goal_handle_.reset();
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          post_visual_servo_ready_pose_completed_.store(true);
          RCLCPP_INFO(this->get_logger(), "[READY][POST] returned to ready pose");
          return;
        }

        RCLCPP_WARN(
          this->get_logger(),
          "[READY][POST] failed with code %d on attempt %d/%d",
          static_cast<int>(result.code), attempt_number, ready_pose_max_attempts_);
        if (attempt_number >= ready_pose_max_attempts_) {
          post_visual_servo_ready_pose_completed_.store(true);
          RCLCPP_ERROR(
            this->get_logger(),
            "[READY][POST] failed after %d attempts. Staying in the current pose.",
            ready_pose_max_attempts_);
        }
      };

    RCLCPP_INFO(
      this->get_logger(),
      "[READY][POST] returning to ready pose after visual-servo grasp (%d/%d)",
      attempt_number, ready_pose_max_attempts_);
    moveit_goal_active_.store(true);
    post_visual_servo_ready_pose_goal_active_.store(true);
    move_group_client_->async_send_goal(goal, send_goal_options);
    return true;
  }

  bool ensureMoveGroupClientReady()
  {
    if (!move_group_client_) {
      move_group_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
        this, move_group_action_);
    }

    auto wait_duration =
      std::chrono::milliseconds(static_cast<int>(moveit_action_wait_sec_ * 1000));
    if (move_group_client_->wait_for_action_server(wait_duration)) {
      return true;
    }

    const std::string fallback_action = "/move_group";
    if (move_group_action_ != fallback_action) {
      auto fallback_client =
        rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(this, fallback_action);
      if (fallback_client->wait_for_action_server(wait_duration)) {
        move_group_action_ = fallback_action;
        move_group_client_ = fallback_client;
        RCLCPP_WARN(
          this->get_logger(),
          "[MOVEIT] action server not found on configured name; using fallback '%s'",
          move_group_action_.c_str());
        return true;
      }
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "[MOVEIT] action server not available on '%s'",
      move_group_action_.c_str());
    return false;
  }

  bool sendMoveGroupGoal(const geometry_msgs::msg::PoseStamped & target)
  {
    if (!ensureMoveGroupClientReady()) {
      return false;
    }
    if (gripper_moveit_goal_active_.load()) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[MOVEIT] arm goal skipped while gripper MoveGroup goal is active");
      return false;
    }

    if (moveit_goal_active_.load()) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[MOVEIT] goal already active; skipping new target");
      return false;
    }

    moveit_msgs::action::MoveGroup::Goal goal;
    goal.request.group_name = move_group_name_;
    goal.request.num_planning_attempts = moveit_planning_attempts_;
    goal.request.allowed_planning_time = moveit_planning_time_;
    goal.request.max_velocity_scaling_factor = moveit_velocity_scaling_;
    goal.request.max_acceleration_scaling_factor = moveit_accel_scaling_;
    goal.request.goal_constraints.push_back(buildPoseGoalConstraints(target));
    goal.request.start_state.is_diff = true;

    goal.planning_options.plan_only = false;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = false;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    auto send_goal_options =
      rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr goal_handle)
      {
        if (!goal_handle) {
          RCLCPP_WARN(this->get_logger(), "[MOVEIT] goal was rejected");
          moveit_goal_active_.store(false);
          std::lock_guard<std::mutex> lock(goal_mutex_);
          moveit_goal_handle_.reset();
          return;
        }
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          moveit_goal_handle_ = goal_handle;
        }
        moveit_goal_active_.store(true);
      };

    send_goal_options.result_callback =
      [this](const auto & result) {
        moveit_goal_active_.store(false);
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          moveit_goal_handle_.reset();
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(
            this->get_logger(), "[MOVEIT] goal failed with code %d",
            static_cast<int>(result.code));
        } else {
          RCLCPP_INFO(this->get_logger(), "[MOVEIT] goal succeeded");
        }
      };

    RCLCPP_INFO(
      this->get_logger(),
      "[MOVEIT] sending pose goal frame=%s xyz=(%.4f, %.4f, %.4f) eef_link=%s",
      target.header.frame_id.c_str(),
      target.pose.position.x, target.pose.position.y, target.pose.position.z,
      move_group_eef_link_.c_str());
    moveit_goal_active_.store(true);
    move_group_client_->async_send_goal(goal, send_goal_options);
    return true;
  }

  bool sendJointTrajectoryGoal(
    const rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr & client,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions,
    double duration_sec,
    const std::string & label,
    std::atomic<bool> * active_flag = nullptr,
    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr *
    handle_slot = nullptr)
  {
    if (!client) {
      RCLCPP_WARN(this->get_logger(), "No %s trajectory action client available", label.c_str());
      return false;
    }

    if (!client->wait_for_action_server(std::chrono::milliseconds(200))) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "%s trajectory action server not available", label.c_str());
      return false;
    }

    control_msgs::action::FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start =
      rclcpp::Duration::from_seconds(std::max(0.1, duration_sec));
    goal.trajectory.points.push_back(point);

    auto send_goal_options =
      rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this, label, active_flag,
        handle_slot](rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::
        SharedPtr goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(this->get_logger(), "%s trajectory goal rejected", label.c_str());
          if (active_flag) {
            active_flag->store(false);
          }
          if (handle_slot) {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            handle_slot->reset();
          }
          return;
        }
        if (handle_slot) {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          *handle_slot = goal_handle;
        }
        if (active_flag) {
          active_flag->store(true);
        }
      };

    send_goal_options.result_callback =
      [this, label, active_flag, handle_slot](const auto & result) {
        if (active_flag) {
          active_flag->store(false);
        }
        if (handle_slot) {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          handle_slot->reset();
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(
            this->get_logger(), "%s trajectory failed with code %d",
            label.c_str(), static_cast<int>(result.code));
        } else {
          RCLCPP_DEBUG(this->get_logger(), "%s trajectory succeeded", label.c_str());
        }
      };

    if (active_flag) {
      active_flag->store(true);
    }
    client->async_send_goal(goal, send_goal_options);
    return true;
  }

  void cancelActiveGoals()
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (move_group_client_ && moveit_goal_handle_) {
      move_group_client_->async_cancel_goal(moveit_goal_handle_);
    }
    if (move_group_client_ && gripper_moveit_goal_handle_) {
      move_group_client_->async_cancel_goal(gripper_moveit_goal_handle_);
    }
    if (arm_client_ && arm_goal_handle_) {
      arm_client_->async_cancel_goal(arm_goal_handle_);
    }
  }

  void safetyCheck()
  {
    auto now = this->now();
    checkGripperMoveGroupTimeout();

    if (move_to_ready_on_startup_ && !startup_ready_pose_completed_.load() &&
      startup_ready_pose_goal_active_.load())
    {
      // Keep startup MoveIt goal alive while policy output is intentionally gated.
      last_policy_time_ = now;
      safety_stop_active_ = false;
      return;
    }

    auto time_since_last_policy = (now - last_policy_time_).seconds();

    if (time_since_last_policy > safety_timeout_sec_) {
      // No policy output received recently - stop the robot
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "[SAFETY] no policy output for %.2fs (> %.2fs); stopping robot",
        time_since_last_policy, safety_timeout_sec_);

      geometry_msgs::msg::Twist zero_twist;
      cmd_vel_pub_->publish(zero_twist);
      if (!safety_stop_active_) {
        cancelActiveGoals();
        stopServoCommand();
        safety_stop_active_ = true;
      }
      return;
    }
    safety_stop_active_ = false;
  }

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_ = msg;
  }

  bool isWithinWorkspace(const geometry_msgs::msg::PoseStamped & target) const
  {
    geometry_msgs::msg::PointStamped point_in;
    point_in.header = target.header;
    point_in.point = target.pose.position;

    geometry_msgs::msg::PointStamped point_ws;
    if (target.header.frame_id != workspace_frame_id_) {
      try {
        auto transform = tf_buffer_->lookupTransform(
          workspace_frame_id_, target.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(point_in, point_ws, transform);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(
          this->get_logger(),
          "[TF][WORKSPACE] transform failed: %s", ex.what());
        return false;
      }
    } else {
      point_ws = point_in;
    }

    return point_ws.point.x >= workspace_x_min_ && point_ws.point.x <= workspace_x_max_ &&
           point_ws.point.y >= workspace_y_min_ && point_ws.point.y <= workspace_y_max_ &&
           point_ws.point.z >= workspace_z_min_ && point_ws.point.z <= workspace_z_max_;
  }

  bool resolveEndEffectorTarget(
    const manipulation_msgs::msg::PolicyOutput::SharedPtr msg,
    geometry_msgs::msg::PoseStamped & target_out)
  {
    const std::string reference_frame =
      msg->reference_frame.empty() ? base_frame_ : msg->reference_frame;

    geometry_msgs::msg::PoseStamped pose_in_ref;
    pose_in_ref.header.frame_id = reference_frame;
    pose_in_ref.header.stamp = msg->header.stamp;

    if (!eef_target_is_delta_) {
      // Policy outputs an absolute pose in reference_frame.
      pose_in_ref.pose = msg->eef_target_pose;
    } else {
      // Delta mode: look up current EEF pose in reference_frame, then add the delta
      // to produce an absolute target pose, still expressed in reference_frame.
      geometry_msgs::msg::TransformStamped ee_transform;
      try {
        ee_transform = tf_buffer_->lookupTransform(
          reference_frame, ee_frame_, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(
          this->get_logger(),
          "[TF][EEF] lookup current EEF '%s' in '%s' failed: %s",
          ee_frame_.c_str(), reference_frame.c_str(), ex.what());
        return false;
      }

      tf2::Transform current_ee;
      tf2::fromMsg(ee_transform.transform, current_ee);

      tf2::Vector3 delta_translation(
        msg->eef_target_pose.position.x,
        msg->eef_target_pose.position.y,
        msg->eef_target_pose.position.z);

      tf2::Quaternion delta_rotation(
        msg->eef_target_pose.orientation.x,
        msg->eef_target_pose.orientation.y,
        msg->eef_target_pose.orientation.z,
        msg->eef_target_pose.orientation.w);
      if (delta_rotation.length2() < 1e-12) {
        delta_rotation.setValue(0.0, 0.0, 0.0, 1.0);
      } else {
        delta_rotation.normalize();
      }

      tf2::Vector3 target_translation = current_ee.getOrigin() + delta_translation;
      tf2::Quaternion target_rotation = delta_rotation * current_ee.getRotation();
      target_rotation.normalize();

      pose_in_ref.pose.position.x = target_translation.x();
      pose_in_ref.pose.position.y = target_translation.y();
      pose_in_ref.pose.position.z = target_translation.z();
      pose_in_ref.pose.orientation = tf2::toMsg(target_rotation);
    }

    // Transform target into arm_base_frame_ so MoveIt receives pose constraints
    // in the robot arm base frame (not in the EEF link frame).
    if (reference_frame == arm_base_frame_) {
      target_out = pose_in_ref;
      target_out.header.frame_id = arm_base_frame_;
      return true;
    }

    try {
      auto tf_to_arm_base = tf_buffer_->lookupTransform(
        arm_base_frame_, reference_frame, tf2::TimePointZero);
      tf2::doTransform(pose_in_ref, target_out, tf_to_arm_base);
      target_out.header.frame_id = arm_base_frame_;
      target_out.header.stamp = msg->header.stamp;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "[TF][EEF] transform from '%s' to arm base '%s' failed: %s",
        reference_frame.c_str(), arm_base_frame_.c_str(), ex.what());
      return false;
    }

    return true;
  }

  // Parameters
  std::string base_frame_;
  std::string ee_frame_;
  std::string joint_states_topic_;
  std::string navigate_to_pose_action_;
  std::string follow_joint_trajectory_action_;
  bool use_moveit_;
  std::string move_group_action_;
  std::string move_group_name_;
  std::string move_group_eef_link_;
  std::string gripper_move_group_name_;
  double gripper_move_group_timeout_sec_;
  double moveit_action_wait_sec_;
  double moveit_planning_time_;
  int moveit_planning_attempts_;
  double moveit_velocity_scaling_;
  double moveit_accel_scaling_;
  double moveit_position_tolerance_;
  double moveit_orientation_tolerance_;
  bool move_to_ready_on_startup_;
  std::vector<std::string> ready_pose_joint_names_;
  std::vector<double> ready_pose_joint_positions_;
  double ready_pose_start_delay_sec_;
  double ready_pose_retry_period_sec_;
  int ready_pose_max_attempts_;
  std::vector<std::string> arm_joint_names_;
  double arm_command_duration_sec_;
  std::string gripper_joint_name_;
  std::vector<std::string> gripper_joint_names_;
  double gripper_open_position_;
  double gripper_closed_position_;
  std::vector<double> gripper_open_positions_;
  std::vector<double> gripper_closed_positions_;
  double gripper_command_epsilon_;
  double max_base_velocity_;
  double max_arm_velocity_;
  double safety_timeout_sec_;
  std::string workspace_frame_id_;
  double workspace_x_min_;
  double workspace_x_max_;
  double workspace_y_min_;
  double workspace_y_max_;
  double workspace_z_min_;
  double workspace_z_max_;
  bool eef_target_is_delta_;
  std::string arm_base_frame_;
  std::string arm_execution_mode_;
  std::string servo_cartesian_topic_;
  double servo_publish_rate_hz_;
  double servo_command_horizon_sec_;
  double servo_max_linear_velocity_;
  double servo_max_angular_velocity_;
  bool pause_base_during_servo_;
  std::string servo_start_service_;
  bool wait_for_servo_ready_;
  double servo_ready_timeout_sec_;
  std::string visual_servo_state_topic_;
  bool return_to_ready_after_visual_servo_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS communication
  rclcpp::Subscription<manipulation_msgs::msg::PolicyOutput>::SharedPtr policy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr visual_servo_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_twist_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  rclcpp::TimerBase::SharedPtr servo_publish_timer_;
  rclcpp::TimerBase::SharedPtr ready_pose_timer_;

  // State
  rclcpp::Time last_policy_time_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
  std::optional<double> latched_gripper_command_;
  std::atomic<bool> moveit_goal_active_{false};
  std::atomic<bool> arm_goal_active_{false};
  std::atomic<bool> gripper_moveit_goal_active_{false};
  std::atomic<bool> servo_suppressed_for_gripper_{false};
  std::atomic<uint64_t> gripper_goal_generation_{0};
  std::atomic<bool> startup_ready_pose_completed_{true};
  std::atomic<bool> startup_ready_pose_goal_active_{false};
  int startup_ready_pose_attempts_{0};
  std::mutex goal_mutex_;
  std::mutex servo_mutex_;
  rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr moveit_goal_handle_;
  rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr
    gripper_moveit_goal_handle_;
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
    arm_goal_handle_;
  geometry_msgs::msg::Twist latest_servo_twist_;
  rclcpp::Time servo_command_until_{0, 0, RCL_ROS_TIME};
  std::atomic<bool> servo_command_active_{false};
  std::atomic<bool> servo_zero_sent_{true};
  std::atomic<bool> servo_started_{false};
  std::atomic<bool> servo_start_requested_{false};
  std::atomic<bool> servo_wait_started_{false};
  std::string last_visual_servo_state_;
  std::atomic<bool> post_visual_servo_ready_pose_completed_{false};
  std::atomic<bool> post_visual_servo_ready_pose_goal_active_{false};
  int post_visual_servo_ready_pose_attempts_{0};
  std::chrono::steady_clock::time_point ready_pose_start_time_steady_{
    std::chrono::steady_clock::time_point::min()};
  std::chrono::steady_clock::time_point gripper_goal_start_steady_{
    std::chrono::steady_clock::time_point::min()};
  std::chrono::steady_clock::time_point servo_wait_start_steady_{
    std::chrono::steady_clock::time_point::min()};
  std::chrono::steady_clock::time_point last_post_visual_servo_ready_pose_attempt_steady_{
    std::chrono::steady_clock::time_point::min()};
  bool safety_stop_active_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdapterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
