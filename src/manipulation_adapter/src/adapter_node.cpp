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

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <manipulation_msgs/msg/policy_output.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/planning_options.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>

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
    this->declare_parameter<std::string>("ee_frame", "piper_gripper_base");
    this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    this->declare_parameter<std::string>("navigate_to_pose_action", "/navigate_to_pose");
    this->declare_parameter<std::string>(
      "follow_joint_trajectory_action", "/arm_controller/follow_joint_trajectory");
    this->declare_parameter<bool>("use_moveit", true);
    this->declare_parameter<std::string>("move_group_action", "/move_action");
    this->declare_parameter<std::string>("move_group_name", "arm");
    this->declare_parameter<std::string>("move_group_eef_link", "piper_link6");
    this->declare_parameter<double>("moveit_action_wait_sec", 1.0);
    this->declare_parameter<double>("moveit_planning_time", 2.0);
    this->declare_parameter<int>("moveit_planning_attempts", 3);
    this->declare_parameter<double>("moveit_velocity_scaling", 0.5);
    this->declare_parameter<double>("moveit_accel_scaling", 0.5);
    this->declare_parameter<double>("moveit_position_tolerance", 0.01);
    this->declare_parameter<double>("moveit_orientation_tolerance", 0.1);
    this->declare_parameter<std::vector<std::string>>("arm_joint_names", std::vector<std::string>{});
    this->declare_parameter<double>("arm_command_duration_sec", 1.5);
    this->declare_parameter<std::string>(
      "gripper_follow_joint_trajectory_action", "/gripper_controller/follow_joint_trajectory");
    this->declare_parameter<std::string>("gripper_joint_name", "piper_joint7");
    this->declare_parameter<std::vector<std::string>>("gripper_joint_names",
                                                      std::vector<std::string>{});
    this->declare_parameter<double>("gripper_open_position", 0.035);
    this->declare_parameter<double>("gripper_closed_position", 0.0);
    this->declare_parameter<std::vector<double>>("gripper_open_positions",
                                                 std::vector<double>{});
    this->declare_parameter<std::vector<double>>("gripper_closed_positions",
                                                 std::vector<double>{});
    this->declare_parameter<double>("gripper_command_duration_sec", 0.75);
    this->declare_parameter<double>("gripper_command_epsilon", 0.01);
    this->declare_parameter<double>("max_base_velocity", 0.5);
    this->declare_parameter<double>("max_arm_velocity", 1.0);
    this->declare_parameter<double>("safety_timeout_sec", 2.0);
    this->declare_parameter<double>("workspace_x_min", 0.1);
    this->declare_parameter<double>("workspace_x_max", 0.8);
    this->declare_parameter<double>("workspace_y_min", -0.5);
    this->declare_parameter<double>("workspace_y_max", 0.5);
    this->declare_parameter<double>("workspace_z_min", 0.0);
    this->declare_parameter<double>("workspace_z_max", 1.0);
    this->declare_parameter<bool>("eef_target_is_delta", false);

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
    moveit_action_wait_sec_ = this->get_parameter("moveit_action_wait_sec").as_double();
    moveit_planning_time_ = this->get_parameter("moveit_planning_time").as_double();
    moveit_planning_attempts_ = this->get_parameter("moveit_planning_attempts").as_int();
    moveit_velocity_scaling_ = this->get_parameter("moveit_velocity_scaling").as_double();
    moveit_accel_scaling_ = this->get_parameter("moveit_accel_scaling").as_double();
    moveit_position_tolerance_ = this->get_parameter("moveit_position_tolerance").as_double();
    moveit_orientation_tolerance_ =
      this->get_parameter("moveit_orientation_tolerance").as_double();
    arm_joint_names_ = this->get_parameter("arm_joint_names").as_string_array();
    arm_command_duration_sec_ = this->get_parameter("arm_command_duration_sec").as_double();
    gripper_follow_joint_trajectory_action_ =
      this->get_parameter("gripper_follow_joint_trajectory_action").as_string();
    gripper_joint_name_ = this->get_parameter("gripper_joint_name").as_string();
    gripper_joint_names_ = this->get_parameter("gripper_joint_names").as_string_array();
    gripper_open_position_ = this->get_parameter("gripper_open_position").as_double();
    gripper_closed_position_ = this->get_parameter("gripper_closed_position").as_double();
    gripper_open_positions_ = this->get_parameter("gripper_open_positions").as_double_array();
    gripper_closed_positions_ = this->get_parameter("gripper_closed_positions").as_double_array();
    gripper_command_duration_sec_ =
      this->get_parameter("gripper_command_duration_sec").as_double();
    gripper_command_epsilon_ = this->get_parameter("gripper_command_epsilon").as_double();
    max_base_velocity_ = this->get_parameter("max_base_velocity").as_double();
    max_arm_velocity_ = this->get_parameter("max_arm_velocity").as_double();
    safety_timeout_sec_ = this->get_parameter("safety_timeout_sec").as_double();
    workspace_x_min_ = this->get_parameter("workspace_x_min").as_double();
    workspace_x_max_ = this->get_parameter("workspace_x_max").as_double();
    workspace_y_min_ = this->get_parameter("workspace_y_min").as_double();
    workspace_y_max_ = this->get_parameter("workspace_y_max").as_double();
    workspace_z_min_ = this->get_parameter("workspace_z_min").as_double();
    workspace_z_max_ = this->get_parameter("workspace_z_max").as_double();
    eef_target_is_delta_ = this->get_parameter("eef_target_is_delta").as_bool();

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

    // Create publishers for hardware commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, navigate_to_pose_action_);
    arm_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this, follow_joint_trajectory_action_);
    gripper_client_ =
      rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this, gripper_follow_joint_trajectory_action_);
    if (use_moveit_) {
      move_group_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
        this, move_group_action_);
    }

    // Create safety timer
    safety_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(safety_timeout_sec_ * 1000)),
      std::bind(&AdapterNode::safetyCheck, this));

    RCLCPP_INFO(this->get_logger(), "Adapter node initialized");
    RCLCPP_INFO(this->get_logger(), "  Base frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  EE frame: %s", ee_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  MoveIt enabled: %s", use_moveit_ ? "true" : "false");
    if (use_moveit_) {
      RCLCPP_INFO(this->get_logger(), "  MoveIt action: %s", move_group_action_.c_str());
      RCLCPP_INFO(this->get_logger(), "  MoveIt group: %s", move_group_name_.c_str());
      RCLCPP_INFO(this->get_logger(), "  MoveIt EEF link: %s", move_group_eef_link_.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "  EEF target is delta: %s", eef_target_is_delta_ ? "true" : "false");
  }

private:
  void policyCallback(const manipulation_msgs::msg::PolicyOutput::SharedPtr msg)
  {
    last_policy_time_ = this->now();

    RCLCPP_DEBUG(this->get_logger(), "Received policy output");

    bool arm_command_sent = false;
    if (msg->has_eef_target) {
      if (msg->has_joint_deltas) {
        RCLCPP_DEBUG(this->get_logger(),
                     "Policy output has both EEF target and joint deltas; prioritizing EEF target");
      }
      arm_command_sent = processEndEffectorTarget(msg);
    }

    if (!arm_command_sent && msg->has_joint_deltas) {
      arm_command_sent = processJointDeltas(msg->joint_deltas);
    }

    // Process gripper command if active
    if (msg->gripper_active) {
      processGripperCommand(msg->gripper_command);
    }

    // Process base hint if available
    if (msg->has_base_hint) {
      processBaseCommand(msg->base_velocity_hint);
    }
  }

  bool processEndEffectorTarget(const manipulation_msgs::msg::PolicyOutput::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped target_out;
    if (!resolveEndEffectorTarget(msg, target_out)) {
      return false;
    }

    if (!isWithinWorkspace(target_out.pose.position)) {
      RCLCPP_WARN(
        this->get_logger(),
        "EEF target outside workspace bounds [%.2f, %.2f] [%.2f, %.2f] [%.2f, %.2f]",
        workspace_x_min_, workspace_x_max_, workspace_y_min_, workspace_y_max_,
        workspace_z_min_, workspace_z_max_);
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Processing EEF target at [%.2f, %.2f, %.2f]",
                target_out.pose.position.x,
                target_out.pose.position.y,
                target_out.pose.position.z);

    if (!use_moveit_) {
      RCLCPP_WARN_ONCE(this->get_logger(),
                       "MoveIt disabled; cannot execute EEF pose targets without IK.");
      return false;
    }

    return sendMoveGroupGoal(target_out);
  }

  bool processJointDeltas(const std::vector<double>& joint_deltas)
  {
    if (!last_joint_state_) {
      RCLCPP_WARN(this->get_logger(), "Joint deltas received but no joint state available yet");
      return false;
    }

    std::vector<std::string> joint_names = arm_joint_names_;
    std::vector<double> current_positions;
    current_positions.reserve(joint_names.size());

    if (!joint_names.empty()) {
      std::unordered_map<std::string, double> joint_map;
      joint_map.reserve(last_joint_state_->name.size());
      for (size_t i = 0; i < last_joint_state_->name.size(); ++i) {
        if (i < last_joint_state_->position.size()) {
          joint_map[last_joint_state_->name[i]] = last_joint_state_->position[i];
        }
      }
      for (const auto& name : joint_names) {
        auto it = joint_map.find(name);
        if (it == joint_map.end()) {
          RCLCPP_WARN(this->get_logger(), "Joint '%s' not found in joint states", name.c_str());
          return false;
        }
        current_positions.push_back(it->second);
      }
    } else {
      joint_names = last_joint_state_->name;
      current_positions = last_joint_state_->position;
    }

    if (joint_deltas.size() != joint_names.size()) {
      RCLCPP_WARN(this->get_logger(),
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

  void processGripperCommand(double command)
  {
    double clamped = std::clamp(command, 0.0, 1.0);
    if (last_gripper_command_.has_value() &&
        std::abs(last_gripper_command_.value() - clamped) < gripper_command_epsilon_) {
      return;
    }

    bool sent = false;
    if (!gripper_joint_names_.empty()) {
      if (gripper_open_positions_.size() != gripper_joint_names_.size() ||
          gripper_closed_positions_.size() != gripper_joint_names_.size()) {
        RCLCPP_WARN(this->get_logger(),
                    "Gripper joint names/positions size mismatch (names=%zu, open=%zu, closed=%zu)",
                    gripper_joint_names_.size(),
                    gripper_open_positions_.size(),
                    gripper_closed_positions_.size());
        return;
      }
      std::vector<double> target_positions;
      target_positions.reserve(gripper_joint_names_.size());
      for (size_t i = 0; i < gripper_joint_names_.size(); ++i) {
        double target_position = gripper_closed_positions_[i] +
          clamped * (gripper_open_positions_[i] - gripper_closed_positions_[i]);
        target_positions.push_back(target_position);
      }
      sent = sendJointTrajectoryGoal(
        gripper_client_, gripper_joint_names_, target_positions,
        gripper_command_duration_sec_, "gripper", &gripper_goal_active_, &gripper_goal_handle_);
    } else {
      double target_position = gripper_closed_position_ +
        clamped * (gripper_open_position_ - gripper_closed_position_);
      sent = sendJointTrajectoryGoal(
        gripper_client_, {gripper_joint_name_}, {target_position},
        gripper_command_duration_sec_, "gripper", &gripper_goal_active_, &gripper_goal_handle_);
    }

    if (sent) {
      last_gripper_command_ = clamped;
    }
  }

  void processBaseCommand(const geometry_msgs::msg::Twist& twist)
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
    RCLCPP_DEBUG(this->get_logger(), "Base command: [%.2f, %.2f, %.2f]",
                 safe_twist.linear.x, safe_twist.linear.y, safe_twist.angular.z);
  }

  moveit_msgs::msg::Constraints buildPoseGoalConstraints(
    const geometry_msgs::msg::PoseStamped& target) const
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

    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    orientation_constraint.header = target.header;
    orientation_constraint.link_name = move_group_eef_link_;

    tf2::Quaternion orientation;
    tf2::fromMsg(target.pose.orientation, orientation);
    if (orientation.length2() < 1e-12) {
      orientation.setValue(0.0, 0.0, 0.0, 1.0);
    } else {
      orientation.normalize();
    }
    orientation_constraint.orientation = tf2::toMsg(orientation);

    double orientation_tol = std::max(1e-4, moveit_orientation_tolerance_);
    orientation_constraint.absolute_x_axis_tolerance = orientation_tol;
    orientation_constraint.absolute_y_axis_tolerance = orientation_tol;
    orientation_constraint.absolute_z_axis_tolerance = orientation_tol;
    orientation_constraint.weight = 1.0;
    constraints.orientation_constraints.push_back(orientation_constraint);

    return constraints;
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
        RCLCPP_WARN(this->get_logger(),
                    "MoveIt action server not found on configured name; using fallback '%s'",
                    move_group_action_.c_str());
        return true;
      }
    }

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "MoveIt action server not available on '%s'",
                         move_group_action_.c_str());
    return false;
  }

  bool sendMoveGroupGoal(const geometry_msgs::msg::PoseStamped& target)
  {
    if (!ensureMoveGroupClientReady()) {
      return false;
    }

    if (moveit_goal_active_.load()) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "MoveIt goal already active; skipping new target");
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
      [this](rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(this->get_logger(), "MoveIt goal was rejected");
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
      [this](const auto& result) {
        moveit_goal_active_.store(false);
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          moveit_goal_handle_.reset();
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(this->get_logger(), "MoveIt goal failed with code %d",
                      static_cast<int>(result.code));
        } else {
          RCLCPP_INFO(this->get_logger(), "MoveIt goal succeeded");
        }
      };

    moveit_goal_active_.store(true);
    move_group_client_->async_send_goal(goal, send_goal_options);
    return true;
  }

  bool sendJointTrajectoryGoal(
    const rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr& client,
    const std::vector<std::string>& joint_names,
    const std::vector<double>& positions,
    double duration_sec,
    const std::string& label,
    std::atomic<bool>* active_flag = nullptr,
    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr*
      handle_slot = nullptr)
  {
    if (!client) {
      RCLCPP_WARN(this->get_logger(), "No %s trajectory action client available", label.c_str());
      return false;
    }

    if (!client->wait_for_action_server(std::chrono::milliseconds(200))) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
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
      [this, label, active_flag, handle_slot](rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr goal_handle) {
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
      [this, label, active_flag, handle_slot](const auto& result) {
        if (active_flag) {
          active_flag->store(false);
        }
        if (handle_slot) {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          handle_slot->reset();
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(this->get_logger(), "%s trajectory failed with code %d",
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
    if (arm_client_ && arm_goal_handle_) {
      arm_client_->async_cancel_goal(arm_goal_handle_);
    }
    if (gripper_client_ && gripper_goal_handle_) {
      gripper_client_->async_cancel_goal(gripper_goal_handle_);
    }
  }

  void safetyCheck()
  {
    auto now = this->now();
    auto time_since_last_policy = (now - last_policy_time_).seconds();

    if (time_since_last_policy > safety_timeout_sec_) {
      // No policy output received recently - stop the robot
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "No policy output received for %.2f seconds - stopping robot",
                           time_since_last_policy);

      geometry_msgs::msg::Twist zero_twist;
      cmd_vel_pub_->publish(zero_twist);
      if (!safety_stop_active_) {
        cancelActiveGoals();
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

  bool isWithinWorkspace(const geometry_msgs::msg::Point& point) const
  {
    return point.x >= workspace_x_min_ && point.x <= workspace_x_max_ &&
           point.y >= workspace_y_min_ && point.y <= workspace_y_max_ &&
           point.z >= workspace_z_min_ && point.z <= workspace_z_max_;
  }

  bool resolveEndEffectorTarget(
    const manipulation_msgs::msg::PolicyOutput::SharedPtr msg,
    geometry_msgs::msg::PoseStamped& target_out)
  {
    const std::string reference_frame =
      msg->reference_frame.empty() ? base_frame_ : msg->reference_frame;

    if (!eef_target_is_delta_) {
      geometry_msgs::msg::PoseStamped target_in;
      target_in.header.frame_id = reference_frame;
      target_in.header.stamp = msg->header.stamp;
      target_in.pose = msg->eef_target_pose;

      if (target_in.header.frame_id != base_frame_) {
        try {
          auto transform = tf_buffer_->lookupTransform(
            base_frame_, target_in.header.frame_id, tf2::TimePointZero);
          tf2::doTransform(target_in, target_out, transform);
        } catch (const tf2::TransformException& ex) {
          RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
          return false;
        }
      } else {
        target_out = target_in;
      }

      return true;
    }

    geometry_msgs::msg::TransformStamped ee_transform;
    try {
      ee_transform = tf_buffer_->lookupTransform(
        base_frame_, ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup for current EEF failed: %s", ex.what());
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

    tf2::Vector3 delta_translation_base = delta_translation;
    tf2::Quaternion delta_rotation_base = delta_rotation;

    if (reference_frame != base_frame_) {
      try {
        auto transform = tf_buffer_->lookupTransform(
          base_frame_, reference_frame, tf2::TimePointZero);
        tf2::Quaternion q_base_ref;
        tf2::fromMsg(transform.transform.rotation, q_base_ref);
        if (q_base_ref.length2() < 1e-12) {
          q_base_ref.setValue(0.0, 0.0, 0.0, 1.0);
        } else {
          q_base_ref.normalize();
        }
        tf2::Quaternion q_ref_base = q_base_ref.inverse();
        delta_translation_base = tf2::quatRotate(q_base_ref, delta_translation);
        delta_rotation_base = q_base_ref * delta_rotation * q_ref_base;
        delta_rotation_base.normalize();
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
        return false;
      }
    }

    tf2::Vector3 target_translation = current_ee.getOrigin() + delta_translation_base;
    tf2::Quaternion target_rotation = delta_rotation_base * current_ee.getRotation();
    target_rotation.normalize();

    target_out.header.frame_id = base_frame_;
    target_out.header.stamp = msg->header.stamp;
    target_out.pose.position.x = target_translation.x();
    target_out.pose.position.y = target_translation.y();
    target_out.pose.position.z = target_translation.z();
    target_out.pose.orientation = tf2::toMsg(target_rotation);

    return true;
  }

  // Parameters
  std::string base_frame_;
  std::string ee_frame_;
  std::string joint_states_topic_;
  std::string navigate_to_pose_action_;
  std::string follow_joint_trajectory_action_;
  std::string gripper_follow_joint_trajectory_action_;
  bool use_moveit_;
  std::string move_group_action_;
  std::string move_group_name_;
  std::string move_group_eef_link_;
  double moveit_action_wait_sec_;
  double moveit_planning_time_;
  int moveit_planning_attempts_;
  double moveit_velocity_scaling_;
  double moveit_accel_scaling_;
  double moveit_position_tolerance_;
  double moveit_orientation_tolerance_;
  std::vector<std::string> arm_joint_names_;
  double arm_command_duration_sec_;
  std::string gripper_joint_name_;
  std::vector<std::string> gripper_joint_names_;
  double gripper_open_position_;
  double gripper_closed_position_;
  std::vector<double> gripper_open_positions_;
  std::vector<double> gripper_closed_positions_;
  double gripper_command_duration_sec_;
  double gripper_command_epsilon_;
  double max_base_velocity_;
  double max_arm_velocity_;
  double safety_timeout_sec_;
  double workspace_x_min_;
  double workspace_x_max_;
  double workspace_y_min_;
  double workspace_y_max_;
  double workspace_z_min_;
  double workspace_z_max_;
  bool eef_target_is_delta_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS communication
  rclcpp::Subscription<manipulation_msgs::msg::PolicyOutput>::SharedPtr policy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr gripper_client_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_client_;
  rclcpp::TimerBase::SharedPtr safety_timer_;

  // State
  rclcpp::Time last_policy_time_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
  std::optional<double> last_gripper_command_;
  std::atomic<bool> moveit_goal_active_{false};
  std::atomic<bool> arm_goal_active_{false};
  std::atomic<bool> gripper_goal_active_{false};
  std::mutex goal_mutex_;
  rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr moveit_goal_handle_;
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
    arm_goal_handle_;
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
    gripper_goal_handle_;
  bool safety_stop_active_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdapterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
