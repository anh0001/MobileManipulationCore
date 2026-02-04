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
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <manipulation_msgs/msg/policy_output.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
    this->declare_parameter<std::string>("ee_frame", "piper_gripper");
    this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    this->declare_parameter<std::string>("navigate_to_pose_action", "/navigate_to_pose");
    this->declare_parameter<std::string>(
      "follow_joint_trajectory_action", "/piper_arm_controller/follow_joint_trajectory");
    this->declare_parameter<double>("max_base_velocity", 0.5);
    this->declare_parameter<double>("max_arm_velocity", 1.0);
    this->declare_parameter<double>("safety_timeout_sec", 2.0);
    this->declare_parameter<double>("workspace_x_min", 0.1);
    this->declare_parameter<double>("workspace_x_max", 0.8);
    this->declare_parameter<double>("workspace_y_min", -0.5);
    this->declare_parameter<double>("workspace_y_max", 0.5);
    this->declare_parameter<double>("workspace_z_min", 0.0);
    this->declare_parameter<double>("workspace_z_max", 1.0);

    // Get parameters
    base_frame_ = this->get_parameter("base_frame").as_string();
    ee_frame_ = this->get_parameter("ee_frame").as_string();
    joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();
    navigate_to_pose_action_ = this->get_parameter("navigate_to_pose_action").as_string();
    follow_joint_trajectory_action_ =
      this->get_parameter("follow_joint_trajectory_action").as_string();
    max_base_velocity_ = this->get_parameter("max_base_velocity").as_double();
    max_arm_velocity_ = this->get_parameter("max_arm_velocity").as_double();
    safety_timeout_sec_ = this->get_parameter("safety_timeout_sec").as_double();
    workspace_x_min_ = this->get_parameter("workspace_x_min").as_double();
    workspace_x_max_ = this->get_parameter("workspace_x_max").as_double();
    workspace_y_min_ = this->get_parameter("workspace_y_min").as_double();
    workspace_y_max_ = this->get_parameter("workspace_y_max").as_double();
    workspace_z_min_ = this->get_parameter("workspace_z_min").as_double();
    workspace_z_max_ = this->get_parameter("workspace_z_max").as_double();

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

    // Create safety timer
    safety_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(safety_timeout_sec_ * 1000)),
      std::bind(&AdapterNode::safetyCheck, this));

    RCLCPP_INFO(this->get_logger(), "Adapter node initialized");
    RCLCPP_INFO(this->get_logger(), "  Base frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  EE frame: %s", ee_frame_.c_str());
  }

private:
  void policyCallback(const manipulation_msgs::msg::PolicyOutput::SharedPtr msg)
  {
    last_policy_time_ = this->now();

    RCLCPP_DEBUG(this->get_logger(), "Received policy output");

    // TODO: Implement full adaptation logic
    // For now, just process end-effector target if available
    if (msg->has_eef_target) {
      processEndEffectorTarget(msg);
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

  void processEndEffectorTarget(const manipulation_msgs::msg::PolicyOutput::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped target_in;
    target_in.header.frame_id = msg->reference_frame.empty() ? base_frame_ : msg->reference_frame;
    target_in.header.stamp = msg->header.stamp;
    target_in.pose = msg->eef_target_pose;

    geometry_msgs::msg::PoseStamped target_out;
    if (target_in.header.frame_id != base_frame_) {
      try {
        auto transform = tf_buffer_->lookupTransform(
          base_frame_, target_in.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(target_in, target_out, transform);
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
        return;
      }
    } else {
      target_out = target_in;
    }

    if (!isWithinWorkspace(target_out.pose.position)) {
      RCLCPP_WARN(
        this->get_logger(),
        "EEF target outside workspace bounds [%.2f, %.2f] [%.2f, %.2f] [%.2f, %.2f]",
        workspace_x_min_, workspace_x_max_, workspace_y_min_, workspace_y_max_,
        workspace_z_min_, workspace_z_max_);
      return;
    }

    // TODO: Check if target is reachable
    // TODO: Compute inverse kinematics
    // TODO: Send arm trajectory command

    RCLCPP_INFO(this->get_logger(), "Processing EEF target at [%.2f, %.2f, %.2f]",
                target_out.pose.position.x,
                target_out.pose.position.y,
                target_out.pose.position.z);
  }

  void processGripperCommand(double command)
  {
    // TODO: Send gripper command
    RCLCPP_DEBUG(this->get_logger(), "Gripper command: %.2f", command);
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
    }
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

  // Parameters
  std::string base_frame_;
  std::string ee_frame_;
  std::string joint_states_topic_;
  std::string navigate_to_pose_action_;
  std::string follow_joint_trajectory_action_;
  double max_base_velocity_;
  double max_arm_velocity_;
  double safety_timeout_sec_;
  double workspace_x_min_;
  double workspace_x_max_;
  double workspace_y_min_;
  double workspace_y_max_;
  double workspace_z_min_;
  double workspace_z_max_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS communication
  rclcpp::Subscription<manipulation_msgs::msg::PolicyOutput>::SharedPtr policy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr arm_client_;
  rclcpp::TimerBase::SharedPtr safety_timer_;

  // State
  rclcpp::Time last_policy_time_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdapterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
