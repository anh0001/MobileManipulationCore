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
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>

/**
 * @brief Perception node for processing sensor data
 *
 * This node subscribes to camera images, depth data, and joint states,
 * processes them, and prepares observations for the policy model.
 */
class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode()
  : Node("perception_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("camera_topic", "/camera/color/image_raw");
    this->declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    this->declare_parameter<int>("image_width", 224);
    this->declare_parameter<int>("image_height", 224);

    // Get parameters
    std::string camera_topic = this->get_parameter("camera_topic").as_string();
    std::string depth_topic = this->get_parameter("depth_topic").as_string();
    std::string joint_states_topic = this->get_parameter("joint_states_topic").as_string();
    image_width_ = static_cast<int>(this->get_parameter("image_width").as_int());
    image_height_ = static_cast<int>(this->get_parameter("image_height").as_int());

    // Create subscriptions
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic, 10,
      std::bind(&PerceptionNode::imageCallback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic, 10,
      std::bind(&PerceptionNode::depthCallback, this, std::placeholders::_1));

    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, 10,
      std::bind(&PerceptionNode::jointStatesCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Perception node initialized");
    RCLCPP_INFO(this->get_logger(), "  Camera topic: %s", camera_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Depth topic: %s", depth_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Joint states topic: %s", joint_states_topic.c_str());
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // Convert ROS image to OpenCV format
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      // TODO: Process image (resize, normalize, etc.) for policy input
      // TODO: Publish processed observation
      if (image_width_ > 0 && image_height_ > 0 &&
          (cv_ptr->image.cols != image_width_ || cv_ptr->image.rows != image_height_)) {
        cv::Mat resized;
        cv::resize(cv_ptr->image, resized, cv::Size(image_width_, image_height_));
      }

      RCLCPP_DEBUG(this->get_logger(), "Received image: %dx%d", msg->width, msg->height);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // TODO: Process depth information
    RCLCPP_DEBUG(this->get_logger(), "Received depth image: %dx%d", msg->width, msg->height);
  }

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // TODO: Store joint states for observation
    RCLCPP_DEBUG(this->get_logger(), "Received joint states: %zu joints", msg->name.size());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  int image_width_{224};
  int image_height_{224};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PerceptionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
