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

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <manipulation_msgs/msg/observation.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

/**
 * @brief Perception node for processing sensor data
 *
 * This node subscribes to camera images, depth data, and joint states,
 * processes them, and publishes aggregated observations for the policy model.
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
    this->declare_parameter<std::string>("observation_topic", "/manipulation/observation");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("camera_frame", "");
    this->declare_parameter<std::string>("ee_frame", "");
    this->declare_parameter<int>("image_width", 224);
    this->declare_parameter<int>("image_height", 224);
    this->declare_parameter<int>("depth_width", 0);
    this->declare_parameter<int>("depth_height", 0);
    this->declare_parameter<double>("sync_tolerance_sec", 0.05);
    this->declare_parameter<double>("tf_timeout_sec", 0.05);
    this->declare_parameter<bool>("include_image", true);
    this->declare_parameter<bool>("include_depth", false);
    this->declare_parameter<bool>("include_joint_states", true);
    this->declare_parameter<bool>("include_transforms", true);

    // Get parameters
    std::string camera_topic = this->get_parameter("camera_topic").as_string();
    std::string depth_topic = this->get_parameter("depth_topic").as_string();
    std::string joint_states_topic = this->get_parameter("joint_states_topic").as_string();
    std::string observation_topic = this->get_parameter("observation_topic").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    ee_frame_ = this->get_parameter("ee_frame").as_string();
    image_width_ = static_cast<int>(this->get_parameter("image_width").as_int());
    image_height_ = static_cast<int>(this->get_parameter("image_height").as_int());
    depth_width_ = static_cast<int>(this->get_parameter("depth_width").as_int());
    depth_height_ = static_cast<int>(this->get_parameter("depth_height").as_int());
    sync_tolerance_sec_ = this->get_parameter("sync_tolerance_sec").as_double();
    tf_timeout_sec_ = this->get_parameter("tf_timeout_sec").as_double();
    include_image_ = this->get_parameter("include_image").as_bool();
    include_depth_ = this->get_parameter("include_depth").as_bool();
    include_joint_states_ = this->get_parameter("include_joint_states").as_bool();
    include_transforms_ = this->get_parameter("include_transforms").as_bool();

    if (depth_width_ <= 0) {
      depth_width_ = image_width_;
    }
    if (depth_height_ <= 0) {
      depth_height_ = image_height_;
    }

    last_published_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    // TF buffer/listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create subscriptions
    auto sensor_qos = rclcpp::SensorDataQoS();
    if (include_image_) {
      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, sensor_qos,
        std::bind(&PerceptionNode::imageCallback, this, std::placeholders::_1));
    }

    if (include_depth_) {
      depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic, sensor_qos,
        std::bind(&PerceptionNode::depthCallback, this, std::placeholders::_1));
    }

    if (include_joint_states_) {
      joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic, 10,
        std::bind(&PerceptionNode::jointStatesCallback, this, std::placeholders::_1));
    }

    observation_pub_ = this->create_publisher<manipulation_msgs::msg::Observation>(
      observation_topic, 10);

    RCLCPP_INFO(this->get_logger(), "Perception node initialized");
    RCLCPP_INFO(this->get_logger(), "  Camera topic: %s", camera_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Depth topic: %s", depth_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Joint states topic: %s", joint_states_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Observation topic: %s", observation_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Include image: %s", include_image_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Include depth: %s", include_depth_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Include joint states: %s", include_joint_states_ ? "true" : "false");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_image_ = msg;
    }
    tryPublish();
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_depth_ = msg;
    }
    tryPublish();
  }

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_joint_states_ = msg;
    }
    tryPublish();
  }

  bool validStamp(const rclcpp::Time & stamp) const
  {
    return stamp.nanoseconds() != 0;
  }

  bool withinTolerance(const rclcpp::Time & a, const rclcpp::Time & b) const
  {
    return std::abs((a - b).seconds()) <= sync_tolerance_sec_;
  }

  bool processRgbImage(const sensor_msgs::msg::Image::SharedPtr & msg,
                       sensor_msgs::msg::Image & out)
  {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
        msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat processed = cv_ptr->image;
      if (image_width_ > 0 && image_height_ > 0 &&
          (processed.cols != image_width_ || processed.rows != image_height_)) {
        cv::Mat resized;
        cv::resize(processed, resized, cv::Size(image_width_, image_height_));
        processed = resized;
      }

      cv_bridge::CvImage out_bridge;
      out_bridge.header = msg->header;
      out_bridge.encoding = sensor_msgs::image_encodings::BGR8;
      out_bridge.image = processed;
      out = *out_bridge.toImageMsg();
      return true;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (rgb): %s", e.what());
    }
    return false;
  }

  bool processDepthImage(const sensor_msgs::msg::Image::SharedPtr & msg,
                         sensor_msgs::msg::Image & out)
  {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      cv::Mat processed = cv_ptr->image;
      if (depth_width_ > 0 && depth_height_ > 0 &&
          (processed.cols != depth_width_ || processed.rows != depth_height_)) {
        cv::Mat resized;
        cv::resize(processed, resized, cv::Size(depth_width_, depth_height_), 0.0, 0.0,
                   cv::INTER_NEAREST);
        processed = resized;
      }

      cv_bridge::CvImage out_bridge;
      out_bridge.header = msg->header;
      out_bridge.encoding = msg->encoding;
      out_bridge.image = processed;
      out = *out_bridge.toImageMsg();
      return true;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (depth): %s", e.what());
    }
    return false;
  }

  bool lookupTransform(const std::string & target_frame,
                       const std::string & source_frame,
                       const rclcpp::Time & stamp,
                       geometry_msgs::msg::TransformStamped & out)
  {
    if (!tf_buffer_) {
      return false;
    }
    try {
      out = tf_buffer_->lookupTransform(
        target_frame, source_frame, stamp, tf2::durationFromSec(tf_timeout_sec_));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "TF lookup failed (%s -> %s): %s",
        source_frame.c_str(), target_frame.c_str(), ex.what());
    }
    return false;
  }

  void tryPublish()
  {
    sensor_msgs::msg::Image::SharedPtr image_msg;
    sensor_msgs::msg::Image::SharedPtr depth_msg;
    sensor_msgs::msg::JointState::SharedPtr joint_msg;
    rclcpp::Time last_pub_stamp(0, 0, this->get_clock()->get_clock_type());

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      image_msg = latest_image_;
      depth_msg = latest_depth_;
      joint_msg = latest_joint_states_;
      last_pub_stamp = last_published_stamp_;
    }

    if (include_image_ && !image_msg) {
      return;
    }
    if (include_depth_ && !depth_msg) {
      return;
    }
    if (include_joint_states_ && !joint_msg) {
      return;
    }

    rclcpp::Time anchor_stamp(0, 0, this->get_clock()->get_clock_type());
    if (include_image_) {
      rclcpp::Time image_stamp(image_msg->header.stamp);
      if (!validStamp(image_stamp)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Image stamp is zero; cannot synchronize observation");
        return;
      }
      anchor_stamp = image_stamp;
    } else if (include_depth_) {
      rclcpp::Time depth_stamp(depth_msg->header.stamp);
      if (!validStamp(depth_stamp)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Depth stamp is zero; cannot synchronize observation");
        return;
      }
      anchor_stamp = depth_stamp;
    } else if (include_joint_states_) {
      rclcpp::Time joint_stamp(joint_msg->header.stamp);
      if (!validStamp(joint_stamp)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Joint state stamp is zero; cannot synchronize observation");
        return;
      }
      anchor_stamp = joint_stamp;
    } else {
      return;
    }

    if (anchor_stamp <= last_pub_stamp) {
      return;
    }

    if (include_depth_) {
      rclcpp::Time depth_stamp(depth_msg->header.stamp);
      if (!validStamp(depth_stamp)) {
        return;
      }
      if (!withinTolerance(anchor_stamp, depth_stamp)) {
        return;
      }
    }

    if (include_joint_states_) {
      rclcpp::Time joint_stamp(joint_msg->header.stamp);
      if (!validStamp(joint_stamp)) {
        return;
      }
      if (!withinTolerance(anchor_stamp, joint_stamp)) {
        return;
      }
    }

    manipulation_msgs::msg::Observation observation;
    observation.header.stamp = anchor_stamp;
    observation.header.frame_id = base_frame_;
    observation.base_frame = base_frame_;
    observation.camera_frame = camera_frame_;
    observation.ee_frame = ee_frame_;

    if (include_image_) {
      sensor_msgs::msg::Image processed;
      if (!processRgbImage(image_msg, processed)) {
        return;
      }
      observation.rgb_image = std::move(processed);
      observation.has_rgb_image = true;
      if (observation.camera_frame.empty()) {
        observation.camera_frame = image_msg->header.frame_id;
      }
    } else {
      observation.has_rgb_image = false;
    }

    if (include_depth_) {
      sensor_msgs::msg::Image processed;
      if (!processDepthImage(depth_msg, processed)) {
        return;
      }
      observation.depth_image = std::move(processed);
      observation.has_depth_image = true;
    } else {
      observation.has_depth_image = false;
    }

    if (include_joint_states_) {
      observation.joint_states = *joint_msg;
      observation.has_joint_states = true;
    } else {
      observation.has_joint_states = false;
    }

    if (include_transforms_) {
      observation.has_base_to_camera = false;
      observation.has_base_to_ee = false;

      if (!observation.camera_frame.empty() && !base_frame_.empty()) {
        geometry_msgs::msg::TransformStamped base_to_camera;
        if (lookupTransform(base_frame_, observation.camera_frame, anchor_stamp, base_to_camera)) {
          observation.base_to_camera = base_to_camera;
          observation.has_base_to_camera = true;
        }
      }

      if (!ee_frame_.empty() && !base_frame_.empty()) {
        geometry_msgs::msg::TransformStamped base_to_ee;
        if (lookupTransform(base_frame_, ee_frame_, anchor_stamp, base_to_ee)) {
          observation.base_to_ee = base_to_ee;
          observation.has_base_to_ee = true;
        }
      }
    } else {
      observation.has_base_to_camera = false;
      observation.has_base_to_ee = false;
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (anchor_stamp <= last_published_stamp_) {
        return;
      }
      last_published_stamp_ = anchor_stamp;
    }

    observation_pub_->publish(observation);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<manipulation_msgs::msg::Observation>::SharedPtr observation_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  sensor_msgs::msg::Image::SharedPtr latest_image_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_states_;

  std::mutex data_mutex_;
  rclcpp::Time last_published_stamp_{0, 0, RCL_ROS_TIME};

  int image_width_{224};
  int image_height_{224};
  int depth_width_{224};
  int depth_height_{224};
  double sync_tolerance_sec_{0.05};
  double tf_timeout_sec_{0.05};
  bool include_image_{true};
  bool include_depth_{false};
  bool include_joint_states_{true};
  bool include_transforms_{true};

  std::string base_frame_{"base_link"};
  std::string camera_frame_;
  std::string ee_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PerceptionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
