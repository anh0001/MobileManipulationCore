#!/usr/bin/env python3
# Copyright 2026 MobileManipulationCore Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Policy node for VLA model inference.

This node runs the vision-language-action model (e.g., OpenVLA, LeRobot)
to generate manipulation actions from observations.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from manipulation_msgs.msg import PolicyOutput
from geometry_msgs.msg import Pose, Twist
from cv_bridge import CvBridge
import numpy as np


class PolicyNode(Node):
    """
    ROS 2 node for policy model inference.

    Subscribes to sensor inputs and publishes high-level manipulation actions.
    """

    def __init__(self):
        super().__init__('policy_node')

        # Declare parameters
        self.declare_parameter('model_name', 'openvla-7b')
        self.declare_parameter('model_path', '')
        self.declare_parameter('use_remote', False)
        self.declare_parameter('remote_url', 'http://localhost:5000')
        self.declare_parameter('inference_rate', 10.0)

        # Get parameters
        self.model_name = self.get_parameter('model_name').value
        self.model_path = self.get_parameter('model_path').value
        self.use_remote = self.get_parameter('use_remote').value
        self.remote_url = self.get_parameter('remote_url').value
        inference_rate = self.get_parameter('inference_rate').value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # State variables
        self.latest_image = None
        self.latest_joint_states = None

        # Create subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        # Create publisher
        self.policy_output_pub = self.create_publisher(
            PolicyOutput,
            '/manipulation/policy_output',
            10
        )

        # Create timer for inference loop
        self.inference_timer = self.create_timer(
            1.0 / inference_rate,
            self.inference_callback
        )

        # TODO: Load model
        self.model = None
        if not self.use_remote:
            self.get_logger().info(f'Loading model: {self.model_name}')
            # self.model = self.load_model()
            self.get_logger().warn('Model loading not yet implemented - using stub')
        else:
            self.get_logger().info(f'Using remote inference at: {self.remote_url}')

        self.get_logger().info('Policy node initialized')

    def image_callback(self, msg):
        """Store latest image for inference."""
        self.latest_image = msg

    def joint_states_callback(self, msg):
        """Store latest joint states for inference."""
        self.latest_joint_states = msg

    def inference_callback(self):
        """Run policy inference and publish output."""
        if self.latest_image is None or self.latest_joint_states is None:
            self.get_logger().debug('Waiting for sensor data...')
            return

        try:
            # TODO: Implement actual inference
            # For now, publish a stub output
            output = self.run_inference_stub()

            # Publish policy output
            self.policy_output_pub.publish(output)

        except Exception as e:
            self.get_logger().error(f'Inference error: {str(e)}')

    def run_inference_stub(self):
        """
        Stub inference function.

        TODO: Replace with actual VLA model inference.
        """
        output = PolicyOutput()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'base_link'

        # Stub: output a fixed target
        output.has_eef_target = True
        output.eef_target_pose = Pose()
        output.eef_target_pose.position.x = 0.3
        output.eef_target_pose.position.y = 0.0
        output.eef_target_pose.position.z = 0.3
        output.eef_target_pose.orientation.w = 1.0

        output.gripper_command = 0.5
        output.gripper_active = True

        output.has_base_hint = False
        output.reference_frame = 'base_link'
        output.confidence = 0.0

        return output

    def load_model(self):
        """
        Load the VLA model.

        TODO: Implement model loading from HuggingFace or local checkpoint.
        """
        # Example:
        # from transformers import AutoModel, AutoTokenizer
        # model = AutoModel.from_pretrained(self.model_path or self.model_name)
        # return model
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
