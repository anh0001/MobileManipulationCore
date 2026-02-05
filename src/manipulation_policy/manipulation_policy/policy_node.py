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

import base64
import json
from urllib import error as urlerror
from urllib import request as urlrequest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from manipulation_msgs.msg import PolicyOutput
from geometry_msgs.msg import Pose, Twist
from cv_bridge import CvBridge

try:
    import cv2
except ImportError:  # pragma: no cover - optional dependency
    cv2 = None


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
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('use_fp16', True)
        self.declare_parameter('remote_timeout_sec', 1.0)
        self.declare_parameter('remote_retry_attempts', 3)
        self.declare_parameter('remote_fallback_on_failure', True)
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('joint_states_topic', '/joint_states')

        # Get parameters
        self.model_name = self.get_parameter('model_name').value
        self.model_path = self.get_parameter('model_path').value
        self.use_remote = self.get_parameter('use_remote').value
        self.remote_url = self.get_parameter('remote_url').value.rstrip('/')
        inference_rate = self.get_parameter('inference_rate').value
        self.device = self.get_parameter('device').value
        self.use_fp16 = self.get_parameter('use_fp16').value
        self.remote_timeout_sec = self.get_parameter('remote_timeout_sec').value
        self.remote_retry_attempts = int(self.get_parameter('remote_retry_attempts').value)
        self.remote_fallback_on_failure = self.get_parameter('remote_fallback_on_failure').value
        self.remote_infer_url = f"{self.remote_url}/infer"
        camera_topic = self.get_parameter('camera_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # State variables
        self.latest_image = None
        self.latest_joint_states = None
        self.warned_no_cv2 = False

        # Create subscriptions
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        self.joint_states_sub = self.create_subscription(
            JointState,
            joint_states_topic,
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
            self.get_logger().info(f'Using remote inference at: {self.remote_infer_url}')

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
            if self.use_remote:
                output = self.run_inference_remote()
            else:
                output = self.run_inference_stub()

            if output is None:
                return

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

        output.has_joint_deltas = False
        output.joint_deltas = []

        output.gripper_command = 0.5
        output.gripper_active = True

        output.has_base_hint = False
        output.reference_frame = 'base_link'
        output.confidence = 0.0

        return output

    def run_inference_remote(self):
        """Send observation to remote server and convert response."""
        payload = {
            'reference_frame': 'base_link',
            'joint_states': {
                'name': list(self.latest_joint_states.name),
                'position': list(self.latest_joint_states.position),
                'velocity': list(self.latest_joint_states.velocity),
                'effort': list(self.latest_joint_states.effort),
            },
        }

        if self.latest_image is not None and cv2 is not None:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
                success, encoded = cv2.imencode('.jpg', cv_image)
                if success:
                    payload['image'] = base64.b64encode(encoded.tobytes()).decode('ascii')
                    payload['image_encoding'] = 'jpeg'
                    payload['image_width'] = int(cv_image.shape[1])
                    payload['image_height'] = int(cv_image.shape[0])
            except Exception as exc:  # pragma: no cover - defensive
                self.get_logger().warn(f'Failed to encode image for remote inference: {exc}')
        elif cv2 is None and not self.warned_no_cv2:
            self.get_logger().warn('cv2 not available; sending remote inference without image')
            self.warned_no_cv2 = True

        last_error = None
        for attempt in range(self.remote_retry_attempts):
            try:
                data = json.dumps(payload).encode('utf-8')
                req = urlrequest.Request(
                    self.remote_infer_url,
                    data=data,
                    headers={'Content-Type': 'application/json'},
                )
                with urlrequest.urlopen(req, timeout=self.remote_timeout_sec) as resp:
                    if resp.status != 200:
                        raise urlerror.HTTPError(
                            self.remote_infer_url,
                            resp.status,
                            resp.reason,
                            resp.headers,
                            None,
                        )
                    response_payload = json.loads(resp.read().decode('utf-8'))
                    return self.policy_output_from_dict(response_payload)
            except Exception as exc:
                last_error = exc
                self.get_logger().warn(
                    f'Remote inference attempt {attempt + 1}/{self.remote_retry_attempts} failed: {exc}'
                )

        if self.remote_fallback_on_failure:
            self.get_logger().warn('Remote inference failed; falling back to stub output')
            return self.run_inference_stub()

        self.get_logger().error(f'Remote inference failed: {last_error}')
        return None

    def policy_output_from_dict(self, data):
        """Convert a JSON response into a PolicyOutput message."""
        output = PolicyOutput()
        output.header.stamp = self.get_clock().now().to_msg()

        output.has_eef_target = bool(data.get('has_eef_target', False))
        if output.has_eef_target:
            pose_data = data.get('eef_target_pose', {})
            position = pose_data.get('position', {})
            orientation = pose_data.get('orientation', {})
            output.eef_target_pose = Pose()
            output.eef_target_pose.position.x = float(position.get('x', 0.0))
            output.eef_target_pose.position.y = float(position.get('y', 0.0))
            output.eef_target_pose.position.z = float(position.get('z', 0.0))
            output.eef_target_pose.orientation.x = float(orientation.get('x', 0.0))
            output.eef_target_pose.orientation.y = float(orientation.get('y', 0.0))
            output.eef_target_pose.orientation.z = float(orientation.get('z', 0.0))
            output.eef_target_pose.orientation.w = float(orientation.get('w', 1.0))

        output.has_joint_deltas = bool(data.get('has_joint_deltas', False))
        output.joint_deltas = [float(v) for v in data.get('joint_deltas', [])]

        output.gripper_command = float(data.get('gripper_command', 0.0))
        output.gripper_active = bool(data.get('gripper_active', False))

        output.has_base_hint = bool(data.get('has_base_hint', False))
        if output.has_base_hint:
            base_hint = data.get('base_velocity_hint', {})
            output.base_velocity_hint = Twist()
            linear = base_hint.get('linear', {})
            angular = base_hint.get('angular', {})
            output.base_velocity_hint.linear.x = float(linear.get('x', 0.0))
            output.base_velocity_hint.linear.y = float(linear.get('y', 0.0))
            output.base_velocity_hint.linear.z = float(linear.get('z', 0.0))
            output.base_velocity_hint.angular.x = float(angular.get('x', 0.0))
            output.base_velocity_hint.angular.y = float(angular.get('y', 0.0))
            output.base_velocity_hint.angular.z = float(angular.get('z', 0.0))

        output.reference_frame = str(data.get('reference_frame', 'base_link'))
        output.confidence = float(data.get('confidence', 0.0))
        output.header.frame_id = output.reference_frame

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
