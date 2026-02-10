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
import math
from urllib import error as urlerror
from urllib import request as urlrequest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from manipulation_msgs.msg import Observation, PolicyOutput
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
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('use_observation', True)
        self.declare_parameter('observation_topic', '/manipulation/observation')
        self.declare_parameter('observation_timeout_sec', 0.5)
        self.declare_parameter('task_prompt', '')
        self.declare_parameter('task_prompt_topic', '/manipulation/task_prompt')
        self.declare_parameter('arm_base_frame', 'piper_base_link')
        self.declare_parameter('max_steps', 50)

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
        self.camera_frame = self.get_parameter('camera_frame').value
        # OpenVLA outputs actions in the wrist camera frame (piper_camera_link).
        # The frame follows REP-103 convention: X forward, Z up.
        self.reference_frame = self.camera_frame if self.camera_frame else 'piper_camera_link'
        joint_states_topic = self.get_parameter('joint_states_topic').value
        self.use_observation = self.get_parameter('use_observation').value
        self.observation_topic = self.get_parameter('observation_topic').value
        self.observation_timeout_sec = float(self.get_parameter('observation_timeout_sec').value)
        self.current_task_prompt = self.get_parameter('task_prompt').value
        task_prompt_topic = self.get_parameter('task_prompt_topic').value
        self.max_steps = int(self.get_parameter('max_steps').value)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # State variables
        self.latest_observation = None
        self.latest_raw_image = None
        self.latest_raw_joint_states = None
        self.active_prompt = self.current_task_prompt or None
        self.steps_remaining = 0
        self.warned_no_cv2 = False
        self._warned_waiting_for_sensor_data = False

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

        self.observation_sub = self.create_subscription(
            Observation,
            self.observation_topic,
            self.observation_callback,
            10
        )

        self.task_prompt_sub = self.create_subscription(
            String,
            task_prompt_topic,
            self.task_prompt_callback,
            10
        )

        # Create publisher
        self.policy_output_pub = self.create_publisher(
            PolicyOutput,
            '/manipulation/policy_output',
            10
        )

        # Timer runs inference for the active prompt at inference_rate Hz.
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

    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        """Convert roll/pitch/yaw (rad) to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    @staticmethod
    def _normalize_gripper_value(value):
        """Map gripper values to [0, 1] (supports [-1, 1] and [0, 1])."""
        try:
            val = float(value)
        except (TypeError, ValueError):
            return 0.0

        if val < 0.0 or val > 1.0:
            val = (val + 1.0) * 0.5
        return max(0.0, min(1.0, val))

    def _coerce_action_vector(self, value):
        """Convert a candidate action payload to a 7D float list if possible."""
        if value is None:
            return None

        if hasattr(value, 'tolist'):
            return self._coerce_action_vector(value.tolist())

        if isinstance(value, dict):
            for key in ('action', 'actions', 'action_values', 'values', 'data'):
                converted = self._coerce_action_vector(value.get(key))
                if converted is not None:
                    return converted
            return None

        if isinstance(value, (list, tuple)):
            candidate = value
            if len(candidate) == 1 and isinstance(candidate[0], (list, tuple)):
                candidate = candidate[0]
            if len(candidate) < 7:
                return None
            try:
                out = [float(v) for v in candidate[:7]]
            except (TypeError, ValueError):
                return None
            if all(math.isfinite(v) for v in out):
                return out

        return None

    def _extract_openvla_action(self, data):
        """Extract a raw OpenVLA 7D action from common response shapes."""
        for key in ('action', 'actions', 'action_values', 'openvla_action'):
            converted = self._coerce_action_vector(data.get(key))
            if converted is not None:
                return converted

        result = data.get('result')
        if isinstance(result, dict):
            for key in ('action', 'actions', 'action_values', 'openvla_action'):
                converted = self._coerce_action_vector(result.get(key))
                if converted is not None:
                    return converted

        return None

    def image_callback(self, msg):
        """Store latest image for inference."""
        if self.camera_frame:
            msg.header.frame_id = self.camera_frame
        self.latest_raw_image = msg

    def joint_states_callback(self, msg):
        """Store latest joint states for inference."""
        self.latest_raw_joint_states = msg

    def observation_callback(self, msg):
        """Store latest aggregated observation for inference."""
        self.latest_observation = msg

    def task_prompt_callback(self, msg):
        """Set or overwrite the active task prompt and reset the step budget."""
        prompt = msg.data.strip()
        self.current_task_prompt = prompt
        self._warned_waiting_for_sensor_data = False

        if prompt:
            self.active_prompt = prompt
            self.steps_remaining = self.max_steps
            self.get_logger().info(
                f'Task prompt set: "{prompt}" ({self.max_steps} steps)'
            )
        else:
            self.active_prompt = None
            self.steps_remaining = 0
            self.get_logger().info('Task prompt cleared')

    def _observation_fresh(self, obs):
        if obs is None:
            return False
        if obs.header.stamp.sec == 0 and obs.header.stamp.nanosec == 0:
            return True
        obs_time = rclpy.time.Time.from_msg(obs.header.stamp)
        age = (self.get_clock().now() - obs_time).nanoseconds * 1e-9
        return age <= self.observation_timeout_sec

    def _select_inputs(self):
        if self.use_observation and self._observation_fresh(self.latest_observation):
            obs = self.latest_observation
            image = obs.rgb_image if getattr(obs, 'has_rgb_image', False) else None
            joints = obs.joint_states if getattr(obs, 'has_joint_states', False) else None
            if image is None:
                image = self.latest_raw_image
            if joints is None:
                joints = self.latest_raw_joint_states
            return image, joints
        return self.latest_raw_image, self.latest_raw_joint_states

    def inference_callback(self):
        """Run one inference step for the active prompt, counting down the step budget."""
        if not self.active_prompt or self.steps_remaining <= 0:
            return

        image_msg, joint_states = self._select_inputs()
        if image_msg is None or joint_states is None:
            if not self._warned_waiting_for_sensor_data:
                self.get_logger().info('Prompt active, waiting for sensor data before inference')
                self._warned_waiting_for_sensor_data = True
            return

        self._warned_waiting_for_sensor_data = False

        try:
            if self.use_remote:
                output = self.run_inference_remote(image_msg, joint_states)
            else:
                output = self.run_inference_stub()

            if output is None:
                return

            self.policy_output_pub.publish(output)
            self.steps_remaining -= 1
            self.get_logger().info(
                f'Published policy output for prompt: "{self.active_prompt}" '
                f'(step {self.max_steps - self.steps_remaining}/{self.max_steps})'
            )

            if self.steps_remaining <= 0:
                self.get_logger().info(
                    f'Task "{self.active_prompt}" reached max steps ({self.max_steps}), '
                    f'waiting for next prompt'
                )
                self.active_prompt = None

        except Exception as e:
            self.get_logger().error(f'Inference error: {str(e)}')

    def run_inference_stub(self):
        """
        Stub inference function.

        TODO: Replace with actual VLA model inference.
        """
        output = PolicyOutput()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.reference_frame

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
        output.reference_frame = self.reference_frame
        output.confidence = 0.0

        return output

    def run_inference_remote(self, image_msg, joint_states):
        """Send observation to remote server and convert response."""
        payload = {
            'reference_frame': self.reference_frame,
            'task': self.current_task_prompt,
            'joint_states': {
                'name': list(joint_states.name),
                'position': list(joint_states.position),
                'velocity': list(joint_states.velocity),
                'effort': list(joint_states.effort),
            },
        }

        if image_msg is not None and cv2 is not None:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
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

        if self.current_task_prompt:
            self.get_logger().debug(f'Inferring with task: "{self.current_task_prompt}"')
        else:
            self.get_logger().debug('Inferring with no task prompt set')

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

        has_gripper_command = 'gripper_command' in data
        has_gripper_active = 'gripper_active' in data
        output.gripper_command = self._normalize_gripper_value(data.get('gripper_command', 0.0))
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

        # Actions from OpenVLA are expressed in the wrist camera frame (piper_camera_link),
        # which is set to follow REP-103 convention: X forward, Z up.
        output.reference_frame = str(data.get('reference_frame', self.reference_frame))
        output.confidence = float(data.get('confidence', 0.0))
        output.header.frame_id = output.reference_frame

        # Fallback: accept raw OpenVLA 7D action vectors and convert to delta EEF targets.
        if not output.has_eef_target and not output.has_joint_deltas:
            action_values = self._extract_openvla_action(data)
            if action_values is not None:
                dx, dy, dz, roll, pitch, yaw, gripper = action_values
                qx, qy, qz, qw = self._euler_to_quaternion(roll, pitch, yaw)

                output.has_eef_target = True
                output.eef_target_pose = Pose()
                output.eef_target_pose.position.x = dx
                output.eef_target_pose.position.y = dy
                output.eef_target_pose.position.z = dz
                output.eef_target_pose.orientation.x = qx
                output.eef_target_pose.orientation.y = qy
                output.eef_target_pose.orientation.z = qz
                output.eef_target_pose.orientation.w = qw

                if not has_gripper_command:
                    output.gripper_command = self._normalize_gripper_value(gripper)
                if not has_gripper_active:
                    output.gripper_active = True
                if 'confidence' not in data:
                    output.confidence = 1.0

                self.get_logger().debug(
                    'Converted raw OpenVLA action vector to delta EEF target for adapter'
                )

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
