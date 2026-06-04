#!/usr/bin/env python3
# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""PickObject action server — one-call "pick object X" for AI clients.

Wraps the existing visual-servo grasp pipeline behind a single ROS 2 action so
any caller (an MCP tool, an LLM agent, a CLI, another node) can request a pick
with just an object label and get structured feedback + result.

Sequence per goal (mirrors scripts/grasp_autotune.py run_attempt/classify):
  1. publish the object label to the visual-servo prompt topic
  2. gate off, reset the arm to the look-down capture pose
  3. gate the grasp on (grasp_enabled=true, grasp_auto_loop=false -> single pick)
  4. wait for the state machine to enter the grasp sequence, then reach DONE
  5. classify object_held from the final gripper width, gate off, return

Held detection is heuristic (no force sensor): a jaw blocked by an object stops
short of full close, so width > held_width_threshold => held. Thin/soft objects
can still close fully while holding, so the raw final width is also returned for
the caller to judge.
"""
import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType
from rclpy.action import ActionClient

from manipulation_msgs.action import PickObject

# Active grasp-sequence states (object actually being picked); used to confirm a
# fresh attempt started and to detect a stale prior "DONE".
ACTIVE_STATES = {
    "OPEN_GRIPPER", "GUARDED_APPROACH", "APPROACH_DEPTH", "CLOSE_GRIPPER", "LIFT",
}


class PickOrchestrator(Node):
    def __init__(self):
        super().__init__("pick_orchestrator")
        self.declare_parameter("visual_servo_node", "/visual_servo_node")
        self.declare_parameter("prompt_topic", "/visual_servo/target_prompt")
        self.declare_parameter("state_topic", "/visual_servo/state")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("gripper_joint", "piper_joint7")
        self.declare_parameter("arm_action", "/piper_arm_controller/follow_joint_trajectory")
        self.declare_parameter("arm_joints", [f"piper_joint{i}" for i in range(1, 7)])
        self.declare_parameter("capture_pose", [0.0, 1.2, -0.2, 0.0, -0.35, 0.0])
        self.declare_parameter("default_timeout_sec", 60.0)
        self.declare_parameter("acquire_timeout_sec", 12.0)
        self.declare_parameter("held_width_threshold", 0.012)
        self.declare_parameter("reset_arm_each_pick", True)

        self.vs_node = self.get_parameter("visual_servo_node").value
        self.gripper_joint = self.get_parameter("gripper_joint").value
        self.arm_joints = list(self.get_parameter("arm_joints").value)
        self.capture_pose = list(self.get_parameter("capture_pose").value)
        self.default_timeout = float(self.get_parameter("default_timeout_sec").value)
        self.acquire_timeout = float(self.get_parameter("acquire_timeout_sec").value)
        self.held_threshold = float(self.get_parameter("held_width_threshold").value)
        self.reset_arm = bool(self.get_parameter("reset_arm_each_pick").value)

        self._cb = ReentrantCallbackGroup()
        self.state = ""
        self.width = 0.0

        self.prompt_pub = self.create_publisher(
            String, self.get_parameter("prompt_topic").value, 10)
        self.create_subscription(
            String, self.get_parameter("state_topic").value, self._on_state, 10,
            callback_group=self._cb)
        self.create_subscription(
            JointState, self.get_parameter("joint_states_topic").value, self._on_js, 10,
            callback_group=self._cb)
        self.param_cli = self.create_client(
            SetParameters, self.vs_node + "/set_parameters", callback_group=self._cb)
        self.arm_cli = ActionClient(
            self, FollowJointTrajectory, self.get_parameter("arm_action").value,
            callback_group=self._cb)

        self._server = ActionServer(
            self, PickObject, "/pick_object",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._cb)
        self.get_logger().info("pick_orchestrator ready: action=/pick_object")

    # --- callbacks ---
    def _on_state(self, msg):
        self.state = msg.data

    def _on_js(self, msg):
        try:
            i = msg.name.index(self.gripper_joint)
            self.width = float(msg.position[i])
        except (ValueError, IndexError):
            pass

    # --- helpers ---
    def _set_bool_param(self, name, value, timeout=4.0):
        if not self.param_cli.wait_for_service(timeout_sec=timeout):
            return False
        pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(value))
        req = SetParameters.Request(parameters=[ParamMsg(name=name, value=pv)])
        fut = self.param_cli.call_async(req)
        deadline = time.time() + timeout
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        return fut.done() and fut.result() is not None

    def _move_to_capture(self, timeout=12.0):
        if not self.arm_cli.wait_for_server(timeout_sec=4.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(joint_names=self.arm_joints)
        pt = JointTrajectoryPoint()
        pt.positions = self.capture_pose
        pt.time_from_start.sec = 5
        goal.trajectory.points = [pt]
        sfut = self.arm_cli.send_goal_async(goal)
        deadline = time.time() + timeout
        while not sfut.done() and time.time() < deadline:
            time.sleep(0.02)
        if not sfut.done() or sfut.result() is None or not sfut.result().accepted:
            return False
        rfut = sfut.result().get_result_async()
        while not rfut.done() and time.time() < deadline:
            time.sleep(0.02)
        return rfut.done()

    # --- action execution ---
    def _execute(self, goal_handle):
        req = goal_handle.request
        obj = req.object.strip()
        timeout = req.timeout_sec if req.timeout_sec and req.timeout_sec > 0 else self.default_timeout
        result = PickObject.Result()

        def finish(success, held, message, abort=False):
            self._set_bool_param("grasp_enabled", False)
            self._set_bool_param("grasp_auto_loop", False)
            result.success = success
            result.object_held = held
            result.gripper_width = float(self.width)
            result.message = message
            if abort:
                goal_handle.abort()
            else:
                goal_handle.succeed()
            self.get_logger().info(f"[PICK '{obj}'] {message}")
            return result

        if not obj:
            return finish(False, False, "empty object label", abort=True)
        self.get_logger().info(f"[PICK '{obj}'] starting (timeout={timeout:.0f}s)")

        # 1. gate off, set the prompt. Enable auto_loop so a node sitting at a stale
        #    DONE re-arms (DONE -> IDLE -> ACQUIRE); it is turned back off the moment
        #    this pick starts, so exactly one pick runs (no auto-loop runaway).
        self._set_bool_param("grasp_enabled", False)
        self._set_bool_param("grasp_auto_loop", True)
        self.prompt_pub.publish(String(data=obj))

        # 2. reset arm to the look-down capture pose
        if self.reset_arm:
            self._move_to_capture()

        # 3. gate the grasp on
        self._publish_feedback(goal_handle)
        if not self._set_bool_param("grasp_enabled", True):
            return finish(False, False, f"could not reach {self.vs_node} to enable grasp", abort=True)

        # 4. wait for the grasp sequence to start, then for DONE
        t0 = time.time()
        saw_grasp = False
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._set_bool_param("grasp_enabled", False)
                result.message = "canceled"
                result.gripper_width = float(self.width)
                goal_handle.canceled()
                return result
            self._publish_feedback(goal_handle)
            now = time.time()
            if self.state in ACTIVE_STATES:
                if not saw_grasp:
                    # Pick has started — disable auto_loop so it won't re-fire after DONE.
                    self._set_bool_param("grasp_auto_loop", False)
                saw_grasp = True
            if not saw_grasp and now - t0 > self.acquire_timeout:
                return finish(False, False,
                              f"'{obj}' not acquired within {self.acquire_timeout:.0f}s "
                              "(not detected, or grasp pose out of workspace)", abort=True)
            if saw_grasp and self.state == "DONE":
                time.sleep(1.0)  # let the lift/width settle
                held = self.width > self.held_threshold
                msg = (f"picked '{obj}'; width={self.width:.4f} "
                       f"({'held (jaw blocked)' if held else 'full close — held only if thin/soft'})")
                return finish(True, held, msg)
            if now - t0 > timeout:
                return finish(False, False,
                              f"timeout after {timeout:.0f}s in state '{self.state}' "
                              "(stuck — e.g. grasp target rejected by workspace gate)", abort=True)
            time.sleep(0.1)
        return finish(False, False, "node shutting down", abort=True)

    def _publish_feedback(self, goal_handle):
        fb = PickObject.Feedback()
        fb.state = self.state
        fb.gripper_width = float(self.width)
        goal_handle.publish_feedback(fb)


def main():
    rclpy.init()
    node = PickOrchestrator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
