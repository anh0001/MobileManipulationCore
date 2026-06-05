#!/usr/bin/env python3
# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""skill_server — one ROS 2 action that dispatches every registered robot skill.

Replaces the per-action orchestrator pattern. A single ActionServer on
``/execute_skill`` (manipulation_msgs/action/ExecuteSkill) receives a skill name
+ a JSON params object, looks the skill up in the registry, validates the args
against the skill's declared schema, and runs it. The node provides the shared
robot plumbing (prompt publish, remote params, arm move, live state) so skills
stay small. Adding a skill needs no change to this file.

Run inside a sourced ROS 2 workspace; launched automatically by core_launch.py
in visual-servo mode.
"""
import json
import time
from typing import Any, List

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType

from manipulation_msgs.action import ExecuteSkill
from manipulation_policy.skills import all_skills, get_skill
from manipulation_policy.skills.base import SkillContext


class SkillServer(Node, SkillContext):
    """ActionServer + SkillContext: dispatches skills, owns the ROS plumbing."""

    def __init__(self):
        super().__init__("skill_server")
        # plumbing config (shared by skills via get_param)
        self.declare_parameter("visual_servo_node", "/visual_servo_node")
        self.declare_parameter("prompt_topic", "/visual_servo/target_prompt")
        self.declare_parameter("state_topic", "/visual_servo/state")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("gripper_joint", "piper_joint7")
        self.declare_parameter("arm_action", "/piper_arm_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_action", "/piper_gripper_controller/gripper_cmd")
        self.declare_parameter("arm_joints", [f"piper_joint{i}" for i in range(1, 7)])
        self.declare_parameter("gripper_open_position", 0.07)
        self.declare_parameter("gripper_max_effort", 5.0)
        # skill-tunable params (read by skills through get_param)
        self.declare_parameter("capture_pose", [0.0, 1.2, -0.2, 0.0, -0.35, 0.0])
        self.declare_parameter("default_timeout_sec", 60.0)
        self.declare_parameter("acquire_timeout_sec", 12.0)
        # Held-vs-empty gripper-width gate (no force sensor). Empty full-close lands
        # at ~0.000–0.0022 m; a held thin object (e.g. a toy banana) sits at
        # ~0.005 m+. 0.004 m sits in that gap so thin held objects aren't misread as
        # empty. Thicker objects close on a wider gap, so they clear it easily.
        self.declare_parameter("held_width_threshold", 0.004)
        self.declare_parameter("reset_arm_each_pick", True)
        # pick_and_place: height (m) above the detected destination point at which
        # the held object is released, so it clears the destination instead of
        # bumping it. Approached from the elevated transit pose, then dropped.
        self.declare_parameter("place_clearance_m", 0.15)
        # default drop pose over the box (6 arm joints). Empty -> place skill
        # requires an explicit pose so the arm never swings to a guessed spot.
        self.declare_parameter("place_pose", [0.0])

        # Per-skill tunables: every registered skill may declare a `server_params`
        # map (name -> default). Declare them all here so skills read them via
        # get_param and they stay runtime-settable / config-overridable. A new
        # skill with its own tunables needs no edit to this file.
        for skill in all_skills():
            for pname, pdefault in getattr(skill, "server_params", {}).items():
                if not self.has_parameter(pname):
                    self.declare_parameter(pname, pdefault)

        self._vs_node = self.get_parameter("visual_servo_node").value
        self._gripper_joint = self.get_parameter("gripper_joint").value
        self._arm_joints = list(self.get_parameter("arm_joints").value)

        self._cb = ReentrantCallbackGroup()
        self._state = ""
        self._width = 0.0

        self.prompt_pub = self.create_publisher(
            String, self.get_parameter("prompt_topic").value, 10)
        self.create_subscription(
            String, self.get_parameter("state_topic").value, self._on_state, 10,
            callback_group=self._cb)
        self.create_subscription(
            JointState, self.get_parameter("joint_states_topic").value, self._on_js, 10,
            callback_group=self._cb)
        self.param_cli = self.create_client(
            SetParameters, self._vs_node + "/set_parameters", callback_group=self._cb)
        self.get_param_cli = self.create_client(
            GetParameters, self._vs_node + "/get_parameters", callback_group=self._cb)
        self.arm_cli = ActionClient(
            self, FollowJointTrajectory, self.get_parameter("arm_action").value,
            callback_group=self._cb)
        self.gripper_cli = ActionClient(
            self, GripperCommand, self.get_parameter("gripper_action").value,
            callback_group=self._cb)

        self._server = ActionServer(
            self, ExecuteSkill, "/execute_skill",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._cb)
        names = ", ".join(s.name for s in all_skills())
        self.get_logger().info(
            f"skill_server ready: action=/execute_skill skills=[{names}]")

    # --- subscriptions ---
    def _on_state(self, msg):
        self._state = msg.data

    def _on_js(self, msg):
        try:
            i = msg.name.index(self._gripper_joint)
            self._width = float(msg.position[i])
        except (ValueError, IndexError):
            pass

    # --- SkillContext implementation ---
    @property
    def vs_state(self) -> str:
        return self._state

    @property
    def gripper_width(self) -> float:
        return self._width

    def get_param(self, name: str, default: Any = None) -> Any:
        if not self.has_parameter(name):
            return default
        value = self.get_parameter(name).value
        return default if value is None else value

    def publish_prompt(self, text: str) -> None:
        self.prompt_pub.publish(String(data=text))

    def set_bool_param(self, name: str, value: bool,
                       node: str | None = None, timeout: float = 4.0) -> bool:
        cli = self.param_cli
        if node is not None and node != self._vs_node:
            cli = self.create_client(SetParameters, node + "/set_parameters",
                                     callback_group=self._cb)
        if not cli.wait_for_service(timeout_sec=timeout):
            return False
        pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(value))
        req = SetParameters.Request(parameters=[ParamMsg(name=name, value=pv)])
        fut = cli.call_async(req)
        deadline = time.time() + timeout
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        return fut.done() and fut.result() is not None

    def set_double_array_param(self, name: str, values: List[float],
                               node: str | None = None, timeout: float = 4.0) -> bool:
        cli = self.param_cli
        if node is not None and node != self._vs_node:
            cli = self.create_client(SetParameters, node + "/set_parameters",
                                     callback_group=self._cb)
        if not cli.wait_for_service(timeout_sec=timeout):
            return False
        pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                            double_array_value=[float(v) for v in values])
        req = SetParameters.Request(parameters=[ParamMsg(name=name, value=pv)])
        fut = cli.call_async(req)
        deadline = time.time() + timeout
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        return fut.done() and fut.result() is not None

    def set_double_param(self, name: str, value: float,
                         node: str | None = None, timeout: float = 4.0) -> bool:
        cli = self.param_cli
        if node is not None and node != self._vs_node:
            cli = self.create_client(SetParameters, node + "/set_parameters",
                                     callback_group=self._cb)
        if not cli.wait_for_service(timeout_sec=timeout):
            return False
        pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                            double_value=float(value))
        req = SetParameters.Request(parameters=[ParamMsg(name=name, value=pv)])
        fut = cli.call_async(req)
        deadline = time.time() + timeout
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        return fut.done() and fut.result() is not None

    def get_remote_params(self, names: List[str],
                          node: str | None = None, timeout: float = 4.0) -> dict:
        """Read parameters from a remote node (default: the visual-servo node).

        Returns {name: value} for each requested param, decoding the typed
        ParameterValue. Missing/unset params (PARAMETER_NOT_SET) are omitted.
        Returns {} if the service is unreachable.
        """
        cli = self.get_param_cli
        if node is not None and node != self._vs_node:
            cli = self.create_client(GetParameters, node + "/get_parameters",
                                     callback_group=self._cb)
        if not cli.wait_for_service(timeout_sec=timeout):
            return {}
        fut = cli.call_async(GetParameters.Request(names=list(names)))
        deadline = time.time() + timeout
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        if not fut.done() or fut.result() is None:
            return {}
        out: dict = {}
        for n, pv in zip(names, fut.result().values):
            t = pv.type
            if t == ParameterType.PARAMETER_BOOL:
                out[n] = bool(pv.bool_value)
            elif t == ParameterType.PARAMETER_INTEGER:
                out[n] = int(pv.integer_value)
            elif t == ParameterType.PARAMETER_DOUBLE:
                out[n] = float(pv.double_value)
            elif t == ParameterType.PARAMETER_STRING:
                out[n] = str(pv.string_value)
            elif t == ParameterType.PARAMETER_DOUBLE_ARRAY:
                out[n] = [float(v) for v in pv.double_array_value]
            elif t == ParameterType.PARAMETER_INTEGER_ARRAY:
                out[n] = [int(v) for v in pv.integer_array_value]
            # PARAMETER_NOT_SET and unhandled types are intentionally omitted.
        return out

    def move_arm_to(self, positions: List[float],
                    time_sec: float = 5.0, timeout: float = 12.0) -> bool:
        if not self.arm_cli.wait_for_server(timeout_sec=4.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(joint_names=self._arm_joints)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start.sec = int(time_sec)
        pt.time_from_start.nanosec = int((time_sec - int(time_sec)) * 1e9)
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

    def set_gripper(self, position: float, max_effort: float = 5.0,
                    timeout: float = 10.0) -> bool:
        if not self.gripper_cli.wait_for_server(timeout_sec=4.0):
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)  # must be > 0 to actuate
        sfut = self.gripper_cli.send_goal_async(goal)
        deadline = time.time() + timeout
        while not sfut.done() and time.time() < deadline:
            time.sleep(0.02)
        if not sfut.done() or sfut.result() is None or not sfut.result().accepted:
            return False
        rfut = sfut.result().get_result_async()
        while not rfut.done() and time.time() < deadline:
            time.sleep(0.02)
        return rfut.done()

    def log(self, message: str) -> None:
        self.get_logger().info(message)

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)

    def now(self) -> float:
        return time.time()

    def ok(self) -> bool:
        return rclpy.ok()

    # --- action execution ---
    def _execute(self, goal_handle):
        req = goal_handle.request
        skill_name = (req.skill or "").strip()
        result = ExecuteSkill.Result()

        def fail(message: str):
            result.success = False
            result.message = message
            result.result_json = "{}"
            goal_handle.abort()
            self.get_logger().warn(f"[skill '{skill_name}'] {message}")
            return result

        skill = get_skill(skill_name)
        if skill is None:
            avail = ", ".join(s.name for s in all_skills())
            return fail(f"unknown skill '{skill_name}'; available: {avail}")

        try:
            raw = json.loads(req.params_json) if req.params_json.strip() else {}
            if not isinstance(raw, dict):
                raise ValueError("params_json must be a JSON object")
            params = skill.validate(raw)
        except (json.JSONDecodeError, ValueError) as e:
            return fail(f"bad params: {e}")

        def feedback(state: str, progress: float = 0.0):
            fb = ExecuteSkill.Feedback()
            fb.state = str(state)
            fb.progress = float(progress)
            goal_handle.publish_feedback(fb)

        def is_cancelled():
            return goal_handle.is_cancel_requested

        try:
            sr = skill.execute(self, params, feedback, is_cancelled)
        except Exception as e:  # a skill bug must not crash the server
            self.get_logger().error(f"[skill '{skill_name}'] raised: {e}")
            return fail(f"skill raised: {e}")

        result.success = bool(sr.success)
        result.message = str(sr.message)
        result.result_json = json.dumps(sr.data or {})
        if is_cancelled() and not sr.success:
            goal_handle.canceled()
        elif sr.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(f"[skill '{skill_name}'] {sr.message}")
        return result


def main():
    rclpy.init()
    node = SkillServer()
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
