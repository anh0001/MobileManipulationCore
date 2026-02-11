#!/usr/bin/env python3
"""
Monitor and display /manipulation/policy_output in a readable format.
Shows real-time policy outputs including EEF pose, joint deltas, gripper, and base hints.
"""

import sys
import math
import rclpy
from rclpy.node import Node
from manipulation_msgs.msg import PolicyOutput


# ANSI color codes
RESET   = "\033[0m"
BOLD    = "\033[1m"
DIM     = "\033[2m"
RED     = "\033[91m"
GREEN   = "\033[92m"
YELLOW  = "\033[93m"
CYAN    = "\033[96m"
WHITE   = "\033[97m"
MAGENTA = "\033[95m"
BLUE    = "\033[94m"

TOPIC = "/manipulation/policy_output"


def quat_to_euler_deg(q):
    """Convert quaternion to roll/pitch/yaw in degrees."""
    # Roll (x-axis)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Yaw (z-axis)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def confidence_bar(value: float, width: int = 20) -> str:
    """Render a confidence value as a colored progress bar."""
    filled = int(value * width)
    bar = "█" * filled + "░" * (width - filled)
    if value >= 0.7:
        color = GREEN
    elif value >= 0.4:
        color = YELLOW
    else:
        color = RED
    return f"{color}[{bar}]{RESET} {value:.2f}"


def gripper_bar(value: float, width: int = 10) -> str:
    """Render gripper state as a visual bar (0=closed, 1=open)."""
    filled = int(value * width)
    bar = "▓" * filled + "░" * (width - filled)
    if value > 0.6:
        color = GREEN
    elif value > 0.3:
        color = YELLOW
    else:
        color = RED
    label = "OPEN" if value > 0.6 else ("MID " if value > 0.3 else "CLOSE")
    return f"{color}[{bar}] {label}{RESET} ({value:.3f})"


def format_vel(v: float) -> str:
    color = GREEN if abs(v) > 0.01 else DIM
    return f"{color}{v:+.4f}{RESET}"


class PolicyMonitor(Node):
    def __init__(self):
        super().__init__("policy_monitor")
        self.msg_count = 0
        self.sub = self.create_subscription(
            PolicyOutput, TOPIC, self.callback, 10
        )
        print(f"\n{BOLD}{CYAN}Policy Output Monitor{RESET}")
        print(f"{DIM}Listening on: {TOPIC}{RESET}")
        print(f"{DIM}Press Ctrl+C to exit{RESET}\n")

    def callback(self, msg: PolicyOutput):
        self.msg_count += 1
        stamp = msg.header.stamp
        ts = f"{stamp.sec}.{stamp.nanosec // 1_000_000:03d}"

        lines = []
        lines.append(
            f"\033[2J\033[H"  # clear screen + move cursor to top
            f"{BOLD}{CYAN}╔══════════════════════════════════════════════════╗{RESET}\n"
            f"{BOLD}{CYAN}║        POLICY OUTPUT  #{self.msg_count:<6}  t={ts}s  ║{RESET}\n"
            f"{BOLD}{CYAN}╚══════════════════════════════════════════════════╝{RESET}"
        )

        # --- Confidence ---
        lines.append(
            f"\n{BOLD}{WHITE}Confidence:{RESET}  {confidence_bar(msg.confidence)}"
        )

        # --- Reference Frame ---
        frame = msg.reference_frame if msg.reference_frame else "(none)"
        lines.append(
            f"{BOLD}{WHITE}Ref Frame:{RESET}   {MAGENTA}{frame}{RESET}"
        )

        # --- EEF Delta Pose ---
        eef_label = f"{BOLD}{GREEN}[ACTIVE]{RESET}" if msg.has_eef_target else f"{DIM}[inactive]{RESET}"
        lines.append(f"\n{BOLD}{WHITE}EEF Delta Pose:{RESET}  {eef_label}  {DIM}(delta from current EEF; eef_pose_is_delta=true){RESET}")
        if msg.has_eef_target:
            p = msg.eef_target_pose.position
            o = msg.eef_target_pose.orientation
            roll, pitch, yaw = quat_to_euler_deg(o)
            lines.append(
                f"  {CYAN}ΔPosition (m) {RESET}"
                f"  X: {YELLOW}{p.x:+.4f}{RESET}"
                f"  Y: {YELLOW}{p.y:+.4f}{RESET}"
                f"  Z: {YELLOW}{p.z:+.4f}{RESET}"
            )
            lines.append(
                f"  {CYAN}ΔOrientation  {RESET}"
                f"  Roll: {YELLOW}{roll:+7.2f}°{RESET}"
                f"  Pitch: {YELLOW}{pitch:+7.2f}°{RESET}"
                f"  Yaw: {YELLOW}{yaw:+7.2f}°{RESET}"
            )
            lines.append(
                f"  {DIM}Quaternion    "
                f"  w={o.w:.4f}  x={o.x:.4f}  y={o.y:.4f}  z={o.z:.4f}{RESET}"
            )
        else:
            lines.append(f"  {DIM}(no EEF delta){RESET}")

        # --- Joint Deltas ---
        joint_label = f"{BOLD}{GREEN}[ACTIVE]{RESET}" if msg.has_joint_deltas else f"{DIM}[inactive]{RESET}"
        lines.append(f"\n{BOLD}{WHITE}Joint Deltas (rad):{RESET}  {joint_label}")
        if msg.has_joint_deltas and msg.joint_deltas:
            joint_line = "  "
            for i, delta in enumerate(msg.joint_deltas):
                bar_len = min(10, int(abs(delta) / 0.2 * 10))
                direction = "▶" if delta >= 0 else "◀"
                color = GREEN if abs(delta) > 0.005 else DIM
                joint_line += f"{color}J{i+1}: {delta:+.4f} {direction}{RESET}  "
                if (i + 1) % 3 == 0:
                    lines.append(joint_line)
                    joint_line = "  "
            if joint_line.strip():
                lines.append(joint_line)
        else:
            lines.append(f"  {DIM}(no joint deltas){RESET}")

        # --- Gripper ---
        gripper_label = f"{BOLD}{GREEN}[ACTIVE]{RESET}" if msg.gripper_active else f"{DIM}[inactive]{RESET}"
        lines.append(f"\n{BOLD}{WHITE}Gripper:{RESET}  {gripper_label}")
        lines.append(f"  {gripper_bar(msg.gripper_command)}")

        # --- Base Velocity Hint ---
        base_label = f"{BOLD}{GREEN}[ACTIVE]{RESET}" if msg.has_base_hint else f"{DIM}[inactive]{RESET}"
        lines.append(f"\n{BOLD}{WHITE}Base Velocity Hint:{RESET}  {base_label}")
        if msg.has_base_hint:
            v = msg.base_velocity_hint.linear
            w = msg.base_velocity_hint.angular
            lines.append(
                f"  {CYAN}Linear  (m/s)  {RESET}"
                f"  X: {format_vel(v.x)}"
                f"  Y: {format_vel(v.y)}"
                f"  Z: {format_vel(v.z)}"
            )
            lines.append(
                f"  {CYAN}Angular (r/s)  {RESET}"
                f"  X: {format_vel(w.x)}"
                f"  Y: {format_vel(w.y)}"
                f"  Z: {format_vel(w.z)} {YELLOW}(yaw){RESET}"
            )
        else:
            lines.append(f"  {DIM}(no base hint){RESET}")

        lines.append(f"\n{DIM}{'─' * 52}{RESET}")

        print("\n".join(lines), flush=True)


def main():
    rclpy.init(args=sys.argv)
    node = PolicyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Stopped.{RESET}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
