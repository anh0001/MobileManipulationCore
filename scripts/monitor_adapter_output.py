#!/usr/bin/env python3
"""
Monitor and display adapter outputs in a readable format.
Shows real-time base velocity commands (/cmd_vel) and servo twist commands.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


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

CMD_VEL_TOPIC = "/cmd_vel"
SERVO_TOPIC = "/servo_node/delta_twist_cmds"  # Default servo topic


def format_vel(v: float, threshold: float = 0.001) -> str:
    """Format velocity with color based on magnitude."""
    if abs(v) < threshold:
        return f"{DIM}{v:+.4f}{RESET}"
    elif abs(v) < 0.1:
        return f"{YELLOW}{v:+.4f}{RESET}"
    else:
        return f"{GREEN if v >= 0 else RED}{v:+.4f}{RESET}"


def velocity_bar(value: float, max_val: float = 1.0, width: int = 20) -> str:
    """Render a velocity as a centered bar (-max to +max)."""
    # Normalize to -1 to +1
    normalized = max(-1.0, min(1.0, value / max_val))
    
    # Calculate bar position (center is at width/2)
    center = width // 2
    if normalized >= 0:
        filled_start = center
        filled_end = center + int(normalized * center)
        bar_chars = [" "] * width
        for i in range(filled_start, filled_end):
            bar_chars[i] = "█"
        color = GREEN if abs(normalized) > 0.1 else YELLOW
    else:
        filled_end = center
        filled_start = center + int(normalized * center)
        bar_chars = [" "] * width
        for i in range(filled_start, filled_end):
            bar_chars[i] = "█"
        color = RED if abs(normalized) > 0.1 else YELLOW
    
    bar = "".join(bar_chars)
    bar_chars[center] = "│"  # Center marker
    bar_with_center = "".join(bar_chars)
    
    if abs(value) < 0.001:
        color = DIM
    
    return f"{color}[{bar_with_center}]{RESET}"


class AdapterMonitor(Node):
    def __init__(self):
        super().__init__("adapter_monitor")
        
        self.cmd_vel_count = 0
        self.servo_count = 0
        self.last_cmd_vel = None
        self.last_servo = None
        self.last_cmd_vel_time = None
        self.last_servo_time = None
        
        # Declare parameters
        self.declare_parameter("servo_topic", SERVO_TOPIC)
        servo_topic = self.get_parameter("servo_topic").value
        
        # Create subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist, CMD_VEL_TOPIC, self.cmd_vel_callback, 10
        )
        self.servo_sub = self.create_subscription(
            TwistStamped, servo_topic, self.servo_callback, 10
        )
        
        # Display timer
        self.display_timer = self.create_timer(0.1, self.display)
        
        print(f"\n{BOLD}{CYAN}Adapter Output Monitor{RESET}")
        print(f"{DIM}Monitoring:{RESET}")
        print(f"{DIM}  - {CMD_VEL_TOPIC} (base velocity){RESET}")
        print(f"{DIM}  - {servo_topic} (servo twist){RESET}")
        print(f"{DIM}Press Ctrl+C to exit{RESET}\n")

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel_count += 1
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()

    def servo_callback(self, msg: TwistStamped):
        self.servo_count += 1
        self.last_servo = msg
        self.last_servo_time = self.get_clock().now()

    def display(self):
        """Display current adapter outputs."""
        now = self.get_clock().now()
        
        # Check for timeouts
        cmd_vel_active = (
            self.last_cmd_vel_time is not None and
            (now - self.last_cmd_vel_time).nanoseconds < 1e9  # 1 second
        )
        servo_active = (
            self.last_servo_time is not None and
            (now - self.last_servo_time).nanoseconds < 1e9  # 1 second
        )
        
        lines = []
        lines.append(
            f"\033[2J\033[H"  # clear screen + move cursor to top
            f"{BOLD}{CYAN}╔══════════════════════════════════════════════════════════════════╗{RESET}\n"
            f"{BOLD}{CYAN}║                    ADAPTER OUTPUT MONITOR                       ║{RESET}\n"
            f"{BOLD}{CYAN}╚══════════════════════════════════════════════════════════════════╝{RESET}"
        )
        
        # --- Base Velocity (/cmd_vel) ---
        status = f"{BOLD}{GREEN}[ACTIVE]{RESET}" if cmd_vel_active else f"{DIM}[inactive]{RESET}"
        lines.append(
            f"\n{BOLD}{WHITE}Base Velocity (/cmd_vel):{RESET}  {status}  "
            f"{DIM}(msgs: {self.cmd_vel_count}){RESET}"
        )
        
        if self.last_cmd_vel:
            v = self.last_cmd_vel.linear
            w = self.last_cmd_vel.angular
            
            # Linear velocity
            lines.append(f"\n  {CYAN}Linear Velocity (m/s):{RESET}")
            lines.append(
                f"    X (forward): {format_vel(v.x, 0.001)}  "
                f"{velocity_bar(v.x, 0.5, 30)}"
            )
            lines.append(
                f"    Y (lateral): {format_vel(v.y, 0.001)}  "
                f"{velocity_bar(v.y, 0.5, 30)}"
            )
            lines.append(
                f"    Z (up/down): {format_vel(v.z, 0.001)}  "
                f"{velocity_bar(v.z, 0.5, 30)}"
            )
            
            # Angular velocity
            lines.append(f"\n  {CYAN}Angular Velocity (rad/s):{RESET}")
            lines.append(
                f"    Roll  (X):   {format_vel(w.x, 0.001)}  "
                f"{velocity_bar(w.x, 1.0, 30)}"
            )
            lines.append(
                f"    Pitch (Y):   {format_vel(w.y, 0.001)}  "
                f"{velocity_bar(w.y, 1.0, 30)}"
            )
            lines.append(
                f"    Yaw   (Z):   {format_vel(w.z, 0.001)}  "
                f"{velocity_bar(w.z, 1.0, 30)}"
            )
            
            # Overall status
            moving = any(abs(x) > 0.001 for x in [v.x, v.y, v.z, w.x, w.y, w.z])
            if moving:
                lines.append(f"\n  {GREEN}● BASE MOVING{RESET}")
            else:
                lines.append(f"\n  {DIM}○ Base stationary{RESET}")
        else:
            lines.append(f"  {DIM}(no messages received yet){RESET}")
        
        # --- Servo Twist ---
        servo_status = f"{BOLD}{GREEN}[ACTIVE]{RESET}" if servo_active else f"{DIM}[inactive]{RESET}"
        lines.append(
            f"\n\n{BOLD}{WHITE}Servo Twist (arm end-effector):{RESET}  {servo_status}  "
            f"{DIM}(msgs: {self.servo_count}){RESET}"
        )
        
        if self.last_servo:
            twist = self.last_servo.twist
            v = twist.linear
            w = twist.angular
            
            stamp = self.last_servo.header.stamp
            frame = self.last_servo.header.frame_id or "(no frame)"
            lines.append(f"  {DIM}Frame: {MAGENTA}{frame}{RESET}  Time: {stamp.sec}.{stamp.nanosec//1_000_000:03d}s{RESET}")
            
            # Linear
            lines.append(f"\n  {CYAN}Linear (m/s):{RESET}")
            lines.append(
                f"    X: {format_vel(v.x)}  "
                f"{velocity_bar(v.x, 0.1, 25)}"
            )
            lines.append(
                f"    Y: {format_vel(v.y)}  "
                f"{velocity_bar(v.y, 0.1, 25)}"
            )
            lines.append(
                f"    Z: {format_vel(v.z)}  "
                f"{velocity_bar(v.z, 0.1, 25)}"
            )
            
            # Angular
            lines.append(f"\n  {CYAN}Angular (rad/s):{RESET}")
            lines.append(
                f"    X: {format_vel(w.x)}  "
                f"{velocity_bar(w.x, 0.5, 25)}"
            )
            lines.append(
                f"    Y: {format_vel(w.y)}  "
                f"{velocity_bar(w.y, 0.5, 25)}"
            )
            lines.append(
                f"    Z: {format_vel(w.z)}  "
                f"{velocity_bar(w.z, 0.5, 25)}"
            )
            
            # Overall status
            moving = any(abs(x) > 0.001 for x in [v.x, v.y, v.z, w.x, w.y, w.z])
            if moving:
                lines.append(f"\n  {GREEN}● ARM MOVING{RESET}")
            else:
                lines.append(f"\n  {DIM}○ Arm stationary{RESET}")
        else:
            lines.append(f"  {DIM}(no messages received yet){RESET}")
        
        # Footer
        lines.append(f"\n{DIM}{'─' * 68}{RESET}")
        if not cmd_vel_active and not servo_active:
            lines.append(f"{YELLOW}⚠ No recent messages (check if adapter node is running){RESET}")
        
        print("\n".join(lines), flush=True)


def main():
    rclpy.init(args=sys.argv)
    node = AdapterMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Stopped.{RESET}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
