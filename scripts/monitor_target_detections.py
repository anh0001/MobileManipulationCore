#!/usr/bin/env python3
"""
Monitor and display /manipulation/target_detections in a readable format.
Shows real-time Detection2DArray output: bounding boxes, class labels, and confidence scores.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


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

TOPIC = "/manipulation/target_detections"

# Colors cycled per detection index for easy visual separation
DET_COLORS = [CYAN, MAGENTA, YELLOW, GREEN, BLUE]


def confidence_bar(value: float, width: int = 20) -> str:
    """Render a confidence score as a colored progress bar."""
    filled = int(value * width)
    bar = "█" * filled + "░" * (width - filled)
    if value >= 0.7:
        color = GREEN
    elif value >= 0.4:
        color = YELLOW
    else:
        color = RED
    return f"{color}[{bar}]{RESET} {value:.2f}"


def bbox_ascii(cx: float, cy: float, w: float, h: float, img_w: int = 80, img_h: int = 8) -> list:
    """Render a simple ASCII bounding box scaled to terminal width."""
    # Normalize to terminal grid
    norm_cx = cx / max(1.0, w + cx)  # rough normalization
    x1 = max(0, int((cx - w / 2) / 640 * img_w))
    x2 = min(img_w - 1, int((cx + w / 2) / 640 * img_w))
    y1 = max(0, int((cy - h / 2) / 480 * img_h))
    y2 = min(img_h - 1, int((cy + h / 2) / 480 * img_h))
    bw = max(2, x2 - x1)
    bh = max(1, y2 - y1)

    rows = []
    for row in range(img_h):
        line = [" "] * img_w
        if row == y1 or row == y2:
            for col in range(x1, x1 + bw + 1):
                if 0 <= col < img_w:
                    line[col] = "─"
            if 0 <= x1 < img_w:
                line[x1] = "┌" if row == y1 else "└"
            if 0 <= x1 + bw < img_w:
                line[x1 + bw] = "┐" if row == y1 else "┘"
        elif y1 < row < y2:
            if 0 <= x1 < img_w:
                line[x1] = "│"
            if 0 <= x1 + bw < img_w:
                line[x1 + bw] = "│"
        rows.append("".join(line))
    return rows


class DetectionMonitor(Node):
    def __init__(self):
        super().__init__("detection_monitor")

        self.msg_count = 0
        self.last_msg_time = None
        self.last_msg = None

        self.sub = self.create_subscription(
            Detection2DArray, TOPIC, self.callback, 10
        )
        self.display_timer = self.create_timer(0.1, self.display)

        print(f"\n{BOLD}{CYAN}Target Detection Monitor{RESET}")
        print(f"{DIM}Listening on: {TOPIC}{RESET}")
        print(f"{DIM}Message type: vision_msgs/Detection2DArray{RESET}")
        print(f"{DIM}Press Ctrl+C to exit{RESET}\n")

    def callback(self, msg: Detection2DArray):
        self.msg_count += 1
        self.last_msg = msg
        self.last_msg_time = self.get_clock().now()

    def display(self):
        now = self.get_clock().now()
        active = (
            self.last_msg_time is not None
            and (now - self.last_msg_time).nanoseconds < 2_000_000_000  # 2 seconds
        )

        lines = []
        lines.append(
            f"\033[2J\033[H"  # clear screen + move cursor to top
            f"{BOLD}{CYAN}╔══════════════════════════════════════════════════════════╗{RESET}\n"
            f"{BOLD}{CYAN}║           TARGET DETECTION MONITOR  #{self.msg_count:<6}           ║{RESET}\n"
            f"{BOLD}{CYAN}╚══════════════════════════════════════════════════════════╝{RESET}"
        )

        # Topic / status header
        status = f"{BOLD}{GREEN}[ACTIVE]{RESET}" if active else f"{DIM}[waiting...]{RESET}"
        lines.append(
            f"\n{BOLD}{WHITE}Topic:{RESET}  {DIM}{TOPIC}{RESET}   {status}   "
            f"{DIM}(msgs: {self.msg_count}){RESET}"
        )

        if self.last_msg is None:
            lines.append(f"\n  {DIM}No messages received yet.{RESET}")
            lines.append(f"  {YELLOW}⚠ Check if remote_detection_client is running and prompt is set.{RESET}")
            print("\n".join(lines), flush=True)
            return

        msg = self.last_msg
        stamp = msg.header.stamp
        ts = f"{stamp.sec}.{stamp.nanosec // 1_000_000:03d}"
        frame = msg.header.frame_id or "(no frame)"
        n_det = len(msg.detections)

        lines.append(
            f"  {DIM}Header: t={ts}s  frame={MAGENTA}{frame}{DIM}{RESET}"
        )

        # Detection count summary
        if n_det == 0:
            lines.append(f"\n  {YELLOW}● No detections in last message{RESET}")
        else:
            count_color = GREEN if n_det >= 1 else YELLOW
            lines.append(f"\n  {count_color}{BOLD}● {n_det} detection(s) found{RESET}")

        lines.append(f"\n{DIM}{'─' * 60}{RESET}")

        # Per-detection breakdown
        for i, det in enumerate(msg.detections):
            color = DET_COLORS[i % len(DET_COLORS)]

            # ID and class
            det_id = det.id if det.id else f"det_{i}"
            class_id = ""
            score = 0.0
            if det.results:
                best = max(det.results, key=lambda r: r.hypothesis.score)
                class_id = best.hypothesis.class_id or "(no class)"
                score = best.hypothesis.score

            lines.append(
                f"\n{color}{BOLD}[{i+1}] {class_id}{RESET}"
                f"  {DIM}id={det_id}{RESET}"
            )

            # Confidence bar
            lines.append(
                f"    {BOLD}{WHITE}Score:{RESET}  {confidence_bar(score)}"
            )

            # Bounding box
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            bw = det.bbox.size_x
            bh = det.bbox.size_y
            x1 = cx - bw / 2
            y1 = cy - bh / 2
            x2 = cx + bw / 2
            y2 = cy + bh / 2

            lines.append(
                f"    {BOLD}{WHITE}BBox:{RESET}"
                f"  {color}center=({cx:.1f}, {cy:.1f}){RESET}"
                f"  {color}size=({bw:.1f} x {bh:.1f}){RESET}"
            )
            lines.append(
                f"           "
                f"  {DIM}top-left=({x1:.1f}, {y1:.1f})  bottom-right=({x2:.1f}, {y2:.1f}){RESET}"
            )

            # All hypotheses if more than one
            if len(det.results) > 1:
                lines.append(f"    {DIM}All hypotheses:{RESET}")
                for r in sorted(det.results, key=lambda r: r.hypothesis.score, reverse=True):
                    hyp_color = GREEN if r.hypothesis.score >= 0.7 else (YELLOW if r.hypothesis.score >= 0.4 else RED)
                    lines.append(
                        f"      {hyp_color}{r.hypothesis.class_id or '?':<20}{RESET}"
                        f"  {confidence_bar(r.hypothesis.score, width=15)}"
                    )

        lines.append(f"\n{DIM}{'─' * 60}{RESET}")

        # Staleness warning
        if self.last_msg_time is not None:
            age_ms = (now - self.last_msg_time).nanoseconds / 1_000_000
            if age_ms > 500:
                lines.append(f"{YELLOW}⚠ Last message {age_ms:.0f}ms ago  (check detection rate){RESET}")
            else:
                lines.append(f"{DIM}Last message {age_ms:.0f}ms ago{RESET}")
        else:
            lines.append(f"{YELLOW}⚠ No messages received (check if adapter node is running){RESET}")

        print("\n".join(lines), flush=True)


def main():
    rclpy.init(args=sys.argv)
    node = DetectionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Stopped.{RESET}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
