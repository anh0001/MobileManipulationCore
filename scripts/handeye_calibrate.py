#!/usr/bin/env python3
"""Eye-in-hand hand-eye calibration for the PiPER wrist D405 + ChArUco board.

Moves the arm through a set of SAFE, bounded poses around the down-look capture
pose (j3 kept near -0.2 so the arm never folds toward the lidar), detects the
ChArUco board at each, solves camera<-board with solvePnP, reads base<-TCP from
TF, and runs cv2.calibrateHandEye to get the fixed TCP->camera transform.

Outputs:
- TCP->camera transform (xyz + rpy + quaternion) to paste into the URDF camera
  mount / a static TF.
- Saves raw samples to ~/handeye_samples.npz so the solve can be re-run offline.

Run (with bringup + D405 up, board fixed in view):
  PYTHONNOUSERSITE=1 python3 scripts/handeye_calibrate.py
"""

import math
import os
import time

import numpy as np
import cv2
import cv_bridge
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener

# ---- Board geometry (from the printed calib.io ChArUco) -------------------
SQUARES_X = 10          # columns
SQUARES_Y = 7           # rows
SQUARE_LEN = 0.025      # m (measured)
MARKER_LEN = 0.018      # m (~0.7 * square; approximate is fine for ChArUco)
DICT = cv2.aruco.DICT_4X4_50

RGB_TOPIC = "/piper/wrist_camera/piper_d405/color/image_raw"
INFO_TOPIC = "/piper/wrist_camera/piper_d405/color/camera_info"
ARM_ACTION = "/piper_arm_controller/follow_joint_trajectory"
BASE_FRAME = "piper_base_link"
TCP_FRAME = "piper_tcp"

# ---- SAFE pose sweep: [j1..j6]. j3 fixed near -0.2 (away from lidar), small
# deltas, wide ORIENTATION diversity via j1(yaw)/j5(pitch)/j6(roll). ----------
import json as _json

# Start pose = the hand-guided calibration pose (camera looks well at the board).
_cfg = os.path.join(os.path.dirname(__file__), "..", "config", "calibration_start_pose.json")
START = _json.load(open(_cfg))["joint_positions"]

# Bounded deltas around the start pose. Orientation diversity (what hand-eye
# needs) comes from wrist rolls (j4,j6) + small pitch(j5)/yaw(j1). j2/j3 (the
# arm posture) are held near-fixed so the arm never folds further toward the
# lidar. Poses that lose the board are skipped automatically.
DELTAS = [
    [0.00, 0.00, 0.0, 0.00, 0.00, 0.00],
    [0.00, 0.00, 0.0, 0.45, 0.00, 0.60],
    [0.00, 0.00, 0.0, -0.45, 0.00, -0.60],
    [0.20, 0.00, 0.0, 0.35, 0.25, 0.40],
    [-0.20, 0.00, 0.0, -0.35, 0.25, -0.40],
    [0.20, 0.05, 0.0, -0.30, -0.30, 0.50],
    [-0.20, -0.05, 0.0, 0.30, -0.30, -0.50],
    [0.00, 0.05, 0.0, 0.45, 0.30, -0.60],
    [0.00, -0.05, 0.0, -0.45, -0.35, 0.60],
    [0.15, 0.00, 0.0, 0.20, 0.35, 0.55],
    [-0.15, 0.00, 0.0, -0.20, 0.35, -0.55],
    [0.25, 0.00, 0.0, -0.50, 0.00, 0.20],
    [-0.25, 0.00, 0.0, 0.50, 0.00, -0.20],
    [0.10, 0.05, 0.0, 0.35, -0.25, 0.30],
    [-0.10, -0.05, 0.0, -0.35, 0.30, -0.30],
    [0.00, 0.00, 0.0, 0.00, 0.00, 0.00],
]
POSES = [[round(s + d, 4) for s, d in zip(START, dl)] for dl in DELTAS]

MOVE_SEC = 7            # slow moves
SETTLE_SEC = 2.0
MIN_CORNERS = 6


def make_board():
    d = cv2.aruco.getPredefinedDictionary(DICT)
    try:  # OpenCV >= 4.7
        board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LEN, MARKER_LEN, d)
    except Exception:  # legacy
        board = cv2.aruco.CharucoBoard_create(SQUARES_X, SQUARES_Y, SQUARE_LEN, MARKER_LEN, d)
    return d, board


def detect_charuco(gray, d, board, K, dist):
    """Return (rvec, tvec, n_corners) of board in camera, or (None, None, 0)."""
    try:  # new API
        cd = cv2.aruco.CharucoDetector(board)
        ch_corners, ch_ids, _, _ = cd.detectBoard(gray)
    except Exception:  # legacy API
        try:
            ad = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
            m_corners, m_ids, _ = ad.detectMarkers(gray)
        except Exception:
            m_corners, m_ids, _ = cv2.aruco.detectMarkers(gray, d)
        if m_ids is None or len(m_ids) == 0:
            return None, None, 0
        _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
            m_corners, m_ids, gray, board)
    if ch_ids is None or len(ch_ids) < MIN_CORNERS:
        return None, None, 0 if ch_ids is None else len(ch_ids)
    # Pose: match charuco corners to board object points, solvePnP.
    try:
        obj_pts, img_pts = board.matchImagePoints(ch_corners, ch_ids)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist)
    except Exception:
        rvec = np.zeros((3, 1)); tvec = np.zeros((3, 1))
        ok = cv2.aruco.estimatePoseCharucoBoard(
            ch_corners, ch_ids, board, K, dist, rvec, tvec)
    if not ok:
        return None, None, len(ch_ids)
    return rvec, tvec, len(ch_ids)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def R_to_rpy(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        return (math.atan2(R[2, 1], R[2, 2]),
                math.atan2(-R[2, 0], sy),
                math.atan2(R[1, 0], R[0, 0]))
    return (math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0.0)


def main():
    rclpy.init()
    n = Node("handeye")
    bridge = cv_bridge.CvBridge()
    state = {"img": None, "K": None, "dist": None}

    def img_cb(m):
        state["img"] = m

    def info_cb(m):
        state["K"] = np.array(m.k, dtype=np.float64).reshape(3, 3)
        state["dist"] = np.array(m.d, dtype=np.float64).reshape(1, -1)

    n.create_subscription(Image, RGB_TOPIC, img_cb, 10)
    n.create_subscription(CameraInfo, INFO_TOPIC, info_cb, 10)
    buf = Buffer(); TransformListener(buf, n)
    ac = ActionClient(n, FollowJointTrajectory, ARM_ACTION)
    ac.wait_for_server(timeout_sec=10)
    d, board = make_board()

    # wait for camera_info
    t0 = time.time()
    while state["K"] is None and time.time() - t0 < 8:
        rclpy.spin_once(n, timeout_sec=0.2)
    if state["K"] is None:
        print("NO camera_info"); return
    print("camera K:\n", state["K"])

    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []

    def goto(pos):
        g = FollowJointTrajectory.Goal(); jt = JointTrajectory()
        jt.joint_names = [f"piper_joint{i}" for i in range(1, 7)]
        p = JointTrajectoryPoint(); p.positions = [float(v) for v in pos]
        p.velocities = [0.0] * 6; p.time_from_start = Duration(sec=MOVE_SEC)
        jt.points = [p]; g.trajectory = jt
        fut = ac.send_goal_async(g)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=8)
        time.sleep(MOVE_SEC + SETTLE_SEC)

    for i, pose in enumerate(POSES):
        print(f"\n[{i+1}/{len(POSES)}] moving to {pose} ...")
        goto(pose)
        # fresh frame
        state["img"] = None
        t0 = time.time()
        while state["img"] is None and time.time() - t0 < 4:
            rclpy.spin_once(n, timeout_sec=0.2)
        if state["img"] is None:
            print("  no image, skip"); continue
        cv = bridge.imgmsg_to_cv2(state["img"], "bgr8")
        gray = cv2.cvtColor(cv, cv2.COLOR_BGR2GRAY)
        rvec, tvec, ncor = detect_charuco(gray, d, board, state["K"], state["dist"])
        if rvec is None:
            print(f"  board not detected ({ncor} corners), skip"); continue
        # base<-TCP
        try:
            tf = buf.lookup_transform(BASE_FRAME, TCP_FRAME, rclpy.time.Time())
        except Exception as e:
            print("  no TF base->tcp, skip:", e); continue
        q = tf.transform.rotation; tr = tf.transform.translation
        Rb = quat_to_R(q.x, q.y, q.z, q.w)
        tb = np.array([[tr.x], [tr.y], [tr.z]])
        Rc, _ = cv2.Rodrigues(rvec)
        R_g2b.append(Rb); t_g2b.append(tb)
        R_t2c.append(Rc); t_t2c.append(tvec.reshape(3, 1))
        print(f"  OK  corners={ncor}  board_dist={float(np.linalg.norm(tvec)):.3f} m  TCP=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f})")

    print(f"\nCollected {len(R_g2b)} valid samples.")
    if len(R_g2b) < 4:
        print("Not enough samples (need >=4, ideally >=8). Aborting solve.")
        return

    np.savez(os.path.expanduser("~/handeye_samples.npz"),
             R_g2b=np.array(R_g2b), t_g2b=np.array(t_g2b),
             R_t2c=np.array(R_t2c), t_t2c=np.array(t_t2c))

    results = {}
    for name, method in [("TSAI", cv2.CALIB_HAND_EYE_TSAI),
                         ("PARK", cv2.CALIB_HAND_EYE_PARK),
                         ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD),
                         ("ANDREFF", cv2.CALIB_HAND_EYE_ANDREFF),
                         ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS)]:
        try:
            R_c2g, t_c2g = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=method)
        except cv2.error as e:
            print(f"\n=== {name}: solver failed ({str(e).splitlines()[-1][:60]}) ===")
            continue
        rpy = R_to_rpy(R_c2g)
        results[name] = (R_c2g, t_c2g.reshape(3), rpy)
        print(f"\n=== {name}: TCP->camera ===")
        print(f"  xyz  = [{t_c2g[0,0]:+.4f}, {t_c2g[1,0]:+.4f}, {t_c2g[2,0]:+.4f}] m")
        print(f"  rpy  = [{rpy[0]:+.4f}, {rpy[1]:+.4f}, {rpy[2]:+.4f}] rad")

    # Convergence check: spread of the translation across methods.
    ts = np.array([results[k][1] for k in results])
    spread = float(np.linalg.norm(ts.std(axis=0)))
    med = np.median(ts, axis=0)
    # pick the method closest to the median translation as the representative.
    best = min(results, key=lambda k: np.linalg.norm(results[k][1] - med))
    Rb, tb, rpyb = results[best]
    converged = spread < 0.01  # <1 cm spread => trustworthy
    print(f"\ntranslation spread across methods = {spread*1000:.1f} mm "
          f"({'CONVERGED' if converged else 'NOT converged - needs more pose diversity'})")
    print(f"representative method = {best}")

    # Quaternion (xyzw) from Rb
    tr = Rb[0, 0] + Rb[1, 1] + Rb[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S; qx = (Rb[2, 1] - Rb[1, 2]) / S
        qy = (Rb[0, 2] - Rb[2, 0]) / S; qz = (Rb[1, 0] - Rb[0, 1]) / S
    else:
        i = int(np.argmax([Rb[0, 0], Rb[1, 1], Rb[2, 2]]))
        if i == 0:
            S = math.sqrt(1.0 + Rb[0, 0] - Rb[1, 1] - Rb[2, 2]) * 2
            qw = (Rb[2, 1] - Rb[1, 2]) / S; qx = 0.25 * S
            qy = (Rb[0, 1] + Rb[1, 0]) / S; qz = (Rb[0, 2] + Rb[2, 0]) / S
        elif i == 1:
            S = math.sqrt(1.0 + Rb[1, 1] - Rb[0, 0] - Rb[2, 2]) * 2
            qw = (Rb[0, 2] - Rb[2, 0]) / S; qx = (Rb[0, 1] + Rb[1, 0]) / S
            qy = 0.25 * S; qz = (Rb[1, 2] + Rb[2, 1]) / S
        else:
            S = math.sqrt(1.0 + Rb[2, 2] - Rb[0, 0] - Rb[1, 1]) * 2
            qw = (Rb[1, 0] - Rb[0, 1]) / S; qx = (Rb[0, 2] + Rb[2, 0]) / S
            qy = (Rb[1, 2] + Rb[2, 1]) / S; qz = 0.25 * S

    out = os.path.join(os.path.dirname(__file__), "..", "config", "handeye_calibration.yaml")
    out = os.path.abspath(out)
    with open(out, "w") as f:
        f.write("# Eye-in-hand calibration: PiPER TCP (piper_tcp) -> wrist D405 camera optical frame.\n")
        f.write("# Produced by scripts/handeye_calibrate.py (cv2.calibrateHandEye).\n")
        f.write(f"# samples: {len(R_g2b)}  board: ChArUco {SQUARES_X}x{SQUARES_Y} "
                f"square={SQUARE_LEN} marker={MARKER_LEN} {('DICT_4X4_50')}\n")
        f.write(f"# translation spread across solvers: {spread*1000:.1f} mm "
                f"({'converged' if converged else 'NOT CONVERGED - review before use'})\n")
        f.write("handeye_tcp_to_camera:\n")
        f.write(f"  method: {best}\n")
        f.write(f"  converged: {str(converged).lower()}\n")
        f.write(f"  spread_mm: {spread*1000:.2f}\n")
        f.write(f"  samples: {len(R_g2b)}\n")
        f.write(f"  translation_xyz_m: [{tb[0]:.5f}, {tb[1]:.5f}, {tb[2]:.5f}]\n")
        f.write(f"  rotation_rpy_rad: [{rpyb[0]:.5f}, {rpyb[1]:.5f}, {rpyb[2]:.5f}]\n")
        f.write(f"  rotation_quat_xyzw: [{qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}]\n")
        f.write("  all_methods:\n")
        for k in results:
            R2, t2, r2 = results[k]
            f.write(f"    {k}: {{xyz: [{t2[0]:.5f}, {t2[1]:.5f}, {t2[2]:.5f}], "
                    f"rpy: [{r2[0]:.5f}, {r2[1]:.5f}, {r2[2]:.5f}]}}\n")
    print(f"\nwrote {out}")

    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
