#!/usr/bin/env python3
"""Look-at ORBIT hand-eye calibration for the PiPER wrist D405 + ChArUco board.

Prior wrist-spin sweeps failed: rotations were dominantly about the optical axis
(roll) because spinning the wrist swung the board out of frame -> ill-conditioned
hand-eye (needs >=30deg about >=2 non-parallel axes). This orbits the camera
AROUND the fixed board (azimuth+elevation+distance+roll), each pose pointing the
optical axis AT the board center (look-at), so the board stays in view while the
viewing angle changes a lot. Poses are planned with MoveIt /compute_ik
(collision-checked) using the APPROXIMATE URDF tcp->camera (planning only -- the
calibration measures the TRUE transform from actual board+TCP poses).

Board pose per view: depth-Kabsch (no PnP ambiguity), reused from
handeye_calibrate. Recipe per Codex: dist {0.35,0.43,0.50}, az {-40..+40},
el {25,40,55}, roll {-25,0,25}.

Run (bringup + D405 + move_group up, board fixed in view at the start pose):
  PYTHONNOUSERSITE=1 python3 scripts/handeye_orbit_calibrate.py
"""

import math
import os
import sys
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
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(__file__))
import handeye_calibrate as H  # noqa: E402  (board_pose_depth, make_board, R_to_rpy, etc.)

GROUPS = ["piper_arm", "arm"]
IK_LINK = "piper_tcp"
JOINTS = [f"piper_joint{i}" for i in range(1, 7)]
BOARD_CENTER_OBJ = np.array([H.SQUARES_X * H.SQUARE_LEN / 2.0,
                             H.SQUARES_Y * H.SQUARE_LEN / 2.0, 0.0])
MOVE_SEC = 6
SETTLE = 1.5


def quat_from_R(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / S, (R[0, 2] - R[2, 0]) / S,
                (R[1, 0] - R[0, 1]) / S, 0.25 * S)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        S = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * S, (R[0, 1] + R[1, 0]) / S, (R[0, 2] + R[2, 0]) / S, (R[2, 1] - R[1, 2]) / S)
    if i == 1:
        S = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / S, 0.25 * S, (R[1, 2] + R[2, 1]) / S, (R[0, 2] - R[2, 0]) / S)
    S = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / S, (R[1, 2] + R[2, 1]) / S, 0.25 * S, (R[1, 0] - R[0, 1]) / S)


def look_at_R(cam_pos, target, roll):
    """Optical-frame rotation (base): +Z toward target, +X right, +Y down, + roll."""
    f = target - cam_pos
    f = f / np.linalg.norm(f)
    up = np.array([0.0, 0.0, 1.0])
    x = np.cross(up, f)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0.0, 0.0])
    x = x / np.linalg.norm(x)
    y = np.cross(f, x); y = y / np.linalg.norm(y)
    R = np.column_stack([x, y, f])
    cr, sr = math.cos(roll), math.sin(roll)
    Rz = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1.0]])
    return R @ Rz


def main():
    rclpy.init()
    n = Node("handeye_orbit")
    bridge = cv_bridge.CvBridge()
    st = {"img": None, "depth": None, "K": None, "js": None}
    n.create_subscription(Image, H.RGB_TOPIC, lambda m: st.update(img=m), 10)
    n.create_subscription(Image, H.DEPTH_TOPIC, lambda m: st.update(depth=m), 10)
    n.create_subscription(CameraInfo, H.INFO_TOPIC,
                          lambda m: st.update(K=np.array(m.k).reshape(3, 3)), 10)
    from sensor_msgs.msg import JointState
    n.create_subscription(JointState, "/joint_states", lambda m: st.update(js=m), 10)
    n.create_subscription(JointState, "/piper/joint_states", lambda m: st.update(js=m), 10)
    buf = Buffer(); TransformListener(buf, n)
    ac = ActionClient(n, FollowJointTrajectory, H.ARM_ACTION); ac.wait_for_server(timeout_sec=10)
    ik = n.create_client(GetPositionIK, "/compute_ik"); ik.wait_for_service(timeout_sec=10)
    d, board = H.make_board()

    def spin(sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(n, timeout_sec=0.05)

    t = time.time()
    while (st["K"] is None or st["img"] is None or st["depth"] is None) and time.time() - t < 8:
        rclpy.spin_once(n, timeout_sec=0.2)
    if st["K"] is None:
        print("NO camera_info"); return

    def tf(a, b):
        t0 = time.time()
        while time.time() - t0 < 4:
            rclpy.spin_once(n, timeout_sec=0.2)
            try:
                return buf.lookup_transform(a, b, rclpy.time.Time())
            except Exception:
                pass
        return None

    def T_from_tf(tfm):
        q = tfm.transform.rotation; tr = tfm.transform.translation
        R = H.quat_to_R(q.x, q.y, q.z, q.w)
        return R, np.array([tr.x, tr.y, tr.z])

    # tcp->camera from URDF (planning only)
    tcp_cam = tf("piper_tcp", "piper_camera_optical_frame")
    R_tc, t_tc = T_from_tf(tcp_cam)

    # board center in base from a depth-Kabsch measurement at the current pose.
    cv = bridge.imgmsg_to_cv2(st["img"], "bgr8"); gray = cv2.cvtColor(cv, cv2.COLOR_BGR2GRAY)
    dep = bridge.imgmsg_to_cv2(st["depth"], "passthrough")
    res = H.board_pose_depth(gray, dep, d, board, st["K"])
    if res is None:
        print("board not detected at start pose; aim the camera at the board first"); return
    Rcb, tcb, ninl, rms = res  # board->camera
    center_cam = Rcb @ BOARD_CENTER_OBJ + tcb.reshape(3)
    bc = tf("piper_base_link", "piper_camera_optical_frame")
    R_bc, t_bc = T_from_tf(bc)
    board_center = R_bc @ center_cam + t_bc
    print(f"board center (base) = {board_center.round(3)}  (rms {rms*1000:.1f}mm)")

    # nominal camera bearing from the board (keep camera on the robot side).
    off = (R_bc @ np.zeros(3) + t_bc) - board_center  # current cam pos - center
    az0 = math.atan2(off[1], off[0])

    # candidate poses (Codex recipe)
    dists = [0.35, 0.43, 0.50]
    azs = [math.radians(a) for a in (-40, -25, 0, 25, 40)]
    els = [math.radians(e) for e in (25, 40, 55)]
    rolls = [math.radians(r) for r in (-25, 0, 25)]
    cand = []
    k = 0
    for dist in dists:
        for da in azs:
            for el in els:
                az = az0 + da
                cam_pos = board_center + dist * np.array(
                    [math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
                R_cam = look_at_R(cam_pos, board_center, rolls[k % 3])
                k += 1
                # target base->tcp = base->cam * inv(tcp->cam)
                R_bt = R_cam @ R_tc.T
                t_bt = cam_pos - R_bt @ t_tc
                cand.append((R_bt, t_bt))
    print(f"{len(cand)} candidate poses; solving IK...")

    # IK each, keep reachable+collision-free
    seed = st["js"]
    sols = []
    group = None
    for R_bt, t_bt in cand:
        req = GetPositionIK.Request()
        req.ik_request.group_name = group or GROUPS[0]
        req.ik_request.ik_link_name = IK_LINK
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout = Duration(sec=1)
        if seed is not None:
            req.ik_request.robot_state.joint_state = seed
        ps = PoseStamped(); ps.header.frame_id = "piper_base_link"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = [float(v) for v in t_bt]
        qx, qy, qz, qw = quat_from_R(R_bt)
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = qx, qy, qz, qw
        req.ik_request.pose_stamped = ps
        fut = ik.call_async(req); rclpy.spin_until_future_complete(n, fut, timeout_sec=3)
        r = fut.result()
        if r is None:
            continue
        if r.error_code.val != 1 and group is None:  # try the other group name once
            for g in GROUPS[1:]:
                req.ik_request.group_name = g
                fut = ik.call_async(req); rclpy.spin_until_future_complete(n, fut, timeout_sec=3)
                r = fut.result()
                if r and r.error_code.val == 1:
                    group = g; break
        if r and r.error_code.val == 1:
            if group is None:
                group = req.ik_request.group_name
            name2pos = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
            sols.append([round(name2pos.get(j, 0.0), 4) for j in JOINTS])
    print(f"{len(sols)} reachable+collision-free poses (group={group})")
    if len(sols) < 6:
        print("too few reachable poses; tilt/move the board or relax ranges"); return
    # Order poses nearest-neighbour from the current config to minimise the
    # joint-space sweep between consecutive moves (smaller, safer paths).
    cur = [dict(zip(seed.name, seed.position)).get(j, 0.0) for j in JOINTS] if seed else [0.0] * 6
    ordered = []
    remaining = list(sols)
    ref = np.array(cur)
    while remaining:
        nxt = min(remaining, key=lambda p: float(np.linalg.norm(np.array(p) - ref)))
        ordered.append(nxt); remaining.remove(nxt); ref = np.array(nxt)
    sols = ordered
    if os.getenv("DRY_RUN"):
        for s in sols:
            print("  pose:", s)
        print("DRY_RUN: not executing."); n.destroy_node(); rclpy.shutdown(); return

    # capture loop
    def goto(pos):
        g = FollowJointTrajectory.Goal(); jt = JointTrajectory(); jt.joint_names = JOINTS
        p = JointTrajectoryPoint(); p.positions = [float(v) for v in pos]; p.velocities = [0.0] * 6
        p.time_from_start = Duration(sec=MOVE_SEC); jt.points = [p]; g.trajectory = jt
        fut = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, fut, timeout_sec=8)
        spin(MOVE_SEC + SETTLE)

    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    for i, pos in enumerate(sols):
        print(f"[{i+1}/{len(sols)}] -> {pos}")
        goto(pos)
        st["img"] = None; st["depth"] = None
        t0 = time.time()
        while (st["img"] is None or st["depth"] is None) and time.time() - t0 < 4:
            rclpy.spin_once(n, timeout_sec=0.2)
        if st["img"] is None or st["depth"] is None:
            print("  no rgb/depth, skip"); continue
        cv = bridge.imgmsg_to_cv2(st["img"], "bgr8"); gray = cv2.cvtColor(cv, cv2.COLOR_BGR2GRAY)
        dep = bridge.imgmsg_to_cv2(st["depth"], "passthrough")
        rr = H.board_pose_depth(gray, dep, d, board, st["K"])
        if rr is None or rr[3] > H.MAX_FIT_RMS:
            print("  board rejected, skip"); continue
        Rc, tc, ninl, rms = rr
        tfm = tf("piper_base_link", "piper_tcp")
        if tfm is None:
            print("  no TF, skip"); continue
        Rb, tb = T_from_tf(tfm)
        R_g2b.append(Rb); t_g2b.append(tb.reshape(3, 1))
        R_t2c.append(Rc); t_t2c.append(tc.reshape(3, 1))
        print(f"  OK inliers={ninl} rms={rms*1000:.1f}mm dist={float(np.linalg.norm(tc)):.3f}")

    print(f"\nCollected {len(R_g2b)} samples.")
    if len(R_g2b) < 6:
        print("not enough; aborting solve"); return
    np.savez(os.path.expanduser("~/handeye_orbit_samples.npz"),
             R_g2b=np.array(R_g2b), t_g2b=np.array(t_g2b),
             R_t2c=np.array(R_t2c), t_t2c=np.array(t_t2c))
    # diversity report: pairwise relative rotation angles
    angs = []
    for i in range(len(R_g2b)):
        for j in range(i + 1, len(R_g2b)):
            Rrel = R_g2b[i].T @ R_g2b[j]
            ang = math.degrees(math.acos(max(-1, min(1, (np.trace(Rrel) - 1) / 2))))
            angs.append(ang)
    angs = np.array(angs)
    print(f"relative TCP rotations: median={np.median(angs):.0f}deg max={angs.max():.0f}deg "
          f">30deg pairs={(angs>30).sum()}/{len(angs)}")

    res = {}
    for name, m in [("TSAI", cv2.CALIB_HAND_EYE_TSAI), ("PARK", cv2.CALIB_HAND_EYE_PARK),
                    ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD), ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS)]:
        try:
            R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=m)
        except cv2.error:
            continue
        res[name] = (R, t.reshape(3), H.R_to_rpy(R))
        print(f"{name}: xyz=[{t[0,0]:+.4f},{t[1,0]:+.4f},{t[2,0]:+.4f}] |t|={np.linalg.norm(t):.3f} "
              f"rpy=[{res[name][2][0]:+.3f},{res[name][2][1]:+.3f},{res[name][2][2]:+.3f}]")
    ts = np.array([res[k][1] for k in res]); spread = float(np.linalg.norm(ts.std(0)))
    med = np.median(ts, 0); best = min(res, key=lambda k: np.linalg.norm(res[k][1] - med))
    conv = spread < 0.01
    Rb_, tb_, rpyb = res[best]; qx, qy, qz, qw = quat_from_R(Rb_)
    print(f"spread={spread*1000:.1f}mm best={best} converged={conv}")
    out = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "handeye_calibration.yaml"))
    with open(out, "w") as f:
        f.write("# Eye-in-hand (orbit): piper_tcp -> wrist D405 camera optical frame.\n")
        f.write(f"# scripts/handeye_orbit_calibrate.py  samples={len(R_g2b)} "
                f"spread={spread*1000:.1f}mm rot_median={np.median(angs):.0f}deg\n")
        f.write(f"# STATUS: {'converged' if conv else 'NOT CONVERGED - review'}\n")
        f.write("handeye_tcp_to_camera:\n")
        f.write(f"  method: {best}\n  converged: {str(conv).lower()}\n  spread_mm: {spread*1000:.2f}\n")
        f.write(f"  samples: {len(R_g2b)}\n")
        f.write(f"  translation_xyz_m: [{tb_[0]:.5f}, {tb_[1]:.5f}, {tb_[2]:.5f}]\n")
        f.write(f"  rotation_rpy_rad: [{rpyb[0]:.5f}, {rpyb[1]:.5f}, {rpyb[2]:.5f}]\n")
        f.write(f"  rotation_quat_xyzw: [{qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}]\n")
        f.write("  all_methods:\n")
        for kk in res:
            R2, t2, r2 = res[kk]
            f.write(f"    {kk}: {{xyz: [{t2[0]:.5f}, {t2[1]:.5f}, {t2[2]:.5f}], "
                    f"rpy: [{r2[0]:.5f}, {r2[1]:.5f}, {r2[2]:.5f}]}}\n")
    print("wrote", out)
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
