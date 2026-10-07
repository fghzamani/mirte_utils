#!/usr/bin/env python3
"""Is the robot actually where Nav2 thinks it is?

This is the check that was missing. A wrong initial pose never reports itself as
a localisation error -- it surfaces as a planner failure, because the obstacle
layer paints walls that contradict the static map:

    planner_server: GridBased: failed to create plan, no valid path found.
    planner_server: ... Starting point in lethal space!

Measured on the demo runs, the match score predicts the outcome exactly:

    2026-10-01  73% at the assumed pose  -> 2443 plans, navigated fine
    2026-10-02  34% at the assumed pose  -> 0 plans, every goal rejected
                (best match at yaw -10 deg: the robot was placed rotated)

Method: project each scan endpoint into the map at the assumed pose and count
how many land on a mapped wall (dilated by one cell). Then search nearby poses
for a better match; if a shifted or rotated pose scores much higher, the pose
estimate is wrong by that offset.

IMPORTANT: the scan must be rotated by the composed base_link -> laser
transform, which on Mirte is +90 deg (base_link -> frame_link +90, -> lidar_base
-90, -> laser +90). Omitting it makes a well-localised robot score ~5%.

Usage:
  # live, before sending a goal (needs only the robot, not nav2):
  python3 scripts/check_localization.py
  # against a recorded run:
  python3 scripts/check_localization.py --bag logs/run_<stamp>/bag

Reading the result:
  >70%  well localised, go ahead
  40-70% marginal
  <40%  the robot is not where Nav2 thinks. Rotate/move it to match, or give a
        2D Pose Estimate in RViz, and re-check before sending a goal.
"""

import math, os, sys
import numpy as np, yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

LASER_YAW = math.radians(90.0)      # composed base_link -> laser

import argparse
ap = argparse.ArgumentParser()
ap.add_argument("--bag", help="read the first scan and amcl_pose from a run bag "
                             "instead of the live topics")
ap.add_argument("--pose", nargs=3, type=float, metavar=("X", "Y", "YAW_DEG"),
                help="assumed pose to score against (default: amcl initial_pose "
                     "from demo_params.yaml, or the bag's first /amcl_pose)")
args = ap.parse_args()
bag = args.bag
ws = "/home/forough/phd_projects/mirte_demo"
pr = yaml.safe_load(open(os.path.join(ws, "src/mirte_navigation/params/demo_params.yaml")))
mp = pr["map_server"]["ros__parameters"]["yaml_filename"]
if not os.path.isabs(mp):
    mp = os.path.join(ws, mp)
my = yaml.safe_load(open(mp))
img = os.path.join(os.path.dirname(mp), my["image"])
res, ox, oy = my["resolution"], my["origin"][0], my["origin"][1]
ft, ot = my["free_thresh"], my["occupied_thresh"]
with open(img, "rb") as f:
    assert f.readline().strip() == b"P5"
    l = f.readline()
    while l.startswith(b"#"):
        l = f.readline()
    w, h = map(int, l.split()); f.readline()
    a = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
occ = (255 - a.astype(float)) / 255.0
OCCU = occ >= ot
# dilate walls by one cell so near-misses still count
D = np.zeros_like(OCCU)
for dr in (-1, 0, 1):
    for dc in (-1, 0, 1):
        D |= np.roll(np.roll(OCCU, dr, 0), dc, 1)

scan = None; pose = None
if bag:
    so = rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3")
    rd = rosbag2_py.SequentialCompressionReader()
    rd.open(so, rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    while rd.has_next() and (scan is None or pose is None):
        t, d, ts = rd.read_next()
        if t == "/scan" and scan is None:
            scan = deserialize_message(d, get_message(types[t]))
        elif t == "/amcl_pose" and pose is None:
            m = deserialize_message(d, get_message(types[t]))
            q = m.pose.pose.orientation
            pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                    math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
else:
    # Live: one scan straight off the robot. Works before nav2 is up, which is
    # the point -- check the placement BEFORE launching and sending a goal.
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseWithCovarianceStamped
    rclpy.init()
    n = Node("check_localization")
    got = {}
    n.create_subscription(LaserScan, "/scan", lambda m: got.setdefault("s", m), 10)
    # Score against AMCL's CURRENT belief when nav2 is up, not the configured
    # initial_pose -- otherwise this keeps reporting the old wrong pose after it
    # has been corrected (by RViz or set_initial_pose_from_scan.py).
    # AMCL publishes /amcl_pose only when it updates, and it updates only on
    # motion (update_min_d: 0.05). A stationary robot therefore sends nothing to
    # a late subscriber unless we ask for the latched last value.
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    _q = QoSProfile(depth=5)
    _q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    _q.reliability = QoSReliabilityPolicy.RELIABLE
    n.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                          lambda m: got.setdefault("p", m), _q)
    import time
    t_end = time.time() + 10.0
    while time.time() < t_end and "s" not in got:
        rclpy.spin_once(n, timeout_sec=0.2)
    t_end = time.time() + 3.0          # brief extra wait for a pose, if any
    while time.time() < t_end and "p" not in got:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_node(); rclpy.shutdown()
    if "s" not in got:
        print("no /scan received -- is the robot up?")
        sys.exit(2)
    scan = got["s"]
    if "p" in got:
        q = got["p"].pose.pose.orientation
        pose = (got["p"].pose.pose.position.x, got["p"].pose.pose.position.y,
                math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        print("scoring against live /amcl_pose")

if args.pose:
    pose = (args.pose[0], args.pose[1], math.radians(args.pose[2]))
elif pose is None:
    ip = pr["amcl"]["ros__parameters"].get("initial_pose", {}) or {}
    pose = (float(ip.get("x", 0.0)), float(ip.get("y", 0.0)), float(ip.get("yaw", 0.0)))
    print(f"assumed pose from demo_params initial_pose: "
          f"({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.1f} deg)")

pts = []
for i, rng in enumerate(scan.ranges):
    if rng != rng or not (scan.range_min <= rng <= scan.range_max):
        continue
    if rng < 0.30 or rng > 6.0:
        continue
    pts.append((rng, scan.angle_min + i * scan.angle_increment))

def hitfrac(px, py, yaw):
    hit = tot = 0
    for rng, a_ in pts:
        ang = a_ + yaw + LASER_YAW
        x = px + rng*math.cos(ang); y = py + rng*math.sin(ang)
        c = int((x-ox)/res); r_ = int(h-1-(y-oy)/res)
        if not (0 <= r_ < h and 0 <= c < w):
            continue
        tot += 1
        if D[r_, c]:
            hit += 1
    return (hit/tot if tot else 0.0), tot

base, tot = hitfrac(*pose)
print(f"scan beams used: {tot}")
print(f"assumed pose   : x={pose[0]:+.3f} y={pose[1]:+.3f} yaw={math.degrees(pose[2]):+.1f} deg"
      f"   -> match {base*100:.0f}%")
best = (base, 0.0, 0.0, 0.0)
for dyaw in np.arange(-180, 180, 5):
    for dx in np.arange(-1.0, 1.01, 0.25):
        for dy in np.arange(-1.0, 1.01, 0.25):
            f_, _ = hitfrac(pose[0]+dx, pose[1]+dy, pose[2]+math.radians(dyaw))
            if f_ > best[0]:
                best = (f_, dx, dy, dyaw)
print(f"best nearby pose: dx={best[1]:+.2f} dy={best[2]:+.2f} dyaw={best[3]:+.0f} deg"
      f"   -> match {best[0]*100:.0f}%")
# The search is local (+-1 m, 0.25 m steps). If the winner sits on the boundary
# the real offset is further out, so the reported numbers are a lower bound.
if abs(abs(best[1]) - 1.0) < 1e-6 or abs(abs(best[2]) - 1.0) < 1e-6:
    print("   NOTE: best offset is at the edge of the +-1 m search window, so the")
    print("         true error is larger than this. Treat it as 'badly placed'")
    print("         rather than as a precise correction.")
print()
if best[0] - base > 0.15:
    print(f"VERDICT: pose estimate is WRONG. Match improves {base*100:.0f}% -> "
          f"{best[0]*100:.0f}% at dx={best[1]:+.2f} dy={best[2]:+.2f} "
          f"dyaw={best[3]:+.0f} deg.")
    print("         Move/rotate the robot to match, or set a 2D Pose Estimate in")
    print("         RViz, then re-run this. Until then the planner will reject")
    print("         goals with 'no valid path found' or 'Starting point in")
    print("         lethal space', naming the planner and never the pose.")
    sys.exit(1)
elif base < 0.40:
    print(f"VERDICT: poor match ({base*100:.0f}%) and no better pose nearby -- the")
    print("         map may be stale, or the robot is somewhere not covered by it.")
    sys.exit(1)
print(f"VERDICT: localisation looks good ({base*100:.0f}% of scan endpoints on "
      f"mapped walls).")
