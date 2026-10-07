#!/usr/bin/env python3
"""Find where the robot actually is by matching /scan against the map, and give
that pose to AMCL on /initialpose.

Why this exists: `set_initial_pose: true` with `initial_pose {0,0,0}` tells AMCL
the robot is at the map origin facing 0 deg. When it is not, AMCL starts with a
tight covariance around a wrong pose and `update_min_d: 0.05` means it only
corrects while moving -- so it never recovers before the planner gives up. The
failure is reported as a planner error and never as a pose error:

    planner_server: ... Starting point in lethal space!
    planner_server: GridBased: failed to create plan, no valid path found.

Placing the robot by hand to within a few degrees is not realistic; a 10 deg
heading error alone is ~0.5 m of lateral error over 3 m of travel, which is
enough to put the footprint inside a mapped wall.

Method: brute-force scan matching over the whole map. Candidate positions are
the mapped-free cells; for each candidate (x, y, yaw) the scan endpoints are
projected into the map and scored against a distance transform of the walls
(closer to a wall = better). Coarse pass, then refinement around the winner.

Usage:
  python3 scripts/set_initial_pose_from_scan.py            # find and publish
  python3 scripts/set_initial_pose_from_scan.py --dry-run  # find only
Then check:
  python3 scripts/check_localization.py
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

WS = "/home/forough/phd_projects/mirte_demo"
LASER_YAW = math.radians(90.0)   # composed base_link -> laser; see check_localization.py


def load_map():
    pr = yaml.safe_load(open(os.path.join(WS, "src/mirte_navigation/params/demo_params.yaml")))
    mp = pr["map_server"]["ros__parameters"]["yaml_filename"]
    if not os.path.isabs(mp):
        mp = os.path.join(WS, mp)
    my = yaml.safe_load(open(mp))
    img = os.path.join(os.path.dirname(mp), my["image"])
    with open(img, "rb") as f:
        assert f.readline().strip() == b"P5"
        l = f.readline()
        while l.startswith(b"#"):
            l = f.readline()
        w, h = map(int, l.split())
        f.readline()
        a = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    occ = (255 - a.astype(float)) / 255.0
    return (occ <= my["free_thresh"], occ >= my["occupied_thresh"],
            my["resolution"], my["origin"][0], my["origin"][1], w, h)


def get_scan(timeout=15.0):
    rclpy.init()
    n = Node("pose_from_scan")
    got = {}
    n.create_subscription(LaserScan, "/scan", lambda m: got.setdefault("s", m), 10)
    end = time.time() + timeout
    while time.time() < end and "s" not in got:
        rclpy.spin_once(n, timeout_sec=0.2)
    return n, got.get("s")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="report, do not publish")
    ap.add_argument("--stride", type=int, default=4, help="use every Nth scan beam")
    args = ap.parse_args()

    FREE, OCCU, res, ox, oy, w, h = load_map()
    if not OCCU.any():
        print("map has no occupied cells -- nothing to match against")
        return 2
    # distance (in metres) from every cell to the nearest wall
    dist = distance_transform_edt(~OCCU) * res

    node, scan = get_scan()
    if scan is None:
        print("no /scan received -- is the robot up?")
        rclpy.shutdown()
        return 2

    pts = []
    for i in range(0, len(scan.ranges), args.stride):
        rng = scan.ranges[i]
        if rng != rng or not (scan.range_min <= rng <= scan.range_max):
            continue
        if rng < 0.30 or rng > 6.0:        # 0.30 = the risk node's self-filter
            continue
        pts.append((rng, scan.angle_min + i * scan.angle_increment))
    if len(pts) < 30:
        print(f"only {len(pts)} usable beams -- cannot match")
        rclpy.shutdown()
        return 2
    rngs = np.array([p[0] for p in pts])
    angs = np.array([p[1] for p in pts])
    print(f"matching {len(pts)} beams against {int(OCCU.sum())} wall cells "
          f"over {int(FREE.sum())} free cells ...")

    def score_batch(xs, ys, yaw):
        """Mean wall-distance of all endpoints, for many origins at one yaw."""
        a = angs + yaw + LASER_YAW
        ex = rngs * np.cos(a)
        ey = rngs * np.sin(a)
        X = xs[:, None] + ex[None, :]
        Y = ys[:, None] + ey[None, :]
        C = ((X - ox) / res).astype(np.int32)
        R = (h - 1 - (Y - oy) / res).astype(np.int32)
        ok = (R >= 0) & (R < h) & (C >= 0) & (C < w)
        np.clip(R, 0, h - 1, out=R)
        np.clip(C, 0, w - 1, out=C)
        d = dist[R, C]
        d = np.where(ok, d, 3.0)           # off-map endpoints penalised
        return d.mean(axis=1)

    def search(cx, cy, half, step, yaws):
        gx = np.arange(cx - half, cx + half + 1e-9, step)
        gy = np.arange(cy - half, cy + half + 1e-9, step)
        XX, YY = np.meshgrid(gx, gy)
        xs, ys = XX.ravel(), YY.ravel()
        # only consider origins the robot could occupy
        C = ((xs - ox) / res).astype(int)
        R = (h - 1 - (ys - oy) / res).astype(int)
        inb = (R >= 0) & (R < h) & (C >= 0) & (C < w)
        keep = np.zeros_like(inb)
        keep[inb] = FREE[R[inb], C[inb]]
        xs, ys = xs[keep], ys[keep]
        if len(xs) == 0:
            return None
        best = None
        for yaw in yaws:
            sc = score_batch(xs, ys, yaw)
            k = int(np.argmin(sc))
            if best is None or sc[k] < best[0]:
                best = (float(sc[k]), float(xs[k]), float(ys[k]), float(yaw))
        return best

    # whole map, coarse
    cx = ox + w * res / 2.0
    cy = oy + h * res / 2.0
    half = max(w, h) * res / 2.0
    t0 = time.time()
    best = search(cx, cy, half, 0.20, np.radians(np.arange(0, 360, 10)))
    if best is None:
        print("no free cells to search")
        rclpy.shutdown()
        return 2
    # refine twice
    for hw, st, ys in ((0.30, 0.05, np.radians(np.arange(-12, 12.1, 2))),
                       (0.08, 0.02, np.radians(np.arange(-3, 3.1, 0.5)))):
        r = search(best[1], best[2], hw, st, best[3] + ys)
        if r and r[0] < best[0]:
            best = r
    err, bx, by, byaw = best
    byaw = (byaw + math.pi) % (2 * math.pi) - math.pi
    print(f"search took {time.time()-t0:.1f}s")
    print(f"best pose: x={bx:+.3f}  y={by:+.3f}  yaw={math.degrees(byaw):+.1f} deg")
    # Mirror to the ROS logger: print() goes to the launch console only, which is
    # not captured unless run via run_demo.sh -- so a failure here was invisible
    # in node_logs/ and in the bag's /rosout.
    node.get_logger().info(
        f"scan match: x={bx:+.3f} y={by:+.3f} yaw={math.degrees(byaw):+.1f}deg "
        f"(mean endpoint-to-wall {err:.3f} m)")
    print(f"mean endpoint distance to nearest wall: {err:.3f} m "
          f"({'good' if err < 0.12 else 'WEAK -- treat with suspicion'})")

    if args.dry_run:
        print("\n--dry-run: not published. Re-run without it to send /initialpose.")
        rclpy.shutdown()
        return 0
    if err > 0.25:
        print("\nMatch is too weak to trust; NOT publishing. Move the robot to a")
        print("more distinctive spot (a corner beats the middle of a corridor),")
        print("or re-map if the environment has changed.")
        rclpy.shutdown()
        return 1

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = bx
    msg.pose.pose.position.y = by
    msg.pose.pose.orientation.z = math.sin(byaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(byaw / 2.0)
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.05          # 22 cm 1-sigma in x,y
    cov[35] = 0.03                  # ~10 deg 1-sigma in yaw
    msg.pose.covariance = cov
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    # Wait for AMCL's subscription to be MATCHED before publishing. This process
    # is short-lived: creating the publisher and sending immediately means DDS
    # discovery has not completed, nobody is listening, and the pose is silently
    # dropped. That is exactly what happened when this ran from demo_launch.py --
    # the script completed "successfully" while /amcl_pose stayed at (0, 0, 0)
    # and every goal was refused.
    deadline = time.time() + 15.0
    while time.time() < deadline and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    subs = pub.get_subscription_count()
    if subs == 0:
        node.get_logger().error(
            "no subscriber on /initialpose after 15s -- is amcl up? pose NOT applied")
        print("no subscriber on /initialpose after 15s -- is amcl up? pose NOT applied")
        rclpy.shutdown()
        return 1
    node.get_logger().info(f"/initialpose has {subs} subscriber(s); publishing "
                           f"x={bx:.3f} y={by:.3f} yaw={math.degrees(byaw):.1f}deg")

    # Confirm AMCL actually adopted it, rather than assuming.
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    aq = QoSProfile(depth=5)
    aq.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    aq.reliability = QoSReliabilityPolicy.RELIABLE
    seen = {}
    node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                             lambda m: seen.__setitem__("p", m), aq)
    applied = False
    for attempt in range(6):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        end = time.time() + 1.5
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            p_ = seen.get("p")
            if p_ is not None:
                dx = p_.pose.pose.position.x - bx
                dy = p_.pose.pose.position.y - by
                if math.hypot(dx, dy) < 0.10:
                    applied = True
                    break
        if applied:
            break

    if applied:
        node.get_logger().info("AMCL adopted the pose.")
        print(f"\npublished and CONFIRMED: /amcl_pose is now at the matched pose.")
    else:
        node.get_logger().warn(
            "published to /initialpose but /amcl_pose did not confirm within ~9s; "
            "check localisation before sending a goal")
        print("\npublished, but /amcl_pose did NOT confirm. Check with:")
    print("  python3 scripts/check_localization.py")
    rclpy.shutdown()
    return 0 if applied else 1


if __name__ == "__main__":
    sys.exit(main())
