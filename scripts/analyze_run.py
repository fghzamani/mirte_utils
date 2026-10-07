"""Compare commanded /cmd_vel against measured /odom in a recorded run.

The question the logs cannot answer is whether the robot did what it was told.
This reads a run bag and prints commanded vs achieved velocity, the combined
speed |v| = hypot(vx, vy) (which the envelope's forward-only speed ceiling does
not see), the odometry path, and the AMCL pose track.

What to look for:
  * |v|cmd far above |v|odom      -> MPPI is planning motion the base cannot
                                     produce, so no plan is ever followed
  * vy comparable to vx           -> the robot is crabbing diagonally; the
                                     combined speed is what must stop in time
  * wz_odom opposite to wz_cmd,
    with |v|odom collapsing       -> an impact

Usage:
  docker exec ros2-tiago-mirte-cpu bash -lc \
    'source /opt/ros/humble/setup.bash && cd /home/forough/phd_projects/mirte_demo && \
     python3 scripts/analyze_run.py logs/run_<stamp>/bag'
"""
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
so = rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3')
# The bag is zstd-compressed (--compression-mode file), so the plain
# SequentialReader fails with "file is not a database".
try:
    r = rosbag2_py.SequentialCompressionReader()
    r.open(so, rosbag2_py.ConverterOptions('', ''))
except Exception:
    r = rosbag2_py.SequentialReader()
    r.open(so, rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}

cmd, odom, amcl = [], [], []
while r.has_next():
    topic, data, t = r.read_next()
    ts = t * 1e-9
    if topic == '/cmd_vel':
        m = deserialize_message(data, get_message(types[topic]))
        cmd.append((ts, m.linear.x, m.linear.y, m.angular.z))
    elif topic == '/odom':
        m = deserialize_message(data, get_message(types[topic]))
        p, tw = m.pose.pose, m.twist.twist
        odom.append((ts, p.position.x, p.position.y, tw.linear.x, tw.linear.y, tw.angular.z))
    elif topic == '/amcl_pose':
        m = deserialize_message(data, get_message(types[topic]))
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        amcl.append((ts, p.x, p.y, yaw))

t0 = min(c[0] for c in cmd) if cmd else 0.0
print("=== /cmd_vel: non-zero commands ===")
print(f"{'t':>7}{'vx_cmd':>9}{'vy_cmd':>9}{'wz_cmd':>9}")
nz = [c for c in cmd if abs(c[1]) > 1e-3 or abs(c[2]) > 1e-3 or abs(c[3]) > 1e-3]
for c in nz[:28]:
    print(f"{c[0]-t0:>7.2f}{c[1]:>9.3f}{c[2]:>9.3f}{c[3]:>9.3f}")
print(f"(total non-zero: {len(nz)} of {len(cmd)})")

print("\n=== commanded vs measured (odom sampled at each cmd) ===")
print(f"{'t':>7}{'vx_cmd':>9}{'vx_odom':>9}{'vy_cmd':>9}{'vy_odom':>9}{'wz_cmd':>9}{'wz_odom':>9}")
for c in nz[:28]:
    o = min(odom, key=lambda o: abs(o[0] - c[0])) if odom else None
    if o:
        print(f"{c[0]-t0:>7.2f}{c[1]:>9.3f}{o[3]:>9.3f}{c[2]:>9.3f}{o[4]:>9.3f}{c[3]:>9.3f}{o[5]:>9.3f}")

print("\n=== combined commanded speed vs envelope assumption ===")
import math
print(f"{'t':>7}{'|v|cmd':>9}{'|v|odom':>10}{'vx_max':>9}{'vy_max':>9}")
for c in nz[::4][:14]:
    o = min(odom, key=lambda o: abs(o[0] - c[0]))
    print(f"{c[0]-t0:>7.2f}{math.hypot(c[1],c[2]):>9.3f}{math.hypot(o[3],o[4]):>10.3f}{0.45:>9.2f}{0.5:>9.2f}")
mx = max(nz, key=lambda c: math.hypot(c[1], c[2]))
print(f"peak commanded |v| = {math.hypot(mx[1],mx[2]):.3f} m/s "
      f"(vx={mx[1]:.3f} vy={mx[2]:.3f}) at t={mx[0]-t0:.2f}")

print("\n=== odom during the motion window ===")
mv = [o for o in odom if o[0] >= t0]
if mv:
    print(f"{'t':>7}{'odom_x':>9}{'odom_y':>9}{'|v|':>8}")
    step = max(1, len(mv)//16)
    for o in mv[::step]:
        print(f"{o[0]-t0:>7.2f}{o[1]:>9.3f}{o[2]:>9.3f}{math.hypot(o[3],o[4]):>8.3f}")
    print(f"net displacement during motion: dx={mv[-1][1]-mv[0][1]:+.3f} dy={mv[-1][2]-mv[0][2]:+.3f}")

print("\n=== amcl_pose ===")
for a in amcl[::max(1, len(amcl)//12)] if amcl else []:
    print(f"t={a[0]-t0:>7.2f}  x={a[1]:>7.3f} y={a[2]:>7.3f} yaw={math.degrees(a[3]):>7.1f}deg")
