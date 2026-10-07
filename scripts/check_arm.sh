#!/usr/bin/env bash
# Pre-flight check: is the arm actually able to move?
#
# Why this exists: the joint_trajectory_controller reports
#   "Goal successfully reached!"  /  status: SUCCEEDED
# even when the shoulder_pan and elbow servos never move. The only reliable
# test is the controller's own standing error: /mirte_master_arm_controller/
# controller_state publishes reference (commanded) vs feedback (measured), and
# a persistent error.positions of ~1-2 rad means a servo has cut torque.
#
# On Mirte Master this happens after the arm holds the extended CARRY pose
# under load for a while. A power cycle of the arm clears it.
#
# Run this BEFORE every demo run. If it says STUCK, the tuner cannot tuck the
# arm, so the costmap stays at the (large) carry footprint and the robot will
# fail to fit through the narrow corridor.
#
# Usage:  ./scripts/check_arm.sh          # report only
#         ./scripts/check_arm.sh --move   # also command tucked, then re-check
set -uo pipefail

TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT

if [[ "${1:-}" == "--move" ]]; then
  echo "Commanding TUCKED (home) pose ..."
  ros2 topic pub --once /mirte_master_arm_controller/joint_trajectory \
    trajectory_msgs/msg/JointTrajectory \
    "{joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_joint],
      points: [{positions: [0.0, 0.0, 0.5, 0.0], time_from_start: {sec: 4}}]}" >/dev/null 2>&1
  sleep 6
fi

# Retry: `ros2 topic echo` can emit "A message was lost!!!" instead of the
# message under load (the bag recorder subscribes to these topics too), which
# would otherwise read as an unparseable controller_state and look like a fault.
for _try in 1 2 3; do
  ros2 topic echo /mirte_master_arm_controller/controller_state --once \
    >"$TMP" 2>/dev/null
  grep -q "positions:" "$TMP" && break
done

if [[ ! -s "$TMP" ]]; then
  echo "FAIL: no /mirte_master_arm_controller/controller_state."
  echo "      Is the robot up?   ros2 control list_controllers"
  exit 2
fi

python3 - "$TMP" <<'PY'
import sys, re

text = open(sys.argv[1]).read()

def positions(section):
    m = re.search(section + r":\n(?:.*\n)*?  positions:\n((?:  - .*\n)+)", text)
    if not m:
        return None
    out = []
    for v in re.findall(r"  - (\S+)", m.group(1)):
        try:
            out.append(float(v))
        except ValueError:
            out.append(float("nan"))
    return out

names = re.findall(r"- (\w+_joint)", text)[:4]
ref = positions("reference")
fb = positions("feedback")
err = positions("error")

if not err:
    print("FAIL: could not parse controller_state")
    sys.exit(2)

hdr = "{:<22}{:>11}{:>11}{:>9}".format("joint", "commanded", "measured", "error")
print(hdr)
print("-" * len(hdr))

nan = float("nan")
stuck, unreadable = [], []
for i, e in enumerate(err):
    name = names[i] if i < len(names) else "joint%d" % i
    r = ref[i] if ref and i < len(ref) else nan
    f = fb[i] if fb and i < len(fb) else nan
    flag = ""
    # A NaN error means the driver never read that servo. abs(nan) > 0.25 is
    # False, so without this branch a NaN joint silently passed as healthy and
    # the verdict blamed whichever joint happened to have a real error.
    if e != e or f != f:
        flag = "  <-- UNREADABLE"
        unreadable.append(name)
    elif abs(e) > 0.25:
        flag = "  <-- STUCK"
        stuck.append(name)
    print("{:<22}{:>11.3f}{:>11.3f}{:>9.3f}{}".format(name, r, f, e, flag))

print()
if unreadable:
    print("VERDICT: HARDWARE FAULT -- the driver cannot READ these joints:")
    print("         " + ", ".join(unreadable))
    print("         They report NaN, not a position, so nothing can be verified")
    print("         about the arm and the costmap is held at the CARRY footprint.")
    print("         On the robot:  ros2 control list_hardware_components -v")
    print("         Then POWER-CYCLE -- remove power. A servo in overload")
    print("         shutdown does not clear on a software restart.")
    if stuck:
        print("         (also not following commands: " + ", ".join(stuck) + ")")
    sys.exit(2)
if stuck:
    print("VERDICT: ARM STUCK -- these joints are not following commands:")
    print("         " + ", ".join(stuck))
    print("         POWER-CYCLE THE ARM, then re-run this check.")
    print("         Until it is fixed the tuner cannot tuck, so the costmap stays")
    print("         at the large CARRY footprint and the robot will not fit the")
    print("         narrow corridor.")
    sys.exit(1)

print("VERDICT: arm OK (all joints tracking within 0.25 rad).")
PY
