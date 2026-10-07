#!/usr/bin/env bash
# Command the arm to a named pose and VERIFY it arrived, with retries.
#
# Needed because the joint_trajectory_controller reports "Goal successfully
# reached!" / SUCCEEDED whether or not the servos moved, so a single
# `ros2 topic pub --once` at launch is fire-and-forget: if shoulder_pan stalls
# (it does, under the load of the extended carry pose) the demo silently starts
# from the wrong initial condition.
#
# Verification compares /joint_states against the target within TOL rad. Only
# shoulder_pan and elbow actually distinguish the two poses -- shoulder_lift and
# wrist sit near 0 in both -- so all four are checked.
#
# Usage:  ./scripts/set_arm_pose.sh carry [retries]
#         ./scripts/set_arm_pose.sh tucked
# Exit:   0 = pose reached, 1 = not reached, 2 = no robot
set -uo pipefail

POSE="${1:-carry}"
RETRIES="${2:-4}"
TOL=0.15
JOINTS="[shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_joint]"

case "$POSE" in
  carry)  TARGET="[1.15, -0.030, -1.27, 0.0]" ;;   # achievable pose, see ARM_CONFIGS
  tucked|home) TARGET="[0.0, 0.0, 0.5, 0.0]" ;;
  *) echo "unknown pose '$POSE' (expected: carry | tucked)"; exit 2 ;;
esac

echo "Arm -> '$POSE'  target=$TARGET"

for ((i=1; i<=RETRIES; i++)); do
  ros2 topic pub --once /mirte_master_arm_controller/joint_trajectory \
    trajectory_msgs/msg/JointTrajectory \
    "{joint_names: $JOINTS, points: [{positions: $TARGET, time_from_start: {sec: 4}}]}" \
    >/dev/null 2>&1
  sleep 5

  # Read names and positions via --field: plain `ros2 topic echo` emits YAML
  # BLOCK style ("name:\n- joint_a\n- joint_b"), not inline lists, so parsing
  # the whole message with a bracket regex silently fails and every check then
  # looks like a miss. --field prints a Python literal that is safe to parse.
  # `ros2 topic echo` interleaves its own warnings with the data on stdout, e.g.
  #   A message was lost!!!
  # which appears under load (the bag recorder is now a second subscriber on
  # these topics). Taking `head -1` therefore captured the warning instead of
  # the value and ast.literal_eval died with "SyntaxError: invalid syntax".
  # Select the line that actually looks like data, and re-read a few times in
  # case the one message we asked for was the one that got dropped.
  NAMES=""; POSNS=""
  for _try in 1 2 3; do
    [[ -z "$NAMES" ]] && NAMES="$(ros2 topic echo /joint_states --once --field name \
        2>/dev/null | grep -m1 '^\[')"
    [[ -z "$POSNS" ]] && POSNS="$(ros2 topic echo /joint_states --once --field position \
        2>/dev/null | grep -m1 -E "^(array|\[)")"
    [[ -n "$NAMES" && -n "$POSNS" ]] && break
  done
  if [[ -z "$NAMES" || -z "$POSNS" ]]; then
    echo "  could not read /joint_states (is the robot up?)"
    exit 2
  fi

  RESULT="$(python3 - "$NAMES" "$POSNS" "$TARGET" "$TOL" <<'PYCHK'
import ast, re, sys
names = ast.literal_eval(sys.argv[1])
# Tokenise the bracketed list by commas rather than regex-matching numbers.
# A number regex does NOT match "nan", so NaN entries were silently DROPPED and
# every following value shifted left -- a reported "shoulder_lift = -1.917" was
# really the elbow's reading. Any joint reporting NaN mis-attributed every
# joint after it, which made the per-joint diagnosis actively misleading.
def _tok(txt):
    inner = txt[txt.index("[") + 1:txt.rindex("]")]
    vals = []
    for t in inner.split(","):
        try:
            vals.append(float(t.strip()))
        except ValueError:          # nan, .nan, inf, -inf
            vals.append(float("nan"))
    return vals
pos = _tok(sys.argv[2])[:len(names)]
target = ast.literal_eval(sys.argv[3])
tol = float(sys.argv[4])
want = dict(zip(["shoulder_pan_joint", "shoulder_lift_joint",
                 "elbow_joint", "wrist_joint"], target))
have = dict(zip(names, pos))
# A NaN position means the driver never managed to READ that servo. Retrying
# cannot help, and commanding a servo that is not answering just holds it
# straining -- which is how an overloaded servo goes into thermal shutdown.
nan_j = [j for j in want if j in have and have[j] != have[j]]
if nan_j:
    print("NAN " + ",".join(sorted(nan_j)))
    raise SystemExit(0)
bad, worst = [], 0.0
for j, t in want.items():
    if j not in have:
        bad.append("%s missing from /joint_states" % j)
        continue
    e = have[j] - t
    worst = max(worst, abs(e))
    if abs(e) > tol:
        bad.append("%s off by %+.3f" % (j, e))
print("OK" if not bad else "BAD %.4f %s" % (worst, "; ".join(bad)))
PYCHK
)"

  if [[ "$RESULT" == OK ]]; then
    echo "  reached '$POSE' (attempt $i)"
    exit 0
  fi

  if [[ "$RESULT" == NAN* ]]; then
    echo
    echo "!!! HARDWARE FAULT: these joints report NaN instead of a position:"
    echo "!!!   ${RESULT#NAN }"
    echo "!!! The driver cannot READ them, so commanding them is pointless and"
    echo "!!! keeps them straining. Not retrying."
    echo "!!! On the robot: ros2 control list_hardware_components -v"
    echo "!!! Then POWER-CYCLE (remove power -- a servo in overload shutdown"
    echo "!!! will not clear on a software restart)."
    exit 2
  fi

  # Stall detection. If the worst joint error is no better than last attempt,
  # the servo is not moving and further commands only hold it loaded against a
  # target it cannot reach -- the fast route to an overload shutdown. Stop.
  WORST=$(awk '{print $2}' <<<"$RESULT")
  echo "  attempt $i/$RETRIES: worst error ${WORST} rad -- ${RESULT#BAD * }"
  if [[ -n "${PREV_WORST:-}" ]]; then
    if awk "BEGIN{exit !($WORST > $PREV_WORST - 0.02)}"; then
      echo
      echo "!!! Arm is NOT MOVING (error ${PREV_WORST} -> ${WORST} rad)."
      echo "!!! Stopping rather than holding the servo loaded against a target it"
      echo "!!! cannot reach, which risks an overload shutdown."
      echo "!!!   ./scripts/check_arm.sh     # per-joint standing error"
      exit 1
    fi
  fi
  PREV_WORST="$WORST"
done

echo
echo "!!! Arm did NOT reach '$POSE' after $RETRIES attempts."
echo "!!! The controller reports success regardless, so this is a servo fault:"
echo "!!!   ./scripts/check_arm.sh     # shows the standing error per joint"
echo "!!! POWER-CYCLE THE ARM. The demo will start from the wrong pose until then."
exit 1
