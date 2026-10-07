#!/usr/bin/env bash
# Run the demo stack and capture the COMPLETE console output to a file.
#
# `ros2 launch` prints everything to the terminal, where it scrolls away and the
# interesting lines (tuner ticks, planner/controller errors, arm retries) get
# lost. This tees the whole stream to a timestamped file so a run can be debugged
# afterwards, while still showing it live.
#
# Usage:
#   ./scripts/run_demo.sh                       # full stack, log to logs/
#   ./scripts/run_demo.sh rviz:=false           # any launch args are passed through
#   ./scripts/run_demo.sh tuner:=false my_run   # a bare word becomes the log name
#   LOG_DIR=/tmp ./scripts/run_demo.sh
#
# Afterwards, the useful greps:
#   grep "TUNER TICK"            logs/<file>    # what the tuner decided each tick
#   grep -E "ERROR|WARN"         logs/<file>    # everything that went wrong
#   grep -E "Arm target|Sending arm"  logs/<file>   # arm switching
#   grep -E "Optimizer fail|Failed to make progress|lethal" logs/<file>
#   grep -vE "feedback for unknown goal" logs/<file> > logs/<file>.clean
#                                               # drop the bt_navigator spam
set -uo pipefail

LOG_DIR="${LOG_DIR:-$HOME/phd_projects/mirte_demo/logs}"
mkdir -p "$LOG_DIR"

NAME="demo"
ARGS=()
for a in "$@"; do
  if [[ "$a" == *":="* ]]; then ARGS+=("$a"); else NAME="$a"; fi
done

STAMP="$(date +%Y%m%dT%H%M%S)"
# One directory per run. demo_launch.py reads MIRTE_RUN_NAME and puts the bag,
# the per-node logs and the config snapshot in the same place, so the console
# stream tee'd below sits next to the data it describes.
RUN="${NAME}_${STAMP}"
RUN_DIR="$LOG_DIR/$RUN"
mkdir -p "$RUN_DIR"
export MIRTE_RUN_NAME="$RUN"
LOG="$RUN_DIR/console.log"

{
  echo "### run_demo.sh  $(date -Is)"
  echo "### launch args: ${ARGS[*]:-<none>}"
  echo "### demo_params footprint / inflation:"
  grep -E "^ *footprint:|inflation_radius:|robot_radius:|max_beams:|expected_planner_frequency:" \
    "$HOME/phd_projects/mirte_demo/src/mirte_navigation/params/demo_params.yaml" 2>/dev/null \
    | sed 's/^/###   /'
  echo "### ---------------------------------------------------------------"
} > "$LOG"

# Pre-flight: the FastDDS profile pins the UDP transport to one hardcoded
# address with <useBuiltinTransports>false</>. If that address is not currently
# held by any interface (robot off, or DHCP handed out a different one), the
# whitelist matches nothing, nodes start but never discover each other, and the
# nav stack dies mid-run with stale-transform errors that point nowhere near
# the real cause. Loopback is whitelisted too, so container-local traffic keeps
# working, but the robot link will not.
DDS_XML="${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
if [[ -n "$DDS_XML" && -r "$DDS_XML" ]]; then
  {
    echo "### --- DDS pre-flight ---"
    echo "### profile: $DDS_XML"
    HAVE="$(hostname -I 2>/dev/null)"
    echo "### interfaces: $HAVE"
    MISSING=0
    while read -r a; do
      [[ -z "$a" || "$a" == "127.0.0.1" ]] && continue
      if ! grep -qw "$a" <<<"$HAVE"; then
        echo "!!! whitelisted address $a is NOT held by any interface"
        MISSING=1
      fi
    done < <(grep -oE "<address>[^<]+</address>" "$DDS_XML" | sed -E "s|</?address>||g")
    if [[ "$MISSING" == "1" ]]; then
      echo "!!! The robot link will not work. Check the robot network, or update"
      echo "!!! the <interfaceWhiteList> in $DDS_XML to the current address."
    else
      echo "### all whitelisted addresses present"
    fi
    grep -q "<type>SHM</type>" "$DDS_XML" \
      && echo "### SHM transport: declared (node-to-node traffic stays off the network)" \
      || echo "!!! SHM transport NOT declared: intra-container traffic will be forced over UDP"
    echo "### ------------------------"
  } 2>&1 | tee -a "$LOG"
fi

# Pre-flight: a torque-disabled arm servo cannot be detected from the launch
# output (the trajectory controller reports SUCCEEDED regardless), and it makes
# the whole run fail 70s later because the costmap is stuck on the large carry
# footprint. Catch it here. Warn, do not block -- base-only runs are still useful.
if [[ -x "$(dirname "$0")/check_arm.sh" ]]; then
  echo "### --- arm pre-flight ---" | tee -a "$LOG"
  "$(dirname "$0")/check_arm.sh" 2>&1 | tee -a "$LOG" || {
    echo | tee -a "$LOG"
    echo "!!! Arm is STUCK: the tuner will not be able to tuck it." | tee -a "$LOG"
    echo "!!! Power-cycle the arm for the full demo. Continuing in 5s..." | tee -a "$LOG"
    sleep 5
  }
  echo "### -----------------------" | tee -a "$LOG"
fi

echo "Run directory: $RUN_DIR"
echo "  console.log  full console stream (this terminal)"
echo "  bag/         rosbag2 of the run"
echo "  node_logs/   one log file per node"
echo "  config/      params + map actually used"
echo "(Ctrl-C stops the launch; everything is kept.)"
echo

# stdbuf keeps the stream line-buffered so the log stays complete if we Ctrl-C.
#
# The console is filtered, the LOG FILE IS NOT. tf2 prints
#   Error: TF_NAN_INPUT: Ignoring transform for child_frame_id "shoulder_pan" ...
#   Error: TF_DENORMALIZED_QUATERNION: ... invalid quaternion (nan nan nan nan)
# directly to stderr from buffer_core.cpp, not through the ROS logger, so no
# log-level setting can quieten it. When an arm servo reports NaN every node with
# a TF listener repeats the pair for every bad frame, which is thousands of lines
# and buries the tuner output during a demo. They are harmless to navigation --
# "Ignoring transform" means tf2 drops them and the map->odom->base_link->laser
# chain is untouched -- but they are NOT harmless to the arm: see the
# TF_NAN_INPUT section in mirte_manual.md. Set NO_TF_FILTER=1 to see them live.
TF_NOISE='TF_NAN_INPUT|TF_DENORMALIZED_QUATERNION|buffer_core\.cpp'
if [[ -n "${NO_TF_FILTER:-}" ]]; then
  stdbuf -oL -eL ros2 launch mirte_navigation demo_launch.py "${ARGS[@]}" 2>&1 \
    | stdbuf -oL tee -a "$LOG"
else
  stdbuf -oL -eL ros2 launch mirte_navigation demo_launch.py "${ARGS[@]}" 2>&1 \
    | stdbuf -oL tee -a "$LOG" \
    | stdbuf -oL grep --line-buffered -vE "$TF_NOISE"
fi

echo
echo "Run saved: $RUN_DIR"
# Report what the console hid, so a silent hardware fault is not silent.
if [[ -z "${NO_TF_FILTER:-}" ]]; then
  HID=$(grep -cE "$TF_NOISE" "$LOG" 2>/dev/null || echo 0)
  if [[ "${HID:-0}" -gt 0 ]]; then
    echo
    echo "!!! $HID TF-NaN lines were hidden from the console (kept in console.log)."
    echo "!!! An arm servo is reporting NaN instead of a position. Navigation is"
    echo "!!! unaffected, but the arm state cannot be verified, so the costmap is"
    echo "!!! held at the CARRY footprint. Check and power-cycle the arm:"
    echo "!!!   ros2 topic echo /joint_states --once --field position"
  fi
fi
du -sh "$RUN_DIR" 2>/dev/null | awk '{print "  size: " $1}'
echo "Quick triage:"
echo "  grep 'TUNER TICK' '$LOG' | tail -20"
echo "  grep -E 'ERROR|Aborting|lethal|Optimizer fail|Failed to make progress' '$LOG' | tail -40"
echo "  ls '$RUN_DIR/node_logs'"
echo "  ros2 bag info '$RUN_DIR/bag'"
