#!/usr/bin/env bash
# Keep the Mirte robot clock in sync with this laptop.
#
# WHY THIS MATTERS: the robot has no NTP and its clock drifts (observed: 29 min
# in one session, 1.8 days after a boot). Nav2 nodes run on the laptop clock but
# /scan and /odom carry the robot's timestamps, so any skew makes AMCL's
# map->odom transform look stale. Symptoms:
#   "[tf_help]: Transform data too old when converting from map to odom"
#   "[controller_server]: Reached the goal!"   <-- instantly, robot never moves
#   AMCL: "Failed to transform initial pose in time ... extrapolation"
#
# RUN THIS ON THE LAPTOP (not in the container): it needs ssh+sudo to the robot.
# Requires passwordless ssh (manual section 2.2) and passwordless sudo for date.
#
# Usage:
#   ./scripts/sync_robot_clock.sh            # sync once, then report the offset
#   ./scripts/sync_robot_clock.sh --watch    # resync every 60 s (run during the demo)
#   ./scripts/sync_robot_clock.sh --check    # report offset only, change nothing
#   INTERVAL=30 ./scripts/sync_robot_clock.sh --watch
set -uo pipefail

ROBOT="${ROBOT:-mirte@192.168.45.1}"
INTERVAL="${INTERVAL:-60}"
MODE="${1:-once}"

offset_ms() {
  # Send laptop time, have the robot print the difference in ms.
  local t_local t_robot
  t_local="$(date +%s.%N)"
  t_robot="$(ssh -o ConnectTimeout=5 -o BatchMode=yes "$ROBOT" 'date +%s.%N' 2>/dev/null)" || return 1
  [[ -z "$t_robot" ]] && return 1
  awk -v a="$t_robot" -v b="$t_local" 'BEGIN{printf "%.0f", (a-b)*1000}'
}

do_sync() {
  (sleep 2; date +%s.%N) | ssh -o ConnectTimeout=5 "$ROBOT" \
      "sudo bash -c 'read t; date -s @\$t'" >/dev/null 2>&1
}

report() {
  local o
  if ! o="$(offset_ms)"; then
    echo "$(date +%T)  ERROR: cannot reach $ROBOT over ssh"
    return 1
  fi
  local abs=${o#-}
  if (( abs > 500 )); then
    echo "$(date +%T)  offset ${o} ms  <-- TOO LARGE, nav2 will misbehave"
  else
    echo "$(date +%T)  offset ${o} ms  OK"
  fi
}

case "$MODE" in
  --check)
    report
    ;;
  --watch)
    echo "Syncing $ROBOT every ${INTERVAL}s. Ctrl-C to stop."
    while true; do
      do_sync
      report
      sleep "$INTERVAL"
    done
    ;;
  *)
    echo "Syncing $ROBOT once..."
    do_sync
    sleep 1
    report
    echo
    echo "Tip: run with --watch during the demo so drift cannot bite you mid-run."
    ;;
esac
