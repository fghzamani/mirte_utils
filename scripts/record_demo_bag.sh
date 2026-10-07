#!/usr/bin/env bash
# Record a Mirte demo run for later causal-model work.
#
# Captures three things that must line up in time:
#   1. what the robot SEES   -> /scan, /odom, /tf, /joint_states, costmaps
#   2. the CONFIG applied    -> /tuner_decision (risk + all 7 Table I params),
#                               /speed_limit, /local_costmap/published_footprint
#   3. the RISK state        -> /risk_state, /risk_state_diagnostics
# plus the navigation OUTCOME (/navigate_to_pose action status + feedback, /plan,
# /cmd_vel, /behavior_tree_log) so each run can be labelled success/abort.
#
# /tuner_decision is the important one: without it a bag has the risk state but
# not the treatment, which is exactly the pair the causal model is fitted on.
#
# Usage:
#   ./scripts/record_demo_bag.sh                 # nav + risk + config (no camera)
#   ./scripts/record_demo_bag.sh --with-camera   # also compressed RGB
#   ./scripts/record_demo_bag.sh my_run_name     # custom name
set -euo pipefail

WITH_CAMERA=0
NAME=""
for arg in "$@"; do
  case "$arg" in
    --with-camera) WITH_CAMERA=1 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) NAME="$arg" ;;
  esac
done

OUTDIR="${BAG_DIR:-$HOME/phd_projects/mirte_demo/bags}"
STAMP="$(date +%Y%m%dT%H%M%S)"
NAME="${NAME:-demo}"
OUT="$OUTDIR/${NAME}_${STAMP}"
mkdir -p "$OUTDIR"

# --- what the robot sees -----------------------------------------------------
TOPICS=(
  /scan
  /odom
  /tf
  /tf_static
  /joint_states
  /map                                  # latched; one message is enough
  /local_costmap/costmap_raw            # what the risk node actually reads
  /global_costmap/costmap
)
# --- localization ------------------------------------------------------------
TOPICS+=( /amcl_pose /particle_cloud )
# --- risk state --------------------------------------------------------------
TOPICS+=( /risk_state /risk_state_diagnostics )
# --- config / treatment ------------------------------------------------------
TOPICS+=( /tuner_decision /speed_limit /local_costmap/published_footprint )
# --- navigation outcome ------------------------------------------------------
TOPICS+=(
  /plan
  /cmd_vel
  /navigation_status
  /behavior_tree_log
  /navigate_to_pose/_action/status
  /navigate_to_pose/_action/feedback
)
# --- optional camera (big: only compressed) ----------------------------------
if [[ "$WITH_CAMERA" == "1" ]]; then
  TOPICS+=( /camera/color/image_raw/compressed /camera/color/camera_info )
fi

echo "Recording -> $OUT"
echo "Topics (${#TOPICS[@]}):"
printf '  %s\n' "${TOPICS[@]}"
echo

# Snapshot the EXACT configuration this run used, next to the bag. A bag with
# /tuner_decision carries the per-tick config, but a fixed-config run (tuner
# disabled) does not -- so without this you cannot tell afterwards which config
# the recorded risk belongs to. Always keep the bag and its config together.
SRC="$HOME/phd_projects/mirte_demo/src"
CFGDIR="${OUT}_config"
mkdir -p "$CFGDIR"
cp "$SRC/mirte_navigation/params/demo_params.yaml" "$CFGDIR/" 2>/dev/null || true
cp "$SRC/online_causal_tuner/config/default_tuner_config.yaml" "$CFGDIR/" 2>/dev/null || true
# the active map (path is inside demo_params)
MAPYAML="$(grep -m1 'yaml_filename' "$SRC/mirte_navigation/params/demo_params.yaml" \
           | sed 's/.*yaml_filename: *"\(.*\)".*/\1/')"
if [[ -f "$MAPYAML" ]]; then
  cp "$MAPYAML" "$CFGDIR/" 2>/dev/null || true
  cp "$(dirname "$MAPYAML")/$(grep -m1 '^image:' "$MAPYAML" | awk '{print $2}')" \
     "$CFGDIR/" 2>/dev/null || true
fi
{
  echo "recorded_at: $(date -Is)"
  echo "bag: $(basename "$OUT")"
  echo "map_yaml: $MAPYAML"
  echo "note: demo_params.yaml here is the config active during this run"
} > "$CFGDIR/run_info.yaml"
echo "Config snapshot -> $CFGDIR"
echo
echo "Ctrl-C to stop."
echo

# zstd file compression keeps costmap/scan bags manageable.
exec ros2 bag record \
  --output "$OUT" \
  --compression-mode file \
  --compression-format zstd \
  "${TOPICS[@]}"
