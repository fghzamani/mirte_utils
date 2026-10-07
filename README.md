# mirte_utils — Mirte physical-robot demo workspace

Workspace glue for running the **online causal tuner** on a physical Mirte
Master: the manual, the operator scripts, the container definition, and a
`vcstool` manifest pinning the three package repositories at the state verified
on the robot.

The ROS packages themselves are **not** in this repository. They are pinned by
[`mirte_demo.repos`](mirte_demo.repos) and fetched into `src/`.

## Reproduce the workspace

```bash
git clone git@github.com:fghzamani/mirte_utils.git mirte_demo
cd mirte_demo
vcs import src < mirte_demo.repos          # pinned to tag mirte-demo-2026-10-07
```

Then, inside the container (there is no ROS on the laptop):

```bash
docker exec -it ros2-tiago-mirte-cpu bash
source /opt/ros/humble/setup.bash
cd /home/forough/phd_projects/mirte_demo
colcon build --symlink-install
```

`vcs import` checks out the tag as a detached HEAD. To continue development:

```bash
for d in src/*/; do git -C "$d" checkout mirte_demo; done
```

| directory | ROS package | repository |
|---|---|---|
| `src/mirte_minimal_navigation` | `mirte_navigation` | fork of `MartijnWisse/mirte_navigation` |
| `src/online_causal_tuner` | `online_causal_tuner` | own |
| `src/rct_data_collector` | **`rct_collector`** | own |

Note the directory and package names differ in two of the three — `colcon build
--packages-select` takes the *package* name.

## Run a demo

```bash
./scripts/run_demo.sh                 # pre-flight checks + full capture
./scripts/run_demo.sh corridor_test   # names the run directory
```

Each run writes `logs/<run>/` containing `console.log`, `bag/` (rosbag2),
`node_logs/` (one file per node) and `config/` (the params and map actually
used). None of that is tracked here — see `.gitignore`.

Use `run_demo.sh` rather than `ros2 launch` directly: it captures the console
stream, runs the arm and DDS pre-flight checks, and filters the tf2 NaN spam
from the terminal while keeping it in the log.

## Operator scripts

| script | purpose |
|---|---|
| `run_demo.sh` | launch with pre-flight checks and full per-run capture |
| `check_localization.py` | **run before every goal.** Scores scan-to-map match; a wrong pose surfaces only as a *planner* failure |
| `set_initial_pose_from_scan.py` | locates the robot by scan-matching the map and publishes `/initialpose` |
| `check_arm.sh` | per-joint standing error; distinguishes STUCK from UNREADABLE (NaN) |
| `set_arm_pose.sh` | verified arm move, aborts on NaN, stops if the arm is not moving |
| `check_goal.py` | offline goal reachability against the live map and footprint |
| `analyze_run.py` | commanded vs achieved velocity from a run bag |
| `record_demo_bag.sh` | record against an already-running stack |
| `sync_robot_clock.sh` | clock offset check / resync |

## Container

`container/` holds copies of the container definition and the DDS profile.
They are **copies**: the live setup references
`~/phd_projects/fastdds_eth0.xml` by absolute path through
`FASTRTPS_DEFAULT_PROFILES_FILE`, so moving the original would break a working
system. Keep them in sync, or re-point the variable here.

`fastdds_eth0.xml` is load-bearing. It declares an SHM transport alongside the
eth0-restricted UDP one; without SHM, every message between two nodes inside the
container is forced through the eth0 socket and the nav stack stalls mid-run
with frozen `map->odom` transforms. The UDP whitelist pins one hardcoded
address — if `hostname -I` does not contain it, nothing discovers anything.

## Before you debug anything

[`mirte_manual.md`](mirte_manual.md) is the operational record: every failure
mode seen on this robot, how it presented, and what actually caused it. Most
symptoms here point somewhere other than their cause. The ones that cost the
most time:

- **A wrong initial pose reports itself as a planner failure**, never as a
  localisation error. `check_localization.py` first.
- **`map_saver_cli` resets `free_thresh` to 0.25.** It must be `0.196`, or
  unknown map cells read as free and the planner routes through unmapped walls.
- **The arm trajectory controller returns `SUCCEEDED` even when the servos never
  move**, and reports `nan` positions when it cannot read them. Only
  `controller_state`'s standing error is trustworthy.
- **MPPI sampling std must stay inside the actuation range.** A std larger than
  the velocity limit collapses exploration and the robot stands still.
- **`r_min` has a floor** set by the laser self-filter; it cannot distinguish
  "at the floor distance" from "touching".
