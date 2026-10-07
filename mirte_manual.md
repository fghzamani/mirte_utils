# Mirte Master — Setup and Usage Manual

Personal working manual for connecting to and working with the Mirte Master robot.
Laptop: Ubuntu, internet via Wi-Fi (eduroam), robot via Ethernet cable (`eth0`).

---

## 1. Connecting to the robot

### 1.1 Physical connection

1. Make sure the arm points roughly upward before switching the robot on.
2. Switch the robot on and wait until text appears on the side display.
3. Connect a LAN cable between the robot and the laptop.

### 1.2 Laptop network configuration (one-time)

The robot runs its own DHCP server on its Ethernet port and gives the laptop an address in `192.168.45.x`.
The laptop must be a DHCP **client** on `eth0` (not "Shared to other computers"), and must never use the cable as its default route, so internet stays on Wi-Fi.

```bash
nmcli con mod "Wired connection 1" ipv4.method auto ipv4.never-default yes ipv6.method ignore
nmcli con up "Wired connection 1"
```

Check:

```bash
ip -br addr show eth0        # expect 192.168.45.x (e.g. 192.168.45.35)
ip route | grep default      # must list only wlp0s20f3 (Wi-Fi)
ping -c3 8.8.8.8             # internet still works
```

### 1.3 Terminal access (SSH)

```bash
ssh mirte@192.168.45.1
```

- Password: `mirte_mirte38`
- First connection: answer `yes` to the host-key prompt.
- Exit the robot shell with `exit` or `Ctrl+D`.

Copy files from laptop to robot:

```bash
scp my_file.py mirte@192.168.45.1:~/mirte_ws/src/
```

### 1.4 Web interface (alternative)

- VS Code in browser: `http://192.168.45.1:8000`
- Robot settings page: `http://192.168.45.1`
- Login: user `mirte`; password not yet verified (README default is `mirte_mirte`; try `mirte_mirte38` if that fails)

### 1.5 Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `eth0` has `10.42.0.1` | Laptop is in shared mode. Re-run 1.2. |
| `eth0` has no IPv4 address | Robot not fully booted, or cable. Wait for the side display, then `nmcli con up "Wired connection 1"`. |
| `ping 192.168.45.1` fails but `eth0` has `192.168.45.x` | Robot may ignore ICMP. Try SSH or the browser directly. |
| `ssh: Connection timed out` | Wrong robot address. Check `nmcli -g DHCP4 device show eth0 \| tr '\|' '\n' \| grep dhcp_server_identifier`. |
| `ssh: Connection refused` | SSH server off. Open the web VS Code terminal and run `sudo systemctl enable --now ssh`. |
| `REMOTE HOST IDENTIFICATION HAS CHANGED` | Robot was re-flashed. `ssh-keygen -R 192.168.45.1`, then reconnect. |
| Laptop loses internet when cable is plugged in | `never-default` not set. Re-run 1.2. |

---

## 2. Clock synchronization (every robot boot)

### 2.1 Why

The robot has no active time service and no internet on the cable, so its clock is wrong after boot (it was about one month behind). ROS 2 messages carry the robot's timestamps; TF, Nav2 and costmaps compare them with the laptop's clock. An offset larger than roughly 0.1 s causes "extrapolation into the future" or "message too old" errors. Driving with `cmd_vel` alone does not need this; anything using TF or Nav2 does.

### 2.2 Prerequisite (one-time): SSH without password

A password prompt delays the commands and corrupts the timing. Run on the laptop:

```bash
ssh-copy-id mirte@192.168.45.1
```

### 2.3 Set and check the clock

Run on the laptop (not in the container), after every robot boot, and again before a long demo because the robot clock drifts:

```bash
# set robot clock to laptop time
(sleep 2; date +%s.%N) | ssh mirte@192.168.45.1 "sudo bash -c 'read t; date -s @\$t'"

# measure offset (robot minus laptop)
(sleep 2; date +%s.%N) | ssh mirte@192.168.45.1 'read t; python3 -c "print(f\"offset {($(date +%s.%N)-$t)*1000:.0f} ms\")"'
```

Target: offset within about ±20 ms (achieved: -1 ms).

The `sleep 2` lets SSH finish connecting before the laptop time is read, and `sudo` is started before the time is read; both remove delays that would otherwise leave the robot tens to hundreds of milliseconds behind.

#### Helper script (and why you must not skip this)

```bash
./scripts/sync_robot_clock.sh           # sync once + report offset
./scripts/sync_robot_clock.sh --check   # report offset only
./scripts/sync_robot_clock.sh --watch   # resync every 60 s -- RUN THIS DURING A DEMO
```

The robot has no NTP and its clock drifts badly (observed: **29 minutes** within one
session, and **1.8 days** after a boot). Nav2 runs on the laptop clock while `/scan`
and `/odom` carry the robot's timestamps, so any skew makes AMCL's `map->odom`
transform look stale. The failure is silent and misleading:

```
[tf_help]: Transform data too old when converting from map to odom
[controller_server]: Reached the goal!      <-- instantly, robot never moved
[bt_navigator]: Begin navigating from current location (0.00, 0.00)  <-- bogus pose
```

Nav2 reports **success** without the robot moving a centimetre. If you ever see an
instant "GOAL REACHED" with 0 recoveries, check the clock first.

Pre-flight check before sending any goal:

```bash
ros2 topic delay /odom     # want ~milliseconds, NOT minutes
```

---

## 3. Connecting the Docker container to the robot (ROS 2)

### 3.1 Why

The robot runs its drivers (base, lidar, cameras, arm) and publishes ROS 2 topics. Code in the laptop container sees and uses these topics over the LAN cable if both sides use the same ROS 2 settings. SSH is not involved; it only gives a shell on the robot.

Robot settings (checked on its running nodes): Humble, `ROS_DOMAIN_ID` = 0 (unset), Fast DDS (`rmw_fastrtps_cpp`, default), `ROS_LOCALHOST_ONLY=0`.

The simulation container (`ros2-tiago-dev`) uses `ROS_LOCALHOST_ONLY=1` and can never see the robot. A separate robot container is needed.

### 3.2 Fast DDS profile (one-time)

Restricts ROS 2 traffic to the cable (`eth0`), so it does not go out on eduroam.

```bash
cat > ~/phd_projects/fastdds_eth0.xml << 'XML'
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_eth0</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <address>192.168.45.35</address>
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="eth0_only" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_eth0</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML
```

The `<address>` must equal the laptop's `eth0` address (`ip -br addr show eth0`). If it changes, edit the file and restart the container.

### 3.3 Create the robot container (one-time)

`container_creator.sh` has a `--robot` option. It sets `--ipc host`, `ROS_LOCALHOST_ONLY=0`, `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and `FASTRTPS_DEFAULT_PROFILES_FILE=~/phd_projects/fastdds_eth0.xml`. Without `--robot` it creates a simulation container as before.

```bash
cd <folder with container_creator.sh and Dockerfile>   # mirte_demo
./container_creator.sh --name mirte --robot     # creates ros2-tiago-mirte
```

`--name` sets the container name; without it the name defaults to `ros2-tiago-dev`, which already exists.

### 3.4 Start the container (every session)

On the laptop:

```bash
xhost +local:docker
docker start -ai ros2-tiago-mirte
```

Use `ros2-tiago-mirte` for the robot and `ros2-tiago-dev` for simulation. In the robot container, ROS 2 runs only over `eth0`: with the cable unplugged, even nodes inside the container cannot see each other.

Where to run what:
- Laptop terminal: SSH, clock sync, network settings.
- Container: `ros2 ...`, own code, RViz.

### 3.5 Verify the connection

Inside the container:

```bash
env | grep -E 'ROS_|RMW|FASTRTPS'        # ROS_LOCALHOST_ONLY=0, ROS_DOMAIN_ID=0, rmw_fastrtps_cpp
hostname -I                              # must include 192.168.45.35 (container has no 'ip' tool)
ros2 daemon stop; ros2 topic list --no-daemon
```

Expected topics include `/scan`, `/tf`, `/tf_static`, `/joint_states`, `/mirte_base_controller/cmd_vel`, `/mirte_base_controller/odom`, `/gripper_camera/...`.

Check that data arrives, not only names:

```bash
ros2 topic echo /joint_states --once
ros2 topic hz /scan
```

Two-way test without moving the robot. Container:

```bash
ros2 topic pub /laptop_test std_msgs/msg/String "{data: hello_from_container}" -r 1
```

Robot (SSH): `ros2 topic echo /laptop_test`.

Motion test (robot lifted):

```bash
ros2 topic pub /mirte_base_controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10
```

### 3.6 Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Only `/parameter_events`, `/rosout` | Wrong container (simulation), firewall, or XML address wrong. Check `env`, `sudo ufw allow in on eth0`, and 3.2. |
| Only `/io/...` topics visible; `/scan`, `/tf`, `cmd_vel` missing | Seen once; resolved, exact fix not recorded. Candidates: restart the robot's ROS service while the cable is connected (`systemctl list-units --type=service \| grep -i -E 'ros\|mirte'`, then `sudo systemctl restart <service>`), or reboot the robot with the cable plugged in. |
| Topics listed but `echo` prints nothing | Data blocked: firewall. Restart the container after fixing. |
| `ip: command not found` in container | Image lacks `iproute2`. Use `hostname -I`, or run `ip` on the laptop (same network via `--net host`). |

---

## 4. Mapping a new environment (SLAM)

Workspace: `/home/forough/phd_projects/mirte_demo` (container paths — inside the container `~` is `/root`, so always write absolute paths).
Nav2 and SLAM run in the container; the robot only provides `/scan`, `/tf`, odometry and accepts `cmd_vel`.

### 4.1 One-time setup in the container

```bash
apt install -y ros-humble-topic-tools ros-humble-slam-toolbox ros-humble-navigation2
mkdir -p /home/forough/phd_projects/mirte_demo/src
cd /home/forough/phd_projects/mirte_demo/src
git clone --branch ros2_humble https://github.com/MartijnWisse/mirte_navigation
cd /home/forough/phd_projects/mirte_demo
colcon build --symlink-install --packages-select mirte_navigation
```

Adjust the laser range in `src/mirte_navigation/params/slam_params.yaml` (under `ros__parameters`, four spaces):

```yaml
    min_laser_range: 0.2
    max_laser_range: 12.0
```

The defaults (0.0 and 25.0) exceed the lidar's capability (0.2–16.0 m) and let noise become walls.

### 4.2 Each session

Terminal setup for every container terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/forough/phd_projects/mirte_demo/install/setup.bash
```

1. Cable connected, `ros2 topic hz /scan` shows a steady rate.
2. Clock synced (section 2.3) — SLAM drops scans on a clock offset.

### 4.3 Run

Terminal 1 — SLAM (arm TF errors are noise from the `NaN` joint states; filter them):

```bash
ros2 launch mirte_navigation minimal_slam_launch.py 2>&1 | grep -v -E 'TF_NAN|TF_DENORM|buffer_core'
```

Terminal 2 — RViz (run on the laptop, in a second terminal):

```bash
docker exec -it ros2-tiago-mirte bash -lc 'source /opt/ros/humble/setup.bash && source /home/forough/phd_projects/mirte_demo/install/setup.bash && rviz2 -d /home/forough/phd_projects/mirte_demo/src/mirte_navigation/mirte_rviz.rviz'
```

- Fixed Frame: `map` (not `base_link`, or the map moves with the robot)
- `Map` display, topic `/map`, **Durability Policy: Transient Local** — otherwise nothing is shown
- `LaserScan` `/scan`, `TF`
- View type TopDownOrtho for a stable orientation

Terminal 3 — driving:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/mirte_base_controller/cmd_vel
```

Driving rules: slowly, turns even slower, follow walls at 1–2 m, pause after each turn (scans get dropped otherwise), revisit mapped areas and close a loop at the end, open the doors that belong in the map.

### 4.4 Save the map (before stopping SLAM)

```bash
ros2 run nav2_map_server map_saver_cli -f /home/forough/phd_projects/mirte_demo/src/mirte_navigation/maps/<name>
ls -l /home/forough/phd_projects/mirte_demo/src/mirte_navigation/maps/<name>.*
```

`-f` takes a path without extension and without a trailing slash. If the write fails, save to `/tmp/<name>` first and copy afterwards.

### 4.5 Editing the map (optional)

GIMP on the laptop (`sudo apt install gimp`). Only three pixel values are allowed: 0 occupied, 205 unknown, 254 free. Grayscale mode, pencil at 100% hardness, no anti-aliasing. **Never change the canvas size** — the `origin` in the YAML refers to the original pixel grid, and cropping shifts the map against the robot's coordinates. Save with File → Overwrite.

Check:

```bash
python3 -c "
from PIL import Image; import collections
im=Image.open('<path>/<name>.pgm'); print(im.mode, im.size)
print(collections.Counter(im.getdata()).most_common())"
```

Expect three values, and the same size as before the edit.

### 4.6 Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Grey area moving with the robot | Fixed Frame is `base_link` → set to `map`; `Map` display on Volatile → Transient Local |
| No map, `/map` exists | Check clock, `ros2 topic hz /scan`, and `tf2_echo odom base_footprint` |
| `Message Filter dropping message ... queue is full` | Driving too fast for the sync SLAM node — drive slower |
| `map_saver_cli`: `Unable to open file` | `~` is `/root` in the container → absolute path; or the folder isn't writable |
| RViz map looks blockier than the file | A costmap display (`/global_costmap/costmap`) is on, not `/map` |
| Scan doesn't lie on the walls | Pose estimate imprecise, or the map was cropped and the `origin` no longer matches |

## Navigating with a saved map

Navigation uses the map you saved with SLAM for localization (AMCL) plus the
Nav2 stack (planner, controller, BT navigator). The controller shipped in this
package is Regulated Pure Pursuit at 10 Hz.

### Prerequisites
- A saved map in `maps/` (see "Generating a map with SLAM").
- `mirte_navigation` built and sourced; `/scan` and
  `/mirte_base_controller/odom` publishing.
- SLAM is NOT running. SLAM and AMCL both publish the map→odom TF — never run
  both at once.

### 1. Point the config at your map
Edit `params/minimal_nav2_params.yaml`:
```yaml
map_server:
  ros__parameters:
    yaml_filename: "/home/mirte/mirte_ws/src/mirte_navigation/maps/my_map.yaml"
```
This is the only place the map is set. There is no map launch argument.
If you built with `--symlink-install`, the edit takes effect on next launch with
no rebuild; otherwise rebuild the package.

### 2. Launch navigation
```bash
cd ~/mirte_ws && source install/setup.bash
ros2 launch mirte_navigation minimal_navigation_launch.py
```
This starts map_server, amcl, planner_server, controller_server, bt_navigator,
the lifecycle manager, the two static TFs, and the `/cmd_vel` and `/odom` relays.
The lifecycle manager autostarts, so the stack activates on its own.

To use a different params file without editing the default:
```bash
ros2 launch mirte_navigation minimal_navigation_launch.py \
  params_file:=/path/to/your_params.yaml
```

If `/odom` does not appear in the container, check it from a second container
terminal while navigation is still running:

```bash
ros2 topic list --no-daemon | grep odom
ros2 topic hz /mirte_base_controller/odom
ros2 topic echo /mirte_base_controller/odom --once
ros2 node list | grep relay
```

The robot must publish `/mirte_base_controller/odom`. If that source topic is
publishing but `/odom` is still missing, start the relay manually in one
terminal and check the topic from another:

```bash
ros2 run topic_tools relay /mirte_base_controller/odom /odom
```

Do not press `Ctrl-C`; this command must remain running. Leave this terminal
open and, in a second container terminal, run:

```bash
ros2 topic list --no-daemon | grep odom
ros2 topic echo /odom --once
```

If the manual relay gives no response or `/odom` still does not appear, reset
the ROS 2 daemon, then start the relay again:

```bash
ros2 daemon stop
ros2 daemon start
ros2 run topic_tools relay /mirte_base_controller/odom /odom
```

Leave the relay terminal running. From a second container terminal, verify:

```bash
ros2 topic list --no-daemon | grep odom
ros2 topic echo /odom --once
```

If the source topic does not publish, fix or restart the robot base controller
first. The odometry topic must also have a live TF; verify it with:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Do not replace this with a static `odom` to `base_link` transform: the
transform must change as the robot moves.

### 3. Set the initial pose
AMCL must receive an initial pose before it can publish the `map` frame. If the
launch log says `AMCL cannot publish a pose or update the transform`, set the
pose immediately in RViz with the "2D Pose Estimate" tool (Fixed Frame =
`map`), then click the robot's actual position and drag in its heading.

Alternatively, publish the configured origin pose with the helper node:
```bash
  ros2 run mirte_navigation set_initial_pose
```
(The helper publishes `(0,0,0)` for 20 seconds; edit it or use RViz for any
other pose.)

If using a params file with `set_initial_pose: true`, AMCL starts at its
configured `initial_pose`. This is only suitable when the robot is actually at
that pose; otherwise use RViz to set the real pose.

### 4. Send a goal
- RViz: "2D Goal Pose" tool, click the target on the map.
- CLI:
```bash
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, \
    orientation: {w: 1.0}}}}"
```
To see live action feedback and the final navigation status in the terminal,
send the goal with `--feedback`:
```bash
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, \
  orientation: {w: 1.0}}}}"
```
The final result reports `Goal finished with status: SUCCEEDED` when the robot
reaches the goal. `ABORTED` or `CANCELED` means it did not complete the goal.
In RViz, the saved configuration displays the global `/plan` and local
`/local_plan` paths; the green and red paths show the route and current local
controller trajectory.

### 5. Confirm it's working
```bash
ros2 topic echo /cmd_vel --once     # controller is commanding motion
ros2 lifecycle get /amcl            # should report 'active'
```
In RViz you should see the AMCL particle cloud, a global path (planner), and a
local path (controller) once a goal is sent.

### Notes / gotchas
- COLLISION RISK IN DEFAULT CONFIG: the costmaps use `robot_radius: 0.22` but
  `inflation_radius: 0.15`, which is smaller than the radius. Cells 0.15–0.22 m
  from an obstacle carry no cost, so the planner can route the robot's body into
  contact. Raise `inflation_radius` to at least `0.25` (and set
  `cost_scaling_factor` accordingly) in BOTH `local_costmap` and `global_costmap`
  before any autonomous run near obstacles.
- One TF source only: stop SLAM before launching navigation.
- If the robot drives poorly (drifts, overshoots), suspect the base PID gains,
  not Nav2 — the launch file warns that gains, including an integrator term, must
  be set in
  `/opt/ros/humble/share/mirte_base_control/config/mirte_base_control.yaml`.

## Moving the arm (footprint poses)

The arm pose changes the robot's footprint: **tucked** (arm stowed, small round
base) vs **carry / side-extended** (arm swung out to the side, larger asymmetric
footprint). These two poses are the `tucked` / `carry` states the causal tuner
treats as a knob, and the poses you capture with
`ros2 run rct_collector measure_footprint`.

Joint roles (arm controller `/mirte_master_arm_controller`, joints
`shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_joint`):
- `shoulder_pan_joint` — **yaw**: swings the whole arm left/right to the side.
- `shoulder_lift_joint` — raises/lowers the arm.
- `elbow_joint` — bends the forearm (reach/height).
- `wrist_joint` — wrist angle.

Commands are sent to the controller topic; one joint at a time is fine (list only
the joint you want). Sign sets direction; watch RViz and stay within servo limits.

### Side-extended (carry) footprint

Run these two, one after the other, to bend the elbow and swing the arm out to
the side:

```bash
# bend the elbow
ros2 topic pub --once /mirte_master_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [elbow_joint], points: [{positions: [-1.5], time_from_start: {sec: 2}}]}"

# swing the arm to the side (shoulder yaw)
ros2 topic pub --once /mirte_master_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [shoulder_pan_joint], points: [{positions: [1.7], time_from_start: {sec: 3}}]}"
```

Or in a single message (all four joints at once):

```bash
ros2 topic pub --once /mirte_master_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_joint], \
  points: [{positions: [1.7, 0.0, -1.5, 0.0], time_from_start: {sec: 3}}]}"
```

### Measuring the side-extended footprint

Only the side-extended state needs measuring (home uses the circular footprint).
Do it in the **container** (where `rct_collector` is built and robot comms work):

1. Put the arm in the side-extended pose with the commands above and wait for it
   to finish moving (watch RViz).
2. With the arm held there, run:

   ```bash
   ros2 run rct_collector measure_footprint --label carry
   ```

   The tool reads the current arm joint angles from `/joint_states` and the live
   TF of every arm/gripper link, projects them onto the ground in `base_link`
   (default; `base_footprint` only exists while the nav launch runs, and the two
   differ by a zero transform), and adds a 0.22 m circular base. It prints two
   ready-to-paste blocks:
   - an **`ARM_CONFIGS["carry"]`** entry (measured footprint polygon + joint targets)
   - a **`FOOTPRINT_GEOMETRY[1.0]`** line (measured `r_circ` and `width`)
3. Paste both into
   `src/online_causal_tuner/online_causal_tuner/online_tuner_node.py`, replacing the
   `carry` placeholders.

Defaults are already set for Mirte (`--base-radius 0.22`,
`--link-filter "(shoulder|elbow|wrist|gripper)"`). If it reports "No arm-link
transforms resolved," pass the real link names explicitly with
`--links shoulder_lift,elbow,wrist,gripper` (check with `ros2 run tf2_tools
view_frames` or `ros2 topic echo /tf_static`).

### Home (tucked) pose

The home pose keeps the arm stowed over the base, so it uses the **simple
circular footprint** the robot already has in the YAML (`robot_radius: 0.22`) —
no measurement needed for this state.

```bash
ros2 topic pub --once /mirte_master_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_joint], \
  points: [{positions: [0.0, 0.0, 0.5, 0.0], time_from_start: {sec: 4}}]}"
```

Notes:
- The same interface is available as an action
  (`/mirte_master_arm_controller/follow_joint_trajectory`,
  `control_msgs/action/FollowJointTrajectory`), which is what the tuner and
  `measure_footprint` use.
- If a single-joint command is rejected as a partial goal, either enable
  `allow_partial_joints_goal` on the controller or send all four joints at once.
- Whichever joint moves most between your tucked and carry poses is the one the
  risk node's `a_t` (arm-extension index) should track — set `arm_extension_joint`
  and the stowed/extended reference angles accordingly.

### The arm reports success but does not move

The `joint_trajectory_controller` reports

```
error_code: 0
error_string: Goal successfully reached!
Goal finished with status: SUCCEEDED
```

**even when the servos never moved.** `ros2 action send_goal` and
`ros2 topic pub .../joint_trajectory` both look like they worked. Do not trust
either as evidence that the arm actually went anywhere.

The only reliable test is the controller's own standing error:

```bash
./scripts/check_arm.sh            # report
./scripts/check_arm.sh --move     # command tucked, then report
```

which reads `/mirte_master_arm_controller/controller_state` and compares
`reference` (commanded) against `feedback` (measured). A healthy joint has an
error near zero. A dead one looks like this:

```
joint                   commanded   measured    error
shoulder_pan_joint          0.000      1.512   -1.512  <-- STUCK
shoulder_lift_joint         0.000     -0.021    0.021
elbow_joint                 0.500     -1.552    2.052  <-- STUCK
wrist_joint                 0.000     -0.010    0.010
```

`shoulder_pan` and `elbow` are the two joints that carry the load in the
extended **carry** pose, and they are the ones that cut torque after holding it
for a while. They are also the only two joints that *distinguish* the two
poses — `shoulder_lift` and `wrist` sit near 0 in both carry and tucked, so a
check that happens to look only at those two will report a false "arm fine".

**Fix: power-cycle the arm.** Nothing in software clears it.

Why it matters for the demo: the tuner will only shrink the costmap footprint
once `/joint_states` *confirms* the arm is tucked (`_envelope_label()` returns
`carry` whenever the physical and target states disagree, which is the safe
direction). So a stuck arm means the footprint stays at the large carry polygon,
the robot will not fit the narrow corridor, and the run ends in
`Failed to make progress` / `GOAL FAILED (aborted)` — with nothing in the log
pointing at the arm. `run_demo.sh` now runs this check before launching, and the
tuner gives up after `MAX_ARM_RETRIES = 5` with a single loud `ARM UNRESPONSIVE`
error instead of silently re-issuing the goal every tick.

## Capturing a full demo log

`ros2 launch` output scrolls away and the interesting lines are spread across
15 nodes. Use the wrapper instead of calling `ros2 launch` directly:

```bash
./scripts/run_demo.sh                     # -> logs/demo_<timestamp>.log
./scripts/run_demo.sh rviz:=false
./scripts/run_demo.sh tuner:=false base_only   # bare word = log name
LOG_DIR=/tmp ./scripts/run_demo.sh
```

It records the launch args and the active footprint/inflation values at the top
of the log, runs the arm pre-flight, then tees the complete stream to the file
while still showing it live. Ctrl-C keeps the log (output is line-buffered via
`stdbuf`, so nothing is lost on interrupt).

Triage greps:

```bash
grep "TUNER TICK" logs/<file> | tail -20          # what the tuner decided
grep -E "ARM UNRESPONSIVE|Arm target|Sending arm" logs/<file>
grep -E "ERROR|Optimizer fail|Failed to make progress|lethal" logs/<file>
grep -vE "feedback for unknown goal" logs/<file> > logs/<file>.clean   # drop BT spam
```

Per-node ROS logs are also kept under `~/.ros/log/` for the same run.

## The nav stack stops mid-run and the logs blame transforms

Signature in the logs:

```
controller_server:  [tf_help]: Transform data too old when converting from map to odom
                    Data time: ...016s, Transform time: ...007s   <-- frozen, never advances
bt_navigator:       Timed out while waiting for action server to acknowledge compute_path_to_pose
                    Node timed out while executing service call to .../clear_entirely_global_costmap
lifecycle_manager:  CRITICAL FAILURE: SERVER map_server IS DOWN after not receiving a
                    heartbeat for 4000 ms. Shutting down related nodes.
```

This is **not** an AMCL, localization, or planner problem, even though every
message points that way. Check these three things before touching nav2:

1. **Is it CPU?** Almost certainly not — check anyway: `nproc`, `cat /proc/loadavg`.
   20 cores at load 0.6 is not starvation.
2. **Did the robot's data keep flowing?** If the risk node's
   `components=9/8, degraded=N/M` counter keeps advancing by exactly
   `10 * interval` through the freeze, `/scan` and `/odom` were healthy and the
   robot link was fine.
3. **Did several unrelated node-to-node paths break at the same instant?**
   AMCL→controller TF, AMCL→RViz TF, and map_server→lifecycle_manager bond are
   three independent pairs. If all three stall together while traffic *from the
   robot* is perfect, the problem is the local DDS transport, not any node.

### The cause: the FastDDS profile

`$FASTRTPS_DEFAULT_PROFILES_FILE` (`phd_projects/fastdds_eth0.xml`) sets

```xml
<useBuiltinTransports>false</useBuiltinTransports>
```

which **also disables shared memory and loopback**. With only a UDPv4
descriptor declared, every message between two nodes inside the container —
`/tf` at ~50 Hz, costmap updates, every service call, every lifecycle bond
heartbeat — is pushed out through the eth0 UDP socket. Under load those buffers
overflow and intra-container traffic starts dropping. The single modest `/scan`
stream from the robot is unaffected, which produces the misleading asymmetry.

The profile now declares an `SHM` transport alongside the UDP one, so
node-to-node traffic never reaches the network. Verify it is live:

```bash
ls /dev/shm/ | grep fastrtps      # segments present => SHM in use
```

A parse error in that XML silently disables the *whole* profile, so always
check after editing:

```bash
ros2 topic list 2>&1 | grep XMLPARSER     # any output = profile is broken
```

(`max_message_size` is **not** a valid SHM tag on Humble's FastDDS; only
`segment_size` is. That one typo disables everything.)

### The second cause: one hardcoded IP

The whitelist pins the UDP transport to `192.168.45.35`. If no interface holds
that address — robot powered off, or DHCP gave a different one — the whitelist
matches nothing and, with builtin transports off, nodes start but never discover
each other. `127.0.0.1` is now whitelisted as well so container-local work
survives, but **the robot link still needs the real address**. Check with:

```bash
hostname -I       # must contain 192.168.45.35 for the robot to talk
```

`run_demo.sh` checks this before launching and prints
`whitelisted address ... is NOT held by any interface`.

### Why the robot stayed stuck

A transient drop cost `map_server` one bond heartbeat, and the lifecycle manager
reacted by shutting down the stack — so the robot stopped permanently rather
than recovering. `bond_timeout` is now `20.0` s (default 4.0) in
`minimal_navigation_launch.py`.

Note that the `lifecycle_manager:` block in `demo_params.yaml` is **dead
config**: the node runs as `lifecycle_manager_navigation` and takes its
parameters inline from the launch file, so the names do not match and nothing in
that YAML block is applied. Edit the launch file.

## The tuner never changes the inflation radius

Symptom: every `TUNER TICK` line reports the same `Inflation: 0.30m` for the
whole run while only `Speed:` moves, the robot strafes sideways at the mouth of
a narrow corridor, never follows the path through it, and the run ends in
`controller_server: Failed to make progress`. With inflation at 0.30 m the
corridor is inflated shut, MPPI can see no low-cost path through the gap, so it
crabs sideways looking for one.

Three separate causes, all fixed on 2026-10-01:

1. **`envelope_only` mode pinned it.** The ablation picks the candidate closest
   to `ENVELOPE_ONLY_SOFTWARE`, which hardcodes
   `inflation_radius: 0.30`. Inflation is now excluded from that fixed set and
   chosen as the *largest admissible* value instead, so the A(R) ceiling
   decides it. Note this mode is active whenever the causal models cannot load
   (see the numpy/sklearn mismatch), which is the normal state right now.

2. **`r_width` was floored at 1.50 m**, so the envelope could not perceive any
   corridor narrower than that:
   `lateral = (1.50 - 0.421)/2 - 0.1063 = 0.433`, and the ceiling
   `max(inflation_floor_m, lateral)` never dropped below 0.43 — 0.30 was always
   admissible. The floor is now just the geometrically valid `2 * r_min`.

3. **The apply gate required a change of >= 0.10 m** while the candidate grid
   steps in 0.05, so every single-step change was silently discarded. The gate
   is now one grid step (0.04) plus a dwell of `INFLATION_DWELL_TICKS = 5`
   (~1 s) so a noisy `r_width` cannot thrash the costmap at tick rate.

Resulting behaviour (tucked footprint, `inflation_floor_m = 0.15`):

| measured `r_width` | inflation applied |
|---|---|
| <= 1.00 m (corridor) | 0.15 |
| 1.20 m | 0.25 |
| >= 1.50 m (open floor) | 0.30 |

The tuner now also pushes inflation **and** footprint to the *global* costmap,
not just the local one. Without that the global planner keeps routing for the
wide carry polygon through a corridor it believes is blocked, and relaxing only
the local costmap cannot help the controller follow a path that was never
planned.

`TUNER TICK` lines now include `r_min=`, `r_width=` and the raw pre-repair
`(raw ...)` value, because none of the geometry driving these decisions was
previously recoverable from the logs.

Beware: `demo_params.yaml` sets `inflation_radius: 0.15` on both costmaps, but
the tuner **overrides it on the first tick**. Lowering it by hand there does
nothing once the tuner is running — the value that matters is the one the tuner
selects.

### Inflation radius must stay above the inscribed radius

If you see this on either costmap, treat it as a real defect, not a warning:

```
The configured inflation radius (0.150) is smaller than the computed inscribed
radius (0.150) of your footprint, it is highly recommended to set inflation
radius to be at least as big as the inscribed radius to avoid collisions
```

Below that threshold the inflation layer stops marking the band around an
obstacle that the robot's own **body** occupies. The visible consequences are:

```
planner_server: GridBased: failed to create plan, invalid use:
                Starting point in lethal space! Cannot create feasible plan..
controller_server: Optimizer fail to compute path
```

and the robot drives **straight into obstacles while the footprint is entirely
correct** — which makes it look like full-body collision avoidance is broken
rather than like a costmap setting.

Mirte's numbers: both footprints have an inscribed radius of **0.140**, and nav2
adds `footprint_padding: 0.01`, so the hard lower bound is **0.150**. The
corridor sets the upper bound — a channel of `corridor - 2*inflation` must still
exceed the 0.421 m lateral span, which for the ~0.84 m lab corridor caps
inflation at ~0.21.

That leaves exactly one safe value for tight spaces: **0.20**. It is now the
`inflation_floor_m`, the smallest entry in the candidate grid (`0.10` and `0.15`
were removed outright so the unsafe regime is unreachable), and the static value
in `demo_params.yaml` for both costmaps.

Resulting selection:

| `r_min` | `r_width` | inflation |
|---|---|---|
| <= 0.50 m | <= 1.00 m | 0.20 |
| 0.60 m | 1.20 m | 0.25 |
| >= 0.75 m | >= 1.50 m | 0.30 |

### `verified='None'` in the tuner ticks

Means `/joint_states` matches *neither* arm pose within
`ARM_JOINT_TOLERANCE_RAD = 0.15` — usually one joint stalled partway. Example:
`shoulder_pan` stopped at 0.528 rad on the way to 0.0 while elbow, lift and
wrist all arrived. `_envelope_label()` then returns `carry`, so the costmap
keeps the **larger** footprint, which is the safe direction. Run
`./scripts/check_arm.sh` to see which joint it is.

## The run always starts in the carry pose

Two independent mechanisms guarantee the documented starting condition, because
each one alone was insufficient.

**1. The launch commands it and verifies it.**
`demo_launch.py` runs `scripts/set_arm_pose.sh carry 4` instead of a bare
`ros2 topic pub --once`. The trajectory controller reports success whether or
not the servos moved, so the script commands the pose, re-reads
`/joint_states`, compares all four joints against the target within 0.15 rad,
and retries up to 4 times before failing loudly. Usable by hand too:

```bash
./scripts/set_arm_pose.sh carry      # verified move to carry
./scripts/set_arm_pose.sh tucked     # verified move to home/tucked
```

**2. The tuner holds carry until the first goal arrives.**
Without this the envelope tucked the arm on the *very first tick*, before any
goal existed, and the demo lost its starting condition:

```
[TUNER TICK #1] ... Arm: target='tucked'
Decision: arm 'carry' -> 'tucked'
```

The cause is the envelope-only selection key. `ENVELOPE_ONLY_SOFTWARE` asks for
`speed_limit_pct = 100`, `_software_distance` is the **primary** sort key, and
the tucked footprint carries a higher speed ceiling than carry — so tucked
always scores closer to 100 and the `-footprint` preference for carry is only a
tie-break that is never reached. Raising the carry thresholds would not fix it;
the comparison never gets that far.

`startup_carry_hold: true` (in `default_tuner_config.yaml`) therefore pins
`arm_label = "carry"` while no goal is active, and releases permanently on the
first goal:

```
Startup carry hold released (goal received): the envelope now controls the arm.
```

Ticks during the hold are labelled
`Reason: STARTUP_CARRY_HOLD (no goal yet; arm pinned to carry)`.

This is safe: the robot is stationary at its initial pose throughout the hold,
and once released the envelope regains full control — including the immediate,
dwell-free `carry -> tucked` retraction path. Set `startup_carry_hold: false`
to let the envelope decide from tick 1.

### Note on the stuck joint

`shoulder_pan` reaches **carry** reliably (verified: `reached 'carry' (attempt
1)` from a standing position of 0.528 rad) but stalls on the way back toward
0 (tucked), getting part-way and stopping. The failure is direction-dependent,
which points at load or a mechanical limit rather than a dead servo.

## The robot drives straight into an obstacle

Checked and ruled out first, in this order — none of these was the cause:

- **Footprint too small.** It is *over*-sized, not under-sized. See below.
- **Inflation below the inscribed radius.** Fixed earlier; the error no longer
  appears on either costmap.
- **Obstacle layer misconfigured.** `observation_sources: scan` on `/scan` with
  `marking: True`, `clearing: True` on both costmaps — correct.
- **Recovery behaviours driving blind.** There is no `behavior_server` in the
  lifecycle set at all, so Spin/BackUp cannot run. The `N recoveries` count in
  the status monitor is BT retries, not motion.
- **Mislocalisation at the start.** The planner's first plan succeeded and the
  robot moved; `Starting point in lethal space!` only began 3.9 s later, so the
  robot drove *into* lethal space rather than starting in it.

### The cause: costmap refresh far slower than control

```
controller_frequency:            20.0 Hz   -> MPPI re-plans every 50 ms
local_costmap update_frequency:   2.0 Hz   -> obstacles refresh every 500 ms
```

MPPI re-planned 20 times a second against obstacle data up to **500 ms stale**.
At 95 % speed (0.43 m/s) that is **0.215 m travelled blind** between updates,
plus the robot's own ~0.14 m half-length: an obstacle could be ~0.35 m closer
than the costmap believed. With `r_min` falling to 0.30 m in the corridor, a
collision is unavoidable no matter how correct the footprint is. Nav2's default
is 5 Hz; 2 Hz is far too slow for this speed.

Now `10.0 Hz` local (blind distance 0.043 m) and `2.0 Hz` global (was 0.5 Hz,
which lagged the planner's obstacle view by 2 s). The container has 20 cores at
~0.6 load, so the cost is irrelevant.

The envelope's speed ceiling also used **pure stopping distance** with no
reaction term, which is why it cleared 95 % speed with an obstacle 0.62 m away.
It now solves `s = v*t + v^2/(2a)` with `sense_latency_s: 0.15`:

| `r_min` | speed cap before | after |
|---|---|---|
| 0.40 m | 52 % | 26 % |
| 0.50 m | 95 % | 87 % |
| >= 0.62 m | 95 % | 95 % |

### Measured geometry (worth knowing)

From live TF with the arm verified in the **carry** pose:

```
base_link -> elbow    [0.086, 0.000, 0.326]
base_link -> wrist    [0.018, 0.000, 0.452]
base_link -> gripper  [0.003, 0.012, 0.493]
```

The "carry" pose points the arm almost **straight up** — its entire horizontal
extent is within |X| <= 0.086, |Y| <= 0.012 of base_link. The carry polygon
claims a bulge out to **Y = +0.303**, so it over-claims by ~0.29 m. That is
conservative (it cannot cause a collision) but it is why the carry footprint
struggles in corridors: 0.513 m of claimed width for a robot that is really
~0.42 m wide with the arm up. Re-measuring carry with
`measure_footprint` would buy back real clearance.

Wheel frames are also **asymmetric** in the URDF:

```
front_left  [ 0.085,  0.153]   rear_left  [-0.085,  0.153]
front_right [ 0.085, -0.098]   rear_right [-0.085, -0.098]
```

so `base_link` is not at the geometric centre (offset ~0.0275 m). The symmetric
+-0.210 footprint over-covers both sides, so this is harmless for navigation,
but do not trust the URDF for real dimensions — measure.

## Every run records itself

`demo_launch.py` now creates one directory per run and puts everything in it.
No separate recording step, and nothing to remember to start.

```
logs/run_20261001T163849/
  console.log        full interleaved console stream (run_demo.sh only)
  bag/               rosbag2, zstd-compressed, 21 topics
  node_logs/         one log file per node
  config/            demo_params.yaml, default_tuner_config.yaml,
                     envelope_constants.json, the map yaml + pgm
  run_info.txt       timestamp, workspace, what was kept
```

```bash
./scripts/run_demo.sh                  # recommended: adds pre-flights + console.log
./scripts/run_demo.sh corridor_test    # names the run directory
ros2 launch mirte_navigation demo_launch.py                 # bag still recorded
ros2 launch mirte_navigation demo_launch.py record:=false   # no bag
```

Why each piece exists:

- **`node_logs/`** — `ROS_LOG_DIR` is pointed at the run directory, so amcl,
  planner_server, controller_server and online_causal_tuner are *separate
  files* instead of interleaved in one terminal. This is what made the
  transport, inflation and costmap-rate problems findable. Launch's own
  `launch.log` still goes to `~/.ros/log`, because launch reads `ROS_LOG_DIR`
  before the launch description runs.
- **`config/`** — a bag is uninterpretable months later without the parameters
  that produced it. Footprint, inflation, speed caps and the envelope constants
  have all changed between runs; the snapshot pins them to the data.
- **`bag/`** — carries `/risk_state` and `/tuner_decision` together, so the
  risk observed and the config applied on that tick are paired and the models
  can be retrained on Mirte data offline.

### Two things that will bite if you change this

**Stop runs with Ctrl-C, not `kill`.** rosbag2 finalises the bag on SIGINT:
you get `bag_0.db3.zstd` plus `metadata.yaml`, and `ros2 bag info` works. On
SIGTERM it does not — the file is left uncompressed with no `metadata.yaml` and
is unreadable until `ros2 bag reindex`.

**`--include-unpublished-topics` is load-bearing.** Recording starts at t=4 s,
before the tuner (t=12 s) exists. Without that flag rosbag2 silently drops
every name in the list that has no publisher yet — `/tuner_decision`,
`/speed_limit`, `/plan` — and the bag comes back missing precisely the topics
worth recording, with no warning. Verified: those topics now appear with
`Count: 0` rather than being absent.

`scripts/record_demo_bag.sh` still works for recording against an
already-running stack, but is no longer needed for the normal path.

### Run directories are owned by root

The container runs as root while the host user is `forough`, so everything the
launch writes under `logs/` comes out root-owned and cannot be deleted or edited
from the laptop without sudo (same as `build/`, `install/` and `log/`). Clean up
from inside the container:

```bash
docker exec ros2-tiago-mirte-cpu bash -lc \
  'cd /home/forough/phd_projects/mirte_demo && rm -rf logs/run_2026*'
```

Reading them from the host is fine; only writing and deleting need the
container.

### "A message was lost!!!" breaking the helper scripts

Symptom, seen in the demo launch once bag recording was added:

```
[bash-15] Traceback (most recent call last):
[bash-15]   File "<stdin>", line 2, in <module>
[bash-15] ast.literal_eval ... <unknown>, line 1
[bash-15]     A message was lost!!!
[bash-15] SyntaxError: invalid syntax
[bash-15]   attempt 1/4:
```

`ros2 topic echo` writes its own warnings to **stdout**, interleaved with the
message, so `... --field name | head -1` captured the warning text instead of
the value and the parser died. The arm then looked unreachable when it was
fine. Both `set_arm_pose.sh` and `check_arm.sh` now select the line that
actually looks like data (`^[` or `^array`) and re-read up to three times, in
case the single message requested was the one dropped.

The warning itself is DDS message loss on `/joint_states`, and it started
appearing because the bag recorder is an additional subscriber on these topics.
It is a warning from `echo`'s own subscription, not evidence that nav2 lost
data — but if navigation starts behaving oddly only when `record:=true`, that
is the first thing to suspect. The recorder already runs with
`--max-cache-size 50000000` to keep write bursts off the subscription path.

Never parse `ros2 topic echo` output with `head -1`.

## It accelerated into the obstacle (the decisive bug)

Found from `logs/run_20261001T164711/bag` with `scripts/analyze_run.py` — not
visible in any log file. Three measurements:

```
peak commanded |v| = 0.640 m/s   (vx 0.427 + vy 0.477)
peak achieved  |v| = 0.350 m/s   (~55% of command, all run)
t=1.29s: wz_odom = -1.017 rad/s against wz_cmd = +0.408,
         |v|odom collapses 0.350 -> 0.035 in 0.21 s   <- the impact
```

Three compounding faults:

**1. The envelope assumed the robot brakes 2.24x harder than measured.**

```python
decel_phys = max(self.decel_limit_mps2, 1.20)   # calibrated value is 0.5355
```

This one line cleared 95 % speed with an obstacle 0.68 m away: available
braking distance was 0.304 m, the ceiling came out 0.693 m/s and the command
was 0.665 m/s, so it passed. With the real 0.5355 m/s^2 the ceiling is
0.496 m/s and that command is rejected. The override is gone; the calibrated
value is used directly (clamped only to keep the sqrt defined).

**2. `vy_max` exceeded `vx_max`** (0.5 > 0.45), so the optimiser could put more
speed sideways than forward and crab diagonally at 0.640 m/s — while the
envelope's speed ceiling, which compares only the *forward* component against
`MAX_VX_LIMIT`, believed the robot was capped at 0.45. **Keep `vy_max <=
vx_max`.** This is the mechanism behind "it moves from the sides and runs into
things".

**3. The limits exceeded what the base can do.** Commanded 0.63 m/s, achieved
0.35 m/s, every tick. That is not harmless: MPPI rolls out trajectories
assuming the command is met, the robot falls 45 % short, and the realised path
diverges from every plan — which is how it ends up off-path and against a wall.
Measured acceleration was 1.18 m/s^2 while `ax_max`/`ay_max` were set to 3.0.

Now sized so the **combined** command stays inside capability —
`hypot(0.30, 0.20) = 0.36` against a measured peak of 0.35:

| | was | now |
|---|---|---|
| `vx_max` | 0.45 | 0.30 |
| `vy_max` | 0.50 | 0.20 |
| `ax_max` / `ay_max` | 3.0 | 1.2 |
| `decel_phys` | `max(cal, 1.20)` | calibrated 0.5355 |

Resulting speed caps, all achievable:

| `r_min` | allowed | commanded `\|v\|` |
|---|---|---|
| <= 0.50 m | 30 % | 0.11 |
| 0.60 m | 50 % | 0.18 |
| 0.68-0.80 m | 70 % | 0.25 |
| >= 1.00 m | 95 % | 0.34 |

### Lesson

Logs showed `Optimizer fail`, `Starting point in lethal space!` and 19
recoveries — all *downstream* of a robot that was commanded 0.64 m/s, managed
0.35, and could not stop in the distance it had. **None of that was visible
without the bag.** Record every run.

```bash
python3 scripts/analyze_run.py logs/run_<stamp>/bag
```

`MAX_VX_LIMIT = 0.7` in `online_tuner_node.py` is still a stale Tiago value and
no longer matches `vx_max`. It is what `speed_limit_pct` is scaled against, so
the percentages above are nominal; it coincidentally approximated the old
combined speed. Worth reconciling, but changing it also moves `e_speed` and
`j_progress`, so it was left alone for now.

## Inflation was MAXIMUM in the tightest spaces (inverted)

Symptom: the run no longer collides, but it wedges and aborts, and the ticks
show the opposite of what the envelope is supposed to do:

```
TUNER TICK #656  r_min=0.30  r_width=0.61  Inflation: 0.30m   <- max, in the tightest spot
```

Cause: A(R) returns an **empty** feasible set there. With `r_width = 0.61` below
`need_width = 0.421 + 2*0.1063 = 0.6336`, gate 1 judges that even the tucked
footprint does not fit, so every candidate is rejected and the fallback fires:

```python
if not feasible_candidates:
    feasible_candidates = [c for c in self.candidates_df
                           if c[FOOTPRINT_KEY] == 0.0 and c[SPEED_LIMIT_KEY] == MIN_SPEED_LIMIT]
```

It constrained footprint and speed but left **inflation free**, and the
envelope-only selection prefers the *largest* admissible inflation — so exactly
when the robot is wedged and needs free cells, it got 0.30 m. The free channel
was `0.61 - 2*0.30 = 0.01 m` for a robot 0.421 m wide: completely closed. That
is the 20 recoveries and the aborted goal.

Fixed: the fallback is now flagged (`_envelope_infeasible`) and the inflation
preference **flips to the smallest** value in that state, with the reason logged
as `ENVELOPE_INFEASIBLE (wedged: min footprint/speed/inflation)`. A `0.17` rung
was added to the grid for this case only — the lowest value still above the
padded inscribed radius of 0.150, so the inflation layer keeps marking the band
the robot's body occupies. It is unreachable through the normal path because the
`r_min < 0.80` rule requires `inflation >= inflation_floor_m` (0.20).

Resulting behaviour — down when tight, back up when open, as intended:

| `r_min` | state | inflation | free channel |
|---|---|---|---|
| < 0.317 m | WEDGED | **0.17** | 0.26 m |
| 0.32-0.55 m | admissible | 0.20 | 0.24-0.70 m |
| 0.58 m | admissible | 0.25 | 0.66 m |
| >= 0.62 m | admissible | **0.30** | 0.64 m+ |

In the failed run (`r_min` median 0.31, i.e. WEDGED) this changes the free
channel from **0.01 m to 0.27 m**.

### Two limits on what `r_min` can tell you

`scan_self_filter_radius_m = 0.30` in the risk node discards every return
closer than 0.30 m, so **`r_min` can never read below 0.30** — it is a floor,
not a measurement. That filter is correct and necessary: the scan contains
returns at 0.249 m, which match the wheel corners
(`hypot(0.135, 0.173) = 0.219` plus wheel width). But it means the envelope
cannot distinguish "0.30 m away" from "touching", and `r_min = 0.30` should be
read as "at or inside the sensing floor".

`r_width` is **not an independent measurement**. The raw value is unusable (it
read 0.30 in the same ticks, below `r_min`, which is geometrically impossible),
so the repaired value is `max(raw, 2*r_min)` and in practice always resolves to
`2*r_min`. Every geometric decision is therefore driven by `r_min` alone. A real
corridor-width estimator in the risk node is the single biggest improvement
available to the envelope.

## Nav2 rejects a goal and the robot never moves

First check whether a plan was ever produced. In the bag:

```
Topic: /plan     Count: 0
Topic: /cmd_vel  Count: 0
```

Zero of both means the robot was never told to move, so this is a **planning**
problem, not a control or tuner problem. The planner log says which kind:

- `Starting point in lethal space!` -> the robot's own pose is in collision
  (see the inflation/inscribed-radius section).
- `no valid path found` -> the goal is unreachable. Diagnose it offline:

```bash
python3 scripts/check_goal.py --robot 0 0 --goal 2.21 3.54
python3 scripts/check_goal.py --robot 0 0 --goal 2.21 3.54 \
    --footprint "[[0.140,0.210],[0.140,-0.210],[-0.140,-0.210],[-0.140,0.210]]"
```

The `--footprint` override answers "would a smaller footprint reach it?" without
touching the running system.

### The lab map is the limiting factor

`lab_map.pgm` is **77.3 % unknown**:

```
map 329x275   free=18747   occupied=1783   UNKNOWN=69945 (77.3 %)
```

and from (0, 0) the robot can reach only **2.5 m2** of the 46.9 m2 of known-free
space, because the map breaks into 9 disconnected passable components at the
carry footprint's circumscribed radius. Two failure modes follow, and both were
seen on 2026-10-02:

| goal | clearance | verdict |
|---|---|---|
| (1.15, 3.18) | 0.45 m | fine, but a **different component** -- no legal path with `allow_unknown: false` |
| (2.21, 3.54) | 0.32 m | **below** the 0.332 m circumscribed radius -- no valid pose exists |
| (2.15, 3.54) | 0.28 m | likewise too tight (this is the goal that "worked" on 10-01; the first plan succeeded only because `tolerance: 0.25` let the planner aim at a nearby pose, and the robot then drove into the wall) |

So Nav2 was right to refuse both. Nothing in the configuration caused it: the
map is byte-identical to the run that navigated (same md5), and the active
inflation was 0.20 in both.

**For a demo today, send a goal inside the reachable region.** `check_goal.py`
lists them; `(1.87, 0.45)` has 0.65 m clearance and is 1.92 m away, which is far
enough to show the tuner adapting.

**To use the far side of the lab, re-map** (section 4). Drive through the
connecting corridor with SLAM so it is recorded as free rather than unknown.
Raising `allow_unknown` is not the answer -- it was disabled precisely because
it routed paths through unmapped walls.

The carry footprint also costs reachable area for nothing: its circumscribed
radius is 0.332 m, but the carry pose holds the arm vertical (measured
`base_link -> gripper = [0.003, 0.012, 0.493]`), so the real shape is close to
tucked. Re-measuring it raises the reachable region from 2.5 to 3.5 m2 and
admits goals down to 0.252 m clearance -- worth doing, though it does not by
itself connect the components.

## The bag now carries the tuner's reasoning

Two additions, because a bag that records what the robot did but not why is
only half useful:

**`/rosout`** -- every node's log messages, timestamped in the bag alongside
`/scan`, `/tf` and `/cmd_vel`. `node_logs/` holds the same text, but only the
bag keeps it aligned with the data. Verified: 855 messages from 14 nodes in a
32 s run, including the tuner's `TUNER TICK` lines.

**`/tuner_decision`** already published the full per-tick trace as JSON; it now
also carries the geometry each decision was derived from:

```
r_min, r_width, r_width_raw, selection_reason, arm_verified, d_goal,
envelope_infeasible
```

Without those the bag could not explain *why* a config was chosen. `r_width` in
particular is a repaired value (`max(raw, 2*r_min)`) that usually differs from
the raw measurement, so both are kept.

Reading them back:

```python
# /rosout -> rcl_interfaces/msg/Log  (fields: name, msg, level)
# /tuner_decision -> std_msgs/msg/String containing JSON
```

Note `/tuner_decision` only appears once per tick and the tuner gates its loop
when idle, so a run with no goal sent yields a single message. That is the idle
gate, not a fault.

## "It worked yesterday for the same goal"

It did, and the goal was not the problem. Yesterday's run reached
`(2.15, 3.49)` with **2443** `/plan` messages; today `(2.21, 3.54)` produced
**zero**. The difference is where the robot physically was:

| | 2026-10-01 (worked) | 2026-10-02 (failed) |
|---|---|---|
| first `/scan` min range | 0.249 m | **0.592 m** |
| beams < 0.5 m | 2 | **0** |
| free cells near (0,0) | 21 % | 31 % |

Both runs set `set_initial_pose: true` with `initial_pose {0,0,0}`, so AMCL was
pinned to the map origin either way -- but the laser saw a completely different
environment, which means the robot was **not standing where AMCL was told it
was**. The obstacle layer then paints walls that contradict the static map and
seals the start region, and the planner reports `no valid path found`. This is
the same failure recorded in the localization note: a wrong AMCL pose shows up
as a planner failure, never as a localization error.

**Before each run, make the scan agree with the map.** Either place the robot
physically at the pose the map was recorded from, or give it a `2D Pose
Estimate` in RViz and confirm the laser scan lies on top of the mapped walls
before sending a goal. If the scan is visibly rotated or offset from the walls,
no amount of tuning will help.

### `check_goal.py` is too conservative -- do not trust its verdict alone

It reports `(2.15, 3.49)` as "too tight, no valid pose there", yet that goal
navigated fine. It tests only the exact goal cell against the **circumscribed**
radius, while Nav2's `SmacPlannerLattice` has `tolerance: 0.25` (it will aim at
a nearby valid pose) and checks the real footprint per orientation, which fits
where a circumscribed circle does not. Use it to compare candidate goals and to
spot disconnected components, not as proof that a goal is unreachable.

## Why navigation keeps failing on easy goals: the pose estimate

The goals were never the problem. A wrong initial pose does not report itself as
a localisation error -- it surfaces as a *planner* failure, because the obstacle
layer paints walls that contradict the static map:

```
planner_server: GridBased: failed to create plan, no valid path found.
planner_server: ... Starting point in lethal space!
```

Measured across the demo runs, scan-to-map match at the assumed pose predicts
the outcome exactly:

| run | match at assumed pose | best offset | result |
|---|---|---|---|
| 2026-10-01 165624 | **73 %** | -5 deg | 2443 `/plan` msgs, navigated |
| 2026-10-02 081501 | 36 % | -10 deg | **0** plans, goal rejected |
| 2026-10-02 082049 | 34 % | -10 deg | **0** plans, goal rejected |
| 2026-10-02 083956 | 34 % | -10 deg | **0** plans, goal rejected |

Nothing in the configuration changed between them -- the map is byte-identical
and the inflation was 0.20 throughout. **The robot was placed about 10 degrees
rotated** from where it stood on 10-01. A 10 deg heading error over 3 m of
travel is roughly 0.5 m of lateral error, which is enough to put the footprint
inside a mapped wall.

`set_initial_pose: true` with `initial_pose {0,0,0}` makes this easy to hit: AMCL
is *told* it is at the origin facing 0 deg and starts with a tight covariance,
and because `update_min_d: 0.05` only updates on motion, it never gets the
movement needed to recover before the planner gives up.

### Check it before every run

```bash
python3 scripts/check_localization.py                       # live, robot only
python3 scripts/check_localization.py --bag logs/run_<stamp>/bag
```

It projects each scan endpoint into the map at the assumed pose, counts how many
land on a mapped wall, then searches nearby poses for a better match.

```
>70 %   well localised, go ahead
40-70 % marginal
<40 %   the robot is not where Nav2 thinks
```

If it reports a better pose at some offset, rotate or move the robot to match,
or give a `2D Pose Estimate` in RViz, and re-check before sending a goal.

**The laser rotation is load-bearing.** The composed `base_link -> laser`
transform is **+90 deg** (`base_link -> frame_link` +90, `-> lidar_base` -90,
`-> laser` +90). My first version of this check ignored it and scored a
well-localised robot at 5 %, which looked like catastrophic mislocalisation and
was simply a missing transform. Any script that projects `/scan` into the map
must apply it.

The search window is +-1 m in 0.25 m steps; when the winning offset sits on that
boundary the real error is larger, and the tool says so.

## Every goal rejected, robot refuses to move: fix the pose automatically

Signature -- the goal is irrelevant, every goal fails the same way:

```
planner_server: ... Starting point in lethal space!     (x28, two different goals)
```

`Starting point in lethal space` means the **robot's own footprint** is inside a
mapped wall, so no plan can exist from where Nav2 believes it is standing.

Measured on 2026-10-02: AMCL was told `(0, 0, 0 deg)` by `set_initial_pose`,
while scan matching put the robot at **(0.42, 1.35, 142 deg)** -- 1.4 m and
142 deg away. Scan-to-map match was **11 %** at the assumed pose and **95 %** at
the true one, confirmed by two independent methods.

### The fix: let the robot find itself

```bash
python3 scripts/set_initial_pose_from_scan.py            # locate and publish
python3 scripts/set_initial_pose_from_scan.py --dry-run  # locate only
python3 scripts/check_localization.py                    # verify
```

It brute-force matches the scan against the map (candidate positions = mapped
free cells, 10 deg yaw steps, then two refinement passes) and publishes the
winner to `/initialpose`. The whole search takes ~0.2 s. It refuses to publish a
weak match rather than asserting a wrong pose, and reports the mean
endpoint-to-wall distance so the quality is visible (<0.12 m is good; the
2026-10-02 fix scored 0.024 m).

`demo_launch.py` now runs this automatically at t=7 s, once AMCL is active and
before the tuner starts:

```
ros2 launch mirte_navigation demo_launch.py                          # on by default
ros2 launch mirte_navigation demo_launch.py auto_initial_pose:=false # opt out
```

So the robot no longer has to be placed anywhere in particular. If you prefer to
set the pose by hand in RViz, pass `auto_initial_pose:=false` and use `2D Pose
Estimate`, then verify with `check_localization.py` before sending a goal.

### Caveat

Scan matching needs distinctive geometry. In the middle of a long featureless
corridor several poses score alike, and the match can be confidently wrong --
which is why the script refuses to publish above 0.25 m mean error. A corner or
a doorway gives a much stronger fix than open floor.

## The arm comes back out: what the demo now shows

The run has a shape an industrial audience can read without explanation.

| phase | clearance (`r_min`) | arm | speed | inflation |
|---|---|---|---|---|
| idle, before the goal | any | **carry** (held) | - | - |
| open floor | >= 1.00 m | **carry** | 95 % | 0.30 |
| narrowing | 0.65-0.80 m | **carry** | 50-70 % | 0.25-0.30 |
| tight | 0.45-0.55 m | **tucked** | 30-50 % | 0.20 |
| wedged (A(R) empty) | < 0.32 m | tucked | 30 % | **0.17** |
| arrived | any | **carry** (restored) | - | - |

So: the robot sets off carrying, retracts the arm only where the space demands
it, opens the costmap up when it gets wedged, and puts the arm back out as soon
as it fits again -- and unconditionally on arrival.

### Why the arm never came back before

`_software_distance` was the **primary** sort key in the envelope-only
selection. `ENVELOPE_ONLY_SOFTWARE` asks for `speed_limit_pct = 100` and the
tucked footprint carries a higher speed ceiling, so tucked always scored closer
to 100 and carry was never re-adopted: the arm retracted once and stayed
retracted for the rest of the run. The footprint is now the primary key, so
carry wins whenever A(R) admits it. That is safe because A(R) already refuses
carry unless the space genuinely fits it (`r_width >= width + 2*delta +
envelope_hysteresis` and `r_min >= carry_adopt_r_min` = 0.65 m); the only cost
is the lower speed ceiling that comes with the larger envelope, which is the
right trade.

### Restoring carry on arrival

Arrival is detected on `/navigate_to_pose/_action/status` (`STATUS_SUCCEEDED =
4`); the feedback distance merely stops updating, which is indistinguishable
from a stall. On arrival the robot is stationary, so extending is safe however
tight the goal is -- and because the carry pose actually holds the arm
**vertical** (measured `base_link -> gripper = [0.003, 0.012, 0.493]`), it
sweeps almost nothing. A new goal (`STATUS_EXECUTING = 2`) hands control back to
the envelope.

One trap worth knowing: the idle gate had to be exempted for this. On arrival
the action feedback stops, so `d_goal` goes `None` within ~2 s while the robot
is stationary -- the gate fired before the tucked -> carry switch (2 confirming
ticks plus the switch dwell) ever reached the arm, leaving the robot tucked at
the goal. The gate now keeps ticking while `_goal_reached` and the arm is not
yet verified in carry.

### Lines to point at during the demo

```bash
grep -E "Decision: arm|GOAL REACHED|ENVELOPE_INFEASIBLE" logs/run_<stamp>/console.log
grep "TUNER TICK" logs/run_<stamp>/console.log | tail -20
```

Each tick prints the clearance it measured and every knob it set, so the
decision chain is visible live:

```
[TUNER TICK #N] d_goal=2.41m | Arm: target='tucked' (verified='tucked')
  | Speed: 30% | Inflation: 0.20m | r_min=0.48 r_width=0.96 (raw 0.75)
  | Reason: ENVELOPE_ONLY_ABLATION
```

## Aborted with no collision: a phantom footprint and no recovery server

Two independent faults, both fixed on 2026-10-02.

### 1. Nav2 had no recovery behaviours at all

`minimal_navigation_launch.py` started map_server, amcl, planner_server,
controller_server and bt_navigator -- but **no `behavior_server`**. The
behaviour tree duly called `Spin`, `BackUp`, `DriveOnHeading` and `Wait`, found
no action server, failed instantly, and the goal was aborted the moment the
footprint touched a lethal cell. The "20 recoveries" in the status monitor were
BT retries against nothing, which is why the robot never backed off and never
continued.

`behavior_server` is now launched and registered with the lifecycle manager,
with a parameter block in `demo_params.yaml`. Verified: `/backup`, `/spin`,
`/drive_on_heading` and `/wait` all appear in `ros2 action list`, and
`Managed nodes are active`. `simulate_ahead_time: 2.0` makes the behaviours
check the costmap before moving, so a recovery cannot drive blind, and the
rotational limits are matched to the base's measured capability rather than the
nav2 defaults.

### 2. The costmap kept the CARRY envelope around a tucked robot

The two footprints are genuinely different, and that difference is the point of
the demo:

| | lateral span | circumscribed radius |
|---|---|---|
| tucked | 0.421 m | 0.253 m |
| carry (side-extended) | **0.513 m** | **0.332 m** |

What went wrong is not the polygon but *which* polygon was in force.
`_envelope_label()` returns the **larger** of the physical and target arm states,
so the costmap grows before the arm extends -- correct while the arm is actually
moving. But with the arm stuck, `target='carry'` and `verified='tucked'` persist
forever (visible in the bag as `arm=carry/tucked`), so a robot that is
verifiably tucked carries a footprint 0.092 m wider than itself. That phantom
margin entered lethal cells and aborted runs in which nothing was touched.

Fixed: once the arm is declared unresponsive (`MAX_ARM_RETRIES` exceeded),
`_envelope_label()` trusts the verified physical label instead of growing to an
envelope the arm will never reach, and says so:

```
Arm unresponsive: using the VERIFIED 'tucked' envelope instead of growing to
'carry'. The costmap now matches the robot's real shape.
```

While the arm is healthy the grow-before-extend rule is unchanged.

### A measurement mistake worth recording

I briefly "corrected" the carry polygon to the base rectangle on the strength of
two `tf2_echo` reads -- gripper at `[0.003, 0.012, 0.493]` in verified carry and
`[0.002, 0.013, 0.492]` in tucked -- concluding that the carry pose folds the arm
vertically and adds nothing. **That was wrong and is reverted.** The two readings
cannot both be live: the gripper sits 0.077 m off the vertical `shoulder_pan`
axis, so swinging pan through 1.51 rad must move it 0.106 m, and those readings
differ by 0.001 m. `robot_state_publisher` was not running reliably, so one read
was stale.

Lesson: **`tf2_echo` output is not evidence unless the publisher is confirmed
live.** Check `ros2 node list | grep state_publisher` first, and prefer
`scripts/measure_footprint` -- which commands the pose, waits, and verifies
against `/joint_states` -- over reading transforms by hand.
### Re-verify the carry polygon when the robot is back

The polygon in `ARM_CONFIGS["carry"]` was measured on 2026-09-29 with the arm in
the side-extended pose. If the arm pose has changed since, re-measure rather
than reasoning from transforms:

```bash
./scripts/set_arm_pose.sh carry        # verified against /joint_states
python3 -m rct_collector.measure_footprint --base-frame base_link \
    --link-filter "(shoulder|elbow|wrist|gripper)"
```

It prints ready-to-paste `ARM_CONFIGS` and `FOOTPRINT_GEOMETRY` blocks.

## The envelope reasoner now trades arm / speed / inflation

Previously it could not. The speed ceiling used a single hardcoded
`r_front = 0.27` for **both** footprints, so carry and tucked cleared exactly the
same speed; `v_lat` never binds (`v_stop` is always tighter), so tucking bought
nothing and the arm decision collapsed onto the hand-set `carry_adopt_r_min`
threshold.

Clearance is measured from the laser, so the robot's own extent must come off
it -- and that extent depends on the arm. `r_front` is now the candidate
footprint's own circumscribed radius (tucked 0.253, carry 0.332), so carry has
0.079 m less room in which to stop:

| `r_min` | v_ceil tucked | v_ceil carry | reasoner picks |
|---|---|---|---|
| 0.45 m | 0.24 | **0.06** | tucked -- carry would be 4x slower |
| 0.55-0.70 m | 0.38-0.53 | 0.27-0.46 | tucked, 50-70 % |
| 0.80 m | 0.61 | 0.55 | **carry** -- both clear 70 %, tie goes to arm-out |
| >= 1.00 m | 0.75 | 0.70 | carry, 95 % |

Selection order is **progress first, arm second**: `_software_distance` is
dominated by the speed term, so the candidate that gets closest to full speed
wins, and only then is an extended arm preferred. That is what makes it a
choice. Making the footprint the primary key (as it briefly was) forces carry
whenever it is admissible and removes the trade-off entirely -- do not do that.

So all three knobs now move together for the same reason: in a tight space the
reasoner tucks the arm **and** drops the speed **and** lowers the inflation,
because each one buys passability; in the open it extends the arm and takes the
speed back.

## The arm can hit things the costmap cannot see

```
laser (sensing plane)    z=0.107  ####
shoulder_pan             z=0.160  ######
elbow                    z=0.326  #############
wrist                    z=0.452  ##################
gripper                  z=0.492  ###################
gripper_finger_l         z=0.535  #####################
```

**Every part of the arm sits above the laser plane.** The costmap is a 2D slice
at 0.107 m, so an obstacle that exists only higher up -- a table edge, a shelf, a
stacked box, any overhang -- is simply not in the costmap. No footprint, however
accurately measured, can protect the arm from an obstacle that was never
observed. If the arm strikes something the base would have driven past cleanly,
this is the first thing to check, not the footprint.

Mitigations, in order of effort:

1. **Demo placement (today).** Use obstacles with a profile at 0.107 m -- boxes
   standing on the floor, not tables or shelves. Then the laser sees what the
   arm will meet and the carry footprint does its job.
2. **Keep the arm in only where it is tight.** Already the behaviour above: the
   arm extends only once there is >= ~0.80 m of clearance.
3. **Feed the depth camera into the costmap (the real fix).** The TF tree already
   carries `camera_rgb_frame` / `camera_depth_frame` / `camera_depth_optical_frame`,
   so the hardware is there. A `nav2_costmap_2d::VoxelLayer` or a second
   `observation_sources` entry taking the depth cloud with
   `max_obstacle_height` ~0.6 m would put arm-height obstacles into the costmap.
   That is the only approach that makes the extended footprint meaningful in a
   cluttered industrial cell, and it is a post-demo task.

## Tuner saturated in narrow spaces ("still stuck")

Symptom: the tuner *is* adapting -- speed 70 -> 50 -> 30 %, inflation
0.30 -> 0.25 -> 0.20 -> 0.17 -- but it bottoms out and the robot still cannot
pass. The controller says:

```
controller_server: Optimizer fail to compute path   (x14)
```

That is MPPI finding no acceptable trajectory, not a planner failure. Two causes,
both of which took a knob away from the tuner in exactly the hard case.

### 1. Inflation was keyed to `r_width`, the untrustworthy channel

```
TUNER TICK #283  r_min=0.32  r_width=1.16 (raw 1.16)  ->  Inflation: 0.25m
```

The ceiling came only from `lateral = (r_width - width)/2 - delta`. With a tight
gap on one side and open space on the other, `r_width` read 1.16 m while the
nearest obstacle was 0.32 m away, so 0.25 m of inflation was admitted -- and the
robot's own centre then sits inside the inflated band around that near obstacle.
Every MPPI sample is high-cost and the optimiser fails.

Inflation is now also bounded by `r_min - clearance_margin`, which keys it to
the measurement that is actually reliable:

| `r_min` | inflation before | after |
|---|---|---|
| 0.30 m | 0.20 | **0.17** (wedged) |
| 0.31-0.32 m | **0.25** | **0.20** |
| 0.39-0.45 m | 0.20 | 0.25 |
| 0.71 m | 0.30 | 0.30 |

The old `r_min < 0.80 -> inflation >= inflation_floor_m (0.20)` rule was also
backwards: it blocked the 0.17 rung in precisely the tight spaces that rung
exists for. Replaced by `INFLATION_HARD_FLOOR_M = 0.155`, the real constraint
(the footprint's padded inscribed radius, below which the inflation layer stops
marking the robot's own body band).

### 2. CostCritic was pinned above nav2's default in constrictions

Gate 4 required `CostCritic >= 5.0` whenever `margin < 0.30` or `r_min < 0.80`.
nav2's own default is **3.81**, so in a corridor -- where every cell is inflated
and the robot must tolerate passing close to move at all -- the tuner was forced
to weight obstacle cost *higher* than anywhere else, and lost the knob. With the
arm already tucked, speed at 30 % and inflation at the floor, there was nothing
left to give.

Relaxed to 3.81. Safety does not rest on this weight: `consider_footprint: true`
means MPPI still rejects any trajectory whose actual footprint collides; the
weight only decides how close it is willing to pass. `PathAlign` keeps its floor
of 15.0, because staying on the planned line through a corridor is genuinely
wanted.

Net effect at the stuck tick (`r_min = 0.32`):

| knob | before | after |
|---|---|---|
| arm | tucked | tucked |
| speed | 30 % | 30 % |
| inflation | 0.25 | **0.20** |
| CostCritic | 5.0 (forced) | **3.81** |
| PathAlign | 15.0 (forced) | 15.0 |

## TF_NAN_INPUT on the arm frames

```
Error: TF_NAN_INPUT: Ignoring transform for child_frame_id "shoulder_pan" ...
       because of a nan value in the transform (nan nan nan) (nan nan nan nan)
Error: TF_DENORMALIZED_QUATERNION: Ignoring transform for child_frame_id "wrist" ...
```

Reported by every node with a TF listener (rviz2, risk_state_node,
online_tuner_node) because they all consume the same broken transforms. The
cause is upstream of all of them -- from the bag:

```
/joint_states, 153 messages:
  shoulder_lift_joint   nan   (153/153)
  wrist_joint           nan   (153/153)
  shoulder_pan_joint    nan   (153/153)
  elbow_joint           -1.9167    <- the only arm joint reporting
```

Three of the four arm servos report NaN instead of a position. That is a servo
or driver fault, not a software problem: **power-cycle the arm.**

### Why it matters more than it looks

NaN fails every comparison, so `_verified_arm_label()` returns `None`, and
`_envelope_label()` then holds the **CARRY** envelope for the entire run. The
robot is given an oversized footprint it can never shed, it stops fitting
through gaps it would otherwise clear, and nothing in the nav logs points at the
arm. Previously this was completely silent; it now reports itself:

```
ARM JOINTS REPORTING NaN: shoulder_lift_joint, shoulder_pan_joint, wrist_joint.
The arm state cannot be verified, so the costmap is held at the CARRY envelope
and the arm decision is disabled. This is a servo or driver fault --
POWER-CYCLE THE ARM.
```

Check directly with:

```bash
ros2 topic echo /joint_states --once --field position
```

`nan` for any of `shoulder_pan` / `shoulder_lift` / `elbow` / `wrist` means the
arm cannot be trusted and the footprint will stay oversized.

## auto_initial_pose ran but published into the void

Symptom: goals refused with `no valid path found`, `/amcl_pose` containing a
single message at `(0, 0, 0)`, and scan-to-map match 42 % when the true pose was
10 degrees away -- yet `set_initial_pose_from_scan.py` had apparently run fine.

Two faults, both mine:

1. **The pose was published before anyone was listening.** The script is
   short-lived: it created the publisher and sent immediately, so DDS discovery
   had not yet matched AMCL's subscription and the message was dropped. The
   script then exited reporting success. It now waits for
   `get_subscription_count() > 0` (up to 15 s), republishes up to six times, and
   **confirms against `/amcl_pose`** before claiming the pose was applied --
   exiting non-zero if not.

2. **Its output was invisible.** It used only `print()`, which goes to the
   launch console -- not captured unless launched via `run_demo.sh`. In
   `node_logs/` it left an empty file, and it never appeared in the bag's
   `/rosout`, so the failure could not be seen after the fact. It now mirrors
   its findings to the ROS logger.

Verified: with no AMCL running it now fails loudly and returns 1 --

```
[ERROR] [pose_from_scan]: no subscriber on /initialpose after 15s -- is amcl up?
                          pose NOT applied
```

### The TF-NaN spam is now filtered from the console

`auto_initial_pose` has nothing to do with these errors -- they come from the
robot, not the launch file. The chain is:

```
arm servo driver  ->  nan in /joint_states  ->  robot_state_publisher  ->  nan TF
```

and `robot_state_publisher` runs **on the robot**, not in the container
(`ps -eo cmd | grep robot_state_publisher` inside the container returns
nothing), so nothing in `demo_launch.py` can prevent it.

**Navigation is not affected.** "Ignoring transform" means tf2 discards those
frames, and only arm links are involved. Verified while the NaN was active:

```
odom      -> base_link   OK
base_link -> laser       OK  [0.001, -0.000, 0.107]
```

tf2 prints these from `buffer_core.cpp` straight to stderr, not through the ROS
logger, so no `--log-level` can quieten them -- and every node with a TF
listener repeats the pair for every bad frame, which buries the tuner output.
`run_demo.sh` now filters them from the **console only**; `console.log` keeps
everything, and the run summary reports how many were hidden so the fault cannot
pass unnoticed:

```
!!! 4182 TF-NaN lines were hidden from the console (kept in console.log).
!!! An arm servo is reporting NaN instead of a position. ...
```

Use `NO_TF_FILTER=1 ./scripts/run_demo.sh` to see them live.

### Fixing it on the robot

The servo driver is not reporting a position for three of the four arm joints,
while `elbow_joint` reads normally. That pattern -- one joint fine, three
unreadable -- points at the servos themselves rather than a broken daisy chain
(a chain break would take out everything downstream of it). It follows the
earlier "stuck" behaviour, where `shoulder_pan` held position but refused to
move: overload -> torque cut -> no response at all.

From the laptop (SSH needs a password, so this cannot be scripted from the
container):

```bash
ssh mirte@192.168.45.1
ros2 control list_hardware_components      # is the arm interface active or in error?
ros2 control list_controllers              # is mirte_master_arm_controller active?
ros2 topic echo /joint_states --once --field position
```

If the hardware component reports an error state, restarting the robot's own
bringup is worth trying before a full power cycle. If the NaN persists across a
power cycle, it is the servos or their wiring, not software.

Until it is fixed, expect: no arm motion, no footprint adaptation, and the
costmap held at the CARRY polygon. The base navigation, speed adaptation and
inflation adaptation all still work -- the demo can run on those alone with
`arm_to_carry:=false`.

### The driver is healthy; three servos are not answering

`ros2 control list_hardware_components` on the robot reports everything
**active**, and all five controllers active -- so ros2_control and the
`MirteMasterArmHWInterface` plugin are fine. They simply never get a reading
back from three of the four arm servos, and NaN is what the interface leaves in
the state when a read never succeeds:

```
shoulder_lift_joint   .nan
elbow_joint           -1.9167     <- answers
wrist_joint           .nan
shoulder_pan_joint    .nan
gripper_joint         0.0775      <- answers (separate component)
```

One joint answering while three do not argues against a broken daisy chain -- a
chain break takes out everything downstream. It follows the earlier "stuck"
behaviour, where a joint held position but refused to move: overload -> torque
cut -> no response.

**A software restart will not clear this.** Remove power.

### Two bugs this exposed in the helper scripts

**1. NaN values were silently dropped, mis-attributing every joint after them.**
`set_arm_pose.sh` parsed `--field position` with a number regex, and a number
regex does not match `nan`. So from

```
array('d', [0.0775, nan, -1.9167, nan, 0.0063, nan, ...])
```

it recovered 6 values for 9 names and zipped them in order, reporting
`shoulder_lift = -1.917` -- which is really the **elbow's** reading. Every
per-joint diagnosis was shifted whenever any joint was NaN. The list is now
tokenised by commas so NaN keeps its slot. **Never parse a numeric array with a
number regex** -- `nan` and `inf` are valid values that such a regex discards.

**2. `check_arm.sh` passed NaN joints as healthy.** `abs(nan) > 0.25` is
`False`, so a joint that could not be read was not flagged and the verdict
blamed whichever joint happened to have a real error. NaN is now reported
separately as `UNREADABLE`, with its own HARDWARE FAULT verdict (exit 2).

### Stall detection: stop commanding a servo that is not moving

`demo_launch.py` called `set_arm_pose.sh carry 4`, which commanded the pose four
times with 5 s waits -- about 20 s of a servo straining against a target it
cannot reach, which is how a servo reaches thermal shutdown. That almost
certainly made the fault worse across runs.

The script now:

* **aborts immediately** (exit 2) if any joint reports NaN -- the driver cannot
  read it, so commanding it is pointless and only keeps it loaded;
* **stops early** (exit 1) if the worst joint error does not improve by at least
  0.02 rad between attempts:

```
!!! Arm is NOT MOVING (error 1.8870 -> 1.8870 rad).
!!! Stopping rather than holding the servo loaded against a target it
!!! cannot reach, which risks an overload shutdown.
```

## "No valid path found" on a goal that IS reachable

The goal was fine, localisation was marginal-but-usable (67 %), the controller
never saw a path (zero controller errors), and the planner refused 14 times:

```
planner_server: GridBased: failed to create plan, no valid path found.   (x14)
NAV: GOAL FAILED (aborted) after 9.2 s, 19 recoveries
```

Proof the path existed -- flood-fill the **actual published global costmap** from
the robot to the goal, over cells the robot's centre may occupy:

```
robot(0,0)        cell=(106,70)   cost=0
goal(1.74,2.60)   cell=(158,105)  cost=82      (of 99 = inscribed inflation)
reachable from robot (cost<99): 13553 cells = 33.9 m2
goal reachable by flood fill?   YES
```

(Note the scaling: `/global_costmap/costmap` is an `OccupancyGrid` where
**100 = lethal, 99 = inscribed-inflated, -1 = unknown** -- not the raw 0-255
costmap values. Thresholding at 253 silently treats lethal cells as free.)

### Cause: a turning-radius constraint the robot does not have

```yaml
plugin: "nav2_smac_planner/SmacPlannerLattice"
lattice_filepath: ".../5cm_resolution/0.5m_turning_radius/omni/output.json"
```

The lattice planner searches over precomputed motion primitives built for a
minimum **turning radius**, and nav2 only ships 0.5 m and 1 m variants -- 0.5 m
was already the smaller. Mirte is a mecanum base: it strafes and spins in place,
and the controller is already configured `motion_model: "Omni"`. In ~0.8 m lab
corridors no sequence of 0.5 m arcs reaches the goal, so A* exhausted its
expansions and reported no path -- for a path that plainly existed. The planner
was enforcing a non-holonomic constraint the hardware does not have.

Switched to **`nav2_smac_planner/SmacPlanner2D`**: grid A* with no kinematic
constraint, which is the right model for a holonomic base. MPPI (`Omni`,
`consider_footprint: true`) does the kinematic and collision work at execution
time, which is where it belongs. The old lattice block is kept commented in
`demo_params.yaml` -- restore it only if the robot is ever driven as a
differential base, and then with the `diff` primitives.

Verified live on the exact goal that had failed 14 times:

```
Created global planner plugin GridBased of type nav2_smac_planner/SmacPlanner2D
Goal accepted with ID: 6ad6b5bc...
"no valid path found" occurrences: 0
```

### Also confirmed: the initial-pose fix now works end to end

```
[pose_from_scan]: AMCL adopted the pose.
best pose: x=+0.120  y=+0.080  yaw=-3.0 deg
published and CONFIRMED: /amcl_pose is now at the matched pose.
```

Previously it published before DDS discovery matched AMCL's subscription and
exited claiming success. It now waits for a subscriber, republishes, and
confirms against `/amcl_pose`.

## One-sided footprint, and an envelope that can shed it

### The footprint was wide on BOTH sides; the arm only extends on one

**Superseded 2026-10-05: the footprints now come from the CAD meshes.**
Both polygons are the ground projection of every `mirte_master_description`
mesh, posed through the URDF in `base_link` at the tucked and carry joints.
The polygons carry no margin of their own (nav2's `footprint_padding` 0.01 is
added on top), and all ~2.2M mesh vertices lie inside them.

The earlier "measured wheel centres" (`y=+0.153` left, `-0.098` right) were the
wheel **joint frames**, not the wheel centres: the left and right wheel meshes
have their origins on opposite faces of the wheel. In reality the base is
**symmetric** about `base_link`:

```
chassis     x -0.140 .. +0.143    y -0.115 .. +0.115
front cam   x        .. +0.147    (|y| < 0.085)
wheels      x -0.137 .. +0.137    y -0.152 .. -0.094 / +0.099 .. +0.157
```

So the old `Y: -0.128 .. +0.183` was shifted 0.026 m to the left: it claimed
space beside the left wheels and left the outer 0.024 m of the right wheels
**outside** the footprint.

Arm extent at the carry joints `[1.15, -0.030, -1.27, 0.0]` (mesh bounds):

```
wrist     x 0.116 .. 0.188   y 0.115 .. 0.204
gripper   x 0.099 .. 0.222   y 0.144 .. 0.242
finger_l  x 0.133 .. 0.169   y 0.227 .. 0.267
```

The arm reaches **+Y and +X only**, so the carry polygon grows on one side
and both polygons share the same right edge:

| | verts | X | Y | lateral | r_circ | inscribed |
|---|---|---|---|---|---|---|
| tucked | 4 | -0.140 .. +0.147 | -0.152 .. +0.157 | 0.309 | 0.215 | 0.140 |
| carry | 7 | -0.140 .. +0.222 | -0.152 .. +0.267 | 0.419 | 0.329 | 0.140 |

(was 0.311 / 0.388 lateral from the joint-frame estimate, and 0.421 / 0.513
before that, both symmetric). The inscribed radius is unchanged at 0.140, so
the inflation floor (0.140 + padding 0.01 < 0.17) still holds.

### The envelope can now shrink the footprint when the arm stalls

This is the behaviour that was missing. `_envelope_label()` returned the larger
of the physical and target arm states, and a servo that stops **part-way**
matches neither pose within `ARM_JOINT_TOLERANCE_RAD`, so
`_verified_arm_label()` returned `None` and the costmap was pinned to **carry**
for the rest of the run -- a footprint the robot could never shed, which wedged
it in corridors it would otherwise clear and ended the goal in failure.

The footprint is now derived from **where the arm actually is**. The arm swings
about the vertical `shoulder_pan` axis, so its lateral reach is
`R*sin(pan)` with `R = 0.250` (from y=0.230 at pan=1.169):

```
|pan| <= 0.822 rad (47 deg)  ->  reach <= 0.183  ->  the arm is inside the base
                                 -> the TUCKED polygon is correct
|pan| >  0.822 rad           ->  the arm sticks out -> CARRY
```

With the arm stalled at `pan = 0.461`, reach is **0.111 m** -- comfortably
inside the base -- so the narrow footprint applies and the robot keeps moving.
Previously this exact state forced the carry polygon.

Two safeguards kept:

* while the arm is actively swinging **out** (`target == carry` and inside the
  transition window) the carry polygon is held, so the costmap grows *before*
  the arm does;
* a missing or NaN `shoulder_pan` still falls back to carry, because then the
  arm's position is genuinely unknown.

### The carry joints were unreachable

`[1.5582, -0.0297, -1.5188, -0.0349]` cannot be held by these servos. They drive
part-way and stall, so `verified` never matched `carry` and the demo never
started with the arm out. Changed to `[1.15, -0.030, -1.27, 0.0]`, and the carry
polygon above was measured at that pose.

**Caveat: the servo is degrading.** Successive attempts reach progressively
less far -- `pan 1.169`, then `0.461` for the same command. Each attempt leaves
it weaker, which is consistent with thermal derating. Expect the arm to be
unreliable until it is serviced; the footprint logic above is what keeps the
navigation working regardless.

### Why the footprint still did not change: a one-joint reach model

From `run_20261005T140022` (1189 `/joint_states`, no NaN):

```
first:  shoulder_pan=+1.583   elbow=-1.280
last:   shoulder_pan=+1.583   elbow=+0.492      <- pan FROZEN, the elbow moved
verified='None' on all 166 ticks
```

`shoulder_pan` was stuck fully rotated out while the **elbow** was the joint
actually moving, and the reach model keyed on pan alone
(`reach = R*sin(pan)`). It therefore reported `carry` for every state and the
footprint never changed -- which is exactly what was observed.

**Two joints matter.** Pan sets the azimuth, but the elbow decides whether the
arm reaches horizontally at all: folded up it is nearly vertical, and panning a
vertical arm barely projects sideways. Horizontal reach from the shoulder,
interpolated on the elbow:

```
elbow = +0.50 (tucked, folded up) -> r_h = 0.077 m   [gripper x=0.002 vs shoulder x=0.079]
elbow = -1.27 (carry,  extended)  -> r_h = 0.250 m   [y=0.230 at pan=1.169]
lateral reach = r_h * |sin(pan)|
```

| pan | elbow | r_h | lateral reach | footprint | |
|---|---|---|---|---|---|
| 1.583 | +0.492 | 0.078 | **0.078** | **tucked** | the stuck state in that run |
| 1.583 | -1.280 | 0.250 | 0.250 | carry | same pan, elbow extended |
| 1.150 | -1.270 | 0.250 | 0.228 | carry | commanded carry |
| 0.461 | -1.096 | 0.233 | 0.104 | tucked | earlier partial |
| 0.000 | +0.500 | 0.077 | 0.000 | tucked | home |

The pan-only model returned `carry` for all five. The footprint now follows the
elbow as well, so it changes as the arm folds and unfolds -- and the tick log
says why:

```
Arm is between poses; using the 'tucked' envelope from the measured arm pose
(pan=+1.583 elbow=+0.492 -> 0.078 m lateral reach) rather than assuming carry.
```
