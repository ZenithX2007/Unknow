# AI Handoff: Web Navigation / Ground-Truth Localization

## Goal

Project: `/home/logic/Unknow-feature-map-stable-llm-web-control`

Current target mode:

- Headless Gazebo
- Web control: `http://127.0.0.1:8000`
- rosbridge: `ws://127.0.0.1:9090`
- Browser map click sends Nav2 `/navigate_to_pose`
- Front/driver camera streaming
- Lightweight local LLM planning only; no RViz, YOLO, EPSILON

Launch command:

```bash
cd /home/logic/Unknow-feature-map-stable-llm-web-control
./run_llm_web_navigation.sh
```

## Latest Root Cause

Nav2 previously received an incorrect robot position, e.g.:

```text
Begin navigating from current location (1.84, -0.85) to (31.12, -0.23)
```

`(1.84, -0.85)` is a lidar mounting-link pose, not the vehicle pose. This led to:

```text
Failed to make progress
[follow_path] [ActionServer] Aborting handle.
```

The source topic `/gen0_model/links/poses` is a `PoseArray` bridged from Gazebo `Pose_V`. Bridge conversion discards link names, and array order is not stable. Fixed `pose_index` values are therefore invalid:

- Old `pose_index: 15` selected the GPS-link local offset near `(-0.25, 0)`.
- Later `pose_index: 1` selected a lidar mounting offset near `(1.84, -0.85)`.

## Fix Applied

Modified:

- `gen0_gz_sim_ros2/sweeper_integration/sweeper_integration/ground_truth_odometry.py`
- `gen0_gz_sim_ros2/sweeper_integration/config/interfaces.yaml`
- `gen0_gz_sim_ros2/sweeper_integration/config/nav2_ground_truth.yaml`

`ground_truth_odometry.py` now uses `pose_index: -1` as tracking mode:

1. The first `PoseArray` pose closest to the Gazebo spawn position is selected.
2. Each subsequent pose is the one closest to the previous selected vehicle world position.
3. The selected ROS message is not modified in place; an independent `Odometry` message is published.
4. It publishes both `/odom` and `odom -> base_footprint` TF.

Configured vehicle initial pose:

```yaml
pose_index: -1
initial_x: -20.6991
initial_y: -22.4324
initial_z: 2.85
```

The spawn pose comes from:

`gen0_gz_sim_ros2/gen0_main/worlds/my_map/my_map.sdf`.

The Nav2 local/global inflation radius was changed from `0.8` to `1.2`, because the reported vehicle inscribed radius is `1.15m`.

## Validation Already Done

Passed:

```bash
python3 -m py_compile gen0_gz_sim_ros2/sweeper_integration/sweeper_integration/ground_truth_odometry.py
bash -n run_llm_web_navigation.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sweeper_integration
```

The launcher sources `install/setup.bash`, so the rebuilt package should be used on the next launch.

## Required Next Check

The current running process predates the fix. Stop it with `Ctrl+C`, relaunch, and inspect the `ground_truth_odometry` log. It must show approximately:

```text
Initial vehicle pose: (-20.70, -22.43, 2.xx)
```

It must not show `(1.84, -0.85)` or `(-0.25, 0.00)`.

Then submit a modest nearby map goal first and verify:

- `/odom` coordinates change as the vehicle moves.
- `odom -> base_footprint` is connected in TF.
- Nav2 does not report `Failed to make progress`.
- Browser robot marker follows the actual vehicle.

Useful commands after sourcing the project:

```bash
source /opt/ros/humble/setup.bash
source /home/logic/Unknow-feature-map-stable-llm-web-control/install/setup.bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

## Other Known Issues

1. The launch occasionally fails to start its HTTP server because port 8000 is already occupied:

```text
OSError: [Errno 98] Address already in use
```

This is independent of the Nav2 localization defect. An older `python3 -m http.server 8000` is usually still serving the UI. Process cleanup in `scripts/manage_llm_web_stack.sh` may need further hardening if the issue persists.

2. Native Gazebo `/odom` was bridged but did not produce callbacks in the earlier `odom_tf_broadcaster`; ground-truth odometry remains intentionally enabled in `run_llm_web_navigation.sh`:

```bash
ground_truth_odometry:=true
```

3. Camera status: driver camera compressor is `640x360` at 8 FPS. Browser balanced mode intentionally throttles the displayed rate to about 4 FPS; observed near-3-FPS display is expected.

4. The browser map was updated previously to replace its preview map with ROS `/map` once received. The browser manual velocity topic is `/cmd_vel`, matching the lightweight Gazebo bridge.

## Related Files

- `run_llm_web_navigation.sh`
- `scripts/manage_llm_web_stack.sh`
- `gen0_gz_sim_ros2/sweeper_integration/launch/web_control_manual.launch.py`
- `gen0_gz_sim_ros2/sweeper_integration/launch/interfaces.launch.py`
- `gen0_gz_sim_ros2/sweeper_integration/config/interfaces.yaml`
- `gen0_gz_sim_ros2/sweeper_integration/config/nav2_ground_truth.yaml`
- `gen0_gz_sim_ros2/sweeper_integration/sweeper_integration/ground_truth_odometry.py`
- `web_control/app.js`
