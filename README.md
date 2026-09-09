# Gen0 Gazebo ROS 2 Demo

This workspace contains the Gen0 Gazebo simulation and ROS 2 Humble packages.
The current validated setup includes Gazebo Fortress, RViz, simulated 3D LiDAR,
FAST-LIO 3D SLAM, terrain/local-costmap support, and the upstream autonomous
mapping demo packages.

## Requirements

- Ubuntu 22.04 or WSL2 with ROS 2 Humble
- Gazebo Fortress / Ignition Gazebo 6
- NVIDIA GPU rendering recommended for Gazebo
- `python3-colcon-common-extensions`
- `ros-humble-ros-gz`
- `ros-humble-rviz2`
- `ros-humble-tf-transformations`
- `ros-humble-pcl-ros`
- `ros-humble-slam-toolbox`
- `ros-humble-navigation2`
- `ros-humble-nav2-bringup`
- `ros-humble-pointcloud-to-laserscan`

Install the Python requirements after cloning:

```bash
pip3 install -r requirements.txt
```

## Build

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash

cmake -S gen0_gz_sim_ros2/gz_plugins -B gen0_gz_sim_ros2/gz_plugins/build
cmake --build gen0_gz_sim_ros2/gz_plugins/build -j2

colcon build --symlink-install --packages-up-to gen0_main fast_lio
source install/setup.bash
```

`build/`, `install/`, `log/`, and `runtime_logs/` are generated locally and are
ignored by git.

## One-Command 3D SLAM Demo

After dependencies are installed and the workspace is built, the tested 3D SLAM
flow can be started with:

```bash
cd ~/gen0_gz_sim_ros2
./run_gen0_3d_slam.sh
```

The script launches Gazebo, FAST-LIO, the mapping drive node, a lightweight RViz
point-cloud preview, and RViz. It also clears software-rendering environment
variables and selects the NVIDIA D3D12 adapter by default. The default world is
`my_map`; actor scenarios are only loaded when `GEN0_ACTORS_SCENARIO` is set.

Useful runtime overrides:

```bash
GEN0_WORLD=san_roundabout GEN0_ACTORS_SCENARIO=walking_actors ./run_gen0_3d_slam.sh
GEN0_GPU_ADAPTER= ./run_gen0_3d_slam.sh
GEN0_SIMULATED_LIDAR=false ./run_gen0_3d_slam.sh
GEN0_SIM_LIDAR_MAX_RANGE=15 GEN0_SIM_LIDAR_MAX_POINTS=4000 ./run_gen0_3d_slam.sh
GEN0_SIM_LIDAR_SURFACE_SAMPLES=800000 GEN0_SIM_LIDAR_WORLD_VOXEL_SIZE=0.12 ./run_gen0_3d_slam.sh
GEN0_DRIVE_SPEED=0.15 ./run_gen0_3d_slam.sh
GEN0_MAPPING_DRIVE=false ./run_gen0_3d_slam.sh
GEN0_CAMERA_VIEW=false ./run_gen0_3d_slam.sh
GEN0_CAMERA_TOPIC=/gen0_model/front_camera ./run_gen0_3d_slam.sh
GEN0_PREVIEW_MAX_POINTS=25000 ./run_gen0_3d_slam.sh
GEN0_FAST_LIO_PCD_SAVE=true GEN0_FAST_LIO_MAP_FILE_PATH=/tmp/my_map_prior.pcd ./run_gen0_3d_slam.sh
GEN0_RELOCALIZATION=true GEN0_PRIOR_MAP_PATH=/path/to/prior_map.pcd ./run_gen0_3d_slam.sh
```

The launcher opens a resizable **Gen0 Sweeper - Front Camera** window by
default. It displays the bridged `/gen0_model/front_camera` stream, its native
resolution, and the measured frame rate. Press `q` or `Esc` to close only the
camera viewer. Set `GEN0_CAMERA_VIEW=false` when running without WSLg/X11.

`GEN0_SIMULATED_LIDAR=false` disables the OBJ-based simulated LiDAR fallback and
uses only Gazebo's `/gen0_model/front3d/lidar/points` output. This is useful
when verifying that RViz is not showing a precomputed world-mesh point source.
The default simulated LiDAR fallback is front-facing and range-limited so RViz
shows map growth instead of a 360-degree world-model sample on the first scan.
It also samples OBJ faces, not only raw OBJ vertices, to make road boundaries
and ground surfaces visually denser.
When the fallback is enabled, it publishes on
`/gen0_mapping/simulated_front3d/lidar/points`; FAST-LIO, RViz raw preview, and
mapping-drive safety use that topic through `GEN0_FRONT3D_SOURCE_TOPIC` so it
does not mix with Gazebo's raw point-cloud bridge.

To create a prior map that matches the current Gazebo world, start normal SLAM
with FAST-LIO map saving enabled:

```bash
GEN0_FAST_LIO_PCD_SAVE=true \
GEN0_FAST_LIO_MAP_FILE_PATH=/tmp/my_map_prior.pcd \
./run_gen0_3d_slam.sh
```

After the map has enough coverage, call:

```bash
ros2 service call /gen0_mapping/save_fast_lio_map std_srvs/srv/Trigger {}
```

Then restart with `GEN0_RELOCALIZATION=true` and
`GEN0_PRIOR_MAP_PATH=/tmp/my_map_prior.pcd`.

In Gen0 relocalization mode, ICP now transforms incoming LiDAR points from
`front_3d_lidar_link` into `base_link` before matching the prior map. The
default offset is `(1.9, 0.0, 1.9)` and SCURM's original 180-degree Livox roll
correction is disabled for the Gen0 simulator. Useful overrides:

```bash
GEN0_RELOCALIZATION_FITNESS_SCORE_THRESHOLD=1.0
GEN0_RELOCALIZATION_MAX_CORRESPONDENCE_DISTANCE=2.0
GEN0_RELOCALIZATION_CONVERGED_COUNT_THRESHOLD=3
GEN0_RELOCALIZATION_INPUT_CLOUD_TO_BASE_X=1.9
GEN0_RELOCALIZATION_INPUT_CLOUD_TO_BASE_Z=1.9
GEN0_RELOCALIZATION_LEGACY_LIVOX_ROLL_180=false
```

When relocalization is enabled, `run_gen0_3d_slam.sh` waits for FAST-LIO
odometry before starting mapping drive or trash cleanup. If it times out, check
`runtime_logs/fast_lio_3d_slam.log` for the latest ICP fitness score.

The 3D SLAM wrapper disables `spawn.launch.py` ground-truth localization and
its static `odom -> base_link` fallback by default. In the relocalized Nav2
flow, ICP/FAST-LIO owns `map -> odom -> base_link`. Do not enable
`GEN0_GROUND_TRUTH_LOCALIZATION=true` for this flow: `/pose_publisher` publishes
a Gazebo-derived `map -> odom` transform and makes RViz/Nav2 see the map frame
move with the vehicle.

`GEN0_MAPPING_DRIVE=false` keeps the relocalized SLAM stack running without the
automatic mapping-drive velocity source. Use that mode before handing vehicle
control to Nav2.

## SCURM Nav2 Integration Check

The SCURM `sentry_bringup` NavigateToPose and NavigateThroughPoses behavior
trees are installed with `gen0_main` and are the defaults in
`gen0_navigation.launch.py`. For a launch-level check against the running
relocalized SLAM stack:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gen0_main gen0_navigation.launch.py \
  use_respawn:=false \
  start_vehicle_interface:=false \
  odom_topic:=/gen0_mapping/fast_lio/odom
```

The checked wrapper waits for FAST-LIO odometry and `odom -> base_link` TF
before starting Nav2:

```bash
GEN0_WORKSPACE=$PWD ./run_gen0_nav2.sh
```

For actual Nav2 control, start the SLAM stack without mapping drive first, then
launch navigation with its vehicle interface enabled:

```bash
GEN0_RELOCALIZATION=true \
GEN0_PRIOR_MAP_PATH=/tmp/my_map_prior.pcd \
GEN0_MAPPING_DRIVE=false \
./run_gen0_3d_slam.sh
```

```bash
GEN0_WORKSPACE=$PWD \
GEN0_NAV2_START_VEHICLE_INTERFACE=true \
./run_gen0_nav2.sh
```

The Gen0 Gazebo steering command direction now matches ROS' positive
`Twist.angular.z` convention after conversion, so `cmdvel_to_vehicle` defaults
to `GEN0_VEHICLE_ANGULAR_Z_SIGN=1.0`. If a future vehicle model turns the
wrong way, override that value when launching Nav2 or mapping drive.
The vehicle adapter defaults to the SCURM-style responsibility split: Nav2's
costmap/controller handles obstacle avoidance, while `cmdvel_to_vehicle` only
does steering conversion, sign correction, and moderate command limits. Forward
velocity is capped at `0.65 m/s` by default. The front-laser hard stop remains
available for explicit safety tests, but it is disabled by default because it
can mask controller output while tuning local costmaps. Angular velocity is
limited to `0.12 rad/s`, matching Gen0's roughly `4.5 m` minimum turning radius
at the default speed. Useful overrides:

```bash
GEN0_VEHICLE_MAX_FORWARD_SPEED=0.45
GEN0_VEHICLE_FRONT_STOP_ENABLED=true
GEN0_VEHICLE_FRONT_STOP_DISTANCE=0.65
GEN0_VEHICLE_FRONT_SLOW_DISTANCE=1.5
```

The Nav2 controller now uses MPPI with an Ackermann motion model and
`min_turning_r=4.5`, so generated controls match the Gen0 steering geometry.
The goal checker uses `xy_goal_tolerance=0.35` and ignores final yaw for
position-only RViz goals. Do not expect centimeter-exact final placement until
a dedicated docking/final-approach controller is added.

The default Nav2 obstacle source is the Gen0 2D LaserScan pair. To test the
next SCURM migration stage, switch Nav2's own local costmap to
`costmap_intensity` on `/gen0_mapping/terrain_map`; in this mode the global
costmap also marks accumulated terrain obstacles from
`/gen0_mapping/terrain_map_ext` so the planner does not rely on the blank
temporary map alone:

```bash
./stop_gen0_nav2.sh

GEN0_WORKSPACE=$PWD \
GEN0_NAV2_START_VEHICLE_INTERFACE=true \
GEN0_NAV2_COSTMAP_SOURCE=scurm_terrain \
./run_gen0_nav2.sh
```

Set `GEN0_NAV2_START_VEHICLE_INTERFACE=true` before launching
`run_gen0_nav2.sh`; changing it in the shell after Nav2 is already running does
not start `cmdvel_to_vehicle`.
The wrapper waits for `/gen0_mapping/terrain_map` and
`/gen0_mapping/terrain_map_ext` before launching Nav2 in `scurm_terrain` mode.
If the map is still visibly incomplete, let the SLAM/terrain stack run a little
longer before sending the first goal.

By default, `gen0_navigation.launch.py` generates `/tmp/gen0_my_map_nav.yaml`
and `/tmp/gen0_my_map_nav.pgm`, a blank occupancy map covering the Gen0
`my_map` coordinate range. This keeps the global costmap in bounds while
testing the SCURM behavior trees, Nav2 lifecycle, and local obstacle handling.
Because this map is intentionally blank, the global planner will not follow
road boundaries by itself. Pass `map:=/path/to/aligned_map.yaml` when validating
against a real global occupancy map with static obstacles or road constraints.
If the vehicle physically collides with a wall, FAST-LIO/terrain mapping can be
polluted by the impact and nearby surface returns. Stop and restart the 3D SLAM
stack after a collision before judging relocalization or map quality.

For a fresh clone on another machine, run the build section once before using
`run_gen0_3d_slam.sh`. The script intentionally requires `install/setup.bash`
so it does not hide build or dependency failures.

If an old run is still alive, the script exits before launching a second stack.
This prevents stale `pointcloud_accumulator_preview` or simulated LiDAR nodes
from publishing an already accumulated RViz point cloud. Run
`./stop_gen0_3d_slam.sh` to clean up a previous run.

## Manual 3D SLAM Flow

Use separate terminals and source ROS plus the workspace in each terminal.

### 1. Gazebo

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

unset LIBGL_ALWAYS_SOFTWARE
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
unset MESA_GL_VERSION_OVERRIDE
unset QT_XCB_GL_INTEGRATION
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

ros2 launch gen0_main spawn.launch.py \
  world:=my_map \
  rviz:=false \
  ground_truth_localization:=false \
  static_odom_base:=false \
  render_env:=unset \
  d3d12_adapter:=NVIDIA
```

### 2. FAST-LIO 3D SLAM

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gen0_main gen0_fast_lio_mapping.launch.py \
  rviz:=false \
  simulated_lidar:=true \
  terrain_analysis:=false \
  local_costmap:=false
```

### 3. Mapping Drive

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gen0_main gen0_mapping_drive.launch.py \
  enabled:=true \
  drive_speed:=0.25
```

### 4. RViz Preview

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run gen0_main pointcloud_accumulator_preview --ros-args \
  -p input_topic:=/gen0_mapping/cloud_registered \
  -p output_topic:=/gen0_mapping/rviz/fast_lio_map \
  -p max_points:=40000 \
  -p voxel_size:=0.65 \
  -p publish_period:=1.0
```

Then open:

```bash
rviz2 -d gen0_gz_sim_ros2/gen0_main/config/gen0_3d_mapping_preview.rviz
```

## Validation Checks

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /gen0_mapping/cloud_registered
ros2 topic hz /gen0_mapping/rviz/fast_lio_map
ros2 topic echo /gen0_mapping/drive_status --once
```

Expected signs of success are a Gazebo simulation window, a moving vehicle, a
green FAST-LIO path in RViz, and a downsampled 3D point-cloud map on
`/gen0_mapping/rviz/fast_lio_map`.

## Autonomous Mapping Demo

The upstream 2D autonomous mapping flow is also included. Build the related
packages first:

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  explore_lite_msgs \
  explore_lite \
  behavior_ext_plugins \
  gen0_main \
  gen0_interface \
  sweeper_integration
source install/setup.bash
```

Then follow the terminal workflow in `docs/demo_workflow.md`.

## Documentation

- `docs/demo_workflow.md`: full autonomous mapping terminal workflow
- `docs/my_map_startup_guide.md`: startup guide for the `my_map` world
- `docs/system_overview.md`: package map and algorithm notes
- `docs/file_inventory.md`: source, generated output, and third-party inventory
- `docs/validation_report_2026-07-20.md`: validation notes from the upstream branch

## Validated LLM Task Agent (MVP)

`src/gen0_llm_agent` converts natural-language commands into a strictly validated
plan and executes only three registered skills: `navigate`, `search_object`, and
`stop`. The LLM never emits ROS interfaces or velocity commands. `navigate` uses
the existing `/navigate_to_pose` Nav2 action, perception subscribes to the
structured `/yolo/detections` output from the existing YOLO node, and the trusted
stop adapter selects the existing velocity mux's highest-priority `stop` mode on
`/epsilon/control_mode`; it never publishes directly to `/cmd_vel`. A valid
`map -> base_link` transform is required before navigation. In simulation,
`/actors/detections` is also mapped to the `person` world-model event, while
camera detections continue to arrive through `/yolo/detections`.

Data flow:

```text
/llm_agent/command (std_msgs/String)
  -> planner -> strict schema validation -> Skill Registry -> Task Executor
  -> /navigate_to_pose (Nav2)
/yolo/detections -> World Model -> person interrupt -> cancel Nav2 -> stop adapter
/llm_agent/status (std_msgs/String JSON)
```

Build and launch:

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install -r requirements.txt
colcon build --symlink-install --packages-select gen0_llm_agent yolo_detector
source install/setup.bash
ros2 launch gen0_llm_agent llm_agent.launch.py provider:=mock
```

Submit a command and inspect status:

```bash
ros2 topic pub --once /llm_agent/command std_msgs/msg/String \
  "{data: '去坐标 (10, 5)，途中如果发现行人就停车。'}"
ros2 topic echo /llm_agent/status
```

The default `mock` provider requires no key and is suitable for CI and Gazebo.
For an OpenAI-compatible endpoint, configure environment variables without
committing secrets:

```bash
export GEN0_LLM_API_KEY='...'
export GEN0_LLM_BASE_URL='https://api.openai.com/v1'
export GEN0_LLM_MODEL='gpt-4o-mini'
ros2 launch gen0_llm_agent llm_agent.launch.py provider:=openai_compatible
```

The emergency service is always available independently of the planner:

```bash
ros2 service call /llm_agent/stop std_srvs/srv/Trigger '{}'
```

`search_object(person)` requires the configured `yolo_detector` weight to contain
a `person` class. The repository's `best_road.pt` contains road-litter classes and
does not contain `person`; set `GEN0_YOLO_MODEL` or the node's `model_path`
parameter to a person-capable YOLO weight when testing the pedestrian-stop flow.
The Gazebo actor stream `/actors/detections` independently provides the simulated
`person` safety event, so pedestrian-stop testing does not depend on a particular
camera model.

### Stable map + pedestrian avoidance + mobile HMI

The stable full-stack launcher now starts the browser/PWA interface, rosbridge,
camera compressors, YOLO and the LLM agent by default, while retaining the backup
branch's FAST-LIO relocalization, static map, Nav2, actor costmap, EPSILON/QCNet
pedestrian avoidance, command mux and final pose guard:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
GEN0_QCNET_BACKEND=constant_velocity ./run_gen0_full_stack.sh
```

Use `GEN0_QCNET_BACKEND=qcnet` only when the QCNet checkpoint and CUDA Python
environment are installed. Optional HMI settings are:

```bash
GEN0_WEB_PORT=8000
GEN0_ROSBRIDGE_PORT=9090
GEN0_LLM_PROVIDER=mock
GEN0_YOLO_MODEL=/absolute/path/to/model.pt
GEN0_ENABLE_YOLO=true
GEN0_START_HMI=true
```

From another computer or phone, forward ports 8000 and 9090 through SSH, open
`http://localhost:8000`, and connect the page to `ws://localhost:9090`. The web
app is installable as a PWA from a supported mobile browser. Manual velocity is
published only to `/web_control/cmd_vel_raw`; the existing EPSILON command mux
selects `teleop` and the existing pose guard remains the sole final publisher to
`/cmd_vel`. Submitting a new LLM navigation task returns the mux to `auto`, and
both stop buttons select the mux's highest-priority `stop` mode.

To attach only the HMI and LLM components to an already-running stable stack:

```bash
ros2 launch sweeper_integration web_control_agent.launch.py
```

### Cloudflare Tunnel deployment for WebToApp

For a phone-installable public endpoint, route one Cloudflare hostname to the
dashboard on `127.0.0.1:8000` and a second hostname to rosbridge on
`127.0.0.1:9090`. Start from `deploy/cloudflare/config.yml.example`, then write
the public WSS endpoint into the browser runtime configuration:

```bash
./deploy/cloudflare/configure_web.sh ros.sweeper.example.com
cloudflared tunnel --config ~/.cloudflared/config.yml run gen0-sweeper
```

Point WebToApp at `https://sweeper.example.com/`, not localhost. The page gives
the deployment-time WSS setting priority, automatically connects when configured,
and refuses insecure `ws://` from an HTTPS page. See `web_control/README_start.md`
for tunnel creation, DNS routing, validation, security, and the legacy AutoDL note.

### Phone APP connecting to each user's local ROS 2 computer

Cloudflare is not required when the phone and computer are on the same Wi-Fi.
The dashboard now defaults to **Local computer** mode: enter only the computer's
LAN IP and keep port `9090`. The page builds and remembers the WebSocket URL.

On the ROS 2 computer:

```bash
GEN0_QCNET_BACKEND=constant_velocity ./run_gen0_full_stack.sh
./deploy/local_mobile/show_connection_info.sh
```

The helper prints the exact IP to enter and checks whether rosbridge is listening.
The rosbridge launch defaults to `0.0.0.0` so a phone on the same LAN can reach it.
For an APK that does not depend on AutoDL or any hosted website, build the offline
web bundle:

```bash
./deploy/local_mobile/build_web_bundle.sh
```

Package `gen0-mobile-web.zip` as Local HTML/Offline Website, not as a wrapper around
an online URL. `roslib` is vendored, so the dashboard itself has no CDN dependency.
See `deploy/local_mobile/README.md` for firewall rules, WebView permissions,
connection steps, and troubleshooting.
