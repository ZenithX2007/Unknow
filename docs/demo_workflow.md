# Gen0 SLAM and Navigation Demo Workflow

This document records the demo workflow that has been verified on the current
WSL + ROS 2 Humble + Gazebo Fortress setup.

The current integration is split into these layers:

- simulator / bridge layer
- interface adapter layer
- perception layer
- mapping layer
- navigation layer

This separation makes it easier to validate Gazebo, interfaces, mapping, and
navigation independently before running the full autonomous demo.

## Verified Result

The following chain has been verified:

```text
Gazebo front 3D lidar
-> /gen0_model/front3d/lidar/points
-> pointcloud_to_laserscan
-> /scan
-> slam_toolbox
-> saved map
-> Nav2
-> local/global costmaps
-> NavfnPlanner global path
-> MPPI local control with Ackermann constraints
-> /cmd_vel
-> cmd_vel_adapter
-> /control/cmd_vel
-> gen0_interface cmdvel_to_vehicle
-> Gazebo vehicle motion
```

The full autonomous demo has also been verified once:

```text
online SLAM map
-> explore_lite frontier selection
-> Nav2 /navigate_to_pose goals
-> /cmd_vel
-> /control/cmd_vel
-> Gazebo vehicle motion while the map grows
```

`/explore/status: exploration_complete` means `explore_lite` currently sees no
eligible frontier in the online map. For this demo, that is an acceptable
completion state. It does not mean the road-aware cleaning behavior is finished.

The current demo map is saved at:

```text
/home/zjxue2007/gen0_maps/san_roundabout_front3d_demo.yaml
/home/zjxue2007/gen0_maps/san_roundabout_front3d_demo.pgm
```

## Important WSL Notes

Keep Gazebo rendering variables separate from RViz rendering variables.

Do not use software rendering for Gazebo:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe
```

Those variables are useful for RViz only. With Gazebo they make the simulation
very slow and can cause visual lag or apparent collision artifacts.

On the current WSL laptop, Gazebo GUI rendering has been verified with the
NVIDIA adapter selected for Mesa D3D12:

```bash
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
```

Leaving that adapter unset can make Qt / GLX fall back to `swrast_dri.so` and
segfault before SLAM, Nav2, or exploration starts. The demo launch also forces
Gazebo to use the OGRE1 render engine through `spawn.launch.py`.

The practical rule is:

- If Gazebo GUI opens and `/scan` has finite values, use GUI.
- If Gazebo GUI crashes, `gazebo_gui:=false` is only a fallback for keeping the
  ROS graph alive. Continue only if `/scan` has finite values.
- If `/scan` is all `inf` or `/map` is `0 x 0`, stop and fix the Gazebo sensor
  input before tuning SLAM, Nav2, or exploration.

## Repository Pieces Used By This Demo

The demo uses these repository files:

- `gen0_gz_sim_ros2/gen0_main/launch/spawn.launch.py`
  - Starts Gazebo with `ign gazebo --render-engine ogre`.
  - Supports `gazebo_gui:=false` and `start_gazebo:=false` for WSL fallback
    and manual Gazebo tests.
  - Starts the ROS-Gazebo bridge and `robot_state_publisher`.
- `gen0_gz_sim_ros2/sweeper_integration/launch/interfaces.launch.py`
  - Starts `/ground_truth_odometry`, `/cmd_vel_adapter`, and static sensor TFs.
- `gen0_gz_sim_ros2/sweeper_integration/config/interfaces.yaml`
  - Configures `/cmd_vel_adapter`.
  - Converts Nav2 pure rotation commands into a slow forward arc for Gen0,
    because the four-wheel steering interface cannot move when
    `linear.x == 0` and only `angular.z` is commanded.
- `gen0_gz_sim_ros2/sweeper_integration/launch/mapping.launch.py`
  - Starts `pointcloud_to_laserscan` and `slam_toolbox`.
- `gen0_gz_sim_ros2/sweeper_integration/config/pointcloud_to_laserscan.yaml`
  - Converts `/gen0_model/front3d/lidar/points` into `/scan`.
- `gen0_gz_sim_ros2/sweeper_integration/config/gen0_slam.yaml`
  - Configures `slam_toolbox` to use `/scan`.
- `gen0_gz_sim_ros2/sweeper_integration/launch/navigation_ground_truth.launch.py`
  - Starts Nav2 using the saved map and ground-truth odometry.
- `gen0_gz_sim_ros2/sweeper_integration/launch/navigation_online_slam.launch.py`
  - Starts `pointcloud_to_laserscan`, `slam_toolbox`, and Nav2 together for
    online mapping while navigating.
- `gen0_gz_sim_ros2/sweeper_integration/config/nav2_online_slam.yaml`
  - Configures Nav2 to use the online `/map` from `slam_toolbox` and the
    point-cloud-derived `/scan`.
  - Uses Regulated Pure Pursuit. This remains available as the legacy online
    SLAM controller configuration.
- `gen0_gz_sim_ros2/sweeper_integration/config/nav2_online_slam_mppi.yaml`
  - Configures the same online `/map`, `/scan`, local costmap, and global
    costmap chain.
  - Uses `nav2_mppi_controller::MPPIController` with the Ackermann motion model,
    forward-only velocity sampling, and a Gen0 turning-radius constraint.
  - Uses a tighter online exploration goal tolerance than the saved-map demo,
    so nearby frontiers are not immediately reported as reached before the
    vehicle moves.
- `gen0_gz_sim_ros2/m-explore-ros2`
  - Local vendor copy of the `m-explore-ros2` `explore_lite` source.
  - It provides the `explore_lite` frontier exploration node.
  - Unused `map_merge`, TurtleBot examples, and nested repository metadata were
    removed from this local copy.
  - Local compatibility patch: `explore/src/costmap_client.cpp` uses the
    latest available TF when reading the robot pose, avoiding millisecond-level
    future extrapolation in simulation.
- `gen0_gz_sim_ros2/sweeper_integration/config/explore_gen0.yaml`
  - Gen0-specific parameters for official `explore_lite`.
  - Uses `base_footprint` and the online `/map`.
- `gen0_gz_sim_ros2/sweeper_integration/launch/explore_gen0.launch.py`
  - Starts the official `explore_lite` executable with Gen0 parameters.
- `gen0_gz_sim_ros2/gen0_interface/gen0_interface/cmdvel_to_vehicle.py`
  - Converts `/control/cmd_vel` into vehicle steering and wheel commands.

## Build

After changing launch/config files, rebuild the changed packages:

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  behavior_ext_plugins \
  gen0_main \
  sweeper_integration \
  gen0_interface

source install/setup.bash
```

## Final Autonomous Mapping Check

Use this sequence after closing all old terminals. It starts the current
frontier-exploration demo from a clean state.

### Terminal 1: Gazebo, Bridge, and Robot Description

Use Gazebo GUI first because the front 3D lidar needs valid rendering output:

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
export IGN_PARTITION=gen0_roundabout_demo
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$PWD/gen0_gz_sim_ros2/gz_plugins/build"
export IGN_GAZEBO_RESOURCE_PATH="$PWD/install/gen0_main/share/gen0_main/meshes"

ros2 launch gen0_main spawn.launch.py \
  world:=san_roundabout \
  rviz:=false
```

Do not add RViz rendering variables to this terminal. If this machine is moved
to a different GPU setup, only change `MESA_D3D12_DEFAULT_ADAPTER_NAME`; keep
the Gazebo and RViz rendering environments separate.

If Gazebo GUI crashes, this fallback keeps only the Gazebo server path alive.
Use it only for diagnosis and continue only if `/scan` contains finite ranges:

```bash
ros2 launch gen0_main spawn.launch.py \
  world:=san_roundabout \
  rviz:=false \
  gazebo_gui:=false
```

If Gazebo fails with an OGRE / D3D12 message such as `Out of GPU memory or
driver refused`, stop here. That failure is in the WSL graphics stack before
SLAM, Nav2, or exploration can run. Restart WSL / WSLg from Windows, then
retest Gazebo before changing SLAM or frontier parameters.

### Terminal 2: Interfaces

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch sweeper_integration interfaces.launch.py
```

### Terminal 3: Vehicle Command Interface

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run gen0_interface cmdvel_to_vehicle
```

### Terminal 4: Online SLAM and Nav2

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch sweeper_integration navigation_online_slam.launch.py \
  nav2_params_file:=$PWD/gen0_gz_sim_ros2/sweeper_integration/config/nav2_online_slam_mppi.yaml
```

Wait until `slam_toolbox` registers the lidar and
`nav2_lifecycle_bringup` reports that Nav2 lifecycle nodes are active.

If `nav2_params_file` is omitted, the launch file uses
`nav2_online_slam.yaml`, which is the older Regulated Pure Pursuit
configuration.

### Terminal 5: Frontier Exploration

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch sweeper_integration explore_gen0.launch.py
```

If needed, resume exploration:

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
```

### Terminal 6: RViz Map View

Use RViz software rendering only in this terminal:

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch sweeper_integration rviz_mapping.launch.py
```

This launch uses `mapping_map_only.rviz` by default. It shows `/map` only, which
is best for final validation because it avoids LaserScan message-filter noise.

### Final Health Checks

Run these in a separate terminal:

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list | grep -E "ros_gz_bridge|ground_truth_odometry|cmd_vel_adapter|vehicle_movement_interface|pointcloud_to_laserscan|slam_toolbox|controller_server|explore_node"
ros2 lifecycle get /controller_server
ros2 param get /controller_server controller_plugins
ros2 param get /controller_server FollowPath.plugin
ros2 param get /planner_server planner_plugins
ros2 param get /planner_server GridBased.plugin
timeout 8 ros2 topic hz /gen0_model/front3d/lidar/points
timeout 8 ros2 topic hz /scan
timeout 20 ros2 topic hz /map
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo --once /explore/status
ros2 topic echo --once /map --field info
ros2 topic echo --once /odom --field pose.pose.position
```

Expected:

- `/controller_server` is `active [3]`.
- `/controller_server FollowPath.plugin` is
  `nav2_mppi_controller::MPPIController` when the MPPI params file is used.
- `/planner_server GridBased.plugin` is `nav2_navfn_planner/NavfnPlanner`.
- `/scan` publishes at a few Hz and contains finite ranges.
- `/map` publishes after `slam_toolbox` starts and has nonzero width/height.
- `map -> base_footprint` prints translation and rotation.
- `/explore/status` is `exploration_started`, `exploration_in_progress`, or
  `exploration_complete`.
- While exploration is in progress, `/cmd_vel` and `/control/cmd_vel` may
  publish nonzero commands and `/odom` should change.

Stop here if `/scan` is all `inf` or `/map` reports `width: 0` and `height: 0`.
That is a Gazebo sensor/rendering input failure, not an exploration tuning issue.

Important: restarting `navigation_online_slam.launch.py` starts a fresh online
SLAM session. The white map in RViz is rebuilt from scratch unless you load a
saved static map with `navigation_ground_truth.launch.py`.

## Terminal 1: Start Gazebo, Bridge, and Robot Description

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
export IGN_PARTITION=gen0_roundabout_demo
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$PWD/gen0_gz_sim_ros2/gz_plugins/build"
export IGN_GAZEBO_RESOURCE_PATH="$PWD/install/gen0_main/share/gen0_main/meshes"

ros2 launch gen0_main spawn.launch.py \
  world:=san_roundabout \
  rviz:=false
```

## Terminal 2: Start Interface Layer

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch sweeper_integration interfaces.launch.py
```

Expected nodes include:

```text
/ground_truth_odometry
/cmd_vel_adapter
/ros_gz_bridge
/robot_state_publisher
```

## Terminal 3: Start Vehicle Command Interface

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run gen0_interface cmdvel_to_vehicle
```

This node consumes `/control/cmd_vel` and sends steering / wheel commands to the
Gazebo model.

## Mapping Workflow

Use this section when creating a new map.

### Terminal 4: Start Mapping

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch sweeper_integration mapping.launch.py
```

Expected mapping logs include:

```text
pointcloud_to_laserscan: Got a subscriber to laserscan
slam_toolbox: Registering sensor: [Custom Described Lidar]
```

### Terminal 5: Open RViz for Mapping

Use software rendering for RViz only.

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

unset MESA_D3D12_DEFAULT_ADAPTER_NAME
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=3.3COMPAT
export QT_XCB_GL_INTEGRATION=none

ros2 launch sweeper_integration rviz_mapping.launch.py
```

RViz settings:

```text
Fixed Frame: map
Map topic: /map
```

The default `rviz_mapping.launch.py` config is map-only. Use
`mapping_minimal.rviz` only when you need to inspect `/scan`; the scan display
can print TF timing warnings under WSL.

### Optional: Keyboard Teleop for Mapping

Use slow movement. Avoid fast spinning because the point-cloud-derived scan can
be only a few Hz under WSL.

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Mapping Health Checks

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

timeout 8 ros2 topic hz /gen0_model/front3d/lidar/points
timeout 8 ros2 topic hz /scan
ros2 topic echo --once /scan sensor_msgs/msg/LaserScan --field ranges
timeout 20 ros2 topic hz /map
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint
```

Expected:

- `/scan` is not all `0.5`.
- `/scan` is not all `inf`.
- `/map` publishes at about `0.5 Hz` if `map_update_interval` is `2.0`.
- `map -> base_footprint` eventually prints translation and rotation.

### Save Map

Save the map before stopping SLAM:

```bash
mkdir -p ~/gen0_maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/gen0_maps/san_roundabout_front3d_demo
```

Expected output files:

```text
~/gen0_maps/san_roundabout_front3d_demo.yaml
~/gen0_maps/san_roundabout_front3d_demo.pgm
```

## Navigation Workflow

Use this section after a map has been saved.

Stop mapping first:

- Stop `mapping.launch.py`.
- Stop `teleop_twist_keyboard`.
- Keep Gazebo, bridge, interfaces, and `cmdvel_to_vehicle` running.

### Start Nav2 With Saved Map

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch sweeper_integration navigation_ground_truth.launch.py \
  map:=/home/zjxue2007/gen0_maps/san_roundabout_front3d_demo.yaml
```

### Nav2 Health Checks

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list | grep -E "map_server|planner_server|controller_server|bt_navigator"
ros2 lifecycle get /map_server
ros2 lifecycle get /controller_server
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint
ros2 action list | grep navigate
```

Expected:

```text
/map_server
/planner_server
/controller_server
/bt_navigator
active [3]
active [3]
/navigate_to_pose
```

### Send a Small Test Goal

Use a nearby goal first. Example from the verified demo:

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 24.0, y: 31.6, z: 0.0}, orientation: {z: -0.113, w: 0.994}}}}"
```

Expected result:

```text
Goal accepted
Goal finished with status: SUCCEEDED
```

### Confirm Vehicle Motion

```bash
ros2 topic echo --once /odom --field pose.pose.position
sleep 5
ros2 topic echo --once /odom --field pose.pose.position
```

The position should change while Nav2 is executing the goal. Near the goal the
motion should become small or stop.

## Online SLAM Navigation Workflow

Use this section when testing the next layer: Nav2 driving while `slam_toolbox`
keeps building the map.

Keep these terminals running first:

- `spawn.launch.py` with `world:=san_roundabout rviz:=false`
- `interfaces.launch.py`
- `cmdvel_to_vehicle`

Stop these before starting online SLAM navigation:

- `mapping.launch.py`
- `navigation_ground_truth.launch.py`
- `teleop_twist_keyboard`

Do not run `mapping.launch.py` at the same time as
`navigation_online_slam.launch.py`; the online launch already starts
`pointcloud_to_laserscan` and `slam_toolbox`.

### Start Online SLAM + Nav2

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch sweeper_integration navigation_online_slam.launch.py
```

To run the currently verified MPPI controller configuration, pass:

```bash
ros2 launch sweeper_integration navigation_online_slam.launch.py \
  nav2_params_file:=$PWD/gen0_gz_sim_ros2/sweeper_integration/config/nav2_online_slam_mppi.yaml
```

Expected nodes include:

```text
/slam_toolbox
/pointcloud_to_laserscan
/controller_server
/planner_server
/bt_navigator
/velocity_smoother
```

### Online SLAM Navigation Health Checks

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list | grep -E "slam_toolbox|controller_server|planner_server|bt_navigator"
ros2 lifecycle get /controller_server
ros2 param get /controller_server FollowPath.plugin
ros2 param get /planner_server GridBased.plugin
timeout 8 ros2 topic hz /scan
timeout 20 ros2 topic hz /map
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint
```

Expected:

- `/controller_server` is `active [3]`.
- `FollowPath.plugin` is `nav2_mppi_controller::MPPIController` when using
  `nav2_online_slam_mppi.yaml`.
- `GridBased.plugin` is `nav2_navfn_planner/NavfnPlanner`.
- `/scan` contains finite values, not all `0.5` and not all `inf`.
- `/map` publishes after `slam_toolbox` registers the scan.
- `map -> base_footprint` prints the robot pose.

### Costmap Health Checks

The Nav2 stack has both a local and a global costmap. The local costmap is an
8 m x 8 m rolling window in the `odom` frame. The global costmap is in the
`map` frame and combines the online SLAM map with `/scan` obstacle updates.

Check that costmap nodes and topics exist:

```bash
ros2 node list | grep -E "local_costmap|global_costmap"
ros2 topic list | grep costmap
```

Expected topics include:

```text
/local_costmap/costmap
/local_costmap/costmap_raw
/global_costmap/costmap
/global_costmap/costmap_raw
```

### Send a Nearby Goal While Mapping

First read the current pose:

```bash
ros2 topic echo --once /odom --field pose.pose.position
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint
```

Then send a small nearby goal in the `map` frame. Use a point already visible in
RViz, not a far unknown area.

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 24.0, y: 31.6, z: 0.0}, orientation: {z: -0.113, w: 0.994}}}}"
```

For each test, replace `x`, `y`, `z`, and `w` with a nearby goal from the
current map. This is still manual goal navigation; autonomous frontier
selection comes after this layer is stable.

## Troubleshooting

### `/scan` is all `0.5`

This is the broken native 2D lidar path in the current WSL/Gazebo rendering
setup. Do not use `/gen0_model/fl/lidar/scan` or `/gen0_model/fr/lidar/scan` for
SLAM in this demo.

Use:

```text
/gen0_model/front3d/lidar/points -> pointcloud_to_laserscan -> /scan
```

### `/scan` is all `inf`

First check that Gazebo is open and the front 3D lidar is publishing:

```bash
timeout 8 ros2 topic hz /gen0_model/front3d/lidar/points
timeout 8 ros2 topic hz /scan
ros2 topic echo --once /scan sensor_msgs/msg/LaserScan --field ranges
```

If Gazebo is not open, recover Gazebo before debugging SLAM. If Gazebo is open
but `/scan` is all `inf`, the SLAM input is invalid.

If the original `ros_gz_sim gz_sim.launch.py` GUI path also fails with `Out of
GPU memory or driver refused`, the current blocker is WSL / GPU rendering
itself, not `slam_toolbox`, Nav2, or `m-explore-ros2`.

### `/map` Does Not Publish

Check `/odom` and TF first:

```bash
timeout 8 ros2 topic hz /odom
timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint
```

If `/odom` is missing, start:

```bash
ros2 launch sweeper_integration interfaces.launch.py
```

If `/odom` was started after `mapping.launch.py`, restart mapping.

### `/map` Is `0 x 0` or Nav2 Says Map Is Malformed

Check `/scan` immediately:

```bash
ros2 topic echo --once /scan sensor_msgs/msg/LaserScan --field ranges
ros2 topic echo --once /map --field info
```

If `/scan` is all `inf`, `slam_toolbox` has no usable obstacle returns. Nav2 may
then print:

```text
Received map message is malformed. Rejecting.
Robot is out of bounds of the costmap.
```

Do not tune `explore_lite` in this state. Restart Gazebo with a rendering mode
that gives finite `/scan` values first.

### Nav2 Does Not Move the Vehicle

Check command flow:

```bash
ros2 topic info /cmd_vel
ros2 topic info /control/cmd_vel
timeout 10 ros2 topic echo /cmd_vel
```

If `/cmd_vel` publishes but the vehicle does not move, verify that
`cmdvel_to_vehicle` is running:

```bash
ros2 node list | grep vehicle_movement_interface
```

Also stop `teleop_twist_keyboard` before Nav2 tests, so teleop does not compete
with Nav2 for `/cmd_vel`.

### RViz Is Slow or Crashes

Use software rendering for RViz:

```bash
unset MESA_D3D12_DEFAULT_ADAPTER_NAME
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=3.3COMPAT
export QT_XCB_GL_INTEGRATION=none
```

Keep Gazebo rendering variables separate from RViz rendering variables.

### RViz Shows OpenGL or TF Timing Errors

These RViz messages are usually display-layer noise in this WSL demo:

```text
Stereo is NOT SUPPORTED
active samplers with a different type refer to the same texture image unit
Message Filter dropping message
Lookup would require extrapolation
```

Interpretation:

- `Stereo is NOT SUPPORTED` is harmless.
- `Trying to create a map of size ...` means RViz is receiving `/map`.
- The GLSL sampler error is an RViz + llvmpipe OpenGL shader issue. If the map
  still appears, it is not a SLAM failure.
- Message-filter and extrapolation errors usually come from the `/scan` display
  and TF timestamps after simulation restarts.

For final validation, open RViz with:

```bash
ros2 launch sweeper_integration rviz_mapping.launch.py
```

That uses `mapping_map_only.rviz`. If you open `mapping_minimal.rviz`, disable
the `LaserScan` display when these timing messages become distracting.

## Next Engineering Steps

This demo proves the full software chain. The map quality is limited by the
current simulated environment and sensor placement:

- road curbs are too low for the current 3D-to-2D scan slice
- the scene is visually and geometrically complex
- WSL rendering limits sensor frequency and simulation performance

Recommended next steps:

- adjust world geometry, especially curb height and road boundaries
- tune or reposition lidar sensors
- tune `pointcloud_to_laserscan` height limits after sensor placement changes
- reduce map area or resolution for faster iteration
- add cleaner launch files for demo startup once the final sensor setup is
  chosen

## Frontier Exploration Workflow

Use this section after online SLAM navigation is already running and a small
manual `/navigate_to_pose` goal has succeeded.

The official repository is copied into this workspace at:

```text
gen0_gz_sim_ros2/m-explore-ros2
```

Most Gen0-specific settings live in `sweeper_integration`.

One local compatibility patch is applied inside the copied official source:

```text
gen0_gz_sim_ros2/m-explore-ros2/explore/src/costmap_client.cpp
```

It reads the latest available TF for the robot pose. This avoids
millisecond-level future extrapolation errors in Gazebo simulation.

### Build Official Explore Package

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-up-to explore_lite
colcon build --symlink-install --packages-select behavior_ext_plugins sweeper_integration

source install/setup.bash
```

### Start Explore

Keep these terminals running first:

- `spawn.launch.py` with `world:=san_roundabout rviz:=false`
- `interfaces.launch.py`
- `cmdvel_to_vehicle`
- `navigation_online_slam.launch.py`

Do not run `teleop_twist_keyboard` while testing autonomous exploration.

Then start the official frontier explorer with Gen0 parameters:

```bash
cd ~/gen0_gz_sim_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch sweeper_integration explore_gen0.launch.py
```

Expected:

- `/explore_node` starts.
- `/explore/frontiers` publishes visualization markers.
- `/navigate_to_pose` receives goals automatically.
- `/cmd_vel` publishes when a frontier goal is being executed.

### Explore Health Checks

```bash
ros2 node list | grep -E "explore_node|slam_toolbox|bt_navigator|controller_server"
ros2 topic info /explore/frontiers
timeout 20 ros2 topic echo /cmd_vel
ros2 action list | grep navigate
```

Pause exploration:

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
```

Resume exploration:

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
```

At this stage exploration is generic frontier exploration. It does not yet know
which unknown space is road and which unknown space is off-road. Road-aware
frontier filtering is a later custom layer.

Current Gen0 explore tuning:

- `map_update_interval: 2.0` updates the SLAM map about every 2 seconds.
- `planner_frequency: 0.25` lets `explore_lite` re-evaluate frontiers about
  every 4 seconds.
- `min_frontier_size: 0.5` keeps smaller frontier gaps eligible for demo
  exploration.
