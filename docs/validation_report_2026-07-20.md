# Autonomous Mapping Validation Report - 2026-07-20

This report records the validation run performed before project organization.
No algorithm code was changed before this run.

## Environment

Workspace:

```text
/home/zjxue2007/gen0_gz_sim_ros2
```

ROS / simulator:

```text
ROS 2 Humble
Gazebo Fortress / Ignition Gazebo 6
WSL
```

Gazebo rendering environment used:

```bash
unset LIBGL_ALWAYS_SOFTWARE
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
unset MESA_GL_VERSION_OVERRIDE
unset QT_XCB_GL_INTEGRATION

export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export IGN_PARTITION=codex整理验证
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$PWD/gen0_gz_sim_ros2/gz_plugins/build"
export IGN_GAZEBO_RESOURCE_PATH="$PWD/install/gen0_main/share/gen0_main/meshes"
```

## Launch Sequence Tested

```bash
ros2 launch gen0_main spawn.launch.py world:=san_roundabout rviz:=false
ros2 launch sweeper_integration interfaces.launch.py
ros2 run gen0_interface cmdvel_to_vehicle
ros2 launch sweeper_integration navigation_online_slam.launch.py
ros2 launch sweeper_integration explore_gen0.launch.py
```

## Passed Checks

Gazebo and bridge:

```text
Gazebo opened with NVIDIA D3D12 adapter.
ros_gz_bridge created the expected sensor and control bridges.
No startup Mesa/swrast crash occurred.
```

Required nodes were present:

```text
/ros_gz_bridge
/robot_state_publisher
/ground_truth_odometry
/cmd_vel_adapter
/vehicle_movement_interface
/pointcloud_to_laserscan
/slam_toolbox
/controller_server
/planner_server
/bt_navigator
/explore_node
```

Nav2 lifecycle states:

```text
/controller_server: active [3]
/planner_server: active [3]
/bt_navigator: active [3]
```

Sensor and map rates:

```text
/gen0_model/front3d/lidar/points: about 1.7 Hz during the sample window
/scan: about 1.4-1.7 Hz during the sample window
/map: 0.5 Hz
```

Map output:

```text
resolution: 0.05 m
width: 1031
height: 956
origin x: -33.97405659143661
origin y: -54.13180431201508
```

Exploration state:

```text
/explore/status: exploration_started
```

Vehicle motion:

```text
/odom before: x=-12.763, y=-25.960
/odom after 8s: x=-13.064, y=-25.930
Nav2 log also showed movement from about (-20.66, -22.45) to (-7.85, -28.63)
during the full exploration run.
```

Velocity command flow:

```text
/cmd_vel published nonzero commands.
/control/cmd_vel published nonzero commands after cmd_vel_adapter.
```

TF:

```text
map -> base_footprint was available.
```

## Important Runtime Issues

The system runs and builds a map, but autonomous exploration is not yet stable.
Observed issues:

```text
controller_server frequently missed its 10 Hz control loop target.
controller_server reported TF future extrapolation.
planner_server repeatedly failed to create some Navfn plans.
explore_lite preempted goals frequently while selecting frontiers.
recovery spin sometimes failed before wait or backup recovery.
```

Interpretation:

```text
The current software chain is functional, but tuning is still needed for stable
long-running autonomous exploration.
```

Likely next tuning areas:

```text
TF timing and transform tolerances
SLAM/map update timing
Nav2 planner tolerance and frontier goal selection
controller frequency relative to WSL/Gazebo performance
frontier blacklist/goal filtering
```

## Shutdown-Only Issues

These appeared after Ctrl-C shutdown:

```text
cmdvel_to_vehicle: rcl_shutdown already called
cmd_vel_adapter: publisher context invalid during final zero command publish
ground_truth_odometry: rcl_shutdown already called
actors_loader: KeyboardInterrupt stack trace
Gazebo GUI: OGRE material shutdown segfault
```

Interpretation:

```text
These are cleanup/shutdown problems, not startup blockers. They should be fixed
before a polished demo, but they did not prevent the validation run.
```

## Validation Conclusion

```text
PASS for current baseline:
Gazebo + bridge + odometry + pointcloud_to_laserscan + slam_toolbox + Nav2 +
explore_lite + vehicle command flow all start and operate together.

LIMITED PASS for autonomy quality:
The vehicle moves and the map grows, but Nav2/explore stability needs tuning.
```

## Post-Cleanup Validation

After cleanup and rebuild, the final verification used:

```text
IGN_PARTITION=codex_cleanup_final
world=san_roundabout
rviz=false
MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
```

Build result:

```text
Gazebo ActorPose plugin: built successfully.
ROS packages: explore_lite_msgs, explore_lite, gen0_main, gen0_interface,
sweeper_integration built successfully.
```

Startup result:

```text
Gazebo GUI opened.
ros_gz_bridge created expected sensor and joint command bridges.
ground_truth_odometry reported PoseArray size 17 with pose_index 15.
nav2_lifecycle_bringup waited for /map, then configured and activated all Nav2
lifecycle nodes automatically.
lifecycle_manager_navigation exited cleanly on shutdown.
```

Final sampled states:

```text
/controller_server: active [3]
/planner_server: active [3]
/bt_navigator: active [3]
/velocity_smoother: active [3]
/scan: about 3.9-4.5 Hz during the sample window
/map: resolution 0.05 m, width 1247, height 1199
map -> base_footprint: available
```

Autonomous exploration result:

```text
explore_lite connected to Nav2 and sent NavigateToPose goals.
Nav2 log showed movement from about (-20.66, -22.45) to about (-1.75, -39.20).
The online map grew during exploration.
explore_lite eventually reported all frontiers traversed/tried and stopped.
```

Cleanup result:

```text
cmdvel_to_vehicle no longer prints rcl_shutdown already called on Ctrl-C.
cmd_vel_adapter and ground_truth_odometry no longer print rcl_shutdown already
called tracebacks on Ctrl-C.
actors_loader was verified with headless Gazebo shutdown and exited cleanly.
```

Remaining issues after cleanup:

```text
NavfnPlanner still fails on some frontier goals.
The controller still misses its desired loop rate under WSL/Gazebo load.
Occasional TF future extrapolation can still appear during exploration.
Gazebo GUI still segfaults in OGRE material cleanup when closing the GUI.
```
