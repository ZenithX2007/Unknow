# SCURM Migration Status

This document records the comparison between:

- Source project: `/home/zjxue2007/SCURM_SentryNavigation`
- Gen0 project: `/home/zjxue2007/Unknow`

## Already Migrated

These SCURM packages are present in `scurm_migration/`:

| Source package | Current location | Status |
| --- | --- | --- |
| `FAST_LIO` | `scurm_migration/FAST_LIO` | Migrated with Gen0 runtime logging additions. |
| `livox_ros_driver2` | `scurm_migration/livox_ros_driver2` | Source-synchronized. |
| `terrain_analysis` | `scurm_migration/terrain_analysis` | Migrated with shutdown handling adapted for ROS 2 launch cleanup. |
| `terrain_analysis_ext` | `scurm_migration/terrain_analysis_ext` | Source-synchronized; original `.pcd` runtime data is not copied. |
| `sensor_scan_generation` | `scurm_migration/sensor_scan_generation` | Source-synchronized. |
| `costmap_intensity` | `scurm_migration/costmap_intensity` | Migrated with noisy out-of-map warning lowered to debug. |
| `behavior_ext_plugins` | `scurm_migration/behavior_ext_plugins` | Migrated and adapted for Gen0 Ackermann/non-holonomic recovery. |
| `icp_relocalization` | `scurm_migration/icp_relocalization` | Migrated in this pass and adapted for Gen0 launch/build. |

## Integrated Into Gen0 Runtime

Current Gen0 launch/config integration uses these SCURM pieces:

| Gen0 file | SCURM function |
| --- | --- |
| `gen0_gz_sim_ros2/gen0_main/launch/gen0_fast_lio_mapping.launch.py` | FAST-LIO, terrain analysis, terrain analysis ext, projected map, local costmap, optional ICP relocalization. |
| `gen0_gz_sim_ros2/gen0_main/config/fast_lio_gen0.yaml` | Gen0 LiDAR/IMU topics and FAST-LIO mapping parameters. |
| `gen0_gz_sim_ros2/gen0_main/config/scurm_local_costmap_gen0.yaml` | SCURM intensity costmap configured for `/gen0_mapping/terrain_map`. |
| `gen0_gz_sim_ros2/gen0_main/launch/gen0_navigation.launch.py` | Gen0 static-map Nav2 launch adapted from SCURM `sentry_bringup/navigation.launch.py`, with selectable Nav2 costmap source. |
| `gen0_gz_sim_ros2/gen0_main/config/nav2_gen0_params.yaml` | Gen0 Nav2 parameters with `BackUpTwzFree`; default obstacle source remains Gen0 2D LaserScan. |
| `gen0_gz_sim_ros2/gen0_main/behavior_tree/*.xml` | SCURM Nav2 behavior trees installed as Gen0 package resources. |
| `gen0_gz_sim_ros2/sweeper_integration/config/nav2_online_slam_mppi.yaml` | Online SLAM Nav2 MPPI config using `BackUpTwzFree`. |

## Deliberately Not Copied Yet

These source packages are not migrated into the Gen0 project:

| Source package | Reason |
| --- | --- |
| `sentry_bringup` | Mostly robot-specific launch, maps, RViz, and Nav2 params. Useful launch/config/behavior-tree parts have been adapted into `gen0_main` and `sweeper_integration`. Large PCD maps remain in the source project. |
| `sentry_description` | Original sentry robot URDF/meshes do not match the Gen0 Gazebo vehicle. |
| `cmd_chassis` | Original chassis command conversion is replaced by `gen0_interface`. |
| `rm_interfaces`, `auto_aim_interfaces` | RoboMaster/autonomy interfaces are not used by the current Gen0 mapping/navigation flow. |
| `rm_decision_cpp` | Behavior-tree decision logic depends on RoboMaster interfaces and the sentry stack. |
| `BehaviorTree.CPP` | Nav2 already provides behavior-tree runtime in this ROS 2 Humble environment. |
| `control_panel` | UI package is not needed for current simulation validation. |

## ICP Relocalization Notes

`icp_relocalization` has been added as a buildable package. The original launch
hard-coded `/home/sentry_ws/...`; the migrated launch defaults to:

```text
/home/zjxue2007/SCURM_SentryNavigation/sentry_bringup/maps/GlobalMap.pcd
```

Override it with `map_path:=...` or `prior_map_path:=...` when using a Gen0
map. The large source `.pcd` maps were not copied into this repository.

Enable ICP relocalization in the Gen0 FAST-LIO launch explicitly:

```bash
ros2 launch gen0_main gen0_fast_lio_mapping.launch.py \
  relocalization:=true \
  prior_map_path:=/path/to/prior_map.pcd \
  relocalization_initial_x:=0.0 \
  relocalization_initial_y:=0.0 \
  relocalization_initial_z:=0.0 \
  relocalization_initial_a:=0.0
```

The default `relocalization:=false` preserves the currently verified 3D SLAM
startup path.

The one-command script exposes the same integration through environment
variables:

```bash
GEN0_WORKSPACE=$PWD \
GEN0_RELOCALIZATION=true \
GEN0_PRIOR_MAP_PATH=/path/to/prior_map.pcd \
GEN0_RELOCALIZATION_INITIAL_X=0.0 \
GEN0_RELOCALIZATION_INITIAL_Y=0.0 \
GEN0_RELOCALIZATION_INITIAL_Z=0.0 \
GEN0_RELOCALIZATION_INITIAL_A=0.0 \
./run_gen0_3d_slam.sh
```

If ICP reports a high fitness score and does not publish `/icp_result`, FAST-LIO
will keep printing `Waiting for initial pose...`. In that state Nav2 costmap TF
warnings such as `base_link` to `odom` being disconnected are expected side
effects: relocalized FAST-LIO has not accepted an initial pose yet.

For Gen0, the migrated ICP node must convert each incoming scan from the front
LiDAR frame into `base_link` before matching the FAST-LIO prior map. The Gen0
launch sets `input_cloud_to_base=(1.9, 0.0, 1.9)`, disables SCURM's legacy
180-degree Livox roll correction, uses a `2.0 m` maximum correspondence
distance, and requires 3 consecutive low-error scans before publishing
`/icp_result`. The one-command script exposes these as:

```bash
GEN0_RELOCALIZATION_MAX_CORRESPONDENCE_DISTANCE=2.0
GEN0_RELOCALIZATION_INPUT_CLOUD_TO_BASE_X=1.9
GEN0_RELOCALIZATION_INPUT_CLOUD_TO_BASE_Y=0.0
GEN0_RELOCALIZATION_INPUT_CLOUD_TO_BASE_Z=1.9
GEN0_RELOCALIZATION_LEGACY_LIVOX_ROLL_180=false
GEN0_RELOCALIZATION_CONVERGED_COUNT_THRESHOLD=3
```

The source SCURM `GlobalMap.pcd` is robot/world specific. For the Gen0 `my_map`
simulation, first generate a matching prior map from a normal SLAM run:

```bash
GEN0_WORKSPACE=$PWD \
GEN0_RELOCALIZATION=false \
GEN0_FAST_LIO_PCD_SAVE=true \
GEN0_FAST_LIO_MAP_FILE_PATH=/tmp/my_map_prior.pcd \
./run_gen0_3d_slam.sh
```

After the vehicle has covered the area, save the accumulated FAST-LIO map from a
second terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /gen0_mapping/save_fast_lio_map std_srvs/srv/Trigger {}
```

Then stop the normal run and start relocalization against the generated map:

```bash
GEN0_WORKSPACE=$PWD \
GEN0_RELOCALIZATION=true \
GEN0_PRIOR_MAP_PATH=/tmp/my_map_prior.pcd \
./run_gen0_3d_slam.sh
```

Tune `GEN0_RELOCALIZATION_INITIAL_X/Y/Z/A` if the saved map frame and current
spawn pose are not close enough for ICP to converge.

## Nav2 Behavior Tree Notes

The SCURM behavior-tree XMLs under
`/home/zjxue2007/SCURM_SentryNavigation/sentry_bringup/behavior_tree` are
copied into:

```text
gen0_gz_sim_ros2/gen0_main/behavior_tree/
```

`gen0_main/setup.py` installs that directory, and
`gen0_navigation.launch.py` exposes:

```text
default_nav_to_pose_bt_xml
default_nav_through_poses_bt_xml
odom_topic
```

The default BT files are:

```text
$(find-pkg-share gen0_main)/behavior_tree/ackermann_scurm_recovery.xml
$(find-pkg-share gen0_main)/behavior_tree/ackermann_scurm_through_poses.xml
```

`ackermann_scurm_recovery.xml` keeps SCURM's
`ComputePath -> SmoothPath -> FollowPath` structure. The original SCURM
`GoalUpdater` is intentionally omitted in the Gen0 tree because live tests
showed it can block ordinary `NavigateToPose` goal acceptance in this stack.

When navigation is launched on top of the relocalized FAST-LIO stack, pass:

```bash
ros2 launch gen0_main gen0_navigation.launch.py \
  odom_topic:=/gen0_mapping/fast_lio/odom
```

For a safe handoff to Nav2 control, start the SLAM stack without the automatic
mapping drive:

```bash
GEN0_RELOCALIZATION=true \
GEN0_PRIOR_MAP_PATH=/tmp/my_map_prior.pcd \
GEN0_MAPPING_DRIVE=false \
./run_gen0_3d_slam.sh
```

The current launch-level validation confirms that Nav2 nodes activate with the
SCURM-aligned behavior tree and `BackUpTwzFree`. For the `scurm_gen0` profile,
`GEN0_NAV2_MAP_SOURCE` now defaults to `projected_map`, so Nav2's `/map` is
relayed from online `/projected_map` and the YAML map_server is moved to
`/map_yaml_unused`. This is the default validation path while the prior
yaml/pcd maps are incomplete or frame-misaligned.

## Nav2 Costmap Source Modes

The source SCURM Nav2 local costmap uses
`costmap_intensity::ObstacleLayerIntensity` on `/terrain_map`. The first Gen0
Nav2 bringup kept the controller and planner stable by using the existing Gen0
2D LaserScan topics:

```text
/gen0_model/fl/lidar/scan
/gen0_model/fr/lidar/scan
```

That default remains:

```bash
GEN0_NAV2_COSTMAP_SOURCE=laser_scan ./run_gen0_nav2.sh
```

The SCURM-aligned Gen0 profile is now the main migration path:

```bash
./stop_gen0_nav2.sh

GEN0_WORKSPACE=$PWD \
GEN0_NAV2_PROFILE=scurm_gen0 \
./run_gen0_nav2.sh
```

Gazebo's native `AckermannSteering` plugin receives `/cmd_vel` directly through
`ros_gz_bridge`. No vehicle command adapter is started.

Navigation execution now uses Gen0 Ackermann-aware validation defaults:

- Nav2 `FollowPath` uses `nav2_mppi_controller::MPPIController` with
  `motion_model: Ackermann` and `min_turning_r: 6.62`.
- Velocity smoothing limits forward speed to `0.65 m/s` and angular speed to
  `0.07 rad/s`, matching the standard front-steering Ackermann geometry.
- Nav2 `xy_goal_tolerance` is `0.35 m`; final yaw is ignored for position-only
  goal testing.

In this mode `gen0_navigation.launch.py` generates
`/tmp/gen0_nav2_costmap_overlay.yaml` and overlays Nav2 params so:

- `/local_costmap` uses `costmap_intensity::ObstacleLayerIntensity`.
- The obstacle input is `/gen0_mapping/terrain_map`.
- `/global_costmap` uses a StaticLayer subscribed to `/map`, which in
  `scurm_gen0` comes from online `/projected_map`.

The standalone SCURM costmap launched by `gen0_fast_lio_mapping.launch.py`
still publishes `/costmap/costmap`. The opt-in mode above is the path that lets
Nav2's own `/local_costmap` consume the migrated SCURM terrain pipeline
directly.

For relocalized Nav2 tests, `run_gen0_3d_slam.sh` must leave
`ground_truth_localization` disabled. The Gazebo `/pose_publisher` publishes a
ground-truth `map -> odom` transform; if it is active alongside the ICP
transform publisher, RViz/Nav2 can appear to rotate or drag the map as the
vehicle moves. The wrapper now defaults both `GEN0_GROUND_TRUTH_LOCALIZATION`
and `GEN0_STATIC_ODOM_BASE` to `false`, and `run_gen0_nav2.sh` refuses to start
unless this ground-truth node is absent.

## Current Validation

Validated on July 30, 2026:

- ICP relocalization accepts `/tmp/my_map_prior.pcd` and FAST-LIO publishes
  `/gen0_mapping/fast_lio/odom`.
- `run_gen0_nav2.sh` waits for `odom -> base_link` before starting Nav2.
- Nav2 reaches `Managed nodes are active`.
- `controller_server` and `planner_server` reach lifecycle state `active [3]`.
- `/local_costmap/costmap_raw` publishes continuously.
- `/compute_path_to_pose` succeeds against the generated Gen0 blank map.
- `/cmd_vel` is the single ROS command topic bridged to Gazebo.
- `/gen0_model/ackermann/odom` is available as native Gazebo Ackermann odometry.

The generated `/tmp/gen0_my_map_nav.yaml` is intentionally blank. It verifies
Nav2 lifecycle, TF, behavior-tree, planner, controller, and local obstacle
plumbing, but it does not encode road boundaries. Road-following global paths
require an aligned occupancy map or a projected-map-to-Nav2-map export stage.

If the vehicle collides with a wall, the subsequent FAST-LIO map and terrain
clouds should be treated as contaminated for that run. Restart the 3D SLAM
stack after a collision before validating map quality or final goal accuracy.

The repeated `Message Filter dropping message ... timestamp ... earlier than all
the data in the transform cache` messages are not a bringup failure in the
validated setup. They indicate old buffered 2D lidar messages being dropped
while Nav2 remains active. The `scurm_terrain` costmap mode is intended to move
Nav2 local obstacle input onto the migrated SCURM terrain pipeline and should be
validated before making it the default.
