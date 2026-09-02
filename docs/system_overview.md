# Gen0 System Overview

This document is a map of the current autonomous mapping and navigation demo.
It is meant to answer: what runs, what algorithm is used, and where to look.

## Current Demo Flow

```text
Gazebo world + Gen0 vehicle
-> ros_gz_bridge
-> /gen0_model/front3d/lidar/points
-> pointcloud_to_laserscan
-> /scan
-> slam_toolbox
-> /map and map -> odom
-> nav2_lifecycle_bringup activates Nav2
-> explore_lite frontier selection
-> Nav2 NavigateToPose goals
-> local and global Nav2 costmaps
-> NavfnPlanner global path
-> MPPI local control with Ackermann constraints
-> /cmd_vel
-> ros_gz_bridge
-> Gazebo AckermannSteering
-> Gen0 steering and wheel joints
```

## Main Startup Sequence

Use `docs/demo_workflow.md` as the runnable command reference. The complete
autonomous mapping check uses these terminals:

```text
1. gen0_main spawn.launch.py
2. sweeper_integration interfaces.launch.py
3. sweeper_integration navigation_online_slam.launch.py
4. sweeper_integration explore_gen0.launch.py
```

## Packages

### gen0_main

Purpose: Gazebo world, vehicle model, sensors, ROS-Gazebo bridge, actor loader.

Important files:

```text
gen0_gz_sim_ros2/gen0_main/launch/spawn.launch.py
gen0_gz_sim_ros2/gen0_main/config/bridge.yaml
gen0_gz_sim_ros2/gen0_main/urdf/gen0_model.sdf
gen0_gz_sim_ros2/gen0_main/worlds/san_roundabout/san_roundabout.sdf
gen0_gz_sim_ros2/gen0_main/models/
gen0_gz_sim_ros2/gen0_main/meshes/
```

Do not start by studying all model and mesh files. They are large Gazebo assets
and are not the best place to learn the navigation algorithm.

### sweeper_integration

Purpose: integration layer for odometry, SLAM, Nav2, RViz, and exploration.

Important files:

```text
gen0_gz_sim_ros2/sweeper_integration/launch/interfaces.launch.py
gen0_gz_sim_ros2/sweeper_integration/launch/navigation_online_slam.launch.py
gen0_gz_sim_ros2/sweeper_integration/launch/explore_gen0.launch.py
gen0_gz_sim_ros2/sweeper_integration/config/pointcloud_to_laserscan.yaml
gen0_gz_sim_ros2/sweeper_integration/config/gen0_slam.yaml
gen0_gz_sim_ros2/sweeper_integration/config/nav2_online_slam.yaml
gen0_gz_sim_ros2/sweeper_integration/config/nav2_online_slam_mppi.yaml
gen0_gz_sim_ros2/sweeper_integration/config/explore_gen0.yaml
gen0_gz_sim_ros2/sweeper_integration/sweeper_integration/ground_truth_odometry.py
gen0_gz_sim_ros2/sweeper_integration/sweeper_integration/nav2_lifecycle_bringup.py
```

This is the first package to read when learning the current navigation system.

### gen0_interface

Purpose: manual mapping-drive and keyboard teleoperation tools. Gazebo's native
Ackermann plugin consumes `/cmd_vel` through `ros_gz_bridge`.

Important file:

```text
gen0_gz_sim_ros2/gen0_interface/gen0_interface/mapping_drive.py
gen0_gz_sim_ros2/gen0_interface/gen0_interface/keyboard_teleop.py
```

### m-explore-ros2

Purpose: third-party `explore_lite` frontier exploration source.

Important note: this is vendored third-party code. Only `explore`,
`explore_lite_msgs`, README, and LICENSE are kept for the current demo.

## Algorithms Currently Used

Runtime note: `navigation_online_slam.launch.py` still defaults to
`nav2_online_slam.yaml`, which uses Regulated Pure Pursuit. The current MPPI
test run was started with `nav2_online_slam_mppi.yaml`, and runtime parameters
confirmed:

```text
/controller_server controller_plugins: ['FollowPath']
/controller_server FollowPath.plugin: nav2_mppi_controller::MPPIController
```

Use runtime parameters as the source of truth when checking which controller is
actually active.

### SLAM

Algorithm/package:

```text
slam_toolbox async_slam_toolbox_node
```

Input:

```text
/scan
/odom
tf: odom -> base_footprint
```

Output:

```text
/map
tf: map -> odom
```

Configuration:

```text
gen0_gz_sim_ros2/sweeper_integration/config/gen0_slam.yaml
```

### Point Cloud To LaserScan

Algorithm/package:

```text
pointcloud_to_laserscan
```

Input:

```text
/gen0_model/front3d/lidar/points
```

Output:

```text
/scan
```

Configuration:

```text
gen0_gz_sim_ros2/sweeper_integration/config/pointcloud_to_laserscan.yaml
```

### Global Planning

Algorithm/package:

```text
Nav2 NavfnPlanner through the GridBased planner plugin
```

Current setting:

```text
use_astar: false
allow_unknown: true
```

Interpretation: this is a traditional grid-costmap global planner. With
`use_astar: false`, it is closer to Dijkstra / wavefront potential planning than
A*.

Configuration:

```text
gen0_gz_sim_ros2/sweeper_integration/config/nav2_online_slam.yaml
gen0_gz_sim_ros2/sweeper_integration/config/nav2_online_slam_mppi.yaml
```

### Local Control

Algorithm/package:

```text
Nav2 MPPIController
```

Interpretation: MPPI samples short forward trajectories, scores them with
critics, and chooses the next velocity command. The current MPPI configuration
uses the Ackermann motion model, no lateral velocity, forward-only motion, and a
minimum turning radius for the Gen0 vehicle.

Current MPPI settings:

```text
motion_model: Ackermann
min_turning_r: 6.62
vx_min: 0.0
vx_max: 0.30
vy_max: 0.0
wz_max: 0.07
critics: Constraint, Cost, Goal, GoalAngle, PathAlign, PathFollow, PathAngle,
         PreferForward, VelocityDeadband
```

Configuration:

```text
gen0_gz_sim_ros2/sweeper_integration/config/nav2_online_slam_mppi.yaml
```

The older online-SLAM configuration remains available:

```text
gen0_gz_sim_ros2/sweeper_integration/config/nav2_online_slam.yaml
```

It uses `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`.

### Costmaps And Obstacle Handling

Yes, the current Nav2 stack has costmaps.

Local costmap:

```text
frame: odom
type: rolling window
size: 8 m x 8 m
resolution: 0.05 m
robot_radius: 0.7 m
plugins: ObstacleLayer, InflationLayer
observation source: /scan
```

Global costmap:

```text
frame: map
resolution: 0.05 m
track_unknown_space: true
plugins: StaticLayer, ObstacleLayer, InflationLayer
map source: /map from slam_toolbox
observation source: /scan
```

Current method:

```text
Nav2 costmap_2d ObstacleLayer
Nav2 costmap_2d InflationLayer
MPPI CostCritic and ConstraintCritic
Nav2 recovery behaviors in the MPPI tree: DriveOnHeading, Wait
```

Input:

```text
/scan
```

Limitation: this is basic costmap-based obstacle handling. There is not yet a
separate measurable safety layer such as collision monitor, emergency stop, or
dynamic obstacle evaluation.

### Sensors

The Gen0 model currently exposes these sensor or truth topics through
`ros_gz_bridge`:

```text
/gen0_model/front3d/lidar/points     PointCloud2, used by current SLAM/Nav2
/scan                                LaserScan derived from the 3D lidar, used
/gen0_model/links/poses              PoseArray, used to generate /odom
/gen0_model/front_camera             Image, bridged but unused
/gen0_model/camera_info              CameraInfo, bridged but unused
/gen0_model/imu/data                 Imu, bridged but unused
/gen0_model/fix                      NavSatFix, bridged but unused
/gen0_model/fl/lidar/scan            LaserScan, bridged but unused in MPPI demo
/gen0_model/fr/lidar/scan            LaserScan, bridged but unused in MPPI demo
```

`/gen0_model/links/poses` is Gazebo ground truth, not a physical sensor model.
It is currently used by `ground_truth_odometry.py` to publish `/odom` and
`odom -> base_footprint`. A more realistic stack would replace or fuse this
with wheel odometry, IMU, GPS, or localization.

### Frontier Exploration

Algorithm/package:

```text
explore_lite
```

Input:

```text
/map
tf: map -> base_footprint
```

Output:

```text
Nav2 /navigate_to_pose goals
/explore/status
/explore/frontiers
```

Configuration:

```text
gen0_gz_sim_ros2/sweeper_integration/config/explore_gen0.yaml
```

Limitation: this is generic frontier exploration. It does not yet understand
trash targets, cleaning coverage, road-only areas, or task priority.

## What Is Not Implemented Yet

The current system does not yet include:

```text
trash detection
trash localization
cleaning or grabbing behavior
cleaned-area accounting
dynamic obstacle safety metrics
emergency-stop response timing
large-model task decomposition
voice / app / BCI interaction
Journey 6 deployment
```

For the competition, the next practical direction is to add:

```text
trash target simulation
trash detection and localization
task state machine: explore -> detect -> navigate -> clean -> mark complete
safety monitor and evaluation logger
```
