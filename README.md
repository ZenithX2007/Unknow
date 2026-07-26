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

After the workspace is built, the tested 3D SLAM flow can be started with:

```bash
cd ~/gen0_gz_sim_ros2
./run_gen0_3d_slam.sh
```

The script launches Gazebo, FAST-LIO, the mapping drive node, a lightweight RViz
point-cloud preview, and RViz. It also clears software-rendering environment
variables and selects the NVIDIA D3D12 adapter by default.

Useful runtime overrides:

```bash
GEN0_WORLD=san_roundabout GEN0_ACTORS_SCENARIO=walking_actors ./run_gen0_3d_slam.sh
GEN0_DRIVE_SPEED=0.15 ./run_gen0_3d_slam.sh
GEN0_PREVIEW_MAX_POINTS=25000 ./run_gen0_3d_slam.sh
```

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
  ground_truth_localization:=true \
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
