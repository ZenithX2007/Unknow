# EPSILON / QCNet Gap Audit

## Current State

- `vehicle_msgs`, `epsilon_core`, `epsilon_planning`, and `qcnet_prediction` are present under `epsilon_migration/` and are discoverable by `colcon`.
- `/home/zjxue2007/QCNet_AV2.ckpt` exists and matches the upstream QCNet AV2 checkpoint shape: `input_dim=2`, `output_dim=2`, `output_head=false`, `history=50`, `future=60`, `modes=6`.
- `epsilon_planning.launch.py` now starts the EPSILON scene bridge, the QCNet prediction bridge, and the integrated EPSILON planner together.
- `qcnet_prediction_node.py` now handles `Lane.child_id` and `Lane.father_id` as arrays and honors `publish_top_k_modes`.
- `epsilon_scene_bridge_node` now can publish dynamic actor vehicles from `PoseStamped` topics, not just the ego vehicle.
- QCNet prediction messages now use the latest dynamic scene frame instead of hard-coding `map`.
- `run_gen0_epsilon.sh` starts only the EPSILON/QCNet layer and performs preflight checks for odometry, projected costmap, and the Nav2 path.
- `run_gen0_full_stack.sh` supervises the 3D SLAM/Gazebo, Nav2, and EPSILON/QCNet scripts from one terminal.
- `stop_gen0_full_stack.sh` stops the supervised full stack and the individual sub-stacks.
- `verify_gen0_epsilon_qcnet.sh` performs static and runtime checks for the EPSILON/QCNet integration.

## Current Boundaries

- The full-stack script starts EPSILON/QCNet without waiting for a Nav2 path by default. EPSILON remains idle until `/plan_smoothed` exists, while Gazebo and Nav2 stay available for a goal.
- With `GEN0_ACTORS_SCENARIO=walking_actors3`, the launcher discovers the live `/actor/pedestrian_N/pose` `PoseStamped` topics and passes all discovered actors to the scene bridge and QCNet. Explicit `GEN0_EPSILON_ACTOR_POSE_TOPICS` still overrides discovery.
- `path_topic` defaults to `/plan_smoothed`. Nav2 supplies the global/reference route; EPSILON supplies the local behavior, obstacle avoidance, trajectory generation, and control. EPSILON is not a replacement for route planning.
- The scene bridge converts the projected costmap and `/gen0_perception/trash_poses` into static circular obstacles. Trash therefore remains a static obstacle as requested. Live actor poses become dynamic vehicles and feed QCNet/EPSILON.
- The full-stack launcher enables the command mux by default. Nav2 publishes `/control/nav2_cmd_vel_raw`, EPSILON publishes `/epsilon/cmd_vel_raw`, the mux publishes `/control/cmd_vel_raw`, and the existing `nav2_pose_guard` remains the only final publisher to `/cmd_vel`.
- QCNet receives a lane graph synthesized from the current reference path, not a full Argoverse lane graph. The pretrained checkpoint is loaded when its Python dependencies are available, but the input remains an integration adapter rather than the original dataset pipeline.
- QCNet uses a latest-snapshot fixed-rate loop: dynamic messages are not queued, and predictions carry the source scene timestamp plus relative future durations. The current host has CPU-only PyTorch, so the real pretrained model is not expected to sustain 10 Hz; use a CUDA-enabled PyTorch build for real-time QCNet. `constant_velocity` is available for plumbing tests.

## Suggested Startup Order

Build the migrated packages first:

```bash
colcon build --packages-select vehicle_msgs epsilon_core qcnet_prediction epsilon_planning
source install/setup.bash
```

```bash
GEN0_WORKSPACE=$PWD \
GEN0_WORLD=my_map \
GEN0_RELOCALIZATION=false \
GEN0_MAPPING_DRIVE=false \
./run_gen0_3d_slam.sh
```

Then start Nav2 and wait until it publishes `/plan_smoothed`:

```bash
GEN0_WORKSPACE=$PWD \
GEN0_NAV2_PROFILE=scurm_gen0 \
./run_gen0_nav2.sh
```

Then start EPSILON/QCNet:

```bash
GEN0_WORKSPACE=$PWD \
GEN0_ACTORS_SCENARIO=walking_actors3 \
GEN0_QCNET_BACKEND=auto \
GEN0_QCNET_CKPT_PATH=/home/zjxue2007/QCNet_AV2.ckpt \
./run_gen0_epsilon.sh
```

For an early plumbing test without QCNet dependencies:

```bash
GEN0_QCNET_BACKEND=constant_velocity ./run_gen0_epsilon.sh
```

## Verification

Static check before launching the stack:

```bash
./verify_gen0_epsilon_qcnet.sh --static
```

Runtime check after Gazebo/SLAM, Nav2, a Nav2 goal, and EPSILON/QCNet are running:

```bash
./verify_gen0_epsilon_qcnet.sh --runtime
```

One-terminal supervised startup:

```bash
cd /home/zjxue2007/Unknow
source /opt/ros/humble/setup.bash
source install/setup.bash
./stop_gen0_full_stack.sh

GEN0_WORLD=my_map \
GEN0_ACTORS_SCENARIO=walking_actors3 \
GEN0_GAZEBO_GUI=true \
GEN0_RVIZ=true \
GEN0_NAV2_RVIZ=true \
GEN0_QCNET_BACKEND=qcnet \
GEN0_QCNET_DEVICE=cuda \
./run_gen0_full_stack.sh
```

This is the preferred integrated launch path for the new stack:
relocalization base -> EPSILON/QCNet -> Nav2 sidecar.
Send a Nav2 goal in RViz after the window opens. Then run the runtime verification from a second terminal.

If you are only validating the planner plumbing without actors:

```bash
GEN0_VERIFY_EXPECT_ACTORS=false \
./verify_gen0_epsilon_qcnet.sh --runtime
```
