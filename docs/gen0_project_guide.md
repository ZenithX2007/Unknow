# Gen0 Project Guide

This document records the current Gen0 runtime structure and verified startup
workflow. The normal full-stack entry point and startup sequence are unchanged.

## Architecture

`run_gen0_full_stack.sh` starts `run_gen0_relocalization.sh`, which starts
Gazebo, the `my_map` world, sensors, FAST-LIO, relocalization, pedestrians,
actor obstacle processing, and the base RViz. When enabled, the full-stack
script then starts Nav2, EPSILON, and the second RViz.

The base vehicle command is `/cmd_vel`. Navigation uses:

```text
Nav2 -> /control/nav2_cmd_vel_raw -> epsilon_cmd_vel_mux_node
     -> /control/cmd_vel_raw -> nav2_pose_guard -> /cmd_vel
     -> ros_gz_bridge -> Gazebo Ackermann steering
```

For Nav2-only diagnosis with EPSILON disabled, the mux is not started and the
command path is shortened to:

```text
Nav2 -> /control/nav2_cmd_vel_raw -> nav2_pose_guard -> /cmd_vel
     -> ros_gz_bridge -> Gazebo Ackermann steering
```

Important logs are under `/home/zjxue2007/Unknow/runtime_logs/`, especially
`relocalization_stack.log`, `relocalization_3d_slam.log`, `gazebo.log`, and
`fast_lio_3d_slam.log`.

## Current Verified State (2026-09-02)

- The normal entry point remains `run_gen0_full_stack.sh`; no startup entry
  point was changed.
- The `my_map` Gazebo vehicle spawn pose is restored to the original pose:
  `x=-20.6991, y=-22.4324, z=2.85, yaw=-0.5406`. The temporary 9 m forward
  spawn experiment was removed.
- The default pedestrian soft-stop hysteresis is now
  `GEN0_ACTOR_SOFT_STOP_MARGIN=1.5` and
  `GEN0_ACTOR_SOFT_STOP_RELEASE_MARGIN=1.9`. An actor is paused only while
  its predicted trajectory enters the vehicle safety envelope; the release
  threshold is larger to prevent rapid stop/restart oscillation.
- Moving actor poses are supplied to `/gen0_mapping/actor_obstacles` for Nav2.
  They are intentionally excluded from the simulated lidar input so that
  `odom_registered_scan` and terrain mapping do not accumulate a horizontal
  trail when an actor crosses the road.
- The Nav2 local inflation radius in the EPSILON profile is `0.6 m`, reduced
  from `1.2 m` to avoid unnecessarily closing narrow passages. The final
  `nav2_pose_guard` remains an emergency collision gate; normal pedestrian
  clearance and detours are handled by the actor costmap and Nav2 controller.
- The last parameter/documentation change was committed and pushed to
  `origin/gen0_humble` as commit `39ebd5b`. Other current worktree changes are
  intentionally not included in that commit.

## Normal Base Startup

Use this first to check Gazebo, map, sensors, vehicle, or pedestrians. Nav2 and
EPSILON are deliberately disabled.

```bash
cd /home/zjxue2007/Unknow
./stop_gen0_full_stack.sh
GEN0_WORLD=my_map \
GEN0_ACTOR_SOFT_STOP=true \
GEN0_ACTOR_SOFT_STOP_MARGIN=1.5 \
GEN0_ACTOR_SOFT_STOP_RELEASE_MARGIN=1.9 \
GEN0_START_EPSILON=false \
GEN0_START_NAV2=false \
./run_gen0_full_stack.sh
```

## Keyboard Control

In a second terminal while the base terminal is running:

```bash
cd /home/zjxue2007/Unknow
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run gen0_interface keyboard_teleop
```

Keys: `w` forward, `a` left, `d` right, `s` reverse, space stop, `q` quit.
The implementation is
`gen0_gz_sim_ros2/gen0_interface/gen0_interface/keyboard_teleop.py`.

## Automatic Low-Speed Drive

Do not run this together with keyboard teleoperation.

```bash
cd /home/zjxue2007/Unknow
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch gen0_main gen0_mapping_drive.launch.py enabled:=true drive_speed:=0.35 stop_distance:=2.5 slow_distance:=5.0
```

It reads `/gen0_model/front3d/lidar/points`, publishes `/cmd_vel`, slows inside
5 m, and stops inside 2.5 m. Implementation:
`gen0_gz_sim_ros2/gen0_interface/gen0_interface/mapping_drive.py`.

Stop with `Ctrl+C`, then publish a zero command on `/cmd_vel`.

## Nav2 and EPSILON

Use this only after base startup and manual vehicle motion are working:

```bash
cd /home/zjxue2007/Unknow
./stop_gen0_full_stack.sh
source /home/zjxue2007/miniconda3/etc/profile.d/conda.sh
conda activate yolo
PYTHONNOUSERSITE=1 \
GEN0_WORLD=my_map \
GEN0_ACTOR_SOFT_STOP=true \
GEN0_ACTOR_SOFT_STOP_MARGIN=1.5 \
GEN0_ACTOR_SOFT_STOP_RELEASE_MARGIN=1.9 \
GEN0_START_EPSILON=true \
GEN0_START_NAV2=true \
GEN0_QCNET_BACKEND=qcnet \
GEN0_QCNET_DEVICE=cuda \
./run_gen0_full_stack.sh
```

Set the initial pose in Nav2 RViz, then use `2D Goal Pose`, or send a nearby
goal in the `map` frame with the `/navigate_to_pose` action.

### QCNet Python Environment

The base shell environment may not contain PyTorch. Starting Nav2 and EPSILON
from `(base)` can therefore fail during the QCNet CUDA preflight with:
`PyTorch import failed: No module named 'torch'`. This is an environment issue,
not a Gazebo or Nav2 startup failure.

Activate the project environment before starting the full navigation stack:

```bash
source /home/zjxue2007/miniconda3/etc/profile.d/conda.sh
conda activate yolo
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

The CUDA check should print a PyTorch version and `True`. Then run the Nav2 and
EPSILON command above with `PYTHONNOUSERSITE=1`. The base-only startup with
`GEN0_START_EPSILON=false GEN0_START_NAV2=false` does not require QCNet.

For a temporary non-QCNet integration test only, use
`GEN0_QCNET_BACKEND=constant_velocity`; this does not validate the QCNet model
or GPU inference path.

### Nav2-only Control Verification

On 2026-09-02, Nav2-only startup was verified with
`GEN0_START_EPSILON=false` and `GEN0_START_NAV2=true`. Do not set
`GEN0_EPSILON_MUX_OUTPUT_CMD_VEL_TOPIC` to the Nav2 topic. When EPSILON is
disabled, `run_gen0_full_stack.sh` connects the Nav2 raw topic directly to the
pose guard.

The verification showed that Nav2 generated non-zero commands, the pose guard
reported `Pose guard cleared; forwarding Nav2 velocity commands.`, and
`/cmd_vel` had one publisher and one subscriber. The publisher was
`nav2_pose_guard`; the subscriber was the Gazebo `ros_gz_bridge`.

Seeing `Unknown topic /control/nav2_cmd_vel_raw` after the stack has stopped is
expected because the Nav2 publisher has exited. Similarly, `ros2 topic info
/map` may show zero publishers after shutdown even if `ros2 topic echo /map
--once` receives a transient-local latched map message.

The corresponding runtime log is `runtime_logs/nav2_epsilon.log`. If Nav2
reports `Failed to make progress` while the pose guard is forwarding commands,
the remaining issue is downstream vehicle motion or controller tuning. The
profile also reports that its `0.6 m` inflation radius is smaller than the
approximately `1.05 m` inscribed radius of the configured `4 m x 2 m`
footprint; this warning remains relevant for tuning.

## Pedestrian Loading

For `my_map`, the scenario is
`gen0_gz_sim_ros2/gen0_main/worlds/scenarios/my_map/walking_actors3.sdf`.

Before Gazebo starts, `actors_loader.py` removes existing actors from
`worlds/my_map/my_map.sdf`, copies actors from the selected scenario, and
writes the world file back. It does not move actors or reset the vehicle.
Gazebo then loads the world and the `ActorPose` plugin updates actor poses.

`pedestrian_2` and `pedestrian_3` have identical start and end poses, so they
are stationary in position by design. `pedestrian_6` starts at trajectory time
60 seconds. `pedestrian_1`, `4`, `5`, `7`, `8`, and `9` should change position.

Actor obstacles are published on `/gen0_mapping/actor_obstacles` for costmaps
and navigation-side obstacle processing. `actor_collision_monitor` publishes
passive near-miss and collision events; it does not stop or steer the vehicle.
These topics do not drive pedestrian motion.

## Actor Collision Proxies and Soft Stop

Gazebo actors provide animation and scripted motion, but the actor definitions
in `walking_actors3.sdf` do not contain a physical `<collision>` geometry. A
vehicle could therefore pass through the rendered pedestrian even when the
software distance check detected it.

`actors_loader.py` now adds one hidden kinematic collision model for each
loaded actor. The proxy is named `<actor_name>_collision_proxy` and contains a
1.6 m high, 0.45 m radius cylinder without a visual element. `ActorPose.cc`
keeps this proxy synchronized with the actor's current pose. The proxy is
independent of the optional actor soft-stop switch, so it continues to follow
actors even when `GEN0_ACTOR_SOFT_STOP=false`.

The resulting safety layers are:

```text
Actor animation/script -> ActorPose -> hidden collision proxy -> Gazebo Physics
                              |
                              +-> optional software soft-stop near the vehicle
```

The proxy prevents the vehicle's Gazebo collision geometry from passing
through the pedestrian. It is a physical safety boundary, not a path planner:
the vehicle controller may still continue publishing a forward command and
must be stopped or replanned by the control layer. A fresh Gazebo process is
required after changing or rebuilding `ActorPose`.

For a collision-proxy check, start the normal base system, move the vehicle
slowly with keyboard teleoperation toward a walking actor, and watch both
Gazebo and the actor pose topic. The relevant diagnostics are:

```bash
cd /home/zjxue2007/Unknow
tail -n 0 -f runtime_logs/gazebo.log | grep -E "Actor soft-stop|collision proxy|Skeleton animation"
```

The proxy is intentionally invisible. A physical stop without a software
`Actor soft-stop holding` message means Gazebo physics stopped the vehicle;
the latter message means the actor software boundary also held the pedestrian.

## Reset Semantics

The Gazebo vehicle spawn pose is stored in
`gen0_gz_sim_ros2/gen0_main/worlds/my_map/my_map.sdf` and is currently restored
to the original position: `x=-20.6991, y=-22.4324, z=2.85, yaw=-0.5406`.
An earlier test moved the vehicle 9 m forward along its local +X heading; that
temporary change has been removed. The source world and the installed world
are synchronized, and `gen0_main` was rebuilt after the restoration.

`GEN0_RELOCALIZATION_INITIAL_X/Y/Z/A` only configure the localization initial
guess. They do not move the Gazebo vehicle. RViz shows the localization result,
which can differ from the physical Gazebo pose. A fresh Gazebo process is
required to restore the SDF spawn pose.

## Current Issues

`runtime_logs/gazebo.log` contains repeated `Skeleton animation name not
found: walking` and `talking` errors. The DAE internal animation names do not
consistently match the SDF aliases. This error flood can severely lag Gazebo.

Git history shows that commit `a40d637` changed `ActorPose` from a publisher-only
plugin into a `PreUpdate` plugin that manually called `SetTrajectoryPose` for
every actor whenever `GEN0_ACTOR_SOFT_STOP=true`. Before that commit, Gazebo's
native actor script controlled all actor movement. That behavior caused the
regression where all pedestrians appeared frozen. The current implementation
only creates a manual trajectory override for an actor whose predicted next
pose enters the vehicle safety envelope; distant actors remain under Gazebo's
native script. A complete Gazebo restart is required after every plugin rebuild.

This diagnosis has been reproduced: with `GEN0_ACTOR_SOFT_STOP=false`, the
pedestrians move again and Gazebo no longer exhibits the severe GUI lag. The
current soft-stop implementation is now limited to the actor whose predicted
pose enters the vehicle safety envelope, while the collision proxy remains
active independently.

The current implementation does not yet perform cooperative path planning.
`mapping_drive.py` only changes forward speed from front point-cloud
clearance. Nav2 uses `/gen0_mapping/actor_obstacles` through its actor obstacle
layer. The full-stack default feeds this actor costmap with the live
`/actor/pedestrian_N/pose` topics, so stopped or delayed actors are represented
at their actual positions. These pose topics are intentionally not injected
into the simulated 3D lidar by default: Nav2 needs one lightweight actor cloud,
whereas duplicating actors into the high-volume lidar adds unnecessary load to
FAST-LIO, SCURM, and both costmaps. The actor cloud runs at 5 Hz with a compact
12-angle by 2-height cylinder representation per actor.

The local planner is expected to re-evaluate the updated costmap at runtime;
this still requires validation with a moving Nav2 goal and should not be
confused with a guarantee that the vehicle will physically avoid an actor.
Do not add a second direct publisher to `/cmd_vel`; a future pedestrian safety
gate should sit before the final vehicle command in the Nav2/EPSILON chain.

EPSILON messages about waiting for `ArenaInfoStatic` or `ArenaInfo` are a
navigation-side condition and do not diagnose base Gazebo startup.

## Diagnostics and Stop

After stopping, `pgrep -af "ign gazebo"` should produce no output before a
clean restart. While running, inspect `runtime_logs/gazebo.log`, echo `/cmd_vel`,
and check `/actor/pedestrian_1/pose` and `/actor/pedestrian_4/pose` rates.

Do not run keyboard teleoperation, mapping drive, and Nav2 simultaneously.

Stop the project with:

```bash
cd /home/zjxue2007/Unknow
./stop_gen0_full_stack.sh
```
