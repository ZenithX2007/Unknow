# my_map 启动说明

本文档面向接手该工程的人，记录当前已经验证过的 `my_map` 启动方式和常见配置坑。

当前验证环境：

```text
Ubuntu 22.04 / WSL
ROS 2 Humble
Gazebo Fortress / Ignition Gazebo 6
地图: gen0_main/worlds/my_map/my_map.sdf
渲染引擎: OGRE1, Gazebo 参数写法为 ogre
```

## 关键结论

- `my_map` 已经可以被 Gazebo GUI 正常加载。
- `spawn.launch.py` 的默认地图已经改为 `my_map`。
- Gazebo 启动参数已经强制使用 OGRE1：`--render-engine ogre`。
- WSL 下不要让 Gazebo 使用软件渲染，否则 GUI 会非常卡。
- 当前电脑应使用 NVIDIA 4060 对应的 Mesa D3D12 适配器。
- `LIBGL_ALWAYS_SOFTWARE=1` 只适合 RViz 单独终端，不适合 Gazebo GUI。

## 一键启动

如果只想构建插件、构建 ROS 包并启动 Gazebo，可直接运行：

```bash
cd ~/gen0_gz_sim_ros2_v2
./gzbuild.sh
```

`gzbuild.sh` 当前会做这些事：

```text
1. 编译 gen0_gz_sim_ros2/gz_plugins
2. 设置 IGN_GAZEBO_SYSTEM_PLUGIN_PATH
3. source ROS 2 Humble
4. colcon build
5. source install/setup.bash
6. 取消 Gazebo 不该使用的软件渲染变量
7. 指定 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
8. 启动 my_map
```

## 推荐手动启动

手动启动更适合调试。每次新开终端后执行：

```bash
cd ~/gen0_gz_sim_ros2_v2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

设置 Gazebo GUI 的 NVIDIA 渲染环境：

```bash
unset LIBGL_ALWAYS_SOFTWARE
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
unset MESA_GL_VERSION_OVERRIDE
unset QT_XCB_GL_INTEGRATION
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$PWD/gen0_gz_sim_ros2/gz_plugins/build"
export IGN_GAZEBO_RESOURCE_PATH="$PWD/install/gen0_main/share/gen0_main/meshes"
```

启动 `my_map`：

```bash
ros2 launch gen0_main spawn.launch.py \
  world:=my_map \
  gazebo_gui:=true \
  rviz:=false \
  ground_truth_localization:=true
```

如果只想确认地图能加载、不需要 GUI：

```bash
ros2 launch gen0_main spawn.launch.py \
  world:=my_map \
  gazebo_gui:=false \
  rviz:=false \
  ground_truth_localization:=true
```

## 首次构建

如果 `install/` 不存在，或换了机器，需要先构建：

```bash
cd ~/gen0_gz_sim_ros2_v2
source /opt/ros/humble/setup.bash
cmake -S gen0_gz_sim_ros2/gz_plugins -B gen0_gz_sim_ros2/gz_plugins/build
cmake --build gen0_gz_sim_ros2/gz_plugins/build -j2
colcon build --symlink-install --packages-ignore race_plan_control
source install/setup.bash
```

## 检查是否用上 NVIDIA

启动 Gazebo 前可以检查 OpenGL renderer：

```bash
unset LIBGL_ALWAYS_SOFTWARE
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
unset MESA_GL_VERSION_OVERRIDE
unset QT_XCB_GL_INTEGRATION
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
glxinfo -B | grep -E "OpenGL renderer|Accelerated|Device:"
```

正常情况下应看到类似：

```text
Device: D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)
Accelerated: yes
OpenGL renderer string: D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)
```

如果看到下面这种结果，说明仍然是软件渲染，Gazebo GUI 会很卡：

```text
OpenGL renderer string: llvmpipe
Accelerated: no
```

## 多终端运行完整 demo

如果后续要启动接口、SLAM、Nav2、探索等多个终端，建议所有终端使用同一个 Ignition partition：

```bash
export IGN_PARTITION=my_map_demo
```

终端 1，Gazebo：

```bash
cd ~/gen0_gz_sim_ros2_v2
source /opt/ros/humble/setup.bash
source install/setup.bash
export IGN_PARTITION=my_map_demo

unset LIBGL_ALWAYS_SOFTWARE
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
unset MESA_GL_VERSION_OVERRIDE
unset QT_XCB_GL_INTEGRATION
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$PWD/gen0_gz_sim_ros2/gz_plugins/build"
export IGN_GAZEBO_RESOURCE_PATH="$PWD/install/gen0_main/share/gen0_main/meshes"

ros2 launch gen0_main spawn.launch.py world:=my_map gazebo_gui:=true rviz:=false ground_truth_localization:=true
```

终端 2，接口层：

```bash
cd ~/gen0_gz_sim_ros2_v2
source /opt/ros/humble/setup.bash
source install/setup.bash
export IGN_PARTITION=my_map_demo
ros2 launch sweeper_integration interfaces.launch.py
```

终端 3，车辆命令转换：

```bash
cd ~/gen0_gz_sim_ros2_v2
source /opt/ros/humble/setup.bash
source install/setup.bash
export IGN_PARTITION=my_map_demo
ros2 run gen0_interface cmdvel_to_vehicle
```

后续 SLAM、Nav2、探索流程可参考 `docs/demo_workflow.md`，但把旧文档里的 `world:=san_roundabout` 替换为 `world:=my_map`。

## 快速验证

Gazebo 启动后，新开一个终端：

```bash
cd ~/gen0_gz_sim_ros2_v2
source /opt/ros/humble/setup.bash
source install/setup.bash
ign topic -l | grep /world/default/stats
ros2 node list
```

至少应看到：

```text
/world/default/stats
/ros_gz_bridge
/robot_state_publisher
```

如果启用了 `ground_truth_localization:=true`，还应看到：

```text
/pose_publisher
```

## 常见问题

### Gazebo GUI 能打开但很卡

优先检查是否被软件渲染变量污染：

```bash
env | grep -E "LIBGL_ALWAYS_SOFTWARE|MESA_LOADER_DRIVER_OVERRIDE|GALLIUM_DRIVER|MESA_D3D12_DEFAULT_ADAPTER_NAME"
```

Gazebo 终端里不要有：

```bash
LIBGL_ALWAYS_SOFTWARE=1
MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
GALLIUM_DRIVER=llvmpipe
```

如果 `~/.bashrc` 里全局写了 `export LIBGL_ALWAYS_SOFTWARE=1`，Gazebo 终端必须先 `unset LIBGL_ALWAYS_SOFTWARE`。更好的做法是不要把它作为全局默认，只在 RViz 专用终端里设置。

### 任务管理器里 NVIDIA 不工作

在 WSL 终端检查：

```bash
nvidia-smi
```

如果 `nvidia-smi` 能看到 RTX 4060，但 `glxinfo -B` 显示 `llvmpipe`，说明是 OpenGL 渲染环境变量问题，不是 NVIDIA 驱动完全不可用。

### 不要把 RViz 的软件渲染配置用于 Gazebo

`sweeper_integration/launch/rviz_mapping.launch.py` 会给 RViz 设置软件渲染变量，这是为了让 RViz 稳定显示。这个配置不要复制到 Gazebo 启动终端。

### my_map 材质警告

`my_map.mtl` 里还存在一些 Windows 绝对路径的 `map_Bump` 法线贴图引用。当前测试表明这不阻止地图加载，但后续可以清理这些路径以减少 Gazebo 材质加载警告。

## 当前相关文件

- `gen0_gz_sim_ros2/gen0_main/worlds/my_map/my_map.sdf`
- `gen0_gz_sim_ros2/gen0_main/worlds/my_map/my_map.obj`
- `gen0_gz_sim_ros2/gen0_main/worlds/my_map/my_map.mtl`
- `gen0_gz_sim_ros2/gen0_main/launch/spawn.launch.py`
- `gen0_gz_sim_ros2/gz_plugins/`
- `gzbuild.sh`
