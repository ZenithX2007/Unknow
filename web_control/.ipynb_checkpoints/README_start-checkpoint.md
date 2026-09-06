# Gen0 ROS2 网页控制台

这个目录附带一个静态网页，直接走 ROS 2 的 rosbridge 链路控制当前仓库里的 Gen0 车体：

- 手动驾驶：发布 `geometry_msgs/msg/Twist` 到 `/cmd_vel`
- 自动驾驶：发送 `nav2_msgs/action/NavigateToPose` 到 `/navigate_to_pose`
- 相机查看：订阅 `/gen0_model/front_camera`
- 地图和状态：订阅 `/map` 与 `/odom`

默认 rosbridge 地址为 `ws://localhost:9090`，浏览器页面默认指向这个端口。相机支持
`sensor_msgs/msg/Image` 和 `sensor_msgs/msg/CompressedImage` 两种显示模式。

## 一次性启动入口

本仓库已经补了几套 launch 文件，适合直接把 ROS2 链路和网页一起拉起来：

- `ros2 launch sweeper_integration web_control_manual.launch.py`
  - Gazebo + 接口层 + rosbridge + `python3 -m http.server`
  - 适合手动驾驶测试
- `ros2 launch sweeper_integration web_control_navigation.launch.py`
  - Gazebo + 接口层 + 在线 SLAM + Nav2 + rosbridge + web
  - 适合自动导航测试
- `ros2 launch sweeper_integration web_control_full.launch.py`
  - Gazebo + 接口层 + 静态地图 Nav2 + rosbridge + web

它们会自动启动：

```text
spawn.launch.py -> Gazebo
interfaces.launch.py -> odom / control topic
gen0_navigation.launch.py -> prior_map_2d.yaml /map /navigate_to_pose
rosbridge_websocket_launch.xml -> ws://localhost:9090
python3 -m http.server 8000 --directory web_control
```

浏览器打开：

```text
http://localhost:8000
```

如果需要改端口：

```bash
ros2 launch sweeper_integration web_control_navigation.launch.py rosbridge_port:=9091 web_port:=8001
```

## 启动前准备

先在工作区终端中 source ROS2 环境：

```bash
cd ~/Unknow-gen0_humble
source /opt/ros/humble/setup.bash
source install/setup.bash

unset LIBGL_ALWAYS_SOFTWARE
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export IGN_PARTITION=gen0_roundabout_demo
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$PWD/gen0_gz_sim_ros2/gz_plugins/build"
export IGN_GAZEBO_RESOURCE_PATH="$PWD/install/gen0_main/share/gen0_main/meshes"
```

如果 `rosbridge_server` 没装，先安装：

```bash
sudo apt update
sudo apt install ros-humble-rosbridge-server ros-humble-pointcloud-to-laserscan
```

如果之前启动过老进程，先按 `Ctrl+C` 停掉，再重启；不要重复启动 `spawn.launch.py`、`rosbridge` 或 HTTP 服务，否则会出现端口占用和旧 topic 混用。

## 详细启动顺序（手动版）

### 1. 仅启动 Gazebo + 接口层 + 网页

```bash
cd ~/Unknow-gen0_humble
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch sweeper_integration web_control_manual.launch.py
```

这版不启动 `navigation_online_slam.launch.py`，适合先确认：

- `/cmd_vel` 可以发出速度
- `/control/cmd_vel` 可以继续流向车体
- `/odom` 有里程计
- 网页相机和地图订阅正常

### 2. 需要自动驾驶时，启动导航版

```bash
cd ~/Unknow-gen0_humble
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch sweeper_integration web_control_navigation.launch.py
```

此时会额外带上：

```text
pointcloud_to_laserscan -> /scan
slam_toolbox -> /map
Nav2 -> /navigate_to_pose
```

网页地图从 `/map` 渲染，点击地图即可填入 Nav2 目标点，再点“发送目标”。

### 3. 静态地图一键启动

```bash
cd ~/Unknow-gen0_humble
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch sweeper_integration web_control_full.launch.py
```

默认使用：

```text
~/Unknow-gen0_humble/gen0_gz_sim_ros2/gen0_main/maps/prior_map_2d.yaml
```

启动静态地图 Nav2、Gazebo、接口层、rosbridge 和网页服务。
其中接口层提供 `odom -> base_footprint -> base_link`，Nav2 提供静态
`map -> odom`，用于将 PCD 投影地图坐标与机器人导航坐标连接起来。

## 网页默认接口

网页本身默认使用：

```text
/cmd_vel
/control/cmd_vel
/map
/odom
/gen0_model/front_camera
/navigate_to_pose
```

网页中“停止”按钮会发布零速度，并且会试图取消当前 Nav2 目标；自动驾驶时，Nav2 目标控制最终经过 `/cmd_vel` 进入车体控制链路。

在浏览器中打开 http://localhost:8000/ 即可控制

## 快速检查 ROS2 链路

另开一个已 source 的终端执行：

```bash
ros2 node list | grep -E "ground_truth_odometry|pointcloud_to_laserscan|slam_toolbox|bt_navigator|controller_server|rosbridge"
```

检查地图：

```bash
timeout 8 ros2 topic hz /odom
timeout 8 ros2 topic hz /scan
timeout 8 ros2 topic hz /gen0_model/front3d/lidar/points
ros2 topic echo --once /map --field info
```

检查手动控制：

```bash
ros2 topic info /cmd_vel
ros2 topic info /control/cmd_vel
timeout 10 ros2 topic echo /cmd_vel
timeout 10 ros2 topic echo /control/cmd_vel
```

检查导航：

```bash
ros2 action info /navigate_to_pose
```

## 常见问题

### 网页一直显示“等待地图”

通常说明没有启动 `navigation_online_slam.launch.py`，或者 `/scan` 没有有效数据。先检查：

```bash
ros2 node list | grep -E "pointcloud_to_laserscan|slam_toolbox"
timeout 8 ros2 topic hz /scan
```

如果 `pointcloud_to_laserscan` 没启动，重启导航版 launch 即可。

### 网页可连上 rosbridge，但车不动

检查三段链路：

```text
/cmd_vel -> /control/cmd_vel -> Gazebo 关节桥接
```

快捷方式：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /control/cmd_vel
ros2 node list | grep -E "ground_truth_odometry|vehicle_movement_interface"
```

### 浏览器打开网页后无法连接

确认：

- rosbridge 进程真的在运行
- 端口 `9090` 没被其他程序占用
- 终端里没有因为重启导致旧 WebSocket 连接持有

```bash
ss -lnt | grep 9090
ss -lnt | grep 8000
```

## 最推荐的使用方式

如果你要做网页遥控和自动驾驶，最稳的流程是：

```bash
ros2 launch sweeper_integration web_control_navigation.launch.py
```

在浏览器打开 `http://localhost:8000`，就可以：

- 拖动摇杆手动驱动
- 点击地图设置目标点
- 发送 Nav2 自动驾驶目标
- 监控 `/map`、`/odom`、相机图像

ros2 topic echo --once /map --field info
```

### 相机有图像，但手动驾驶不动

相机只说明 Gazebo 到 ROS 的相机 topic 正常，不代表速度控制链路正常。检查：

```bash
ros2 node list | grep -E "cmd_vel_adapter|vehicle_movement_interface"
timeout 10 ros2 topic echo /cmd_vel
timeout 10 ros2 topic echo /control/cmd_vel
```

### 能点目标，但机器人没有导航

确认 Nav2 lifecycle 已经激活，并且发送目标后有速度输出：

```bash
ros2 action info /navigate_to_pose
timeout 10 ros2 topic echo /cmd_vel
```

如果 Nav2 没有输出 `/cmd_vel`，目标可能未被接受、目标点在未知/障碍区域、TF 不完整，
或 Nav2 lifecycle 没有成功激活。