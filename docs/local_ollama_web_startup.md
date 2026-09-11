# 本地 Ollama 网页与完整仿真启动

本指南使用本机 Ollama 作为自然语言任务规划器，不使用云端 API Key。LLM 只允许生成 `navigate`、`search_object` 和 `stop` 三种经过校验的技能；它不能直接发布底盘速度或任意 ROS 指令。

本文有两种启动方式：先使用“网页 + 本地 LLM”确认连接和任务解析，再使用“完整仿真”查看 Gazebo、RViz、地图、相机和导航。

## 0. 一次性准备

在 Ubuntu 终端执行以下操作。ROS 2 Humble、Ollama 和项目源码必须已经安装。

```bash
sudo apt update
sudo apt install -y ros-humble-nav2-msgs ros-humble-vision-msgs ros-humble-rosbridge-server

ollama pull qwen2.5:3b
ollama list
```

完整仿真还需要以下系统依赖；首次构建完整工作空间前一次安装即可：

```bash
sudo apt install -y \
  ros-humble-pcl-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-ros-gz \
  libprotobuf-dev protobuf-compiler \
  ignition-fortress \
  libgoogle-glog-dev libdw-dev
```

`qwen2.5:3b` 适合中文指令拆解，模型大小约为 2 GB。若内存充足且需要更强的指令遵循能力，可改用 `qwen2.5:7b`，并在后续命令中将模型名改为 `qwen2.5:7b`。

确认 Ollama 服务正在运行：

```bash
systemctl is-active ollama
```

输出应为 `active`。若未启动：

```bash
sudo systemctl start ollama
```

首次构建本地 LLM Agent：

```bash
conda deactivate
cd /home/logic/Unknow-feature-map-stable-llm-web-control
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gen0_llm_agent yolo_detector
```

每一个 ROS 终端都应先执行 `conda deactivate`，否则 Conda 的 Python 可能和 ROS 2 Humble 的 Python 3.10 冲突。

## 1. 网页 + 本地 LLM 联调

此模式用于验证浏览器、rosbridge 和本地模型。没有运行 Gazebo/Nav2 时，网页能显示、任务能被解析，但不能驱动车辆到目标点，也没有实时相机和定位数据。

最省资源的一键方式会启用 `planning_only`，只解析并返回计划，不执行导航：

```bash
conda deactivate
cd /home/logic/Unknow-feature-map-stable-llm-web-control
./run_llm_web_only.sh
```

浏览器打开 `http://localhost:8000`，页面连接 `ws://localhost:9090`。终端出现 `Gen0 LLM agent ready` 后即可提交任务。该脚本不启动 Gazebo、RViz、Nav2、YOLO、相机或地图节点，按 `Ctrl+C` 会一起停止网页、rosbridge 和 Agent。

下面的多终端方式适用于希望分别观察各进程的情况。

### 终端 A：rosbridge

```bash
conda deactivate
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=127.0.0.1 port:=9090
```

保持该终端运行。

### 终端 B：本地 Ollama LLM Agent

```bash
conda deactivate
cd /home/logic/Unknow-feature-map-stable-llm-web-control
./start_local_llm_agent.sh
```

脚本默认使用 `http://127.0.0.1:11434` 和 `qwen2.5:3b`。若使用其他模型：

```bash
GEN0_OLLAMA_MODEL=qwen2.5:7b ./start_local_llm_agent.sh
```

终端出现 `Gen0 LLM agent ready` 后保持运行。

### 终端 C：网页服务

```bash
conda deactivate
cd /home/logic/Unknow-feature-map-stable-llm-web-control
/usr/bin/python3 -m http.server 8000 --directory web_control
```

浏览器打开 `http://localhost:8000`，在页面连接设置中填写 `ws://localhost:9090`。连接成功后可提交：

```text
去坐标（10，5），遇到行人时停止
```

### 观察任务状态

在第四个终端查看 Agent 状态：

```bash
conda deactivate
source /opt/ros/humble/setup.bash
source /home/logic/Unknow-feature-map-stable-llm-web-control/install/setup.bash
ros2 topic echo /llm_agent/status
```

出现 `plan_accepted` 表示本地模型已经把自然语言转换为受限计划。没有 Nav2 服务时，后续导航技能会失败，这是联调模式的预期行为。

按各终端的 `Ctrl+C` 停止联调模式。

## 1.5 无窗口的前方驾驶相机 + 本地 LLM

在确认纯解析正常后，可只增加驾驶视觉相机，不启动 RViz、SLAM、Nav2、EPSILON 或 YOLO。Gazebo 使用无窗口的 headless rendering，因此不会弹出或闪退图形窗口。

先停止第 1 节的轻量模式（在对应终端按 `Ctrl+C`），然后执行：

```bash
conda deactivate
cd /home/logic/Unknow-feature-map-stable-llm-web-control
./run_llm_web_camera.sh
```

等待约 12 秒后，在浏览器打开 `http://localhost:8000`，连接 `ws://localhost:9090`。网页的“驾驶视觉·前方路况相机”应显示 `/gen0_model/driver_camera/compressed` 流；本地 Agent 仍是 `planning_only`，仅显示自然语言解析结果，不会控制车辆。

若端口 `8000` 已被旧网页占用，换用 `8001`：

```bash
GEN0_WEB_PORT=8001 ./run_llm_web_camera.sh
```

停止时在该脚本终端按 `Ctrl+C`。

## 1.6 无窗口网页选点导航

该模式在第 1.5 节相机基础上增加在线 SLAM、Nav2、地图和网页选点导航，但仍不启动 Gazebo 图形窗口、RViz、EPSILON 或 YOLO。网页地图收到 `/map` 后，点击地图可填写目标 X/Y；点击“发送导航目标”会调用 `/navigate_to_pose`，由 Nav2 控制仿真车移动。

首次使用需安装雷达转 2D 扫描的 ROS 包：

```bash
sudo apt update
sudo apt install -y ros-humble-pointcloud-to-laserscan
```

先在相机脚本终端按 `Ctrl+C`，再启动导航：

```bash
conda deactivate
cd /home/logic/Unknow-feature-map-stable-llm-web-control
GEN0_WEB_PORT=8001 ./run_llm_web_navigation.sh
```

等待约 20–40 秒。浏览器打开 `http://localhost:8001`，连接 `ws://localhost:9090`。确认页面地图由“等待数据”变为可绘制状态，网页日志出现 `Nav2 action 已配置为 /navigate_to_pose` 后，再在地图中点击可通行区域、检查 X/Y，并点击“发送导航目标”。

LLM 在此模式仍只做解析，不会因自然语言输入直接启动导航；这是为了先单独验证网页选点和 Nav2 的安全链路。

若要检查 Nav2 是否真的就绪，另开终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/logic/Unknow-feature-map-stable-llm-web-control/install/setup.bash
ros2 action list | grep navigate_to_pose
```

看到 `/navigate_to_pose` 后才能发送选点目标。

## 2. 完整仿真、网页和本地 LLM

此模式启动 Gazebo、定位、静态地图 Nav2、行人避障、命令 mux、网页、rosbridge、YOLO 与本地 LLM。不要同时运行第 1 节中的 rosbridge、网页服务或 `start_local_llm_agent.sh`，否则会占用 `8000`、`9090` 或造成重复节点。

### 2.1 构建完整工作空间

首次完整启动前，安装声明的 ROS 依赖并构建：

```bash
conda deactivate
cd /home/logic/Unknow-feature-map-stable-llm-web-control
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
```

如果 `rosdep` 提示某些第三方源码包没有系统规则，记录报错包名后再处理；不要跳过 `colcon build` 的真实错误。

### 2.2 一条命令启动完整系统

```bash
conda deactivate
cd /home/logic/Unknow-feature-map-stable-llm-web-control
source /opt/ros/humble/setup.bash
source install/setup.bash

GEN0_QCNET_BACKEND=constant_velocity \
GEN0_LLM_PROVIDER=ollama \
GEN0_OLLAMA_BASE_URL=http://127.0.0.1:11434 \
GEN0_OLLAMA_MODEL=qwen2.5:3b \
GEN0_ENABLE_YOLO=false \
GEN0_TRASH_FUSION_DETECTION=false \
./run_gen0_full_stack.sh
```

`constant_velocity` 不需要 QCNet 模型文件或 CUDA，可用于先验证完整链路。首次启动暂时关闭两处 YOLO，避免 Ultralytics/PyTorch 尚未装入 ROS 使用的系统 Python 时阻塞基础仿真。终端会将子进程日志写到 `runtime_logs/`。请保持该终端运行。

启动后会自动提供：

```text
网页:       http://localhost:8000
rosbridge:  ws://localhost:9090
LLM:        本机 Ollama
Gazebo:     仿真车辆与行人场景
RViz:       Nav2、地图、路径与定位可视化
```

Windows 11 的 WSLg 正常启用时，Gazebo 与 RViz 窗口会显示在 Windows 桌面。网页始终从 Windows 浏览器打开 `http://localhost:8000`。

### 2.3 启动成功后的检查

新开终端：

```bash
conda deactivate
cd /home/logic/Unknow-feature-map-stable-llm-web-control
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list
ros2 topic echo --once /llm_agent/status
```

网页连接 `ws://localhost:9090` 后，确认地图、相机和状态出现，再提交任务。只有在 `map -> base_link` TF、定位和 `/navigate_to_pose` Action 就绪后，导航任务才会实际执行。

## 3. 停止完整系统

优先在运行 `run_gen0_full_stack.sh` 的终端按 `Ctrl+C`。若终端丢失或进程没有退出，另开终端执行：

```bash
cd /home/logic/Unknow-feature-map-stable-llm-web-control
./stop_gen0_full_stack.sh
```

## 4. 常见问题

| 现象 | 检查方式 | 处理方式 |
| --- | --- | --- |
| 网页无法连接 | `ss -lnt | grep 9090` | 启动 rosbridge；网页填写 `ws://localhost:9090`。 |
| 本地模型请求失败 | `systemctl is-active ollama`、`ollama list` | 启动 Ollama 服务，确认目标模型已下载。 |
| 任务已接受但不导航 | `ros2 action list | grep navigate_to_pose` | 启动完整仿真；联调模式没有 Nav2 服务。 |
| Gazebo/RViz 没有窗口 | `echo "$DISPLAY"` | 确认 Windows 11 WSLg 或配置 X Server 与 GPU 图形支持。 |
| 端口被占用 | `ss -lnt | grep -E ':8000|:9090'` | 停止旧的网页、rosbridge 或完整栈进程后再启动。 |
