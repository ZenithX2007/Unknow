# 手机连接用户自己电脑上的 ROS 2

此模式不依赖项目维护者的 AutoDL 或 Cloudflare。每位用户在自己的电脑运行 ROS 2，
手机与电脑处于同一个 Wi-Fi 后，APP 直接连接电脑的 rosbridge。

## 1. 电脑启动 ROS 2

```bash
cd ~/Unknow
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
GEN0_QCNET_BACKEND=constant_velocity ./run_gen0_full_stack.sh
```

默认网页端口为 `8000`，rosbridge 为 `0.0.0.0:9090`。不要把 rosbridge 直接映射到公网。

## 2. 显示手机连接信息

另开终端：

```bash
cd ~/Unknow
./deploy/local_mobile/show_connection_info.sh
```

示例输出：

```text
电脑局域网 IP : 192.168.1.105
APP 中填写 IP : 192.168.1.105
rosbridge      : ws://192.168.1.105:9090
```

如果端口没有监听，先检查：

```bash
ss -lntp | grep -E ':8000|:9090'
ros2 node list | grep rosbridge
```

## 3. 手机连接

1. 手机与电脑连接同一个 Wi-Fi，关闭访客网络/AP 隔离。
2. 打开 APP，连接方式选择“本地电脑（同一 Wi-Fi）”。
3. 只填写脚本显示的电脑 IP，例如 `192.168.1.105`。
4. 端口保持 `9090`，点击“连接”。
5. 状态变为“已连接”后，相机、地图和控制功能会自动订阅。

不要填写 `localhost` 或 `127.0.0.1`，它们在手机中代表手机自身。

## 4. 防火墙

仅允许本地网段访问，下面网段应按实际网络修改：

```bash
sudo ufw allow from 192.168.0.0/16 to any port 8000 proto tcp
sudo ufw allow from 192.168.0.0/16 to any port 9090 proto tcp
```

连接失败时先用手机浏览器访问 `http://电脑IP:8000/`。网页能打开但 rosbridge 连不上，
通常是 `9090` 防火墙、Wi-Fi AP 隔离，或者 rosbridge 未启动。

## 5. 生成可离线打包的网页

```bash
./deploy/local_mobile/build_web_bundle.sh
```

生成 `gen0-mobile-web.zip`。WebToApp 必须选择 **Local HTML / Offline Website**，入口为
`index.html`，并启用网络、明文局域网连接和麦克风权限。如果所用 WebToApp 产品只允许
输入在线 URL，就无法实现真正独立于服务器的 APK，应改用 Capacitor/Android Studio
将该 ZIP 中的资源打包到 APK。

如果打包工具有 **Mixed Content / Allow HTTP & WS / Cleartext Traffic** 开关，也必须
开启；部分 WebView 会以内部 HTTPS 地址加载本地文件，没有这个开关仍会阻止局域网
`ws://`。

Android WebView 至少需要：

```xml
<uses-permission android:name="android.permission.INTERNET" />
<uses-permission android:name="android.permission.RECORD_AUDIO" />
<application android:usesCleartextTraffic="true" ... />
```

`usesCleartextTraffic` 是为了允许同一局域网中的 `ws://电脑IP:9090`。公网连接仍应使用
Cloudflare 的 `wss://`，不要开启公网 9090 端口。
