(function () {
  "use strict";

  const state = {
    ros: null,
    connected: false,
    cmdVel: null,
    camera: null,
    mapTopic: null,
    odomTopic: null,
    navClient: null,
    navGoal: null,
    map: null,
    staticMapLoaded: false,
    odom: null,
    goal: null,
    keys: new Set(),
    joystick: {
      active: false,
      pointerId: null,
      linear: 0,
      angular: 0,
    },
    wasMoving: false,
    cameraFpsStarted: 0,
    cameraFpsFrames: 0,
    compressedUrl: null,
  };

  const dom = {};

  document.addEventListener("DOMContentLoaded", init);

  function init() {
    [
      "rosbridgeUrl",
      "connectButton",
      "disconnectButton",
      "connectionBadge",
      "cameraBadge",
      "cameraTopic",
      "cameraMessageType",
      "subscribeCameraButton",
      "cameraFrame",
      "cameraCanvas",
      "cameraPlaceholder",
      "cameraTopicLabel",
      "cameraEncodingLabel",
      "cameraResolutionLabel",
      "cameraFpsLabel",
      "teleopEnabled",
      "cmdVelTopic",
      "applyCmdVelButton",
      "stopButton",
      "joystick",
      "joystickKnob",
      "linearSpeed",
      "linearSpeedValue",
      "angularSpeed",
      "angularSpeedValue",
      "commandReadout",
      "mapBadge",
      "mapTopic",
      "odomTopic",
      "subscribeMapButton",
      "mapFrame",
      "mapCanvas",
      "mapPlaceholder",
      "goalX",
      "goalY",
      "goalYaw",
      "goalFrame",
      "navActionServer",
      "useCurrentPoseButton",
      "sendGoalButton",
      "cancelGoalButton",
      "navigationBadge",
      "goalFeedback",
      "goalProgress",
      "goalDistance",
      "goalRecoveries",
      "robotPose",
      "robotYaw",
      "mapInfo",
      "lastMessage",
      "eventLog",
      "clearLogButton",
    ].forEach((id) => {
      dom[id] = document.getElementById(id);
    });

    const savedUrl = window.localStorage.getItem("gen0.rosbridgeUrl");
    if (savedUrl) {
      dom.rosbridgeUrl.value = savedUrl;
    }

    bindEvents();
    updateSpeedLabels();
    drawCameraPlaceholder();
    drawMap();
    loadStaticMap();
    setBadge(dom.connectionBadge, "offline", "未连接");
    setBadge(dom.cameraBadge, "offline", "等待数据");
    setBadge(dom.mapBadge, "offline", "等待地图");
    setBadge(dom.navigationBadge, "offline", "空闲");
    appendLog("页面已加载，等待连接 rosbridge。");
    window.setInterval(publishTeleopCommand, 100);
    window.addEventListener("resize", () => {
      drawMap();
      if (!state.camera || dom.cameraPlaceholder.hidden) {
        drawCameraPlaceholder();
      }
    });
  }

  function bindEvents() {
    dom.connectButton.addEventListener("click", connectRos);
    dom.disconnectButton.addEventListener("click", () => disconnectRos(true));
    dom.subscribeCameraButton.addEventListener("click", subscribeCamera);
    dom.subscribeMapButton.addEventListener("click", subscribeMapAndOdom);
    dom.applyCmdVelButton.addEventListener("click", createCmdVelPublisher);
    dom.stopButton.addEventListener("click", () => stopRobot(true));
    dom.linearSpeed.addEventListener("input", updateSpeedLabels);
    dom.angularSpeed.addEventListener("input", updateSpeedLabels);
    dom.sendGoalButton.addEventListener("click", sendNavigationGoal);
    dom.cancelGoalButton.addEventListener("click", () => cancelNavigationGoal(true));
    dom.useCurrentPoseButton.addEventListener("click", useCurrentPose);
    dom.clearLogButton.addEventListener("click", () => dom.eventLog.replaceChildren());
    dom.navActionServer.addEventListener("change", createNavigationClient);

    dom.joystick.addEventListener("pointerdown", onJoystickDown);
    dom.joystick.addEventListener("pointermove", onJoystickMove);
    dom.joystick.addEventListener("pointerup", onJoystickUp);
    dom.joystick.addEventListener("pointercancel", onJoystickUp);
    dom.joystick.addEventListener("lostpointercapture", resetJoystick);

    document.addEventListener("keydown", onKeyDown);
    document.addEventListener("keyup", onKeyUp);
    window.addEventListener("blur", () => {
      state.keys.clear();
      stopRobot(false);
    });

    dom.mapCanvas.addEventListener("click", (event) => {
      const point = canvasToWorld(event);
      if (point) {
        setGoalFromPoint(point);
        appendLog(`地图目标已填入 (${point.x.toFixed(2)}, ${point.y.toFixed(2)})。`);
      }
    });
    dom.mapCanvas.addEventListener("dblclick", (event) => {
      const point = canvasToWorld(event);
      if (point) {
        setGoalFromPoint(point);
        sendNavigationGoal();
      }
    });
  }

  function connectRos() {
    if (typeof window.ROSLIB === "undefined") {
      setBadge(dom.connectionBadge, "error", "缺少 roslib");
      appendLog(
        "未加载 roslib.js，请检查网络，或把 roslib.js 放入 web_control 后修改 index.html。",
        "error",
      );
      return;
    }

    disconnectRos(false);
    const url = dom.rosbridgeUrl.value.trim() || "ws://localhost:9090";
    window.localStorage.setItem("gen0.rosbridgeUrl", url);
    setBadge(dom.connectionBadge, "connecting", "连接中");
    appendLog(`正在连接 ${url}。`);

    state.ros = new window.ROSLIB.Ros({ url });
    state.ros.on("connection", () => {
      state.connected = true;
      dom.connectButton.disabled = true;
      dom.disconnectButton.disabled = false;
      setBadge(dom.connectionBadge, "online", "已连接");
      appendLog("rosbridge 连接成功。");
      createCmdVelPublisher();
      subscribeCamera();
      subscribeMapAndOdom();
      createNavigationClient();
    });
    state.ros.on("error", (error) => {
      const detail = error && error.message ? `: ${error.message}` : "";
      setBadge(dom.connectionBadge, "error", "连接错误");
      appendLog(`rosbridge 错误${detail}`, "error");
    });
    state.ros.on("close", () => {
      state.connected = false;
      dom.connectButton.disabled = false;
      dom.disconnectButton.disabled = true;
      teardownRos();
      setBadge(dom.connectionBadge, "offline", "连接关闭");
      setBadge(dom.cameraBadge, "offline", "等待数据");
      setBadge(dom.mapBadge, "offline", "等待地图");
      setBadge(dom.navigationBadge, "offline", "空闲");
      appendLog("rosbridge 连接已关闭。", "warn");
    });
  }

  function disconnectRos(logEvent) {
    if (state.ros) {
      try {
        state.ros.close();
      } catch (error) {
        appendLog(`关闭 rosbridge 时出现异常: ${error.message}`, "warn");
      }
    }
    state.connected = false;
    dom.connectButton.disabled = false;
    dom.disconnectButton.disabled = true;
    teardownRos();
    setBadge(dom.connectionBadge, "offline", "未连接");
    if (logEvent) {
      appendLog("已断开 rosbridge。");
    }
  }

  function teardownRos() {
    unsubscribe(state.camera);
    unsubscribe(state.mapTopic);
    unsubscribe(state.odomTopic);
    if (state.cmdVel && state.cmdVel.unadvertise) {
      try {
        state.cmdVel.unadvertise();
      } catch (error) {
        // The socket may already be closed.
      }
    }
    state.camera = null;
    state.mapTopic = null;
    state.odomTopic = null;
    state.cmdVel = null;
    closeNavigationGoalSocket();
    state.navClient = null;
    state.navGoal = null;
    state.odom = null;
    state.wasMoving = false;
    resetJoystick();
    clearCamera();
  }

  function unsubscribe(topic) {
    if (!topic || !topic.unsubscribe) {
      return;
    }
    try {
      topic.unsubscribe();
    } catch (error) {
      // The socket may already be closed.
    }
  }

  function createCmdVelPublisher() {
    if (!state.connected || !state.ros) {
      appendLog("请先连接 rosbridge，再应用速度 topic。", "warn");
      return;
    }
    if (state.cmdVel && state.cmdVel.unadvertise) {
      try {
        state.cmdVel.unadvertise();
      } catch (error) {
        // Ignore an already closed topic.
      }
    }
    const topicName = dom.cmdVelTopic.value.trim() || "/cmd_vel";
    state.cmdVel = new window.ROSLIB.Topic({
      ros: state.ros,
      name: topicName,
      messageType: "geometry_msgs/msg/Twist",
      queue_size: 1,
    });
    appendLog(`速度输出已切换到 ${topicName}。`);
  }

  function subscribeCamera() {
    if (!state.connected || !state.ros) {
      appendLog("请先连接 rosbridge，再订阅相机。", "warn");
      return;
    }
    unsubscribe(state.camera);
    const topicName = dom.cameraTopic.value.trim() || "/gen0_model/front_camera";
    const compressed = dom.cameraMessageType.value === "compressed";
    state.camera = new window.ROSLIB.Topic({
      ros: state.ros,
      name: topicName,
      messageType: compressed
        ? "sensor_msgs/msg/CompressedImage"
        : "sensor_msgs/msg/Image",
      throttle_rate: 80,
      queue_length: 1,
    });
    dom.cameraTopicLabel.textContent = topicName;
    setBadge(dom.cameraBadge, "connecting", "订阅中");
    state.camera.subscribe(handleCameraMessage);
    appendLog(
      `相机已订阅 ${topicName} (${compressed ? "CompressedImage" : "Image"})。`,
    );
  }

  function handleCameraMessage(message) {
    try {
      if (dom.cameraMessageType.value === "compressed") {
        renderCompressedImage(message);
      } else {
        renderRawImage(message);
      }
      updateCameraRate();
      setBadge(dom.cameraBadge, "online", "图像正常");
      dom.lastMessage.textContent = "相机";
    } catch (error) {
      setBadge(dom.cameraBadge, "error", "图像解析失败");
      appendLog(`相机帧解析失败: ${error.message}`, "error");
    }
  }

  function updateCameraRate() {
    const now = performance.now();
    state.cameraFpsFrames += 1;
    if (!state.cameraFpsStarted) {
      state.cameraFpsStarted = now;
    }
    if (now - state.cameraFpsStarted >= 1000) {
      const fps =
        (state.cameraFpsFrames * 1000) / (now - state.cameraFpsStarted);
      dom.cameraFpsLabel.textContent = `${fps.toFixed(1)} FPS`;
      state.cameraFpsStarted = now;
      state.cameraFpsFrames = 0;
    }
  }

  function renderRawImage(message) {
    const width = Number(message.width);
    const height = Number(message.height);
    if (!width || !height) {
      throw new Error("Image 缺少有效 width/height");
    }

    const encoding = String(message.encoding || "rgb8").toLowerCase();
    const bytesPerPixel = encoding === "mono8" || encoding === "8uc1"
      ? 1
      : encoding === "rgba8" || encoding === "bgra8"
        ? 4
        : 3;
    const rowStep = Number(message.step) || width * bytesPerPixel;
    const bytes = toUint8Array(message.data);
    const canvas = dom.cameraCanvas;
    const context = canvas.getContext("2d");
    const imageData = context.createImageData(width, height);
    const output = imageData.data;

    for (let y = 0; y < height; y += 1) {
      const inputRow = y * rowStep;
      for (let x = 0; x < width; x += 1) {
        const source = inputRow + x * bytesPerPixel;
        const target = (y * width + x) * 4;
        let red;
        let green;
        let blue;

        if (bytesPerPixel === 1) {
          red = bytes[source] || 0;
          green = red;
          blue = red;
        } else if (encoding === "bgr8" || encoding === "bgra8" || encoding === "8uc3") {
          blue = bytes[source] || 0;
          green = bytes[source + 1] || 0;
          red = bytes[source + 2] || 0;
        } else {
          red = bytes[source] || 0;
          green = bytes[source + 1] || 0;
          blue = bytes[source + 2] || 0;
        }

        output[target] = red;
        output[target + 1] = green;
        output[target + 2] = blue;
        output[target + 3] = 255;
      }
    }

    canvas.width = width;
    canvas.height = height;
    context.putImageData(imageData, 0, 0);
    dom.cameraEncodingLabel.textContent = encoding;
    dom.cameraResolutionLabel.textContent = `${width} x ${height}`;
    dom.cameraPlaceholder.hidden = true;
  }

  function renderCompressedImage(message) {
    const bytes = toUint8Array(message.data);
    const format = String(message.format || "jpeg").toLowerCase();
    const mime = format.includes("png") ? "image/png" : "image/jpeg";
    const url = URL.createObjectURL(new Blob([bytes], { type: mime }));
    if (state.compressedUrl) {
      URL.revokeObjectURL(state.compressedUrl);
    }
    state.compressedUrl = url;
    const image = new Image();
    image.onload = () => {
      const canvas = dom.cameraCanvas;
      canvas.width = image.naturalWidth;
      canvas.height = image.naturalHeight;
      canvas.getContext("2d").drawImage(image, 0, 0);
      dom.cameraEncodingLabel.textContent = format;
      dom.cameraResolutionLabel.textContent =
        `${image.naturalWidth} x ${image.naturalHeight}`;
      dom.cameraPlaceholder.hidden = true;
    };
    image.onerror = () => {
      setBadge(dom.cameraBadge, "error", "图像解码失败");
      appendLog(`无法解码 ${format} 图像。`, "error");
    };
    image.src = url;
  }

  function clearCamera() {
    if (state.compressedUrl) {
      URL.revokeObjectURL(state.compressedUrl);
      state.compressedUrl = null;
    }
    state.cameraFpsStarted = 0;
    state.cameraFpsFrames = 0;
    dom.cameraFpsLabel.textContent = "-- FPS";
    dom.cameraEncodingLabel.textContent = "--";
    dom.cameraResolutionLabel.textContent = "-- x --";
    dom.cameraPlaceholder.hidden = false;
    drawCameraPlaceholder();
  }

  function drawCameraPlaceholder() {
    const canvas = dom.cameraCanvas;
    const rect = dom.cameraFrame.getBoundingClientRect();
    canvas.width = Math.max(320, Math.round(rect.width || 640));
    canvas.height = Math.max(180, Math.round(rect.height || 360));
    const context = canvas.getContext("2d");
    context.fillStyle = "#18201b";
    context.fillRect(0, 0, canvas.width, canvas.height);
  }

  function subscribeMapAndOdom() {
    if (!state.connected || !state.ros) {
      appendLog("请先连接 rosbridge，再订阅地图和里程计。", "warn");
      return;
    }
    unsubscribe(state.mapTopic);
    unsubscribe(state.odomTopic);

    const mapName = dom.mapTopic.value.trim() || "/map";
    const odomName = dom.odomTopic.value.trim() || "/odom";
    state.mapTopic = new window.ROSLIB.Topic({
      ros: state.ros,
      name: mapName,
      messageType: "nav_msgs/msg/OccupancyGrid",
      throttle_rate: 500,
      queue_length: 1,
    });
    state.odomTopic = new window.ROSLIB.Topic({
      ros: state.ros,
      name: odomName,
      messageType: "nav_msgs/msg/Odometry",
      throttle_rate: 100,
      queue_length: 1,
    });
    state.mapTopic.subscribe(handleMapMessage);
    state.odomTopic.subscribe(handleOdomMessage);
    appendLog(`地图和里程计已订阅 ${mapName} / ${odomName}。`);
  }

  async function loadStaticMap() {
    try {
      const staticMapUrl = new URL("static_map/", document.baseURI);
      const yamlResponse = await fetch(
        new URL("prior_map_2d.yaml", staticMapUrl),
        {
        cache: "no-store",
        },
      );
      if (!yamlResponse.ok) {
        throw new Error(`HTTP ${yamlResponse.status}`);
      }
      const yaml = await yamlResponse.text();
      const imageMatch = yaml.match(/^image:\s*(.+)$/m);
      const resolutionMatch = yaml.match(/^resolution:\s*([^\s]+)$/m);
      const imageName = imageMatch && imageMatch[1].trim();
      const resolution = Number(resolutionMatch && resolutionMatch[1]);
      const origin = yaml.match(/^origin:\s*\[([^,]+),\s*([^,]+),/m);
      if (!imageName || !resolution || !origin) {
        throw new Error("地图 YAML 缺少 image、resolution 或 origin");
      }

      const image = new Image();
      await new Promise((resolve, reject) => {
        image.onload = resolve;
        image.onerror = () => reject(new Error(`无法读取地图图片 ${imageName}`));
        image.src = new URL(imageName, staticMapUrl).href;
      });
      const raster = document.createElement("canvas");
      raster.width = image.naturalWidth;
      raster.height = image.naturalHeight;
      raster.getContext("2d").drawImage(image, 0, 0);
      state.map = {
        width: image.naturalWidth,
        height: image.naturalHeight,
        resolution,
        originX: Number(origin[1]),
        originY: Number(origin[2]),
        data: null,
        raster,
        transform: null,
      };
      state.staticMapLoaded = true;
      dom.mapInfo.textContent =
        `${image.naturalWidth} x ${image.naturalHeight} @ ${resolution.toFixed(2)}m`;
      dom.mapPlaceholder.hidden = true;
      setBadge(dom.mapBadge, "online", "静态底图正常");
      appendLog("已加载 prior_map.pcd 转换的 Nav2 静态栅格底图。");
      drawMap();
    } catch (error) {
      setBadge(dom.mapBadge, "error", "底图加载失败");
      appendLog(`静态底图加载失败：${error.message}`, "error");
    }
  }

  function handleMapMessage(message) {
    const info = message.info || {};
    const width = Number(info.width);
    const height = Number(info.height);
    const resolution = Number(info.resolution);
    if (!width || !height || !resolution) {
      appendLog("收到的地图缺少有效尺寸或分辨率。", "warn");
      return;
    }

    const rosMap = {
      width,
      height,
      resolution,
      originX: Number(
        info.origin && info.origin.position && info.origin.position.x,
      ) || 0,
      originY: Number(
        info.origin && info.origin.position && info.origin.position.y,
      ) || 0,
      data: signedOccupancyData(message.data),
      raster: null,
      transform: null,
    };
    buildMapRaster(rosMap);
    if (!state.staticMapLoaded) {
      state.map = rosMap;
      dom.mapInfo.textContent =
        `${width} x ${height} @ ${resolution.toFixed(2)}m`;
      dom.mapPlaceholder.hidden = true;
    }
    dom.lastMessage.textContent = "地图";
    if (!state.staticMapLoaded) {
      setBadge(dom.mapBadge, "online", "地图正常");
    }
    drawMap();
  }

  function handleOdomMessage(message) {
    const poseContainer = message.pose || {};
    const pose = poseContainer.pose || message.pose;
    if (!pose || !pose.position || !pose.orientation) {
      return;
    }

    state.odom = {
      x: Number(pose.position.x) || 0,
      y: Number(pose.position.y) || 0,
      yaw: quaternionToYaw(pose.orientation),
    };
    dom.robotPose.textContent =
      `${state.odom.x.toFixed(2)}, ${state.odom.y.toFixed(2)}`;
    dom.robotYaw.textContent =
      `${radToDeg(state.odom.yaw).toFixed(1)} deg`;
    dom.lastMessage.textContent = "里程计";
    drawMap();
  }

  function buildMapRaster(map) {
    if (!map) {
      return;
    }

    const raster = document.createElement("canvas");
    raster.width = map.width;
    raster.height = map.height;
    const context = raster.getContext("2d");
    const imageData = context.createImageData(raster.width, raster.height);
    const pixels = imageData.data;

    for (let mapY = 0; mapY < map.height; mapY += 1) {
      for (let mapX = 0; mapX < map.width; mapX += 1) {
        const value = map.data[mapY * map.width + mapX];
        const displayY = map.height - 1 - mapY;
        const index = (displayY * map.width + mapX) * 4;
        let red = 216;
        let green = 224;
        let blue = 218;

        if (value >= 0 && value < 55) {
          const shade = Math.round(250 - value * 1.1);
          red = shade;
          green = shade;
          blue = shade;
        } else if (value >= 55) {
          const shade = Math.round(65 - Math.min(value - 55, 45) * 0.45);
          red = shade;
          green = Math.max(32, shade - 8);
          blue = Math.max(30, shade - 14);
        }

        pixels[index] = red;
        pixels[index + 1] = green;
        pixels[index + 2] = blue;
        pixels[index + 3] = 255;
      }
    }
    context.putImageData(imageData, 0, 0);
    map.raster = raster;
  }

  function drawMap() {
    const canvas = dom.mapCanvas;
    const rect = dom.mapFrame.getBoundingClientRect();
    const cssWidth = Math.max(320, rect.width || 640);
    const cssHeight = Math.max(220, rect.height || 400);
    const dpr = Math.max(1, window.devicePixelRatio || 1);
    const width = Math.round(cssWidth * dpr);
    const height = Math.round(cssHeight * dpr);

    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }

    const context = canvas.getContext("2d");
    context.setTransform(dpr, 0, 0, dpr, 0, 0);
    context.clearRect(0, 0, cssWidth, cssHeight);
    context.fillStyle = "#28312b";
    context.fillRect(0, 0, cssWidth, cssHeight);

    if (!state.map || !state.map.raster) {
      return;
    }

    const scale = Math.min(
      (cssWidth - 24) / state.map.width,
      (cssHeight - 24) / state.map.height,
    );
    const drawWidth = state.map.width * scale;
    const drawHeight = state.map.height * scale;
    const offsetX = (cssWidth - drawWidth) / 2;
    const offsetY = (cssHeight - drawHeight) / 2;
    state.map.transform = { scale, offsetX, offsetY };

    context.imageSmoothingEnabled = false;
    context.drawImage(
      state.map.raster,
      offsetX,
      offsetY,
      drawWidth,
      drawHeight,
    );

    if (state.odom) {
      const point = worldToCanvas(state.odom.x, state.odom.y);
      drawPoseMarker(context, point.x, point.y, state.odom.yaw, "#ef9f35", "机器人");
    }
    if (state.goal) {
      const point = worldToCanvas(state.goal.x, state.goal.y);
      drawPoseMarker(context, point.x, point.y, state.goal.yaw, "#e15b5b", "目标");
    }
  }

  function drawPoseMarker(context, x, y, yaw, color, label) {
    context.save();
    context.translate(x, y);
    context.rotate(-yaw);
    context.fillStyle = color;
    context.strokeStyle = "#ffffff";
    context.lineWidth = 2;
    context.beginPath();
    context.moveTo(13, 0);
    context.lineTo(-8, -7);
    context.lineTo(-4, 0);
    context.lineTo(-8, 7);
    context.closePath();
    context.fill();
    context.stroke();
    context.restore();

    context.save();
    context.fillStyle = "#ffffff";
    context.font = "700 11px system-ui, sans-serif";
    context.shadowColor = "#000000";
    context.shadowBlur = 3;
    context.fillText(label, x + 9, y - 10);
    context.restore();
  }

  function canvasToWorld(event) {
    if (!state.map || !state.map.transform) {
      appendLog("地图尚未到达，暂时无法选取目标。", "warn");
      return null;
    }

    const rect = dom.mapCanvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    const { scale, offsetX, offsetY } = state.map.transform;
    const mapX = (x - offsetX) / scale;
    const mapY = state.map.height - (y - offsetY) / scale;
    if (
      mapX < 0 ||
      mapY < 0 ||
      mapX > state.map.width ||
      mapY > state.map.height
    ) {
      return null;
    }

    return {
      x: state.map.originX + mapX * state.map.resolution,
      y: state.map.originY + mapY * state.map.resolution,
    };
  }

  function worldToCanvas(x, y) {
    const { scale, offsetX, offsetY } = state.map.transform;
    const mapX = (x - state.map.originX) / state.map.resolution;
    const mapY = (y - state.map.originY) / state.map.resolution;
    return {
      x: offsetX + mapX * scale,
      y: offsetY + (state.map.height - mapY) * scale,
    };
  }

  function setGoalFromPoint(point) {
    dom.goalX.value = point.x.toFixed(2);
    dom.goalY.value = point.y.toFixed(2);
    state.goal = {
      x: point.x,
      y: point.y,
      yaw: degToRad(Number(dom.goalYaw.value) || 0),
    };
    drawMap();
  }

  function useCurrentPose() {
    if (!state.odom) {
      appendLog("尚未收到 /odom，无法填入当前位置。", "warn");
      return;
    }
    dom.goalX.value = state.odom.x.toFixed(2);
    dom.goalY.value = state.odom.y.toFixed(2);
    dom.goalYaw.value = radToDeg(state.odom.yaw).toFixed(1);
    dom.goalFrame.value = "odom";
    state.goal = { ...state.odom };
    drawMap();
    appendLog("已将当前位置填入导航目标，坐标系切换为 odom。");
  }

  function createNavigationClient() {
    if (!state.connected || !state.ros) {
      return;
    }
    const serverName =
      dom.navActionServer.value.trim() || "/navigate_to_pose";
    state.navClient = {
      serverName,
      actionType: "nav2_msgs/action/NavigateToPose",
    };
    appendLog(`Nav2 action 已配置为 ${serverName}。`);
  }

  function sendNavigationGoal() {
    if (!state.connected || !state.navClient) {
      appendLog("请先连接 rosbridge，并确认 Nav2 action server 已启动。", "warn");
      return;
    }

    const x = Number(dom.goalX.value);
    const y = Number(dom.goalY.value);
    const yawDegrees = Number(dom.goalYaw.value);
    const frame = dom.goalFrame.value.trim() || "map";
    if (![x, y, yawDegrees].every(Number.isFinite)) {
      appendLog("目标 X、Y、朝向必须是有效数字。", "warn");
      return;
    }

    cancelNavigationGoal(false);
    state.goal = { x, y, yaw: degToRad(yawDegrees) };
    drawMap();

    const goal = createNavigationGoal();
    state.navGoal = goal;
    openNavigationGoalSocket(goal, {
      pose: {
        header: {
          frame_id: frame,
          stamp: { sec: 0, nanosec: 0 },
        },
        pose: {
          position: { x, y, z: 0 },
          orientation: yawToQuaternion(degToRad(yawDegrees)),
        },
      },
      behavior_tree: "",
    });

    setBadge(dom.navigationBadge, "active", "目标已发送");
    dom.goalFeedback.textContent =
      `(${x.toFixed(2)}, ${y.toFixed(2)}) / ${frame}`;
    dom.goalProgress.style.width = "12%";
    dom.goalDistance.textContent = "--";
    dom.goalRecoveries.textContent = "0";
    appendLog(
      `已发送 Nav2 目标: x=${x.toFixed(2)}, y=${y.toFixed(2)}, yaw=${yawDegrees.toFixed(1)} deg。`,
    );
  }

  function cancelNavigationGoal(logEvent) {
    const goal = state.navGoal;
    if (!goal) {
      return;
    }
    const sent = sendNavigationGoalCancel(goal);
    goal.finished = true;
    if (sent) {
      closeNavigationGoalSocket(goal);
    }
    state.navGoal = null;
    setBadge(dom.navigationBadge, "offline", "已取消");
    dom.goalFeedback.textContent = "目标已取消";
    dom.goalProgress.style.width = "0";
    if (logEvent) {
      appendLog("已请求取消当前 Nav2 目标。", "warn");
    }
  }

  function handleGoalStatus(status) {
    const value =
      status && typeof status.status === "number" ? status.status : status;
    const label = goalStatusLabel(value);
    dom.goalFeedback.textContent = label;
    if (value === 4) {
      setBadge(dom.navigationBadge, "online", "目标成功");
      dom.goalProgress.style.width = "100%";
    } else if (value === 5 || value === 6) {
      setBadge(dom.navigationBadge, "error", value === 5 ? "目标取消" : "目标失败");
    } else {
      setBadge(dom.navigationBadge, "active", label);
    }
  }

  function handleGoalFeedback(feedback) {
    const distance = feedback && (feedback.distance_remaining !== undefined
      ? feedback.distance_remaining
      : feedback.distanceRemaining);
    const recoveries = feedback && (feedback.number_of_recoveries !== undefined
      ? feedback.number_of_recoveries
      : feedback.numberOfRecoveries);
    if (Number.isFinite(Number(distance))) {
      const numericDistance = Number(distance);
      dom.goalDistance.textContent = `${numericDistance.toFixed(2)} m`;
      if (state.goal && state.odom) {
        const currentDistance = Math.hypot(
          state.goal.x - state.odom.x,
          state.goal.y - state.odom.y,
        );
        const totalDistance = currentDistance + numericDistance;
        const progress = totalDistance > 0.001
          ? clamp(1 - numericDistance / totalDistance, 0.05, 0.98)
          : 1;
        dom.goalProgress.style.width = `${Math.round(progress * 100)}%`;
      }
    }
    if (Number.isFinite(Number(recoveries))) {
      dom.goalRecoveries.textContent = String(recoveries);
    }
    dom.goalFeedback.textContent = "导航中";
  }

  function handleGoalResult(message) {
    const status = Number(message && message.status);
    const values = message && message.values;
    if (message && message.result === false) {
      const detail = typeof values === "string"
        ? values
        : JSON.stringify(values || {});
      setBadge(dom.navigationBadge, "error", "目标失败");
      dom.goalFeedback.textContent = "导航目标发送失败";
      appendLog(`Nav2 目标发送失败: ${detail}`, "error");
      return;
    }

    if (status === 4 || Number.isNaN(status)) {
      setBadge(dom.navigationBadge, "online", "目标完成");
      dom.goalFeedback.textContent = "目标完成";
      dom.goalProgress.style.width = "100%";
      appendLog("Nav2 返回目标完成。");
      return;
    }

    if (status === 5) {
      setBadge(dom.navigationBadge, "error", "目标取消");
      dom.goalFeedback.textContent = "导航已取消";
      appendLog("Nav2 导航被取消。", "warn");
      return;
    }

    if (status === 6) {
      setBadge(dom.navigationBadge, "error", "目标失败");
      dom.goalFeedback.textContent = "导航失败";
      appendLog("Nav2 导航失败。", "error");
      return;
    }

    setBadge(dom.navigationBadge, "error", `返回码 ${status}`);
    dom.goalFeedback.textContent = `导航结束，返回码 ${status}`;
    appendLog(`Nav2 导航结束，返回码 ${status}。`, "warn");
  }

  function createNavigationGoal() {
    return {
      id: `gen0-nav-${Date.now()}-${Math.random().toString(16).slice(2)}`,
      socket: null,
      finished: false,
      pendingCancel: false,
      action: state.navClient.serverName,
      actionType: state.navClient.actionType,
    };
  }

  function openNavigationGoalSocket(goal, goalMessage) {
    const url = dom.rosbridgeUrl.value.trim() || "ws://localhost:9090";
    const socket = new WebSocket(url);
    goal.socket = socket;

    socket.addEventListener("open", () => {
      if (state.navGoal !== goal || goal.finished) {
        return;
      }
      socket.send(JSON.stringify({
        op: "send_action_goal",
        action: goal.action,
        action_type: goal.actionType,
        feedback: true,
        id: goal.id,
        args: goalMessage,
      }));
      if (goal.pendingCancel) {
        sendNavigationGoalCancel(goal);
      }
    });

    socket.addEventListener("message", (event) => {
      let message;
      try {
        message = JSON.parse(event.data);
      } catch (error) {
        appendLog(`导航响应解析失败: ${error.message}`, "warn");
        return;
      }

      if (goal.finished && state.navGoal !== goal) {
        return;
      }
      if (message.id && message.id !== goal.id) {
        return;
      }
      if (message.op === "action_feedback") {
        handleGoalFeedback(message.values || {});
      } else if (message.op === "action_result") {
        goal.finished = true;
        handleGoalResult(message);
        closeNavigationGoalSocket(goal);
        if (state.navGoal === goal) {
          state.navGoal = null;
        }
      } else if (message.op === "status" && Number(message.level) >= 2) {
        appendLog(`Nav2: ${message.msg || "收到状态消息"}`, "warn");
      }
    });

    socket.addEventListener("error", (error) => {
      if (state.navGoal !== goal || goal.finished) {
        return;
      }
      setBadge(dom.navigationBadge, "error", "连接错误");
      dom.goalFeedback.textContent = "导航连接错误";
      appendLog(`Nav2 WebSocket 错误: ${error.message || "unknown"}`, "error");
    });

    socket.addEventListener("close", () => {
      if (state.navGoal === goal && !goal.finished) {
        setBadge(dom.navigationBadge, "offline", "连接关闭");
        dom.goalFeedback.textContent = "导航连接已关闭";
        appendLog("Nav2 WebSocket 连接已关闭。", "warn");
        state.navGoal = null;
      }
    });
  }

  function sendNavigationGoalCancel(goal) {
    if (!goal.socket) {
      goal.pendingCancel = true;
      return false;
    }

    const payload = JSON.stringify({
      op: "cancel_action_goal",
      action: goal.action,
      id: goal.id,
    });

    if (goal.socket.readyState === WebSocket.OPEN) {
      try {
        goal.socket.send(payload);
      } catch (error) {
        appendLog(`取消 Nav2 目标失败: ${error.message}`, "error");
      }
      window.setTimeout(() => {
        if (
          goal.socket &&
          goal.socket.readyState !== WebSocket.CLOSED &&
          goal.socket.readyState !== WebSocket.CLOSING
        ) {
          goal.socket.close();
        }
      }, 150);
      return true;
    } else if (goal.socket.readyState === WebSocket.CONNECTING) {
      goal.pendingCancel = true;
      goal.socket.addEventListener("open", () => {
        try {
          goal.socket.send(payload);
          window.setTimeout(() => {
            if (
              goal.socket &&
              goal.socket.readyState !== WebSocket.CLOSED &&
              goal.socket.readyState !== WebSocket.CLOSING
            ) {
              goal.socket.close();
            }
          }, 150);
        } catch (error) {
          appendLog(`取消 Nav2 目标失败: ${error.message}`, "error");
        }
      }, { once: true });
      return false;
    } else {
      return false;
    }
  }

  function closeNavigationGoalSocket(goal = state.navGoal) {
    if (!goal || !goal.socket) {
      return;
    }
    try {
      if (
        goal.socket.readyState === WebSocket.OPEN ||
        goal.socket.readyState === WebSocket.CONNECTING
      ) {
        goal.socket.close();
      }
    } catch (error) {
      appendLog(`关闭 Nav2 socket 失败: ${error.message}`, "warn");
    }
  }

  function publishTeleopCommand() {
    const command = getManualCommand();
    updateCommandReadout(command.linear, command.angular);
    if (!state.connected || !state.cmdVel) {
      return;
    }

    const moving =
      Math.abs(command.linear) > 0.001 || Math.abs(command.angular) > 0.001;
    if (!dom.teleopEnabled.checked && !state.wasMoving) {
      return;
    }
    if (moving || state.wasMoving) {
      publishTwist(command.linear, command.angular);
    }
    state.wasMoving = moving;
  }

  function publishTwist(linear, angular) {
    if (!state.connected || !state.cmdVel) {
      return;
    }
    state.cmdVel.publish(
      new window.ROSLIB.Message({
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular },
      }),
    );
  }

  function stopRobot(cancelGoal) {
    state.keys.clear();
    resetJoystick();
    state.wasMoving = true;
    publishTwist(0, 0);
    window.setTimeout(() => publishTwist(0, 0), 80);
    window.setTimeout(() => publishTwist(0, 0), 160);
    updateCommandReadout(0, 0);
    if (cancelGoal) {
      cancelNavigationGoal(true);
    }
  }

  function getManualCommand() {
    let linearAxis = state.joystick.active ? state.joystick.linear : 0;
    let angularAxis = state.joystick.active ? state.joystick.angular : 0;
    if (!state.joystick.active) {
      if (state.keys.has("w") || state.keys.has("arrowup")) {
        linearAxis += 1;
      }
      if (state.keys.has("s") || state.keys.has("arrowdown")) {
        linearAxis -= 1;
      }
      if (state.keys.has("a") || state.keys.has("arrowleft")) {
        angularAxis += 1;
      }
      if (state.keys.has("d") || state.keys.has("arrowright")) {
        angularAxis -= 1;
      }
    }
    return {
      linear: clamp(linearAxis, -1, 1) * Number(dom.linearSpeed.value),
      angular: clamp(angularAxis, -1, 1) * Number(dom.angularSpeed.value),
    };
  }

  function updateCommandReadout(linear, angular) {
    dom.commandReadout.textContent =
      `x ${linear.toFixed(2)} m/s · z ${angular.toFixed(2)} rad/s`;
  }

  function updateSpeedLabels() {
    dom.linearSpeedValue.textContent =
      `${Number(dom.linearSpeed.value).toFixed(2)} m/s`;
    dom.angularSpeedValue.textContent =
      `${Number(dom.angularSpeed.value).toFixed(2)} rad/s`;
  }

  function onJoystickDown(event) {
    if (!dom.teleopEnabled.checked) {
      return;
    }
    state.joystick.active = true;
    state.joystick.pointerId = event.pointerId;
    dom.joystick.setPointerCapture(event.pointerId);
    updateJoystick(event);
  }

  function onJoystickMove(event) {
    if (!state.joystick.active || state.joystick.pointerId !== event.pointerId) {
      return;
    }
    updateJoystick(event);
  }

  function onJoystickUp() {
    resetJoystick();
  }

  function updateJoystick(event) {
    const rect = dom.joystick.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;
    const maxDistance = rect.width * 0.34;
    let dx = event.clientX - centerX;
    let dy = event.clientY - centerY;
    const distance = Math.hypot(dx, dy);
    if (distance > maxDistance) {
      const scale = maxDistance / distance;
      dx *= scale;
      dy *= scale;
    }
    state.joystick.linear = clamp(-dy / maxDistance, -1, 1);
    state.joystick.angular = clamp(-dx / maxDistance, -1, 1);
    dom.joystickKnob.style.transform =
      `translate(-50%, -50%) translate(${dx}px, ${dy}px)`;
  }

  function resetJoystick() {
    state.joystick.active = false;
    state.joystick.pointerId = null;
    state.joystick.linear = 0;
    state.joystick.angular = 0;
    dom.joystickKnob.style.transform = "translate(-50%, -50%)";
  }

  function onKeyDown(event) {
    if (["INPUT", "TEXTAREA", "SELECT", "BUTTON"].includes(event.target.tagName)) {
      return;
    }
    const key = event.key.toLowerCase();
    const movementKeys = [
      "w",
      "a",
      "s",
      "d",
      "arrowup",
      "arrowdown",
      "arrowleft",
      "arrowright",
    ];
    if (movementKeys.includes(key)) {
      event.preventDefault();
      if (dom.teleopEnabled.checked) {
        state.keys.add(key);
      }
    } else if (event.code === "Space") {
      event.preventDefault();
      stopRobot(true);
    }
  }

  function onKeyUp(event) {
    state.keys.delete(event.key.toLowerCase());
  }

  function createNavigationClientOnRosReady() {
    createNavigationClient();
  }

  function setBadge(element, type, label) {
    element.className = `status-badge status-badge--${type}`;
    element.textContent = label;
  }

  function appendLog(message, level) {
    const item = document.createElement("div");
    item.className = `event-item${level ? ` event-item--${level}` : ""}`;
    const time = document.createElement("time");
    time.textContent = new Date().toLocaleTimeString("zh-CN", {
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
    });
    const text = document.createElement("span");
    text.textContent = message;
    item.append(time, text);
    dom.eventLog.prepend(item);
    while (dom.eventLog.children.length > 80) {
      dom.eventLog.lastElementChild.remove();
    }
  }

  function toUint8Array(data) {
    if (data instanceof Uint8Array) {
      return data;
    }
    if (Array.isArray(data)) {
      return Uint8Array.from(data, (value) => Number(value) & 255);
    }
    if (typeof data === "string") {
      const binary = window.atob(data);
      const bytes = new Uint8Array(binary.length);
      for (let index = 0; index < binary.length; index += 1) {
        bytes[index] = binary.charCodeAt(index);
      }
      return bytes;
    }
    if (data && data.buffer instanceof ArrayBuffer) {
      return new Uint8Array(data.buffer, data.byteOffset, data.byteLength);
    }
    return new Uint8Array();
  }

  function signedOccupancyData(data) {
    if (Array.isArray(data)) {
      return data.map((value) => Number(value));
    }
    return Array.from(toUint8Array(data), (value) =>
      value > 127 ? value - 256 : value,
    );
  }

  function quaternionToYaw(quaternion) {
    const x = Number(quaternion.x) || 0;
    const y = Number(quaternion.y) || 0;
    const z = Number(quaternion.z) || 0;
    const w = Number.isFinite(Number(quaternion.w))
      ? Number(quaternion.w)
      : 1;
    return Math.atan2(
      2 * (w * z + x * y),
      1 - 2 * (y * y + z * z),
    );
  }

  function yawToQuaternion(yaw) {
    return {
      x: 0,
      y: 0,
      z: Math.sin(yaw / 2),
      w: Math.cos(yaw / 2),
    };
  }

  function goalStatusLabel(status) {
    return {
      0: "未知",
      1: "已接受",
      2: "执行中",
      3: "取消中",
      4: "已成功",
      5: "已取消",
      6: "已中止",
    }[status] || `状态 ${status}`;
  }

  function clamp(value, minimum, maximum) {
    return Math.min(maximum, Math.max(minimum, value));
  }

  function degToRad(degrees) {
    return (degrees * Math.PI) / 180;
  }

  function radToDeg(radians) {
    return (radians * 180) / Math.PI;
  }

  window.gen0WebControl = {
    connect: connectRos,
    disconnect: () => disconnectRos(true),
    stop: () => stopRobot(true),
    createNavigationClient: createNavigationClientOnRosReady,
  };
})();
