import math
import os
import sys
import time
from collections import deque
from collections.abc import Iterable as IterableABC
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from vehicle_msgs.msg import ArenaInfoDynamic
from vehicle_msgs.msg import ArenaInfoStatic
from vehicle_msgs.msg import PredictedState
from vehicle_msgs.msg import PredictedTrajectory
from vehicle_msgs.msg import PredictedTrajectoryArray


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def duration_from_sec(seconds: float) -> Duration:
    seconds = max(0.0, float(seconds))
    sec = int(math.floor(seconds))
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1000000000:
        sec += 1
        nanosec -= 1000000000
    return Duration(sec=sec, nanosec=nanosec)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def interpolate_angle(a0: float, a1: float, alpha: float) -> float:
    return wrap_angle(a0 + wrap_angle(a1 - a0) * alpha)


def safe_float(value: float, default: float = 0.0) -> float:
    return float(value) if math.isfinite(float(value)) else default


@dataclass
class VehicleParamSnapshot:
    width: float
    length: float
    wheel_base: float
    front_suspension: float
    rear_suspension: float
    max_steering_angle: float
    d_cr: float
    max_longitudinal_acc: float
    max_lateral_acc: float


@dataclass
class AgentSample:
    stamp: float
    x: float
    y: float
    heading: float
    velocity: float
    acceleration: float
    curvature: float
    steer: float
    type_name: str
    param: VehicleParamSnapshot


class QcnetPredictionNode(Node):
    def __init__(self) -> None:
        super().__init__("qcnet_prediction_node")
        self._declare_parameters()
        self._read_parameters()

        self.history: Dict[int, deque] = {}
        self.latest_lane_net = None
        self.latest_frame_id = "map"
        self.model = None
        self.torch = None
        self.hetero_data_type = None
        self.torch_device = None
        self.active_backend = "constant_velocity"
        self.qcnet_dependency_error = None
        self.warned_messages = set()
        self.latest_dynamic_scene: Optional[ArenaInfoDynamic] = None
        self.last_prediction_scene_stamp = 0.0

        self._try_load_qcnet()

        static_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.static_sub = self.create_subscription(
            ArenaInfoStatic,
            self.arena_info_static_topic,
            self._on_static_scene,
            static_qos,
        )
        self.dynamic_sub = self.create_subscription(
            ArenaInfoDynamic,
            self.arena_info_dynamic_topic,
            self._on_dynamic_scene,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.prediction_pub = self.create_publisher(
            PredictedTrajectoryArray,
            self.predicted_trajectories_topic,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.prediction_timer = self.create_timer(
            1.0 / self.prediction_rate, self._prediction_timer_callback
        )

        self.get_logger().info(
            "QCNet prediction bridge ready: backend=%s active=%s ckpt=%s rate=%.1fHz"
            % (self.backend, self.active_backend, self.ckpt_path, self.prediction_rate)
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("backend", "qcnet")
        self.declare_parameter("qcnet_root", "/home/zjxue2007/QCNet")
        self.declare_parameter("ckpt_path", "/home/zjxue2007/QCNet_AV2.ckpt")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("ego_id", 0)
        self.declare_parameter("arena_info_static_topic", "/epsilon/arena_info_static")
        self.declare_parameter("arena_info_dynamic_topic", "/epsilon/arena_info_dynamic")
        self.declare_parameter("predicted_trajectories_topic", "/epsilon/predicted_trajectories")
        self.declare_parameter("prediction_rate", 5.0)
        self.declare_parameter("history_steps", 50)
        self.declare_parameter("future_steps", 60)
        self.declare_parameter("prediction_dt", 0.1)
        self.declare_parameter("min_history_steps", 3)
        self.declare_parameter("max_history_age", 6.0)
        self.declare_parameter("max_predicted_agents", 16)
        self.declare_parameter("publish_top_k_modes", 1)
        self.declare_parameter("source_position_is_rear_axle", True)
        self.declare_parameter("output_position_is_rear_axle", False)
        self.declare_parameter("default_vehicle_width", 1.9)
        self.declare_parameter("default_vehicle_length", 4.88)
        self.declare_parameter("default_vehicle_wheel_base", 2.8)
        self.declare_parameter("default_vehicle_d_cr", 1.34)

    def _read_parameters(self) -> None:
        self.backend = str(self.get_parameter("backend").value)
        self.qcnet_root = str(self.get_parameter("qcnet_root").value)
        self.ckpt_path = str(self.get_parameter("ckpt_path").value)
        self.device = str(self.get_parameter("device").value)
        self.ego_id = int(self.get_parameter("ego_id").value)
        self.arena_info_static_topic = str(self.get_parameter("arena_info_static_topic").value)
        self.arena_info_dynamic_topic = str(self.get_parameter("arena_info_dynamic_topic").value)
        self.predicted_trajectories_topic = str(
            self.get_parameter("predicted_trajectories_topic").value
        )
        self.prediction_rate = max(1.0, float(self.get_parameter("prediction_rate").value))
        self.history_steps = int(self.get_parameter("history_steps").value)
        self.future_steps = int(self.get_parameter("future_steps").value)
        self.prediction_dt = float(self.get_parameter("prediction_dt").value)
        self.min_history_steps = int(self.get_parameter("min_history_steps").value)
        self.max_history_age = float(self.get_parameter("max_history_age").value)
        self.max_predicted_agents = int(self.get_parameter("max_predicted_agents").value)
        self.publish_top_k_modes = max(1, int(self.get_parameter("publish_top_k_modes").value))
        self.source_position_is_rear_axle = bool(
            self.get_parameter("source_position_is_rear_axle").value
        )
        self.output_position_is_rear_axle = bool(
            self.get_parameter("output_position_is_rear_axle").value
        )
        self.default_param = VehicleParamSnapshot(
            width=float(self.get_parameter("default_vehicle_width").value),
            length=float(self.get_parameter("default_vehicle_length").value),
            wheel_base=float(self.get_parameter("default_vehicle_wheel_base").value),
            front_suspension=0.93,
            rear_suspension=1.10,
            max_steering_angle=0.4,
            d_cr=float(self.get_parameter("default_vehicle_d_cr").value),
            max_longitudinal_acc=1.0,
            max_lateral_acc=1.2,
        )

    def _try_load_qcnet(self) -> None:
        if self.backend not in ("auto", "qcnet"):
            self.active_backend = "constant_velocity"
            return
        env_hint = os.path.join(self.qcnet_root, "environment.yml")
        if not os.path.isdir(self.qcnet_root):
            message = "qcnet_root missing: %s (QCNet env: %s)" % (self.qcnet_root, env_hint)
            self.qcnet_dependency_error = message
            if self.backend == "qcnet":
                raise RuntimeError(message)
            self._warn_once("QCNet backend unavailable, using constant_velocity: %s" % message)
            return
        if not os.path.isfile(self.ckpt_path):
            message = "ckpt_path missing: %s (QCNet env: %s)" % (self.ckpt_path, env_hint)
            self.qcnet_dependency_error = message
            if self.backend == "qcnet":
                raise RuntimeError(message)
            self._warn_once("QCNet backend unavailable, using constant_velocity: %s" % message)
            return

        if self.qcnet_root not in sys.path:
            sys.path.insert(0, self.qcnet_root)

        try:
            import torch
        except Exception as exc:
            self.qcnet_dependency_error = "torch: %s" % exc
            if self.backend == "qcnet":
                raise RuntimeError(
                    "QCNet backend requested but PyTorch is unavailable: %s (QCNet env: %s)"
                    % (exc, env_hint)
                ) from exc
            self._warn_once(
                "QCNet backend unavailable, using constant_velocity: missing PyTorch: %s "
                "(QCNet env: %s)" % (exc, env_hint)
            )
            return

        try:
            from torch_geometric.data import HeteroData
        except Exception as exc:
            self.qcnet_dependency_error = "torch_geometric.data.HeteroData: %s" % exc
            if self.backend == "qcnet":
                raise RuntimeError(
                    "QCNet backend requested but torch_geometric is unavailable: %s "
                    "(QCNet env: %s)" % (exc, env_hint)
                ) from exc
            self._warn_once(
                "QCNet backend unavailable, using constant_velocity: missing torch_geometric: %s "
                "(QCNet env: %s)" % (exc, env_hint)
            )
            return

        try:
            from predictors import QCNet

            self.torch = torch
            self.hetero_data_type = HeteroData
            requested_device = self.device.strip().lower()
            if requested_device == "auto":
                resolved_device = "cuda" if torch.cuda.is_available() else "cpu"
            else:
                resolved_device = self.device
            if resolved_device.startswith("cuda") and not torch.cuda.is_available():
                raise RuntimeError(
                    "CUDA was requested but this PyTorch installation has no CUDA support"
                )
            self.torch_device = torch.device(resolved_device)
            if requested_device == "auto" and resolved_device == "cpu":
                self._warn_once(
                    "QCNet device=auto resolved to CPU; install a CUDA-enabled PyTorch "
                    "build for real-time inference"
                )
            self.model = QCNet.load_from_checkpoint(
                checkpoint_path=self.ckpt_path,
                map_location=self.torch_device,
            )
            self.model.to(self.torch_device)
            self.model.eval()
            self.history_steps = int(getattr(self.model, "num_historical_steps", self.history_steps))
            self.future_steps = int(getattr(self.model, "num_future_steps", self.future_steps))
            self.active_backend = "qcnet"
            self.get_logger().info(
                "loaded QCNet checkpoint: history_steps=%d future_steps=%d device=%s"
                % (self.history_steps, self.future_steps, self.torch_device)
            )
        except Exception as exc:
            self.qcnet_dependency_error = "predictors.QCNet / checkpoint load: %s" % exc
            if self.backend == "qcnet":
                raise RuntimeError(
                    "QCNet backend initialization failed: %s (QCNet env: %s)"
                    % (exc, env_hint)
                ) from exc
            self._warn_once(
                "QCNet backend unavailable, using constant_velocity: %s (QCNet env: %s)"
                % (exc, env_hint)
            )
            self.model = None
            self.torch = None
            self.hetero_data_type = None
            self.torch_device = None
            self.active_backend = "constant_velocity"

    def _on_static_scene(self, msg: ArenaInfoStatic) -> None:
        self.latest_lane_net = msg.lane_net

    def _on_dynamic_scene(self, msg: ArenaInfoDynamic) -> None:
        stamp = stamp_to_sec(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        if msg.header.frame_id:
            self.latest_frame_id = msg.header.frame_id

        for vehicle in msg.vehicle_set.vehicles:
            vehicle_id = int(vehicle.id.data)
            state = vehicle.state
            if not math.isfinite(state.vec_position.x) or not math.isfinite(state.vec_position.y):
                continue

            param = self._param_from_msg(vehicle.param)
            sample = AgentSample(
                stamp=stamp,
                x=safe_float(state.vec_position.x),
                y=safe_float(state.vec_position.y),
                heading=safe_float(state.angle),
                velocity=safe_float(state.velocity),
                acceleration=safe_float(state.acceleration),
                curvature=safe_float(state.curvature),
                steer=safe_float(state.steer),
                type_name=vehicle.type.data or vehicle.subclass.data or "vehicle",
                param=param,
            )
            self._append_sample(vehicle_id, sample)

        # Keep only the newest scene. Inference runs from a fixed-rate timer so
        # bursts of actor messages cannot create an inference queue.
        self.latest_dynamic_scene = msg

    def _prediction_timer_callback(self) -> None:
        msg = self.latest_dynamic_scene
        if msg is None:
            return

        stamp = stamp_to_sec(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        if stamp <= self.last_prediction_scene_stamp + 1e-6:
            return

        start_time = time.monotonic()
        prediction_msg = self._predict(stamp)
        self.prediction_pub.publish(prediction_msg)
        self.last_prediction_scene_stamp = stamp

        elapsed = time.monotonic() - start_time
        if elapsed > (1.0 / self.prediction_rate):
            self.get_logger().warning(
                "prediction cycle overran: elapsed=%.3fs period=%.3fs"
                % (elapsed, 1.0 / self.prediction_rate)
            )

    def _append_sample(self, vehicle_id: int, sample: AgentSample) -> None:
        history = self.history.setdefault(vehicle_id, deque(maxlen=max(self.history_steps * 2, 4)))
        if history and abs(history[-1].stamp - sample.stamp) < 1e-4:
            history[-1] = sample
        else:
            history.append(sample)

        while history and sample.stamp - history[0].stamp > self.max_history_age:
            history.popleft()

    def _predict(self, stamp: float) -> PredictedTrajectoryArray:
        if self.active_backend == "qcnet":
            try:
                msg = self._predict_qcnet(stamp)
                if msg.trajectories:
                    return msg
                self._warn_once("QCNet produced no live trajectories; falling back to constant velocity")
            except Exception as exc:
                self._warn_once("QCNet prediction failed; falling back to constant velocity: %s" % exc)
        return self._predict_constant_velocity(stamp)

    def _predict_constant_velocity(self, stamp: float) -> PredictedTrajectoryArray:
        msg = self._new_prediction_array_msg(stamp)
        for vehicle_id in self._prediction_vehicle_ids(stamp):
            if vehicle_id == self.ego_id:
                continue
            history = self.history.get(vehicle_id)
            if not history:
                continue
            current = history[-1]
            vx, vy = self._estimate_velocity_xy(history)
            speed = math.hypot(vx, vy)
            heading = math.atan2(vy, vx) if speed > 1e-3 else current.heading
            center_x, center_y = self._center_from_sample(current)

            states = []
            for step in range(1, self.future_steps + 1):
                dt = step * self.prediction_dt
                pred_x = center_x + vx * dt
                pred_y = center_y + vy * dt
                states.append(
                    self._make_predicted_state(
                        dt=dt,
                        center_x=pred_x,
                        center_y=pred_y,
                        heading=heading,
                        velocity=speed,
                        acceleration=0.0,
                        curvature=0.0,
                        steer=0.0,
                        param=current.param,
                    )
                )
            msg.trajectories.append(
                self._make_predicted_trajectory(vehicle_id, current.type_name, current.param, 1.0, states)
            )
        return msg

    def _predict_qcnet(self, stamp: float) -> PredictedTrajectoryArray:
        if self.model is None or self.torch is None or self.hetero_data_type is None:
            return self._new_prediction_array_msg(stamp)
        data, agent_ids, current_samples = self._build_qcnet_data(stamp)
        if data is None:
            return self._new_prediction_array_msg(stamp)

        torch = self.torch
        with torch.no_grad():
            data = data.to(self.torch_device)
            pred = self.model(data)
            loc_refine_pos = pred["loc_refine_pos"].detach().cpu()
            pi = torch.softmax(pred["pi"].detach().cpu(), dim=-1)
            loc_refine_head = pred.get("loc_refine_head")
            if loc_refine_head is not None:
                loc_refine_head = loc_refine_head.detach().cpu()

        msg = self._new_prediction_array_msg(stamp)
        for agent_index, vehicle_id in enumerate(agent_ids):
            if vehicle_id == self.ego_id:
                continue
            current = current_samples[agent_index]
            if not bool(current):
                continue
            mode_probs = pi[agent_index]
            mode_indices = torch.argsort(mode_probs, descending=True)[: self.publish_top_k_modes]
            for mode_index in mode_indices.tolist():
                states = self._qcnet_states_to_msg(
                    loc_refine_pos[agent_index, mode_index],
                    loc_refine_head[agent_index, mode_index] if loc_refine_head is not None else None,
                    current,
                )
                probability = float(mode_probs[mode_index].item())
                msg.trajectories.append(
                    self._make_predicted_trajectory(
                        vehicle_id, current.type_name, current.param, probability, states
                    )
                )
        return msg

    def _build_qcnet_data(self, stamp: float):
        if self.latest_lane_net is None:
            self._warn_once("waiting for ArenaInfoStatic before QCNet prediction")
            return None, [], []

        agent_ids = self._prediction_vehicle_ids(stamp, include_ego=True)
        predicted_ids = [vehicle_id for vehicle_id in agent_ids if vehicle_id != self.ego_id]
        if not predicted_ids:
            return None, [], []
        if self.ego_id in agent_ids:
            agent_ids = [self.ego_id] + [vehicle_id for vehicle_id in agent_ids if vehicle_id != self.ego_id]

        torch = self.torch
        data = self.hetero_data_type()
        num_agents = len(agent_ids)
        total_steps = self.history_steps + self.future_steps
        position = torch.zeros(num_agents, total_steps, 2, dtype=torch.float)
        heading = torch.zeros(num_agents, total_steps, dtype=torch.float)
        velocity = torch.zeros(num_agents, total_steps, 2, dtype=torch.float)
        valid_mask = torch.zeros(num_agents, total_steps, dtype=torch.bool)
        predict_mask = torch.zeros(num_agents, total_steps, dtype=torch.bool)
        agent_type = torch.zeros(num_agents, dtype=torch.uint8)
        agent_category = torch.zeros(num_agents, dtype=torch.uint8)
        agent_name = []
        current_samples: List[Optional[AgentSample]] = []

        target_times = [
            stamp - (self.history_steps - 1 - step) * self.prediction_dt
            for step in range(self.history_steps)
        ]
        for agent_index, vehicle_id in enumerate(agent_ids):
            history = self.history.get(vehicle_id, deque())
            samples = self._interpolate_history(history, target_times)
            current = history[-1] if history else None
            current_samples.append(current)
            agent_name.append("AV" if vehicle_id == self.ego_id else str(vehicle_id))
            if current is not None:
                agent_type[agent_index] = self._agent_type_index(current.type_name)
                agent_category[agent_index] = 3 if vehicle_id != self.ego_id else 2
            for step, sample in enumerate(samples):
                if sample is None:
                    continue
                center_x, center_y = self._center_from_sample(sample)
                position[agent_index, step, 0] = center_x
                position[agent_index, step, 1] = center_y
                heading[agent_index, step] = sample.heading
                vx, vy = self._velocity_from_sample(sample)
                velocity[agent_index, step, 0] = vx
                velocity[agent_index, step, 1] = vy
                valid_mask[agent_index, step] = True
            if self.history_steps > 1:
                valid_mask[agent_index, 1:self.history_steps] = (
                    valid_mask[agent_index, : self.history_steps - 1]
                    & valid_mask[agent_index, 1:self.history_steps]
                )
                valid_mask[agent_index, 0] = False
            if vehicle_id != self.ego_id and valid_mask[agent_index, self.history_steps - 1]:
                predict_mask[agent_index, self.history_steps:] = True

        data["agent"]["num_nodes"] = num_agents
        data["agent"]["av_index"] = agent_ids.index(self.ego_id) if self.ego_id in agent_ids else 0
        data["agent"]["valid_mask"] = valid_mask
        data["agent"]["predict_mask"] = predict_mask
        data["agent"]["id"] = agent_name
        data["agent"]["type"] = agent_type
        data["agent"]["category"] = agent_category
        data["agent"]["position"] = position
        data["agent"]["heading"] = heading
        data["agent"]["velocity"] = velocity

        if not self._fill_qcnet_map(data):
            return None, [], []
        return data, agent_ids, current_samples

    def _fill_qcnet_map(self, data) -> bool:
        torch = self.torch
        lanes = [lane for lane in self.latest_lane_net.lanes if len(lane.points) >= 2]
        if not lanes:
            self._warn_once("lane net is empty for QCNet")
            return False

        lane_to_index = {int(lane.id): index for index, lane in enumerate(lanes)}
        polygon_position = []
        polygon_orientation = []
        polygon_type = []
        polygon_is_intersection = []
        point_position = []
        point_orientation = []
        point_magnitude = []
        point_type = []
        point_side = []
        point_to_polygon_src = []
        point_to_polygon_dst = []

        for polygon_index, lane in enumerate(lanes):
            pts = [(float(point.x), float(point.y)) for point in lane.points]
            dx = pts[1][0] - pts[0][0]
            dy = pts[1][1] - pts[0][1]
            polygon_position.append([pts[0][0], pts[0][1]])
            polygon_orientation.append(math.atan2(dy, dx))
            polygon_type.append(0)
            polygon_is_intersection.append(1)

            for point_index in range(len(pts) - 1):
                x0, y0 = pts[point_index]
                x1, y1 = pts[point_index + 1]
                seg_dx = x1 - x0
                seg_dy = y1 - y0
                point_to_polygon_src.append(len(point_position))
                point_to_polygon_dst.append(polygon_index)
                point_position.append([x0, y0])
                point_orientation.append(math.atan2(seg_dy, seg_dx))
                point_magnitude.append(math.hypot(seg_dx, seg_dy))
                point_type.append(16)
                point_side.append(2)

        edge_src = []
        edge_dst = []
        edge_type = []
        for lane in lanes:
            dst = lane_to_index[int(lane.id)]
            for source_lane_id in self._lane_id_values(lane.father_id):
                self._append_lane_edge(edge_src, edge_dst, edge_type, lane_to_index, source_lane_id, dst, 1)
            for source_lane_id in self._lane_id_values(lane.child_id):
                self._append_lane_edge(edge_src, edge_dst, edge_type, lane_to_index, source_lane_id, dst, 2)
            self._append_lane_edge(edge_src, edge_dst, edge_type, lane_to_index, int(lane.l_lane_id), dst, 3)
            self._append_lane_edge(edge_src, edge_dst, edge_type, lane_to_index, int(lane.r_lane_id), dst, 4)

        data["map_polygon"]["num_nodes"] = len(lanes)
        data["map_polygon"]["position"] = torch.tensor(polygon_position, dtype=torch.float)
        data["map_polygon"]["orientation"] = torch.tensor(polygon_orientation, dtype=torch.float)
        data["map_polygon"]["type"] = torch.tensor(polygon_type, dtype=torch.uint8)
        data["map_polygon"]["is_intersection"] = torch.tensor(polygon_is_intersection, dtype=torch.uint8)
        data["map_point"]["num_nodes"] = len(point_position)
        data["map_point"]["position"] = torch.tensor(point_position, dtype=torch.float)
        data["map_point"]["orientation"] = torch.tensor(point_orientation, dtype=torch.float)
        data["map_point"]["magnitude"] = torch.tensor(point_magnitude, dtype=torch.float)
        data["map_point"]["type"] = torch.tensor(point_type, dtype=torch.uint8)
        data["map_point"]["side"] = torch.tensor(point_side, dtype=torch.uint8)
        data["map_point", "to", "map_polygon"]["edge_index"] = torch.tensor(
            [point_to_polygon_src, point_to_polygon_dst], dtype=torch.long
        )
        data["map_polygon", "to", "map_polygon"]["edge_index"] = torch.tensor(
            [edge_src, edge_dst], dtype=torch.long
        )
        data["map_polygon", "to", "map_polygon"]["type"] = torch.tensor(edge_type, dtype=torch.uint8)
        return True

    @staticmethod
    def _append_lane_edge(edge_src, edge_dst, edge_type, lane_to_index, source_lane_id, dst, relation):
        if source_lane_id in lane_to_index:
            edge_src.append(lane_to_index[source_lane_id])
            edge_dst.append(dst)
            edge_type.append(relation)

    @staticmethod
    def _lane_id_values(value) -> List[int]:
        if value is None:
            return []
        if isinstance(value, (str, bytes)):
            return []
        if isinstance(value, IterableABC):
            lane_ids = []
            for item in value:
                try:
                    lane_ids.append(int(item))
                except (TypeError, ValueError):
                    continue
            return lane_ids
        try:
            return [int(value)]
        except (TypeError, ValueError):
            return []

    def _qcnet_states_to_msg(self, local_positions, local_headings, current: AgentSample):
        states = []
        origin_x, origin_y = self._center_from_sample(current)
        cos_h = math.cos(current.heading)
        sin_h = math.sin(current.heading)
        prev_x = origin_x
        prev_y = origin_y
        prev_speed = max(0.0, current.velocity)
        for step in range(local_positions.shape[0]):
            dt = (step + 1) * self.prediction_dt
            local_x = float(local_positions[step, 0].item())
            local_y = float(local_positions[step, 1].item())
            center_x = origin_x + local_x * cos_h - local_y * sin_h
            center_y = origin_y + local_x * sin_h + local_y * cos_h
            if local_headings is not None:
                heading = wrap_angle(current.heading + float(local_headings[step, 0].item()))
            else:
                dx = center_x - prev_x
                dy = center_y - prev_y
                heading = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-4 else current.heading
            speed = math.hypot(center_x - prev_x, center_y - prev_y) / max(self.prediction_dt, 1e-3)
            acceleration = (speed - prev_speed) / max(self.prediction_dt, 1e-3)
            states.append(
                self._make_predicted_state(
                    dt=dt,
                    center_x=center_x,
                    center_y=center_y,
                    heading=heading,
                    velocity=speed,
                    acceleration=acceleration,
                    curvature=0.0,
                    steer=0.0,
                    param=current.param,
                )
            )
            prev_x = center_x
            prev_y = center_y
            prev_speed = speed
        return states

    def _make_predicted_state(
        self,
        dt: float,
        center_x: float,
        center_y: float,
        heading: float,
        velocity: float,
        acceleration: float,
        curvature: float,
        steer: float,
        param: VehicleParamSnapshot,
    ) -> PredictedState:
        state = PredictedState()
        state.time_from_start = duration_from_sec(dt)
        if self.output_position_is_rear_axle:
            state.position.x = center_x - param.d_cr * math.cos(heading)
            state.position.y = center_y - param.d_cr * math.sin(heading)
        else:
            state.position.x = center_x
            state.position.y = center_y
        state.position.z = 0.0
        state.heading = heading
        state.curvature = curvature
        state.velocity = velocity
        state.acceleration = acceleration
        state.steer = steer
        return state

    def _make_predicted_trajectory(
        self,
        vehicle_id: int,
        type_name: str,
        param: VehicleParamSnapshot,
        probability: float,
        states: Iterable[PredictedState],
    ) -> PredictedTrajectory:
        trajectory = PredictedTrajectory()
        trajectory.id.data = int(vehicle_id)
        trajectory.type.data = type_name or "vehicle"
        trajectory.probability = float(probability)
        self._fill_param_msg(trajectory.param, param)
        trajectory.states = list(states)
        return trajectory

    def _new_prediction_array_msg(self, stamp: float) -> PredictedTrajectoryArray:
        msg = PredictedTrajectoryArray()
        sec = int(math.floor(stamp))
        nanosec = int(round((stamp - sec) * 1e9))
        if nanosec >= 1000000000:
            sec += 1
            nanosec -= 1000000000
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec
        msg.header.frame_id = self.latest_frame_id
        return msg

    def _prediction_vehicle_ids(self, stamp: float, include_ego: bool = False) -> List[int]:
        ego_sample = self.history.get(self.ego_id, deque())
        ego = ego_sample[-1] if ego_sample else None
        candidates = []
        for vehicle_id, history in self.history.items():
            if not history:
                continue
            if stamp - history[-1].stamp > self.max_history_age:
                continue
            if len(history) < self.min_history_steps and vehicle_id != self.ego_id:
                continue
            if vehicle_id == self.ego_id:
                if include_ego:
                    candidates.append((vehicle_id, 0.0))
                continue
            if ego is not None:
                dx = history[-1].x - ego.x
                dy = history[-1].y - ego.y
                distance = math.hypot(dx, dy)
            else:
                distance = 0.0
            candidates.append((vehicle_id, distance))
        candidates.sort(key=lambda item: item[1])
        return [vehicle_id for vehicle_id, _ in candidates[: self.max_predicted_agents]]

    def _interpolate_history(self, history: deque, target_times: List[float]) -> List[Optional[AgentSample]]:
        if not history:
            return [None] * len(target_times)
        samples = list(history)
        result: List[Optional[AgentSample]] = []
        cursor = 0
        for target_time in target_times:
            while cursor + 1 < len(samples) and samples[cursor + 1].stamp < target_time:
                cursor += 1
            if cursor + 1 < len(samples) and samples[cursor].stamp <= target_time <= samples[cursor + 1].stamp:
                before = samples[cursor]
                after = samples[cursor + 1]
                span = max(after.stamp - before.stamp, 1e-6)
                alpha = (target_time - before.stamp) / span
                result.append(self._lerp_sample(before, after, target_time, alpha))
            elif abs(samples[-1].stamp - target_time) <= self.prediction_dt * 1.5:
                result.append(samples[-1])
            else:
                result.append(None)
        return result

    @staticmethod
    def _lerp_sample(a: AgentSample, b: AgentSample, stamp: float, alpha: float) -> AgentSample:
        return AgentSample(
            stamp=stamp,
            x=a.x + (b.x - a.x) * alpha,
            y=a.y + (b.y - a.y) * alpha,
            heading=interpolate_angle(a.heading, b.heading, alpha),
            velocity=a.velocity + (b.velocity - a.velocity) * alpha,
            acceleration=a.acceleration + (b.acceleration - a.acceleration) * alpha,
            curvature=a.curvature + (b.curvature - a.curvature) * alpha,
            steer=a.steer + (b.steer - a.steer) * alpha,
            type_name=b.type_name,
            param=b.param,
        )

    def _center_from_sample(self, sample: AgentSample) -> Tuple[float, float]:
        if self.source_position_is_rear_axle:
            return (
                sample.x + sample.param.d_cr * math.cos(sample.heading),
                sample.y + sample.param.d_cr * math.sin(sample.heading),
            )
        return sample.x, sample.y

    @staticmethod
    def _velocity_from_sample(sample: AgentSample) -> Tuple[float, float]:
        return (
            sample.velocity * math.cos(sample.heading),
            sample.velocity * math.sin(sample.heading),
        )

    def _estimate_velocity_xy(self, history: deque) -> Tuple[float, float]:
        if len(history) >= 2:
            newest = history[-1]
            older = history[-2]
            dt = newest.stamp - older.stamp
            if dt > 1e-3:
                x0, y0 = self._center_from_sample(older)
                x1, y1 = self._center_from_sample(newest)
                return (x1 - x0) / dt, (y1 - y0) / dt
        return self._velocity_from_sample(history[-1])

    def _param_from_msg(self, param_msg) -> VehicleParamSnapshot:
        def value_or_default(value, default):
            value = float(value)
            return value if value > 0.0 and math.isfinite(value) else default

        return VehicleParamSnapshot(
            width=value_or_default(param_msg.width, self.default_param.width),
            length=value_or_default(param_msg.length, self.default_param.length),
            wheel_base=value_or_default(param_msg.wheel_base, self.default_param.wheel_base),
            front_suspension=value_or_default(
                param_msg.front_suspension, self.default_param.front_suspension
            ),
            rear_suspension=value_or_default(param_msg.rear_suspension, self.default_param.rear_suspension),
            max_steering_angle=value_or_default(
                param_msg.max_steering_angle, self.default_param.max_steering_angle
            ),
            d_cr=value_or_default(param_msg.d_cr, self.default_param.d_cr),
            max_longitudinal_acc=value_or_default(
                param_msg.max_longitudinal_acc, self.default_param.max_longitudinal_acc
            ),
            max_lateral_acc=value_or_default(param_msg.max_lateral_acc, self.default_param.max_lateral_acc),
        )

    @staticmethod
    def _fill_param_msg(param_msg, param: VehicleParamSnapshot) -> None:
        param_msg.width = param.width
        param_msg.length = param.length
        param_msg.wheel_base = param.wheel_base
        param_msg.front_suspension = param.front_suspension
        param_msg.rear_suspension = param.rear_suspension
        param_msg.max_steering_angle = param.max_steering_angle
        param_msg.d_cr = param.d_cr
        param_msg.max_longitudinal_acc = param.max_longitudinal_acc
        param_msg.max_lateral_acc = param.max_lateral_acc

    @staticmethod
    def _agent_type_index(type_name: str) -> int:
        normalized = (type_name or "").lower()
        if "pedestrian" in normalized or "person" in normalized:
            return 1
        if "motor" in normalized:
            return 2
        if "cycl" in normalized or "bike" in normalized:
            return 3
        if "bus" in normalized:
            return 4
        if "static" in normalized:
            return 5
        if "vehicle" in normalized or "car" in normalized or "truck" in normalized:
            return 0
        return 9

    def _warn_once(self, text: str) -> None:
        if text in self.warned_messages:
            return
        self.warned_messages.add(text)
        self.get_logger().warning(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QcnetPredictionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
