import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.actions import SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration


def quaternion_from_rpy(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def spawn_request(model_sdf, name, pose):
    x, y, z, roll, pitch, yaw = [float(value) for value in pose]
    qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
    return (
        f'sdf_filename: "{model_sdf}"\n'
        f'name: "{name}"\n'
        "allow_renaming: false\n"
        "pose {\n"
        f"  position {{ x: {x:.4f} y: {y:.4f} z: {z:.4f} }}\n"
        f"  orientation {{ x: {qx:.8f} y: {qy:.8f} z: {qz:.8f} w: {qw:.8f} }}\n"
        "}\n"
    )


def spawn_actions(context, *args, **kwargs):
    package_share = get_package_share_directory("gen0_main")
    world = LaunchConfiguration("world").perform(context)
    scenario = LaunchConfiguration("trash_scenario").perform(context)
    world_name = LaunchConfiguration("gazebo_world_name").perform(context)

    scenario_path = os.path.join(
        package_share,
        "worlds",
        "trash_scenarios",
        world,
        f"{scenario}.json",
    )
    if not os.path.exists(scenario_path):
        return [
            LogInfo(
                msg=(
                    f"[trash_spawn] Scenario not found: {scenario_path}; "
                    "skipping trash spawn"
                )
            )
        ]

    with open(scenario_path, "r", encoding="utf-8") as scenario_file:
        items = json.load(scenario_file)

    actions = []
    for index, item in enumerate(items):
        model = item["model"]
        name = item.get("name", f"{model}_{index}")
        pose = item["pose"]
        model_sdf = os.path.join(package_share, "models", model, "model.sdf")
        if not os.path.exists(model_sdf):
            actions.append(LogInfo(msg=f"[trash_spawn] Missing model: {model_sdf}"))
            continue

        actions.append(
            TimerAction(
                period=1.0 + index * 0.3,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ign",
                            "service",
                            "-s",
                            f"/world/{world_name}/create",
                            "--reqtype",
                            "ignition.msgs.EntityFactory",
                            "--reptype",
                            "ignition.msgs.Boolean",
                            "--timeout",
                            "5000",
                            "--req",
                            spawn_request(model_sdf, name, pose),
                        ],
                        output="screen",
                    )
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="my_map",
                description="World scenario namespace for trash placement.",
            ),
            DeclareLaunchArgument(
                "trash_scenario",
                default_value="small_trash",
                description="Trash scenario JSON filename without extension.",
            ),
            DeclareLaunchArgument(
                "gazebo_world_name",
                default_value="default",
                description="Gazebo world name used by the create service.",
            ),
            DeclareLaunchArgument(
                "partition",
                default_value="gen0_scurm_demo",
                description="Gazebo transport partition used by the running simulation.",
            ),
            SetEnvironmentVariable("IGN_PARTITION", LaunchConfiguration("partition")),
            SetEnvironmentVariable("GZ_PARTITION", LaunchConfiguration("partition")),
            OpaqueFunction(function=spawn_actions),
        ]
    )
