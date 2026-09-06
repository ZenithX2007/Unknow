from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    backend = LaunchConfiguration("backend")
    qcnet_root = LaunchConfiguration("qcnet_root")
    ckpt_path = LaunchConfiguration("ckpt_path")
    device = LaunchConfiguration("device")
    prediction_rate = LaunchConfiguration("prediction_rate")

    default_params_file = PathJoinSubstitution(
        [FindPackageShare("qcnet_prediction"), "config", "qcnet_prediction.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="QCNet prediction bridge parameter file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                choices=["true", "false"],
                description="Use the shared Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "backend",
                default_value="qcnet",
                description="Prediction backend: auto, qcnet, or constant_velocity.",
            ),
            DeclareLaunchArgument(
                "qcnet_root",
                default_value="/home/zjxue2007/QCNet",
                description="Path to the original QCNet repository.",
            ),
            DeclareLaunchArgument(
                "ckpt_path",
                default_value="/home/zjxue2007/QCNet_AV2.ckpt",
                description="Path to the QCNet checkpoint.",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="cuda",
                description="Torch device used by QCNet when available.",
            ),
            DeclareLaunchArgument(
                "prediction_rate",
                default_value="5.0",
                description="QCNet prediction loop frequency in Hz.",
            ),
            Node(
                package="qcnet_prediction",
                executable="qcnet_prediction_node",
                name="qcnet_prediction_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "backend": backend,
                        "qcnet_root": qcnet_root,
                        "ckpt_path": ckpt_path,
                        "device": device,
                        "prediction_rate": prediction_rate,
                    },
                ],
            ),
        ]
    )
