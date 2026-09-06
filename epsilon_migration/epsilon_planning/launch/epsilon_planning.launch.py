from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    desired_velocity = LaunchConfiguration("desired_velocity")
    use_scene_bridge = LaunchConfiguration("use_scene_bridge")
    path_topic = LaunchConfiguration("path_topic")
    costmap_topic = LaunchConfiguration("costmap_topic")
    object_pose_topic = LaunchConfiguration("object_pose_topic")
    actor_pose_topics = LaunchConfiguration("actor_pose_topics")
    use_scenario_actor_publisher = LaunchConfiguration("use_scenario_actor_publisher")
    actor_scenario_path = LaunchConfiguration("actor_scenario_path")
    scenario_actor_pose_topic_prefix = LaunchConfiguration("scenario_actor_pose_topic_prefix")
    scenario_actor_publish_rate = LaunchConfiguration("scenario_actor_publish_rate")
    scenario_actor_frame_id = LaunchConfiguration("scenario_actor_frame_id")
    scenario_actor_world_sdf_path = LaunchConfiguration("scenario_actor_world_sdf_path")
    scenario_actor_world_vehicle_name = LaunchConfiguration("scenario_actor_world_vehicle_name")
    scenario_actor_world_to_output = LaunchConfiguration("scenario_actor_world_to_output")
    scenario_actor_output_origin_xy = LaunchConfiguration("scenario_actor_output_origin_xy")
    predicted_trajectories_topic = LaunchConfiguration("predicted_trajectories_topic")
    use_qcnet_prediction = LaunchConfiguration("use_qcnet_prediction")
    qcnet_params_file = LaunchConfiguration("qcnet_params_file")
    qcnet_backend = LaunchConfiguration("qcnet_backend")
    qcnet_root = LaunchConfiguration("qcnet_root")
    qcnet_ckpt_path = LaunchConfiguration("qcnet_ckpt_path")
    qcnet_device = LaunchConfiguration("qcnet_device")
    prediction_rate = LaunchConfiguration("prediction_rate")
    dynamic_publish_period = LaunchConfiguration("dynamic_publish_period")
    path_timeout = LaunchConfiguration("path_timeout")
    use_cmd_vel_mux = LaunchConfiguration("use_cmd_vel_mux")
    control_source = LaunchConfiguration("control_source")
    nav2_cmd_vel_topic = LaunchConfiguration("nav2_cmd_vel_topic")
    mux_output_cmd_vel_topic = LaunchConfiguration("mux_output_cmd_vel_topic")
    control_mode_topic = LaunchConfiguration("control_mode_topic")
    selected_source_topic = LaunchConfiguration("selected_source_topic")
    epsilon_status_topic = LaunchConfiguration("epsilon_status_topic")
    epsilon_status_timeout = LaunchConfiguration("epsilon_status_timeout")
    fallback_to_nav2 = LaunchConfiguration("fallback_to_nav2")

    default_params_file = PathJoinSubstitution(
        [FindPackageShare("epsilon_planning"), "config", "epsilon_planning.yaml"]
    )
    default_qcnet_params_file = PathJoinSubstitution(
        [FindPackageShare("qcnet_prediction"), "config", "qcnet_prediction.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="EPSILON ROS 2 planner parameter file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                choices=["true", "false"],
                description="Use the shared Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/epsilon/cmd_vel_raw",
                description="Raw EPSILON Twist output before command arbitration.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="Odometry source for ego state override.",
            ),
            DeclareLaunchArgument(
                "desired_velocity",
                default_value="1.5",
                description="EUDM task desired velocity in m/s.",
            ),
            DeclareLaunchArgument(
                "use_scene_bridge",
                default_value="true",
                description="Start the generic Path/OccupancyGrid/Odometry to EPSILON scene bridge.",
            ),
            DeclareLaunchArgument(
                "path_topic",
                default_value="/plan_smoothed",
                description="Reference path used by the scene bridge to build EPSILON LaneNet.",
            ),
            DeclareLaunchArgument(
                "costmap_topic",
                default_value="/projected_costmap",
                description="OccupancyGrid used by the scene bridge to build EPSILON static obstacles.",
            ),
            DeclareLaunchArgument(
                "object_pose_topic",
                default_value="/gen0_perception/trash_poses",
                description="Optional PoseArray converted by the scene bridge into circular static obstacles.",
            ),
            DeclareLaunchArgument(
                "actor_pose_topics",
                default_value="",
                description="Comma-separated PoseStamped topics converted by the scene bridge into dynamic vehicles.",
            ),
            DeclareLaunchArgument(
                "use_scenario_actor_publisher",
                default_value="false",
                choices=["true", "false"],
                description="Publish interpolated actor poses from a Gazebo actor scenario SDF.",
            ),
            DeclareLaunchArgument(
                "actor_scenario_path",
                default_value="",
                description="Gazebo actor scenario SDF used by the scenario actor pose publisher.",
            ),
            DeclareLaunchArgument(
                "scenario_actor_pose_topic_prefix",
                default_value="/epsilon/scenario_actor",
                description="Prefix for scenario actor PoseStamped topics.",
            ),
            DeclareLaunchArgument(
                "scenario_actor_publish_rate",
                default_value="10.0",
                description="Scenario actor PoseStamped publish rate in Hz.",
            ),
            DeclareLaunchArgument(
                "scenario_actor_frame_id",
                default_value="map",
                description="Frame id used for scenario actor PoseStamped topics.",
            ),
            DeclareLaunchArgument(
                "scenario_actor_world_sdf_path",
                default_value="",
                description="World SDF used to transform Gazebo world actor poses into the output frame.",
            ),
            DeclareLaunchArgument(
                "scenario_actor_world_vehicle_name",
                default_value="gen0_model",
                description="Vehicle model name in the world SDF used as the output-frame origin.",
            ),
            DeclareLaunchArgument(
                "scenario_actor_world_to_output",
                default_value="true",
                choices=["true", "false"],
                description="Apply the same world-to-output transform used by the actor obstacle costmap.",
            ),
            DeclareLaunchArgument(
                "scenario_actor_output_origin_xy",
                default_value="0.0,0.0",
                description="Output-frame origin offset used by the world-to-output actor transform.",
            ),
            DeclareLaunchArgument(
                "predicted_trajectories_topic",
                default_value="/epsilon/predicted_trajectories",
                description="External predicted trajectories consumed by the integrated planner.",
            ),
            DeclareLaunchArgument(
                "use_qcnet_prediction",
                default_value="true",
                description="Start the QCNet/constant-velocity prediction bridge.",
            ),
            DeclareLaunchArgument(
                "qcnet_params_file",
                default_value=default_qcnet_params_file,
                description="QCNet prediction bridge parameter file.",
            ),
            DeclareLaunchArgument(
                "qcnet_backend",
                default_value="qcnet",
                description="Prediction backend: auto, qcnet, or constant_velocity.",
            ),
            DeclareLaunchArgument(
                "qcnet_root",
                default_value="/home/zjxue2007/QCNet",
                description="Path to the original QCNet repository.",
            ),
            DeclareLaunchArgument(
                "qcnet_ckpt_path",
                default_value="/home/zjxue2007/QCNet_AV2.ckpt",
                description="Path to the QCNet checkpoint.",
            ),
            DeclareLaunchArgument(
                "qcnet_device",
                default_value="cuda",
                description="Torch device used by QCNet when available.",
            ),
            DeclareLaunchArgument(
                "prediction_rate",
                default_value="5.0",
                description="QCNet prediction loop frequency in Hz.",
            ),
            DeclareLaunchArgument(
                "dynamic_publish_period",
                default_value="0.1",
                description="Scene bridge dynamic-scene publish period in seconds.",
            ),
            DeclareLaunchArgument(
                "path_timeout",
                default_value="30.0",
                description="Maximum wall-time age in seconds for the latest Nav2 path before EPSILON treats LaneNet as stale.",
            ),
            DeclareLaunchArgument(
                "use_cmd_vel_mux",
                default_value="false",
                choices=["true", "false"],
                description="Start the Nav2/EPSILON command arbiter.",
            ),
            DeclareLaunchArgument(
                "control_source",
                default_value="nav2",
                choices=["auto", "nav2", "epsilon", "stop"],
                description="Initial command source selected by the arbiter. auto selects EPSILON only while it reports a fresh valid control output.",
            ),
            DeclareLaunchArgument(
                "nav2_cmd_vel_topic",
                default_value="/control/nav2_cmd_vel_raw",
                description="Separated Nav2 raw velocity input to the arbiter.",
            ),
            DeclareLaunchArgument(
                "mux_output_cmd_vel_topic",
                default_value="/control/cmd_vel_raw",
                description="Arbiter output consumed by nav2_pose_guard.",
            ),
            DeclareLaunchArgument(
                "control_mode_topic",
                default_value="/epsilon/control_mode",
                description="Optional String topic for auto, nav2, epsilon, or stop mode changes.",
            ),
            DeclareLaunchArgument(
                "selected_source_topic",
                default_value="/epsilon/selected_control_source",
                description="String diagnostics topic published by the command arbiter with the selected command source.",
            ),
            DeclareLaunchArgument(
                "epsilon_status_topic",
                default_value="/epsilon/status",
                description="EPSILON planner status topic used by automatic command arbitration.",
            ),
            DeclareLaunchArgument(
                "epsilon_status_timeout",
                default_value="0.6",
                description="Maximum age of a successful EPSILON status for automatic command selection.",
            ),
            DeclareLaunchArgument(
                "fallback_to_nav2",
                default_value="true",
                choices=["true", "false"],
                description="Fall back to the fresh Nav2 command when EPSILON is unavailable.",
            ),
            Node(
                package="epsilon_planning",
                executable="scenario_actor_pose_publisher.py",
                name="epsilon_scenario_actor_pose_publisher",
                output="screen",
                condition=IfCondition(use_scenario_actor_publisher),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "actors_scenario_path": actor_scenario_path,
                        "topic_prefix": scenario_actor_pose_topic_prefix,
                        "frame_id": scenario_actor_frame_id,
                        "publish_rate": scenario_actor_publish_rate,
                        "world_sdf_path": scenario_actor_world_sdf_path,
                        "world_vehicle_name": scenario_actor_world_vehicle_name,
                        "transform_world_to_output": scenario_actor_world_to_output,
                        "output_origin_xy": scenario_actor_output_origin_xy,
                    }
                ],
            ),
            Node(
                package="epsilon_planning",
                executable="epsilon_scene_bridge_node",
                name="epsilon_scene_bridge_node",
                output="screen",
                condition=IfCondition(use_scene_bridge),
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "path_topic": path_topic,
                        "costmap_topic": costmap_topic,
                        "object_pose_topic": object_pose_topic,
                        "actor_pose_topics": actor_pose_topics,
                        "odom_topic": odom_topic,
                        "dynamic_publish_period": dynamic_publish_period,
                        "path_timeout": path_timeout,
                    },
                ],
            ),
            Node(
                package="qcnet_prediction",
                executable="qcnet_prediction_node",
                name="qcnet_prediction_node",
                output="screen",
                condition=IfCondition(use_qcnet_prediction),
                parameters=[
                    qcnet_params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "backend": qcnet_backend,
                        "qcnet_root": qcnet_root,
                        "ckpt_path": qcnet_ckpt_path,
                        "device": qcnet_device,
                        "predicted_trajectories_topic": predicted_trajectories_topic,
                        "prediction_rate": prediction_rate,
                    },
                ],
            ),
            Node(
                package="epsilon_planning",
                executable="epsilon_integrated_planner_node",
                name="epsilon_integrated_planner_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "cmd_vel_topic": cmd_vel_topic,
                        "odom_topic": odom_topic,
                        "desired_velocity": desired_velocity,
                        "predicted_trajectories_topic": predicted_trajectories_topic,
                    },
                ],
            ),
            Node(
                package="epsilon_planning",
                executable="epsilon_cmd_vel_mux_node",
                name="epsilon_cmd_vel_mux_node",
                output="screen",
                condition=IfCondition(use_cmd_vel_mux),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "epsilon_cmd_vel_topic": cmd_vel_topic,
                        "nav2_cmd_vel_topic": nav2_cmd_vel_topic,
                        "output_cmd_vel_topic": mux_output_cmd_vel_topic,
                        "control_source": control_source,
                        "control_mode_topic": control_mode_topic,
                        "selected_source_topic": selected_source_topic,
                        "epsilon_status_topic": epsilon_status_topic,
                        "epsilon_status_timeout": epsilon_status_timeout,
                        "fallback_to_nav2": fallback_to_nav2,
                    }
                ],
            ),
        ]
    )
