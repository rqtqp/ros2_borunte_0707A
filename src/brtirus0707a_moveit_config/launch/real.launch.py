"""Real-arm MoveIt 2 bringup (Phase 4).

Drives the physical BRTIRUS0707A through the SAME MoveIt config as the sim, by
pointing topic_based_ros2_control at the borunte0707a_driver topics instead of
Isaac Sim:

    driver joint_state_publisher --(/hw_joint_states)--> TopicBasedSystem (state)
    TopicBasedSystem (command) --(/joint_command)--> driver motion_bridge --> arm

`/hw_joint_states` is deliberately separate from `/joint_states` (which the
joint_state_broadcaster owns) to avoid two publishers on one topic.

SAFETY: the motion bridge starts with dry_run:=true (logs, no motion). Launch
with `dry_run:=false` only with an operator at the e-stop and the calibration
confirmed (see borunte0707a_driver/calibration_helper).

    ros2 launch brtirus0707a_moveit_config real.launch.py            # dry-run
    ros2 launch brtirus0707a_moveit_config real.launch.py dry_run:=false speed_pct:=5.0
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Topic wiring between the driver and topic_based_ros2_control.
HW_STATE_TOPIC = "/hw_joint_states"     # driver -> TopicBasedSystem (feedback)
HW_COMMAND_TOPIC = "/joint_command"     # TopicBasedSystem -> driver (targets)


def load_text(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def generate_launch_description():
    moveit_config_pkg = get_package_share_directory("brtirus0707a_moveit_config")
    initial_positions_path = os.path.join(
        moveit_config_pkg, "config", "initial_positions.yaml"
    )

    dry_run = LaunchConfiguration("dry_run")
    speed_pct = LaunchConfiguration("speed_pct")
    max_rate_hz = LaunchConfiguration("max_rate_hz")
    max_step_deg = LaunchConfiguration("max_step_deg")
    goal_settle_sec = LaunchConfiguration("goal_settle_sec")
    send_path = LaunchConfiguration("send_path")
    path_waypoint_deg = LaunchConfiguration("path_waypoint_deg")
    path_smooth = LaunchConfiguration("path_smooth")
    path_max_points = LaunchConfiguration("path_max_points")
    chunk_path = LaunchConfiguration("chunk_path")
    stop_command = LaunchConfiguration("stop_command")

    # Robot description with the hardware bridge pointed at the driver topics.
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                PathJoinSubstitution([
                    FindPackageShare("brtirus0707a_moveit_config"),
                    "config",
                    "brtirus0707a.urdf.xacro",
                ]),
                " initial_positions_file:=", initial_positions_path,
                " joint_commands_topic:=", HW_COMMAND_TOPIC,
                " joint_states_topic:=", HW_STATE_TOPIC,
            ]),
            value_type=str,
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": load_text(
            "brtirus0707a_moveit_config", "config/brtirus0707a.srdf"
        )
    }
    robot_description_kinematics = load_yaml(
        "brtirus0707a_moveit_config", "config/kinematics.yaml"
    )
    robot_description_planning = load_yaml(
        "brtirus0707a_moveit_config", "config/joint_limits.yaml"
    )
    ompl_planning = load_yaml("brtirus0707a_moveit_config", "config/ompl_planning.yaml")
    moveit_controllers = load_yaml(
        "brtirus0707a_moveit_config", "config/moveit_controllers.yaml"
    )
    # MoveIt (Humble) wants the OMPL config NESTED under the pipeline name plus a
    # planning_pipelines list; passing it flat leaves move_group unable to find
    # pipeline "ompl", so it silently falls back to CHOMP.
    ompl_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_planning,
    }

    move_group_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_pipeline,
        moveit_controllers,
        {"publish_robot_description": True},
        {"publish_robot_description_semantic": True},
    ]

    ros2_controllers_path = PathJoinSubstitution([
        FindPackageShare("brtirus0707a_moveit_config"), "config", "ros2_controllers.yaml",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("brtirus0707a_moveit_config"), "config", "brtirus0707a_moveit.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "dry_run", default_value="true",
            description="motion bridge dry-run; set false ONLY with operator at e-stop",
        ),
        DeclareLaunchArgument("speed_pct", default_value="5.0"),
        DeclareLaunchArgument("max_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("max_step_deg", default_value="10.0"),
        DeclareLaunchArgument(
            "goal_settle_sec", default_value="0.5",
            description="point-to-point: send MoveIt's final goal as one AddRCC "
                        "after the streamed setpoint holds still this long; 0 = stream",
        ),
        DeclareLaunchArgument(
            "send_path", default_value="true",
            description="follow MoveIt's planned path: send the trajectory waypoints "
                        "as one multi-point AddRCC instead of just the endpoint "
                        "(set false for endpoint-only point-to-point)",
        ),
        DeclareLaunchArgument("path_waypoint_deg", default_value="5.0"),
        DeclareLaunchArgument("path_smooth", default_value="1"),
        DeclareLaunchArgument(
            "chunk_path", default_value="false",
            description="send the FULL path as sequential AddRCC chunks (faithful, "
                        "brief stop per chunk) instead of one downsampled AddRCC",
        ),
        DeclareLaunchArgument(
            "path_max_points", default_value="8",
            description="cap waypoints per AddRCC; the controller rejects long "
                        "instruction lists (>=~10 pts unreliable)",
        ),
        DeclareLaunchArgument(
            "stop_command", default_value="actionStop",
            description="control command the /stop service sends (actionStop / "
                        "actionPause / stopButton)",
        ),
        SetEnvironmentVariable(
            name="LD_LIBRARY_PATH",
            value=[
                "/opt/ros/humble/opt/rviz_ogre_vendor/lib:",
                EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
            ],
        ),

        # --- Real hardware driver (borunte0707a_driver) ---
        # Real joint feedback -> the topic TopicBasedSystem reads as state.
        Node(
            package="borunte0707a_driver",
            executable="joint_state_publisher",
            name="borunte0707a_joint_state_publisher",
            remappings=[("joint_states", HW_STATE_TOPIC)],
            parameters=[{"publish_rate_hz": 20.0}],
            output="screen",
        ),
        # TopicBasedSystem command stream -> AddRCC (rate-limited, gated).
        Node(
            package="borunte0707a_driver",
            executable="motion_bridge",
            name="borunte0707a_motion_bridge",
            parameters=[{
                "command_topic": HW_COMMAND_TOPIC,
                "dry_run": dry_run,
                "speed_pct": speed_pct,
                "max_rate_hz": max_rate_hz,
                "max_step_deg": max_step_deg,
                "goal_settle_sec": goal_settle_sec,
                "send_path": send_path,
                "path_waypoint_deg": path_waypoint_deg,
                "path_smooth": path_smooth,
                "path_max_points": path_max_points,
                "chunk_path": chunk_path,
                "stop_command": stop_command,
            }],
            output="screen",
        ),

        # --- Standard MoveIt 2 / ros2_control stack ---
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, ros2_controllers_path],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["brtirus0707a_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=move_group_params,
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                robot_description_planning,
            ],
            output="screen",
        ),
    ])
