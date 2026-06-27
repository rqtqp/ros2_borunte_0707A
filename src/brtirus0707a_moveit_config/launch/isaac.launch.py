import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_text(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as handle:
        return handle.read()


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def generate_launch_description():
    moveit_config_pkg = get_package_share_directory("brtirus0707a_moveit_config")
    initial_positions_path = os.path.join(
        moveit_config_pkg, "config", "initial_positions.yaml"
    )

    # Robot description that includes ros2_control hardware bridging to Isaac Sim
    # via /isaac_joint_states (Isaac → ros2_control) and /isaac_joint_commands (ros2_control → Isaac)
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                PathJoinSubstitution([
                    FindPackageShare("brtirus0707a_moveit_config"),
                    "config",
                    "brtirus0707a.urdf.xacro",
                ]),
                " initial_positions_file:=",
                initial_positions_path,
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
    ompl_planning = load_yaml(
        "brtirus0707a_moveit_config", "config/ompl_planning.yaml"
    )
    moveit_controllers = load_yaml(
        "brtirus0707a_moveit_config", "config/moveit_controllers.yaml"
    )

    move_group_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_planning,
        moveit_controllers,
        {"publish_robot_description": True},
        {"publish_robot_description_semantic": True},
        {"default_planning_pipeline": "ompl"},
        {"planning_plugin": "ompl_interface/OMPLPlanner"},
    ]

    ros2_controllers_path = PathJoinSubstitution([
        FindPackageShare("brtirus0707a_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("brtirus0707a_moveit_config"),
        "config",
        "brtirus0707a_moveit.rviz",
    ])

    return LaunchDescription([
        SetEnvironmentVariable(
            name="LD_LIBRARY_PATH",
            value=[
                "/opt/ros/humble/opt/rviz_ogre_vendor/lib:",
                EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
            ],
        ),
        # Publishes /tf and /tf_static from robot_description + joint_states
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        # ros2_control node — uses topic_based_ros2_control hardware plugin
        # which bridges /isaac_joint_states → hardware state
        # and hardware commands → /isaac_joint_commands
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, ros2_controllers_path],
            output="screen",
        ),
        # Broadcasts joint states from ros2_control hardware to /joint_states
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
            ],
            output="screen",
        ),
        # JointTrajectoryController — receives FollowJointTrajectory goals from MoveIt
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "brtirus0707a_controller",
                "--controller-manager", "/controller_manager",
            ],
            output="screen",
        ),
        # MoveIt move_group — plans and sends trajectory goals to brtirus0707a_controller
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=move_group_params,
            output="screen",
        ),
        # RViz with MoveIt plugin — no joint_state_publisher_gui, Isaac owns joint states
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
