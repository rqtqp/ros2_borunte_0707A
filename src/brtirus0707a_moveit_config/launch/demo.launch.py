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
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("brtirus0707a_description"),
                            "urdf",
                            "brtirus0707a.urdf.xacro",
                        ]
                    ),
                ]
            ),
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

    move_group_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_planning,
        {"publish_robot_description": True},
        {"publish_robot_description_semantic": True},
    ]

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("brtirus0707a_moveit_config"),
            "config",
            "brtirus0707a_moveit.rviz",
        ]
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value=[
                    "/opt/ros/humble/opt/rviz_ogre_vendor/lib:",
                    EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
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
        ]
    )
