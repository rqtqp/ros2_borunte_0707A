"""View the model in RViz driven by the REAL arm (read-only, no motion).

Unlike display.launch.py (which uses joint_state_publisher_gui sliders, with no
connection to the controller), this runs the driver's joint_state_publisher so
RViz tracks the physical arm's live pose. Use it for the Stage 2 visual SIGN
check: jog the arm on the pendant and confirm the model moves the same way.

    ros2 launch brtirus0707a_description view_real.launch.py

There is NO joint_state_publisher(_gui) here on purpose -- only the driver may
publish /joint_states, otherwise two publishers fight over the topic.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    model_path = PathJoinSubstitution(
        [FindPackageShare("brtirus0707a_description"), "urdf", "brtirus0707a.urdf.xacro"]
    )
    default_rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("brtirus0707a_description"), "launch", "brtirus0707a.rviz"]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", model_path]), value_type=str
        )
    }

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value=[
                    "/opt/ros/humble/opt/rviz_ogre_vendor/lib:",
                    EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
                ],
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config_path),
            # Model -> TF
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            # REAL arm telemetry -> /joint_states (calibrated sign/offset applied)
            Node(
                package="borunte0707a_driver",
                executable="joint_state_publisher",
                name="borunte0707a_joint_state_publisher",
                parameters=[{"publish_rate_hz": 20.0}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
