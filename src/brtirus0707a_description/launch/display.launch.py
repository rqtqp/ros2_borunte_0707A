from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
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
    model = LaunchConfiguration("model")
    j1_axis_x = LaunchConfiguration("j1_axis_x")
    j1_axis_y = LaunchConfiguration("j1_axis_y")
    j1_axis_z = LaunchConfiguration("j1_axis_z")
    j1_origin_x = LaunchConfiguration("j1_origin_x")
    j1_origin_y = LaunchConfiguration("j1_origin_y")
    j1_origin_z = LaunchConfiguration("j1_origin_z")
    rviz_config = LaunchConfiguration("rviz_config")
    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    default_model_path = PathJoinSubstitution(
        [
            FindPackageShare("brtirus0707a_description"),
            "urdf",
            "brtirus0707a.urdf.xacro",
        ]
    )
    default_rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("brtirus0707a_description"),
            "launch",
            "brtirus0707a.rviz",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    model,
                    " j1_origin_x:=",
                    j1_origin_x,
                    " j1_origin_y:=",
                    j1_origin_y,
                    " j1_origin_z:=",
                    j1_origin_z,
                    " j1_axis_x:=",
                    j1_axis_x,
                    " j1_axis_y:=",
                    j1_axis_y,
                    " j1_axis_z:=",
                    j1_axis_z,
                ]
            ),
            value_type=str,
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
            DeclareLaunchArgument(
                "model",
                default_value=default_model_path,
                description="Absolute path to the BRTIRUS0707A URDF/Xacro file.",
            ),
            DeclareLaunchArgument(
                "j1_origin_x",
                default_value="-0.064848",
                description="J1 joint origin X in the base link frame.",
            ),
            DeclareLaunchArgument(
                "j1_origin_y",
                default_value="0.0579775",
                description="J1 joint origin Y in the base link frame.",
            ),
            DeclareLaunchArgument(
                "j1_origin_z",
                default_value="0.055131",
                description="J1 joint origin Z in the base link frame.",
            ),
            DeclareLaunchArgument(
                "j1_axis_x",
                default_value="0",
                description="J1 joint axis X in the J1 joint frame.",
            ),
            DeclareLaunchArgument(
                "j1_axis_y",
                default_value="0",
                description="J1 joint axis Y in the J1 joint frame.",
            ),
            DeclareLaunchArgument(
                "j1_axis_z",
                default_value="1",
                description="J1 joint axis Z in the J1 joint frame.",
            ),
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="Use joint_state_publisher_gui for interactive joint sliders.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 with the robot model.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config_path,
                description="RViz2 config file.",
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
                condition=IfCondition(use_gui),
                output="screen",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=UnlessCondition(use_gui),
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
