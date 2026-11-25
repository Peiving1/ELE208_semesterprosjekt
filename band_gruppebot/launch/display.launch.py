"""Vis band_gruppebot i RViz2 med glidebrytere for leddene (krav 5.i, uten Gazebo).

Bruk:
    ros2 launch band_gruppebot display.launch.py

Sjekk URDF-en mot MATLAB (alle ledd i 0 skal gi [1.81, 0, 0.16], som fkine):
    ros2 run tf2_ros tf2_echo dh_base tcp_link

Tegn tf-treet (posegrafen) til rapporten:
    ros2 run tf2_tools view_frames
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("band_gruppebot")
    xacro_file = os.path.join(pkg_dir, "urdf", "band_gruppebot.urdf.xacro")
    rviz_file = os.path.join(pkg_dir, "rviz", "display.rviz")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " prefix:=", LaunchConfiguration("prefix")]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("prefix", default_value="",
                              description="Prefiks på ledd og lenker i URDF-en."),
        DeclareLaunchArgument("gui", default_value="true",
                              description="Glidebrytere for leddene (false = alle ledd i 0)."),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Start RViz2."),

        # URDF + /joint_states -> /tf
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        # Glidebrytere som publiserer /joint_states
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("gui")),
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            condition=UnlessCondition(LaunchConfiguration("gui")),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_file],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
