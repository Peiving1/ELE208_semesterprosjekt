"""band_gruppebot i Gazebo: robot, samlebånd, esker, bord og kamera (krav 5.i - 5.iii).

Bruk:
    ros2 launch band_gruppebot gazebo.launch.py                # styres fra MATLAB
    ros2 launch band_gruppebot gazebo.launch.py rviz:=true
    ros2 launch band_gruppebot gazebo.launch.py kamera:=true   # kamerabilde
    ros2 launch band_gruppebot gazebo.launch.py gui:=false     # uten vindu

Rekkefølge:
    1. Gazebo starter med worlds/samleband.sdf
    2. Roboten settes inn fra /robot_description
    3. joint_state_broadcaster og arm_controller startes
    4. celle.py starter (bånd, kamera og vakuum), og eskene settes inn på lageret.
       celle.py mater dem inn på båndet én og én
    5. Roboten står i ventestillingen til MATLAB sender en bane
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Eskene: navn, farge, x, y, z, yaw [rad] ved start (i world, dvs. z målt fra gulvet).
# Alle starter på lageret (pallen). celle.py mater dem inn på båndet én og én,
# med tilfeldig rotasjon og sideposisjon.
ESKER = [
    ("eske1", "0.80 0.55 0.25", 2.00, -3.95, 0.152, 0.0),
    ("eske2", "0.75 0.50 0.22", 2.00, -3.57, 0.152, 0.0),
    ("eske3", "0.85 0.60 0.30", 2.00, -3.19, 0.152, 0.0),
    ("eske4", "0.78 0.52 0.24", 2.00, -2.81, 0.152, 0.0),
    ("eske5", "0.82 0.58 0.28", 2.00, -2.43, 0.152, 0.0),
    ("eske6", "0.76 0.54 0.26", 2.00, -2.05, 0.152, 0.0),
]


def generate_launch_description():
    package_name = "band_gruppebot"
    package_path = get_package_share_directory(package_name)

    xacro_file = os.path.join(package_path, "urdf", "band_gruppebot.urdf.xacro")
    eske_file = os.path.join(package_path, "models", "eske.sdf.xacro")
    world_file = os.path.join(package_path, "worlds", "samleband.sdf")
    rviz_file = os.path.join(package_path, "rviz", "display.rviz")
    bridge_file = os.path.join(package_path, "config", "bro.yaml")
    # package://band_gruppebot/meshes/... slås opp herfra av Gazebo
    gazebo_resource_path = os.path.dirname(package_path)

    args = [
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Start RViz2 i tillegg til Gazebo."),
        DeclareLaunchArgument("kamera", default_value="false",
                              description="La Gazebo lage kamerabilde på /kamera/image."),
        DeclareLaunchArgument("gui", default_value="true",
                              description="Vis Gazebo-vinduet (false = bare simulering)."),
    ]

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " sim_gazebo:=true",
            " kamera_sensor:=", LaunchConfiguration("kamera"),
        ]),
        value_type=str,
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    gz_launch = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": "-r " + world_file}.items(),
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    gazebo_sim_uten_vindu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": "-r -s " + world_file}.items(),
        condition=UnlessCondition(LaunchConfiguration("gui")),
    )

    node_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_band_gruppebot",
        arguments=["-world", "samleband", "-topic", "robot_description", "-name", "band_gruppebot"],
        output="screen",
    )

    # Bro mellom Gazebo og ROS 2: klokke, eskeposisjoner, vakuum, båndfart, kamerabilde
    node_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bro",
        parameters=[{"config_file": bridge_file}],
        output="screen",
    )
    # Tjeneste for å flytte en eske (brukes når celle.py mater esker inn på båndet)
    node_bridge_set_pose = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bro_set_pose",
        arguments=["/world/samleband/set_pose@ros_gz_interfaces/srv/SetEntityPose"],
        output="screen",
    )

    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    spawn_esker = []
    for navn, farge, x, y, z, yaw in ESKER:
        spawn_esker.append(Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_" + navn,
            arguments=[
                "-world", "samleband",
                "-string", Command(["xacro ", eske_file, " navn:=", navn, ' farge:="', farge, '"']),
                "-name", navn,
                "-x", str(x), "-y", str(y), "-z", str(z), "-Y", str(yaw),
            ],
            output="screen",
        ))

    node_celle = Node(
        package=package_name,
        executable="celle.py",
        name="celle",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        args
        + [
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gazebo_resource_path),
            gazebo_sim,
            gazebo_sim_uten_vindu,
            node_bridge,
            node_bridge_set_pose,
            node_robot_state_publisher,
            node_spawn_robot,
            # Controllerne startes når roboten er på plass i Gazebo
            RegisterEventHandler(OnProcessExit(target_action=node_spawn_robot, on_exit=[spawner_jsb])),
            RegisterEventHandler(OnProcessExit(target_action=spawner_jsb, on_exit=[spawner_arm])),
            # Når armen holdes i startstillingen: start cellen (bånd, kamera, vakuum), og
            # sett inn eskene 2 s etterpå (celle.py må være i gang før eskene kommer)
            RegisterEventHandler(OnProcessExit(
                target_action=spawner_arm,
                on_exit=[node_celle, TimerAction(period=2.0, actions=spawn_esker)],
            )),
            node_rviz,
        ]
    )
