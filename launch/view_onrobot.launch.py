#!/usr/bin/env python3
"""
Filename: view_onrobot.launch.py
Author: Tú + Grok (adaptado)
Description:
    Launch file para visualizar y simular grippers OnRobot.
    Soporta: rg2, rg6, 2fg7, 3fg15
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Configuraciones
    description_package = LaunchConfiguration("description_package")
    onrobot_type = LaunchConfiguration("onrobot_type").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)
    ns = LaunchConfiguration("ns").perform(context)
    sim_gazebo = LaunchConfiguration("sim_gazebo").perform(context) == "true"
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context) == "true"

    # Generar robot_description con xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", "onrobot.urdf.xacro"]),
            " ",
            f"onrobot_type:={onrobot_type}",
            " ",
            f"prefix:={prefix}",
            " ",
            f"name:=onrobot",
            " ",
            f"use_fake_hardware:={str(use_fake_hardware).lower()}",
            " ",
            f"sim_gazebo:={str(sim_gazebo).lower()}",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_onrobot.rviz"]
    )

    # Nodes
    nodes = []

    # Robot State Publisher
    nodes.append(
        Node(
            namespace=ns,
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        )
    )

    # Joint State Publisher GUI
    nodes.append(
        Node(
            namespace=ns,
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        )
    )

    # RViz
    nodes.append(
        Node(
            namespace=ns,
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        )
    )

    # Gazebo (opcional)
    if sim_gazebo:
        # Start Gazebo
        nodes.append(
            Node(
                package="gazebo_ros",
                executable="gzserver",
                output="screen",
                arguments=["--verbose", "-s", "libgazebo_ros_factory.so"],
            )
        )
        nodes.append(
            Node(
                package="gazebo_ros",
                executable="gzclient",
                output="screen",
            )
        )
        
        # Spawn gripper in Gazebo
        nodes.append(
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-topic", f"/{ns}/robot_description", "-entity", f"onrobot_{onrobot_type}"],
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    declared_arguments = []

    # onrobot_type
    declared_arguments.append(
        DeclareLaunchArgument(
            "onrobot_type",
            default_value="2fg7",
            description="Tipo de gripper OnRobot.",
            choices=["rg2", "rg6", "2fg7", "3fg15"]
        )
    )

    # description_package
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="onrobot_description",
            description="Paquete con URDF/XACRO."
        )
    )

    # prefix
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefijo para nombres de joints."
        )
    )

    # namespace
    declared_arguments.append(
        DeclareLaunchArgument(
            "ns",
            default_value="onrobot",
            description="Namespace para los nodos."
        )
    )

    # sim_gazebo
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="false",
            description="Iniciar Gazebo para simulación.",
            choices=["true", "false"]
        )
    )

    # use_fake_hardware
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Usar hardware falso para testing.",
            choices=["true", "false"]
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
