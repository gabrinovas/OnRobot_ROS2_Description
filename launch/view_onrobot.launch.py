#!/usr/bin/env python3
"""
Filename: view_onrobot.launch.py
Author: Tú + Grok (adaptado)
Description:
    Launch file para visualizar y simular grippers OnRobot (incluyendo VGC10).
    Soporta: rg2, rg6, 2fg7, 3fg15, vgc10 (1/4 cups), vg10
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
    num_cups = LaunchConfiguration("num_cups").perform(context)

    # Ajustar onrobot_type para compatibilidad con carpetas
    mesh_type = "vg10" if onrobot_type == "vg10" else "vgc10"

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
            f"use_fake_hardware:=true",
            " ",
            f"sim_gazebo:={str(sim_gazebo).lower()}",
            " ",
            f"num_cups:={num_cups}",
            " ",
            f"mesh_type:={mesh_type}",
            " ",
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
        nodes.append(
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-topic", f"/{ns}/robot_description", "-entity", "onrobot_gripper"],
                output="screen",
            )
        )
        nodes.append(
            Node(
                package="gazebo_ros",
                executable="gzserver",
                output="screen",
                arguments=["--verbose"],
            )
        )
        nodes.append(
            Node(
                package="gazebo_ros",
                executable="gzclient",
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
            choices=["rg2", "rg6", "2fg7", "3fg15", "vgc10", "vg10"]
        )
    )

    # num_cups (solo para vgc10)
    declared_arguments.append(
        DeclareLaunchArgument(
            "num_cups",
            default_value="4",
            description="Número de ventosas para VGC10: 1 o 4.",
            choices=["1", "4"]
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
