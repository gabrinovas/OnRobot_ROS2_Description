#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Show one OnRobot gripper in RViz, with its joints driven from a small window."""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from onrobot_description.config_loader import load_config, package_file, require
from onrobot_description.grippers import gripper_names
from onrobot_description.launch_arguments import declare_arguments

PACKAGE = 'onrobot_description'


def launch_setup(context, *args, **kwargs):
    config = load_config(package_file(PACKAGE, 'config', 'onrobot_description.yaml'))
    namespace = LaunchConfiguration('ns')
    gui = LaunchConfiguration('gui')
    description_file = package_file(PACKAGE, require(config, 'view', 'description_file'))
    rviz_config = package_file(PACKAGE, require(config, 'view', 'rviz_config'))
    robot_description = {'robot_description': ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', description_file,
        ' onrobot_type:=', LaunchConfiguration('onrobot_type'),
        ' prefix:=', LaunchConfiguration('prefix'),
    ]), value_type=str)}

    def node(package, executable, **options):
        return Node(namespace=namespace, package=package, executable=executable, **options)

    return [
        node('joint_state_publisher_gui', 'joint_state_publisher_gui',
             name='joint_state_publisher_gui', output='screen', condition=IfCondition(gui)),
        node('joint_state_publisher', 'joint_state_publisher',
             name='joint_state_publisher', output='screen', condition=UnlessCondition(gui)),
        node('robot_state_publisher', 'robot_state_publisher',
             output='both', parameters=[robot_description]),
        node('rviz2', 'rviz2', name='rviz2', output='log', arguments=['-d', rviz_config]),
    ]


def generate_launch_description():
    config = load_config(package_file(PACKAGE, 'config', 'onrobot_description.yaml'))
    arguments = declare_arguments(
        config, 'view_launch_arguments', choices={'onrobot_type': gripper_names()})
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
