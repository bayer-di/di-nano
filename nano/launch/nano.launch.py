#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from yaml import safe_load


def generate_launch_description():
    params_file = get_package_share_directory("nano") + '/nano.yaml'

    ld = LaunchDescription()

    condition = IfCondition(LaunchConfiguration('start'))

    with open(params_file, 'r') as f:
        params = safe_load(f)

    nano_node = Node(
        package="nano",
        executable="nano",
        name="nano",
        output='screen',
        parameters=[params],
        condition=condition
    )

    ld.add_action(nano_node)

    return ld
