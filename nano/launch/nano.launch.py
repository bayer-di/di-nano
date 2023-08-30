#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    def_conf_path = f"{get_package_share_directory('nano')}/nano-amr-002.yaml"
    ld = LaunchDescription()

    condition = IfCondition(LaunchConfiguration('start'))

    nano_node = Node(
        package="nano",
        executable="nano",
        name="nano",
        output='screen',
        parameters=[
            { 'conf_file': def_conf_path}
        ],
        condition=condition
    )

    ld.add_action(nano_node)

    return ld
