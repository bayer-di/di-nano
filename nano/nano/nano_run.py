#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

"""
机器人环境使用
"""

import rclpy

from .nano_node import NanoNode


def main(args=None):
    """运行脚本"""
    rclpy.init(args=args)
    nano_node = NanoNode()
    rclpy.spin(nano_node)
    rclpy.shutdown()


if __name__ == '__main__':
    """启动脚本"""
    main()
