#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

"""
MQTT CLIENT 全局缓存, 运行中使用, key in ['local','cloud']
"""
mqtt_client_cache = {}

"""
ROS2 Node 对象缓存, 运行中使用, key in ['nano']
"""
nano_node_cache = {}


"""
ros2 参数后，解析后缓存的 配置对象, 供给 fastapi 启动的时候使用, key in ['obj']
"""
nano_setting_cache = {}
