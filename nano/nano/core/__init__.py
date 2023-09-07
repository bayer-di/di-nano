#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

from .caches.global_cache import nano_setting_cache
from .logs import sys_log, init_log
from .mqtt import init_mqtt


def create_app():
    conf = nano_setting_cache['obj']

    # 初始化日志
    init_log(conf=conf)
    
    init_mqtt(conf=conf)

    print(f"Startup with {conf.environment}!")
    sys_log.info(f"Startup with {conf.environment}!")

