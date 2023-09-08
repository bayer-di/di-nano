#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

import os
from yaml import safe_load


def absolute_path(path: str):
    """获取文件的绝对路径"""
    if os.path.isabs(path):
        return path
    return os.path.join(os.getcwd(), path)


def load_yaml(path: str) -> dict:
    with open(path, 'r') as f:
        return safe_load(f)


def auto_mkdir(file_path: str):
    log_dir = os.path.dirname(file_path)
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
