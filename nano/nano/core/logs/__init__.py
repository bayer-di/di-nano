#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

import os
import logging
from logging import handlers

from ..config import Settings
from ..utils import get_file_absolute_path

sys_log = logging.getLogger('sys-log')
access_log = logging.getLogger('access-log')


def get_logging_level(conf: Settings) -> int:
    """获取日志级别"""
    level = conf.log_level

    if level == 'DEBUG':
        return logging.DEBUG
    if level == 'INFO':
        return logging.INFO
    if level in ('WARNING', 'WARN'):
        return logging.WARN
    if level == 'ERROR':
        return logging.ERROR

    return logging.WARN


def init_log(conf: Settings):
    """初始化日志"""
    level = get_logging_level(conf=conf)

    file_path = get_file_absolute_path(conf.sys_log_file)
    
    os.makedirs(os.path.dirname(file_path), exist_ok=True)

    formatter = logging.Formatter(
        '%(asctime)s - '
        '[%(levelname)s] - '
        '%(process)d(%(thread)d) - '
        '%(pathname)s:%(lineno)d - '
        '%(message)s'
    )

    sys_log.setLevel(level=level)
    sys_log.handlers.clear()
    file_handler = _get_log_handler(filename=conf.sys_log_file, level=level, formatter=formatter)
    sys_log.addHandler(file_handler)

    access_log.setLevel(level=level)
    access_log.handlers.clear()
    access_handler = _get_log_handler(filename=conf.access_log_file, level=level, formatter=formatter)
    access_log.addHandler(access_handler)


def _get_log_handler(filename: str, level: int, formatter: logging.Formatter) -> handlers.TimedRotatingFileHandler:
    file_handler = handlers.TimedRotatingFileHandler(
        filename=filename,
        when='D',
        interval=1,
        backupCount=7,
        delay=True,
        encoding='utf-8'
    )
    file_handler.setLevel(level=level)
    file_handler.setFormatter(fmt=formatter)
    return file_handler
