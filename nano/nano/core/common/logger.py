#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""
import os
from logging import handlers, Formatter, getLogger, DEBUG, INFO, WARN, ERROR
from .config import Settings
from .utils import absolute_path

class Logger():
    
    """自定义日志组件"""
    def __init__(self, settings: Settings):
        self.level = self._level(settings=settings)
        self.formatter = Formatter(
            '%(asctime)s - '
            '[%(levelname)s] - '
            '%(process)d(%(thread)d) - '
            '%(pathname)s:%(lineno)d - '
            '%(message)s'
        )
        file_path = absolute_path(settings.sys_log_file)
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        
        self.file_handler = self._handler(filename=settings.sys_log_file)

        sys_log = getLogger('sys-log')
        sys_log.setLevel(level=self.level)
        sys_log.handlers.clear()
        sys_log.addHandler(self.file_handler)
        self.sys_log=sys_log


    # 获取日志级别
    def _level(self, settings: Settings) -> int:
    
        level = settings.log_level

        if level == 'DEBUG':
            return DEBUG
        if level == 'INFO':
            return INFO
        if level in ('WARNING', 'WARN'):
            return WARN
        if level == 'ERROR':
            return ERROR

        return WARN

    # 获取日志 handler
    def _handler(self, filename: str) -> handlers.TimedRotatingFileHandler:
        file_handler = handlers.TimedRotatingFileHandler(
            filename=filename,
            when='D',
            interval=1,
            backupCount=7,
            delay=True,
            encoding='utf-8'
        )
        file_handler.setLevel(level=self.level)
        file_handler.setFormatter(fmt=self.formatter)
        return file_handler