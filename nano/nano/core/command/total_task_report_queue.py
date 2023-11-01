#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""
from collections import OrderedDict
from threading import RLock

from ..common.logger import Logger


class ReportItem:
    def __init__(self, report: any, max_count: int, platform: str):
        self.report = report
        self.max_count = max_count
        self.counter = 0
        self.platform = platform


class ReportOrderedDict():

    def __init__(self, logger: Logger):
        self.logger = logger
        self.cache = OrderedDict()
        self.lock = RLock()

    def add(self, key: str, max_count: int, platform: str, value: any):
        with self.lock:
            if key not in self.cache:
                self.cache[key] = ReportItem(value, max_count, platform)

    def remove(self, key: str):
        self._remove(key=key, trigger='ack')

    def _remove(self, key: str, trigger: str):
        with self.lock:
            if key in self.cache:
                self.logger.sys_log.info(f'{trigger} remove-> {key}')
                del self.cache[key]

    def get(self, key: str):
        with self.lock:
            if key in self.cache:
                return self.cache[key]

    def process(self, callback):
        with self.lock:
            items = list(self.cache.items())
            for key, obj in items:
                if obj.counter >= obj.max_count:
                    self._remove(key=key, trigger='max')
                else:
                    obj.counter += 1
                    callback(obj.report, obj.platform)
