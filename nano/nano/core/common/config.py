#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

from functools import lru_cache
from pydantic import BaseSettings, Field
from typing import Optional, ClassVar
from yaml import safe_load

from .utils import absolute_path


class MqttInfo(BaseSettings):
    """mqtt broker 信息"""
    host: Optional[str] = Field(..., env='host')
    port: Optional[str] = Field(..., env='port')
    username: Optional[str] = Field(..., env='username')
    password: Optional[str] = Field(..., env='password')


class MqttSettings(BaseSettings):
    """mqtt配置类"""
    local: MqttInfo = Field(..., env='local')
    cloud: MqttInfo = Field(..., env='cloud')


class Settings(BaseSettings):
    """配置类"""
    # API 版本
    API_V1: ClassVar[str] = '/api/v1'

    name: str = Field(..., env='name')

    version: str = Field(..., env='version')

    device_no: str = Field(..., env='device_no')

    task_label: str = Field(..., env='task_label')

    environment: str = Field(..., env='environment')

    log_level: str = Field(..., env='log_level')

    sys_log_file: str = Field(..., env='sys_log_file')

    maps_path: str = Field(..., env='maps_path')

    road_path: str = Field(..., env='road_path')

    mqtt: MqttSettings = Field(..., env='mqtt')


class FactoryConfig:
    """Factory Config"""

    def __init__(self, conf_data: Optional[dict]):
        self.conf_data = conf_data

    def __call__(self):
        return Settings(**self.conf_data)  # type: ignore


@lru_cache
def get_configs(f: str):
    """加载一下环境文件"""
    with open(absolute_path(f), 'r') as file:
        m = safe_load(file)
        return FactoryConfig(conf_data=m)()
