#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

from enum import Enum, unique
from gmqtt.mqtt.constants import SubAckReasonCode
from pydantic import BaseModel
from typing import Union


@unique
class MqttClientType(Enum):
    """MQTT客户端类型枚举"""
    LOCAL = 'local'
    CLOUD = 'cloud'


class MqttMsgReq(BaseModel):
    """消息类"""
    topic: str
    msg: str
    qos: int = SubAckReasonCode.QOS1
    client: Union[str, None] = None
