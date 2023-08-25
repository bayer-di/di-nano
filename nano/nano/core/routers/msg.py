#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

from fastapi import APIRouter

from ..mqtt import all_publish
from ..schemas.mqtt_msg import MqttMsgReq
from ..schemas.response import Response

router = APIRouter()


@router.post('/publish', response_model_exclude_none=True)
async def publish(msg: MqttMsgReq) -> Response:
    """发布消息"""
    all_publish(msg)
    return Response.success(message='发送成功') # type: ignore
