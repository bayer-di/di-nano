#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

from enum import Enum, unique
from pydantic import BaseModel
from typing import Union

@unique
class MessageType(Enum):
    """业务消息类型,因为所有的 ROS 数据统一汇报到一个 上行 MQTT topic"""
    # 心跳
    ping = 'ping'

    # ACK 返回
    cmd_ack = 'cmd_ack'

    # agv 状态 
    robo_status = 'robo_status'

    # 电量
    battery_info = 'battery_info'

    # agv 故障
    robo_faults = 'robo_faults'

    # 总任务
    total_task_report = 'total_task_report'

    # 子任务
    sub_task_report = 'sub_task_report'

    # 结果: 产量检测
    res_yield_calculate = 'res_yield_calculate'

    # ... TODO: 新增业务类型


@unique
class CmdType(Enum):
    """业务指令消息类型"""
    # 地图更新
    maps_updated = 'maps_updated'

    # 路网更新
    road_distribution = 'road_distribution'

    # 下发任务数据
    deploy_task = 'deploy_task'

    # 控制指令
    ctrl = 'ctrl'

    # 心跳响应
    pong = 'pong'

    # 消息报告回执
    task_report_receipt = 'task_report_receipt'



class AgvUpMsg(BaseModel):
    """agv上行消息"""
    trace_id: str
    device_no: str
    msg_type: str
    ts: int
    data: object


class AgvDownMsg(BaseModel):
    """agv下行消息"""
    unique_id: str
    msg_type: str
    data: object
    # TODO: 定义消息内容


@unique
class MqttClientType(Enum):
    """MQTT客户端类型枚举"""
    LOCAL = 'local'
    CLOUD = 'cloud'


class MqttMsgReq(BaseModel):
    """消息类"""
    topic: str
    msg: str
    qos: int = 1
    client: Union[str, None] = None