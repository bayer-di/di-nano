#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""
import json

from ..caches.global_cache import nano_node_cache
from ..convert.msg_convert import convert_to_ros_pack, convert_to_mqtt_pack
from ..logs import sys_log

from paho.mqtt.client import Client


def _get_node():
    return nano_node_cache['node']


def cmd_process(client: Client, client_id: str, topic: str, msg: str):
    # 云端 MQTT 下发指令处理 || 本地 MQTT 下发指令处理
    cmd_type, ros_topic, need_ack, data, _ = convert_to_ros_pack(mqtt_topic=topic, data=msg)
    json_data = json.loads(data)
    if need_ack:
        _cmd_ack(client=client, json_data=json_data)
    sys_log.info(f"MQTT => ROS : {topic} => {ros_topic}, 消息内容: {data}")
    _get_node().mqtt_to_ros2(ros_topic=ros_topic, msg=json_data, cmd_type=cmd_type)


def _cmd_ack(client: Client, json_data: dict):
    if 'trace_id' in json_data:
        cmd_ack_msg, _ = convert_to_mqtt_pack(ros_topic='/cmd_ack', trace_id=json_data['trace_id'], data=json.dumps(json_data))
        client.publish(topic=cmd_ack_msg.topic, payload=cmd_ack_msg.msg, qos=cmd_ack_msg.qos) # type: ignore
