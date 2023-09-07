#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

import json

from uuid import uuid4



from .mqtt_client import NanoMQTTClient
from ..caches.global_cache import mqtt_client_cache
from ..command.cmd_down import cmd_process
from ..config import Settings, MqttInfo
from ..logs import sys_log
from ..schemas.mqtt_msg import MqttClientType, MqttMsgReq


def init_mqtt(conf: Settings):
    """MQTT初始化"""
    mec = conf.mqtt
    client_id = f"{conf.device_no}"
    mqttv1 = _start_mqtt_client(mqtt_info=mec.local, conf=conf, client_id=client_id)
    if mqttv1:
        mqtt_client_cache[MqttClientType.LOCAL.value] = mqttv1
    
    mqttv2 = _start_mqtt_client(mqtt_info=mec.cloud, conf=conf, client_id=client_id)
    if mqttv2:
        mqtt_client_cache[MqttClientType.CLOUD.value] = mqttv2


def _start_mqtt_client(mqtt_info: MqttInfo, conf: Settings, client_id: str):
    """MQTT初始化"""
    device_no = conf.device_no
    topic_prefix = conf.name
    task_label = conf.task_label
    env_prefix = 'test' if conf.environment != 'production' else 'prod'

    """/nano/cmd/{device_no}/#"""
    cmd_mqtt_topic = f"/{topic_prefix}/{env_prefix}/cmd/{device_no}/#"
    if mqtt_info and mqtt_info.host and mqtt_info.port:
        print(f"开启 MQTT 链接, {mqtt_info.host}:{mqtt_info.port}, client_id: {client_id}, topic: {cmd_mqtt_topic}")
        sys_log.info(f"开启 MQTT 链接, {mqtt_info.host}:{mqtt_info.port}, client_id: {client_id}, topic: {cmd_mqtt_topic}")
        return NanoMQTTClient(
            client_id=client_id,
            qos=1,
            topics=[cmd_mqtt_topic],
            invoke=cmd_process,
            mqtt_config = {
                "host": mqtt_info.host,
                "port": int(mqtt_info.port),
                "username": mqtt_info.username,
                "password": mqtt_info.password,
                "online_topic": f"/{topic_prefix}/{env_prefix}/up/{device_no}/device_online",
                "online_body": _get_online_msg(device_no=device_no, task_label=task_label),
                "keepalived": 10
            })
    return None


def _get_online_msg(device_no: str, task_label: str) -> str:
    """获取设备上线消息"""
    return json.dumps({'client_id': device_no, 'msg_type': 'device_online', 'data': task_label})


def _get_mqtt_client(key: str) -> NanoMQTTClient:
    """获取客户端"""
    if key and key in mqtt_client_cache:
        return mqtt_client_cache[key]
    return None  # type: ignore


def _local_publish(msg: MqttMsgReq):
    """发送本地 MQTT"""
    if msg:
        msg.client = MqttClientType.LOCAL.value
        client_publish(msg=msg)


def _cloud_publish(msg: MqttMsgReq):
    """发送云端 MQTT"""
    if msg:
        msg.client = MqttClientType.CLOUD.value
        client_publish(msg=msg)


def client_publish(msg: MqttMsgReq):
    """根据 msg 指定的客户端发送MQTT消息"""
    if msg:
        client_type = msg.client
        cache_client = _get_mqtt_client(client_type)  # type: ignore
        if cache_client:
            cache_client.publish(topic=msg.topic, msg=msg.msg, qos=msg.qos)
        else:
            sys_log.debug(f"MQTT客户端类型 {client_type}, 客户端为空, 发送消息 {msg.dict()}")


def all_publish(msg: MqttMsgReq):
    """本地MQTT 和 云端MQTT 都发送"""
    _local_publish(msg=msg)
    _cloud_publish(msg=msg)


async def async_all_publish(msg: MqttMsgReq):
    """异步开启发送"""
    _local_publish(msg=msg)
    _cloud_publish(msg=msg)
