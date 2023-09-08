#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""
import json
from uuid import uuid4
from rosidl_runtime_py import message_to_ordereddict

from ..common.config import Settings
from ..common.logger import Logger
from ..schemas.message import MqttClientType, MqttMsgReq
from .convert import Converter



class UpStream():
    def __init__(self, settings: Settings, logger: Logger, mqtt_clients: dict):
        self.settings = settings
        self.logger = logger
        self.converter = Converter(settings=self.settings) 
        self.mqtt_clients = mqtt_clients

    def ping_callback(self, msg):
        self._ros2_to_mqtt(ros_topic='/ping', msg=msg)

    def robo_status_callback(self, msg):
        self._ros2_to_mqtt(ros_topic='/robo_status', msg=msg)

    def battery_info_callback(self, msg):
        self._ros2_to_mqtt(ros_topic='/battery_info', msg=msg)

    def robo_faults_callback(self, msg):
        self._ros2_to_mqtt(ros_topic='/robo_faults', msg=msg)

    def total_task_report_callback(self, msg):
        self._ros2_to_mqtt(ros_topic='/task/total_task_report', msg=msg)

    def sub_task_report_callback(self, msg):
        self._ros2_to_mqtt(ros_topic='/task/sub_task_report', msg=msg)
        


    def _ros2_to_mqtt(self, ros_topic: str, msg: any):  # type: ignore
        """ros msg 转成 Mqtt msg 转发到 mqtt server"""
        msg_dict = message_to_ordereddict(msg)
        trace_id = msg_dict['trace_id'] if 'trace_id' in msg_dict else f'{uuid4()}'
        data = json.dumps(msg_dict)
        mqtt_msg, count = self.converter.convert_to_mqtt_pack(ros_topic=ros_topic, trace_id=trace_id, data=data)
        if mqtt_msg:
            self.logger.sys_log.info(f"ROS => MQTT : {ros_topic} => {mqtt_msg.topic}, 消息内容: {data}")
            for _ in range(count):
                import asyncio
                asyncio.run(self.async_all_publish(mqtt_msg))
                # self._local_publish(msg=mqtt_msg)
                # self._cloud_publish(msg=mqtt_msg)


    def _local_publish(self, msg: MqttMsgReq):
        """发送本地 MQTT"""
        if msg:
            msg.client = MqttClientType.LOCAL.value
            self.client_publish(msg=msg)


    def _cloud_publish(self, msg: MqttMsgReq):
        """发送云端 MQTT"""
        if msg:
            msg.client = MqttClientType.CLOUD.value
            self.client_publish(msg=msg)

    def _get_mqtt_client(self, key: str):
        """获取客户端"""
        if key and key in self.mqtt_clients:
            return self.mqtt_clients[key]
        return None  # type: ignore

    def client_publish(self, msg: MqttMsgReq):
        """根据 msg 指定的客户端发送MQTT消息"""
        if msg:
            client_type = msg.client
            cache_client = self._get_mqtt_client(client_type)  # type: ignore
            if cache_client:
                cache_client.publish(topic=msg.topic, msg=msg.msg, qos=msg.qos)
            else:
                self.logger.sys_log.debug(f"MQTT客户端类型 {client_type}, 客户端为空, 发送消息 {msg.dict()}")


    async def async_all_publish(self, msg: MqttMsgReq):
        """异步开启发送"""
        self._local_publish(msg=msg)
        self._cloud_publish(msg=msg)