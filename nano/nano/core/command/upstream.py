#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""
import json
from rosidl_runtime_py import message_to_ordereddict
from uuid import uuid4

from .convert import Converter
from .total_task_report_queue import ReportOrderedDict
from ..common.config import Settings
from ..common.logger import Logger
from ..schemas.message import MqttClientType, MqttMsgReq


class UpStream():
    def __init__(self, settings: Settings, logger: Logger, mqtt_clients: dict, report_dict: ReportOrderedDict):
        self.settings = settings
        self.logger = logger
        self.converter = Converter(settings=self.settings)
        self.mqtt_clients = mqtt_clients
        self.report_dict = report_dict

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

    def res_yield_calculated_callback(self, msg):
        self._ros2_to_mqtt(ros_topic='/res_yield_calculated', msg=msg)

    def _ros2_to_mqtt(self, ros_topic: str, msg: any):  # type: ignore
        """ros msg 转成 Mqtt msg 转发到 mqtt server"""
        msg_dict = message_to_ordereddict(msg)
        trace_id = msg_dict['trace_id'] if 'trace_id' in msg_dict else f'{uuid4()}'
        data = json.dumps(msg_dict)
        mqtt_msg, count, platform = self.converter.convert_to_mqtt_pack(ros_topic=ros_topic, trace_id=trace_id, data=data)
        if mqtt_msg:
            self.logger.sys_log.info(f"ROS => MQTT {platform}: {ros_topic} => {mqtt_msg.topic}, 消息内容: {mqtt_msg.msg}")
            import asyncio
            asyncio.run(self.async_all_publish(msg=mqtt_msg, platform=platform))
            if count > 1:
                self.report_dict.add(key=trace_id, max_count=count, platform=platform, value=mqtt_msg)

    def retry_send_to(self, mqtt_msg: MqttMsgReq, platform:str):
        self.logger.sys_log.info(f"RETRY => MQTT {platform}: {mqtt_msg.topic}, 消息内容: {mqtt_msg.msg}")
        import asyncio
        asyncio.run(self.async_all_publish(msg=mqtt_msg, platform=platform))

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

    async def async_all_publish(self, msg: MqttMsgReq, platform: str):
        """异步开启发送"""
        if platform == MqttClientType.ALL.value:
            self._local_publish(msg=msg)
            self._cloud_publish(msg=msg)
        
        if platform == MqttClientType.CLOUD.value:
            self._cloud_publish(msg=msg)

        if platform == MqttClientType.LOCAL.value:
            self._local_publish(msg=msg)