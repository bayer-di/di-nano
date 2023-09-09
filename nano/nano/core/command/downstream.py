#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""
import json

from std_msgs.msg import String
from paho.mqtt.client import Client

from ..common.config import Settings
from ..common.logger import Logger
from ..schemas.message import CmdType

from .convert import Converter
from .file_processor import FileProcess
from .total_task_report_queue import ReportOrderedDict


class DownStream():
    def __init__(self, settings: Settings, logger: Logger, cache_publishers: dict, report_dict: ReportOrderedDict):
        self.settings = settings
        self.logger = logger
        self.converter = Converter(settings=self.settings)
        self.file_processor = FileProcess(settings=self.settings, logger=self.logger)
        self.cache_publishers = cache_publishers
        self.report_dict = report_dict

    async def cmd_process(self, client: Client, client_id: str, topic: str, msg: str):
        # 云端 MQTT 下发指令处理 || 本地 MQTT 下发指令处理
        cmd_type, ros_topic, need_ack, data, _ = self.converter.convert_to_ros_pack(mqtt_topic=topic, data=msg)
        json_data = json.loads(data)
        if need_ack:
            self._cmd_ack(client=client, json_data=json_data)
        self.logger.sys_log.info(f"MQTT => ROS : {topic} => {ros_topic}, 消息内容: {data}")
        # nano_node_cache['node']
        self._mqtt_to_ros2(ros_topic=ros_topic, msg=json_data, cmd_type=cmd_type)


    def _cmd_ack(self, client: Client, json_data: dict):
        if 'trace_id' in json_data:
            cmd_ack_msg, _ = self.converter.convert_to_mqtt_pack(ros_topic='/cmd_ack', trace_id=json_data['trace_id'], data=json.dumps(json_data))
            client.publish(topic=cmd_ack_msg.topic, payload=cmd_ack_msg.msg, qos=cmd_ack_msg.qos) # type: ignore

    def _mqtt_to_ros2(self, ros_topic: str, msg: dict, cmd_type: str):
        """TODO: 需要发布信息数据"""
        if cmd_type in [CmdType.maps_updated.value, CmdType.road_distribution.value]:
            data = msg['data']
            if data and 'url' in data:
                url = data['url']
                version = data['version'] if 'version' in data else None
                if cmd_type == CmdType.maps_updated.value:
                    self.file_processor.read_to_upload(url=url, version=version)
                else:
                    need_publish = self.file_processor.download_to_save(url=url, version=version)
                    if need_publish:
                        version_msg = String()
                        version_msg.data = str(version)
                        self.cache_publishers['/road_network'].publish(version_msg)
        
        if cmd_type in [CmdType.task_report_receipt.value]:
            # 预备从队列中 移除任务报告的发送, 因为任务报告发送完毕
            self.report_dict.remove(key=msg['trace_id'])

        if cmd_type in [CmdType.deploy_task.value, CmdType.ctrl.value, CmdType.pong.value]:
            ros_msg = String()
            ros_msg.data = json.dumps(msg['data'])
            self.cache_publishers[ros_topic].publish(ros_msg)