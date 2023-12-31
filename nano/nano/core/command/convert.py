#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""
import json
import time

from ..common.config import Settings
from ..schemas.message import AgvUpMsg, MessageType, CmdType, MqttMsgReq, MqttClientType

"""
ros to mqtt topic 映射, 定制好后不会再变化 {ros_topic: [mqtt_attr, mqtt_topic, 需要多次发送的次数, 多端发送]}
"""
ros_2_mqtt_topic_map = {
    # 心跳
    '/ping': [MessageType.ping.value, 'ping', 1, MqttClientType.LOCAL.value],

    '/cmd_ack': [MessageType.cmd_ack.value, 'cmd_ack', 1, MqttClientType.LOCAL.value],

    # agv 状态 & 电量
    '/robo_status': [MessageType.robo_status.value, 'robo_status', 1, MqttClientType.LOCAL.value],

    # agv 电量
    '/battery_info': [MessageType.battery_info.value, 'battery_info', 1, MqttClientType.LOCAL.value],

    # agv 故障
    '/robo_faults': [MessageType.robo_faults.value, 'robo_faults', 1, MqttClientType.LOCAL.value],

    # 子任务
    '/task/sub_task_report': [MessageType.sub_task_report.value, 'sub_task_report', 1, MqttClientType.LOCAL.value],

    # 总任务 
    '/task/total_task_report': [MessageType.total_task_report.value, 'total_task_report', 3, MqttClientType.LOCAL.value],

    # 产量检测
    '/res_yield_calculated': [MessageType.res_yield_calculated.value, 'res_yield_calculated', 3, MqttClientType.CLOUD.value]

}

"""
mqtt to ros topic 映射, 定制好后不会再变化 {mqtt_topic_cmd_name: [ros_attr, ros_topic, 是否需要ACK]}
"""
mqtt_2_ros_topic_map = {

    # 地图更新执行
    'maps_updated': [CmdType.maps_updated.value, '/maps_updated', False],

    # 路网指令
    'road_distribution': [CmdType.road_distribution.value, '/road_distribution', False],

    # 任务数据
    'deploy_task': [CmdType.deploy_task.value, '/deploy_task', True],

    # 控制指令
    'ctrl': [CmdType.ctrl.value, '/ctrl', False],

    # 心跳响应指令
    'pong': [CmdType.pong.value, '/pong', False],

    # 任务报告回执
    'task_report_receipt': [CmdType.task_report_receipt.value, '/task_report_receipt', False]
}


class Converter():
    def __init__(self,
                 settings: Settings):
        self.settings = settings

    def _topic_params(self):
        env_prefix = 'test' if self.settings.environment != 'production' else 'prod'
        return self.settings.name, self.settings.device_no, self.settings.task_label, env_prefix

    def convert_to_ros_pack(self, mqtt_topic: str, data: any) -> any:  # type: ignore
        if mqtt_topic:
            cmd_name = mqtt_topic.split('/')[-1]
            if cmd_name in mqtt_2_ros_topic_map.keys():
                d = mqtt_2_ros_topic_map[cmd_name]
                # cmd_type, ros_topic, need_ack, data, cmd_name
                return d[0], d[1], d[2], data, cmd_name

    def convert_to_mqtt_pack(self, ros_topic: str, trace_id: str, data: any) -> any:  # type: ignore
        if ros_topic and ros_topic in ros_2_mqtt_topic_map.keys():
            d = ros_2_mqtt_topic_map[ros_topic]
            """/nano/{env}/up/{device_no}/{topic_name}"""
            if d:
                msg_type = d[0]
                topic = d[1]
                name, device_no, task_label, env_prefix = self._topic_params()
                mqtt_topic = f"/{name}/{env_prefix}/up/{device_no}/{topic}"
                # TODO: 根据消息构建数据
                agv_up_msg = AgvUpMsg(trace_id=trace_id, 
                                      device_no=device_no, 
                                      label=task_label,
                                      msg_type=msg_type, 
                                      data=data, 
                                      ts=int(time.time() * 1000))
                return MqttMsgReq(topic=mqtt_topic, msg=json.dumps(agv_up_msg.dict())), d[2], d[3]
