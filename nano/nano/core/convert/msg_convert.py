#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""
import json
import time

from ..caches.global_cache import nano_setting_cache
from ..schemas.message import AgvUpMsg
from ..schemas.message import MessageType, CmdType
from ..schemas.mqtt_msg import MqttMsgReq
from ..schemas.nano_exception import NanoException

"""
ros to mqtt topic 映射, 定制好后不会再变化 {ros_topic: mqtt_attr}
"""
ros_2_mqtt_topic_map = {
    # 心跳
    '/ping': [MessageType.ping.value, 'ping'],

    '/cmd_ack': [MessageType.cmd_ack.value, 'cmd_ack'],

    # agv 状态 & 电量
    '/robo_status': [MessageType.robo_status.value, 'robo_status'],

    # agv 电量
    '/battery_info': [MessageType.battery_info.value, 'battery_info'],

    # agv 故障
    '/robo_faults': [MessageType.robo_faults.value, 'robo_faults'],

    # 子任务
    '/task/sub_task_report': [MessageType.sub_task_report.value, 'sub_task_report'],

    # 总任务 
    '/task/total_task_report': [MessageType.total_task_report.value, 'total_task_report'],

    # 产量检测
    '/res_yield_calculate': [MessageType.res_yield_calculate.value, 'res_yield_calculate']

}

"""
mqtt to ros topic 映射, 定制好后不会再变化 {mqtt_topic_cmd_name: ros_attr}
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
    'pong': [CmdType.pong.value, '/pong', False]
}


def _get_topic_params():
    env_prefix = 'test' if nano_setting_cache['obj'].environment != 'production' else 'prod'
    return nano_setting_cache['obj'].name, nano_setting_cache['obj'].device_no, env_prefix


def convert_to_ros_pack(mqtt_topic: str, data: any) -> any:  # type: ignore
    if mqtt_topic:
        cmd_name = mqtt_topic.split('/')[-1]
        d = mqtt_2_ros_topic_map[cmd_name]
        # cmd_type, ros_topic, need_ack, data, cmd_name
        return d[0], d[1], d[2], data, cmd_name
    raise NanoException(detail='转换ROS消息错误')


def convert_to_mqtt_pack(ros_topic: str, trace_id: str, data: any) -> MqttMsgReq:  # type: ignore
    d = ros_2_mqtt_topic_map[ros_topic]
    """/nano/{env}/up/{device_no}/{topic_name}"""
    if d:
        msg_type = d[0]
        topic = d[1]
        name, device_no, env_prefix = _get_topic_params()
        mqtt_topic = f"/{name}/{env_prefix}/up/{device_no}/{topic}"
        # TODO: 根据消息构建数据
        agv_up_msg = AgvUpMsg(trace_id=trace_id, device_no=device_no, msg_type=msg_type, data=data,
                              ts=int(time.time() * 1000))
        return MqttMsgReq(topic=mqtt_topic, msg=json.dumps(agv_up_msg.dict()))
    raise NanoException(detail='转换MQTT消息错误')
