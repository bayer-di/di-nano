#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

"""
机器人环境使用
"""

import json
import rclpy

from ament_index_python.packages import get_package_share_directory
from functools import partial
from uuid import uuid4
from rclpy.node import Node
from std_msgs.msg import String, Header
from agv_msgs.msg import BatteryInfoMsg
from device_msgs.msg import RoboStatus, ErrorStatus
from rosidl_runtime_py import message_to_ordereddict
from rclpy.qos import qos_profile_services_default

from .core.convert.msg_convert import convert_to_mqtt_pack
from .core.schemas.message import CmdType
from .core.logs import sys_log
from .core.mqtt import async_all_publish
from .core.utils import load_params_yaml
from .core.fileproc import read_to_upload, download_to_save


class NanoNode(Node):
    """Nano Node"""

    def __init__(self):
        super().__init__('NanoNode')

        # 获取默认配置的路径
        def_conf_path = f"{get_package_share_directory('nano')}/nano.yaml"
        self.declare_parameter('conf_file', def_conf_path)
        # 优先从命令行参数中获取
        self.conf_file = self.get_parameter('conf_file').get_parameter_value().string_value

        # 解析配置文件
        print(f"....加载配置文件:{self.conf_file}")
        self.conf_data = load_params_yaml(self.conf_file)

        self.port = self.conf_data['port']
        self.cache_publishers = {}
        self.cache_subscribers = {}

        self.get_logger().info(f"配置文件路径: {self.conf_file}")
        self.get_logger().info(f"FastAPI端口: {self.port}")
        self.get_logger().info(f"启动参数: {json.dumps(self.conf_data, indent=4)}")

        self.qos = rclpy.qos.QoSProfile(depth=10)

        self.node_start()

    def node_start(self):

        self.ros_pub_init()

        self.ros_sub_init()

        # self.mock_init()

    def mock_init(self):
        """模拟ROS TOPIC"""
        self.mock_publishers = {}
        self.mock_publishers['/robo_status'] = self.create_publisher(RoboStatus, '/robo_status', qos_profile=self.qos)
        self.mock_publishers['/battery_info'] = self.create_publisher(BatteryInfoMsg, '/battery_info',
                                                                      qos_profile=self.qos)
        self.mock_publishers['/robo_faults'] = self.create_publisher(ErrorStatus, '/robo_faults', qos_profile=self.qos)
        self.mock_publishers['/ping'] = self.create_publisher(String, '/ping', qos_profile=self.qos)
        self.mock_publishers['/task/total_task_report'] = self.create_publisher(String, '/task/total_task_report', qos_profile=self.qos)

        self.timer_mocker_status = self.create_timer(1, partial(self.mocker_status_callback))
        self.timer_mocker_battery = self.create_timer(5, partial(self.mocker_battery_callback))
        # self.timer_mocker_faults = self.create_timer(1, partial(self.mocker_faults_callback))
        self.timer_mocker_ping = self.create_timer(0.5, partial(self.mocker_ping_callback))

        self.mock_subscribers = {}
        self.mock_subscribers['/deploy_task'] = self.create_subscription(String, '/deploy_task', self.mocker_deploy_task_callback, qos_profile=self.qos)

    def mocker_deploy_task_callback(self, msg):
        """模拟消费"""
        from .mock_topic import Mocker
        mocker = Mocker()
        resp = mocker.consume(msg.data)
        if resp:
            self.mock_publishers['/task/total_task_report'].publish(resp)


    def mocker_status_callback(self):
        """模拟消息发送"""
        from .mock_topic import Mocker
        mocker = Mocker()
        self.mock_publishers['/robo_status'].publish(mocker.gen_robo_status())

    def mocker_battery_callback(self):
        """模拟消息发送"""
        from .mock_topic import Mocker
        mocker = Mocker()
        self.mock_publishers['/battery_info'].publish(mocker.gen_battery())

    def mocker_faults_callback(self):
        """模拟消息发送"""
        from .mock_topic import Mocker
        mocker = Mocker()
        self.mock_publishers['/robo_faults'].publish(mocker.gen_err())

    def mocker_ping_callback(self):
        """模拟消息发送"""
        from .mock_topic import Mocker
        mocker = Mocker()
        self.mock_publishers['/ping'].publish(mocker.gen_ping())

    def ros_pub_init(self):
        """ros topic 的 发布缓存"""
        # TODO: 创建发布器 
        self.cache_publishers['/deploy_task'] = self.create_publisher(String, '/deploy_task', qos_profile=qos_profile_services_default)
        self.cache_publishers['/road_network'] = self.create_publisher(String, '/road_network', qos_profile=qos_profile_services_default)
        self.cache_publishers['/pong'] = self.create_publisher(String, '/pong', qos_profile=self.qos)

    def ros_sub_init(self):
        """ros topic 的 订阅缓存"""
        # TODO: 创建订阅器
        self.cache_subscribers['/ping'] = self.create_subscription(String, '/ping', self.ping_callback,
                                                                   qos_profile=self.qos)
        self.cache_subscribers['/robo_status'] = self.create_subscription(RoboStatus, '/robo_status',
                                                                          self.robo_status_callback,
                                                                          qos_profile=self.qos)
        self.cache_subscribers['/battery_info'] = self.create_subscription(BatteryInfoMsg, '/battery_info',
                                                                           self.battery_info_callback,
                                                                           qos_profile=self.qos)
        self.cache_subscribers['/robo_faults'] = self.create_subscription(ErrorStatus, '/robo_faults',
                                                                          self.robo_faults_callback,
                                                                          qos_profile=self.qos)
        self.cache_subscribers['/task/total_task_report'] = self.create_subscription(String, 
                                                                                     '/task/total_task_report',
                                                                                     self.total_task_report_callback,
                                                                                     qos_profile=self.qos)
        self.cache_subscribers['/task/sub_task_report'] = self.create_subscription(String, 
                                                                                   '/task/sub_task_report',
                                                                                   self.sub_task_report_callback,
                                                                                   qos_profile=self.qos)

    def _ros2_to_mqtt(self, ros_topic: str, msg: any):  # type: ignore
        """ros msg 转成 Mqtt msg 转发到 mqtt server"""
        msg_dict = message_to_ordereddict(msg)
        trace_id = msg_dict['trace_id'] if 'trace_id' in msg_dict else f'{uuid4()}'
        data = json.dumps(msg_dict)
        mqtt_msg = convert_to_mqtt_pack(ros_topic=ros_topic, trace_id=trace_id, data=data)
        sys_log.info(
            f"ROS 消息触发回调, 向 MQTT 发送消息, roseTopic:{ros_topic} => mqttTopic:{mqtt_msg.topic}, 消息内容: {data}")
        import asyncio
        asyncio.run(async_all_publish(mqtt_msg))

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

    def mqtt_to_ros2(self, ros_topic: str, msg: dict, cmd_type: str):
        """TODO: 需要发布信息数据"""
        if cmd_type in [CmdType.maps_updated.value, CmdType.road_distribution.value]:
            data = msg['data']
            if data and 'url' in data:
                url = data['url']
                version = data['version'] if 'version' in data else None
                if cmd_type == CmdType.maps_updated.value:
                    read_to_upload(url=url, version=version)
                else:
                    download_to_save(url=url, version=version)
                    version_msg = String()
                    version_msg.data = version
                    self.cache_publishers['/road_network'].publish(version_msg)

        if cmd_type in [CmdType.deploy_task.value, CmdType.ctrl.value, CmdType.pong.value]:
            ros_msg = String()
            ros_msg.data = json.dumps(msg['data'])
            self.cache_publishers[ros_topic].publish(ros_msg)
