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
from agv_msgs.msg import BatteryInfoMsg
from ament_index_python.packages import get_package_share_directory
from device_msgs.msg import RoboStatus, ErrorStatus
from functools import partial
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from std_msgs.msg import String
from ai_msgs.msg import AiRes

from .core.command.downstream import DownStream
from .core.command.total_task_report_queue import ReportOrderedDict
from .core.command.upstream import UpStream
from .core.common.config import get_configs, MqttInfo
from .core.common.logger import Logger
from .core.common.utils import load_yaml
from .core.mqtt.mqtt_client import NanoMQTTClient
from .core.schemas.message import MqttClientType
from .mock_topic import Mocker


class NanoNode(Node):
    """Nano Node"""

    def __init__(self):
        super().__init__('NanoNode')

        # 获取默认配置的路径
        def_conf_path = f"{get_package_share_directory('nano')}/nano.yaml"
        self.declare_parameter('conf_file', def_conf_path)
        self.conf_file = self.get_parameter('conf_file').get_parameter_value().string_value

        # 解析配置文件
        self.conf_data = load_yaml(self.conf_file)
        self.conf = get_configs(f=self.conf_file)
        self.get_logger().info(f"配置文件路径: {self.conf_file}")
        self.get_logger().info(f"启动参数: {json.dumps(self.conf_data, indent=4)}")

        self.cache_publishers = {}
        self.cache_subscribers = {}
        self.mqtt_clients = {}

        self.qos = rclpy.qos.QoSProfile(depth=10)

        self.logger = Logger(settings=self.conf)

        self.report_dict = ReportOrderedDict(logger=self.logger)

        self.node_start()
        self.mqtt_start()

        # self.mock_init()

        self.logger.sys_log.info(f"Startup with {self.conf.environment}!")

    def node_start(self):
        self.ros_pub_init()
        self.ros_sub_init()
        self.report_timer_init()

    def ros_pub_init(self):
        """ros topic 的 发布缓存"""
        self.cache_publishers['/deploy_task'] = self.create_publisher(String, '/deploy_task',
                                                                      qos_profile=qos_profile_services_default)
        self.cache_publishers['/road_network'] = self.create_publisher(String, '/road_network',
                                                                       qos_profile=qos_profile_services_default)
        self.cache_publishers['/pong'] = self.create_publisher(String, '/pong', qos_profile=self.qos)

    def ros_sub_init(self):
        """ros topic 的 订阅缓存"""
        self.up_stream = UpStream(settings=self.conf, logger=self.logger, mqtt_clients=self.mqtt_clients,
                                  report_dict=self.report_dict)
        self.cache_subscribers['/ping'] = self.create_subscription(String, '/ping', self.up_stream.ping_callback,
                                                                   qos_profile=self.qos)
        self.cache_subscribers['/robo_status'] = self.create_subscription(RoboStatus, '/robo_status',
                                                                          self.up_stream.robo_status_callback,
                                                                          qos_profile=self.qos)
        self.cache_subscribers['/battery_info'] = self.create_subscription(BatteryInfoMsg, '/battery_info',
                                                                           self.up_stream.battery_info_callback,
                                                                           qos_profile=self.qos)
        self.cache_subscribers['/robo_faults'] = self.create_subscription(ErrorStatus, '/robo_faults',
                                                                          self.up_stream.robo_faults_callback,
                                                                          qos_profile=self.qos)
        self.cache_subscribers['/task/total_task_report'] = self.create_subscription(String, '/task/total_task_report',
                                                                                     self.up_stream.total_task_report_callback,
                                                                                     qos_profile=self.qos)
        self.cache_subscribers['/task/sub_task_report'] = self.create_subscription(String, '/task/sub_task_report',
                                                                                   self.up_stream.sub_task_report_callback,
                                                                                   qos_profile=self.qos)
        self.cache_subscribers['/res_yield_calculated'] = self.create_subscription(AiRes, '/res_yield_calculated',
                                                                                   self.up_stream.res_yield_calculated_callback,
                                                                                   qos_profile=self.qos)
        

    def report_timer_init(self):
        self.timer_task_report = self.create_timer(1, partial(self.scan_task_report_callback))

    def scan_task_report_callback(self):
        self.report_dict.process(callback=self.up_stream.retry_send_to)

    def mqtt_start(self):
        """MQTT初始化"""
        mec = self.conf.mqtt
        client_id = f"{self.conf.device_no}"
        mqttv1 = self._start_mqtt_client(mqtt_info=mec.local, client_id=client_id)
        if mqttv1:
            self.mqtt_clients[MqttClientType.LOCAL.value] = mqttv1

        mqttv2 = self._start_mqtt_client(mqtt_info=mec.cloud, client_id=client_id)
        if mqttv2:
            self.mqtt_clients[MqttClientType.CLOUD.value] = mqttv2

    def _start_mqtt_client(self, mqtt_info: MqttInfo, client_id: str):
        """MQTT初始化"""
        device_no = self.conf.device_no
        topic_prefix = self.conf.name
        task_label = self.conf.task_label
        env_prefix = 'test' if self.conf.environment != 'production' else 'prod'

        down_stream = DownStream(settings=self.conf, logger=self.logger, cache_publishers=self.cache_publishers,
                                 report_dict=self.report_dict)

        """/nano/cmd/{device_no}/#"""
        cmd_mqtt_topic = f"/{topic_prefix}/{env_prefix}/cmd/{device_no}/#"
        if mqtt_info and mqtt_info.host and mqtt_info.port:
            self.get_logger().info(
                f"开启 MQTT 链接, {mqtt_info.host}:{mqtt_info.port}, client_id: {client_id}, topic: {cmd_mqtt_topic}")
            self.logger.sys_log.info(
                f"开启 MQTT 链接, {mqtt_info.host}:{mqtt_info.port}, client_id: {client_id}, topic: {cmd_mqtt_topic}")
            return NanoMQTTClient(
                client_id=client_id,
                qos=1,
                topics=[cmd_mqtt_topic],
                invoke=down_stream.cmd_process,
                mqtt_config={
                    "host": mqtt_info.host,
                    "port": int(mqtt_info.port),
                    "username": mqtt_info.username,
                    "password": mqtt_info.password,
                    "online_topic": f"/{topic_prefix}/{env_prefix}/up/{device_no}/device_online",
                    "online_body": json.dumps(
                        {'client_id': device_no, 'msg_type': 'device_online', 'data': task_label, 'label': task_label}),
                    "keepalived": 10
                },
                logger=self.logger)
        return None

    def mock_init(self):
        """模拟ROS TOPIC"""
        self.mock_publishers = {}
        self.mock_publishers['/robo_status'] = self.create_publisher(RoboStatus, '/robo_status', qos_profile=self.qos)
        self.mock_publishers['/battery_info'] = self.create_publisher(BatteryInfoMsg, '/battery_info',
                                                                      qos_profile=self.qos)
        self.mock_publishers['/robo_faults'] = self.create_publisher(ErrorStatus, '/robo_faults', qos_profile=self.qos)
        self.mock_publishers['/ping'] = self.create_publisher(String, '/ping', qos_profile=self.qos)
        self.mock_publishers['/task/total_task_report'] = self.create_publisher(String, '/task/total_task_report',
                                                                                qos_profile=self.qos)

        self.timer_mocker_status = self.create_timer(1, partial(self.mocker_status_callback))
        self.timer_mocker_battery = self.create_timer(5, partial(self.mocker_battery_callback))
        self.timer_mocker_faults = self.create_timer(1, partial(self.mocker_faults_callback))
        self.timer_mocker_ping = self.create_timer(0.5, partial(self.mocker_ping_callback))

        self.mock_subscribers = {}
        self.mock_subscribers['/deploy_task'] = self.create_subscription(String, '/deploy_task',
                                                                         self.mocker_deploy_task_callback,
                                                                         qos_profile=self.qos)

    def mocker_deploy_task_callback(self, msg):
        """模拟消费"""
        resp = Mocker().consume(msg.data)
        if resp:
            self.mock_publishers['/task/total_task_report'].publish(resp)

    def mocker_status_callback(self):
        """模拟消息发送"""
        self.mock_publishers['/robo_status'].publish(Mocker().gen_robo_status())

    def mocker_battery_callback(self):
        """模拟消息发送"""
        self.mock_publishers['/battery_info'].publish(Mocker().gen_battery())

    def mocker_faults_callback(self):
        """模拟消息发送"""
        self.mock_publishers['/robo_faults'].publish(Mocker().gen_err())

    def mocker_ping_callback(self):
        """模拟消息发送"""
        self.mock_publishers['/ping'].publish(Mocker().gen_ping())
