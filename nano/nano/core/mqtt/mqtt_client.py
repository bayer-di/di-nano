#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

from paho.mqtt.client import Client, MQTTMessage
from inspect import iscoroutinefunction
from typing import Optional

from ..schemas.nano_exception import NanoException
from ..logs import sys_log


class NanoMQTTClient:
    """MQTT Client"""

    def __init__(self,
                 client_id: str,
                 qos: int,
                 topics: list,
                 mqtt_config: dict,
                 invoke: None
                 ) -> None:

        self.client_id = client_id
        self.qos = qos
        self.topics = topics if topics is not None else ['/#']
        self.invoke = invoke
        self.online_topic = mqtt_config['online_topic']
        self.online_body = mqtt_config['online_body']

        client=Client(client_id=self.client_id, clean_session=True, reconnect_on_failure=True)
        client.username_pw_set(username=mqtt_config['username'], password=mqtt_config['password'])
        client.on_connect=self.on_connect
        client.on_disconnect=self.on_disconnect
        client.on_message=self.on_message
        client.on_subscribe=self.on_subscribe

        client.connect(
            host=mqtt_config['host'],
            port=mqtt_config['port'],
            keepalive=mqtt_config['keepalived'])
    
        
        self.mqtt = client

        self.mqtt.loop_start()

    def on_message(self, client: Client, userdata, msg: MQTTMessage):
        
        """消息回调"""
        body = msg.payload.decode('utf-8')
        sys_log.debug(f"message: {client._client_id}, topic: {msg.topic}, 消息内容: {body}")
        if iscoroutinefunction(self.invoke):
            self.invoke(client, client._client_id, msg.topic, body)
        else:
            self.invoke(client, client._client_id, msg.topic, body)

    def on_subscribe(self, client: Client, userdata, mid, granted_qos):
        """订阅回调"""
        sys_log.debug(f"subscribe: {client._client_id}, {mid}")
    
    def on_disconnect(self, client: Client, userdata, rc):
        if rc != 0:
            sys_log.debug("意外断开连接.....")
        else:
            sys_log.debug("正常断开连接.....")
        
        sys_log.warn(f"disconnect{client._client_id} 返回码, {rc}.........重连中........")
        client.reconnect_delay_set(min_delay=1, max_delay=3)

    def on_connect(self, client: Client, userdata, flags, rc):
        if rc == 0:
            """成功连结回调"""
            sys_log.debug(
                f"connected: {client._client_id}[{userdata}], {flags}, {rc}")
            """开启订阅操作"""
            for topic in self.topics:
                self.mqtt.subscribe(topic, qos=self.qos)
            """如果配置了链接 topic,通知服务端进行发路网指令"""
            if self.online_topic and self.online_body:
                """配置"""
                self.publish(topic=self.online_topic, msg=self.online_body, qos=1)  # type: ignore

        else:
            sys_log.warn(f"connect failed: {client._client_id} 返回码, {rc}.........重连中........")
            client.reconnect_delay_set(min_delay=1, max_delay=3)
    

    def publish(self, topic: str, msg: str, qos: int = 0) -> None:
        """发布消息"""
        if self.mqtt.is_connected:
            self.mqtt.publish(topic, msg, qos)
        else:
            sys_log.error(f"MQTT已经断链.....重连ING")
            

    def _is_ip(self, host:str) -> bool:
        import ipaddress
        try:
            ipaddress.ip_address(host)
            return True
        except ValueError:
            return False
