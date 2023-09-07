#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

from fastapi import FastAPI
from fastapi_mqtt import MQTTConfig, FastMQTT
from gmqtt.client import Client
from inspect import iscoroutinefunction
from typing import Optional

from ..logs import sys_log


class FastMQTTClient:
    """MQTT Client"""

    def __init__(self,
                 app: FastAPI,
                 mqtt_conf: MQTTConfig,
                 client_id: str,
                 qos: int,
                 topics: Optional[list] = None,
                 invoke=None,
                 online_topic=None,
                 online_body=None,
                 ) -> None:
        self.app = app
        self.mqtt_conf = mqtt_conf
        self.client_id = client_id
        self.qos = qos
        self.topics = topics if topics is not None else ['/#']
        self.invoke = invoke
        self.online_topic = online_topic
        self.online_body = online_body

        self.mqtt = FastMQTT(config=self.mqtt_conf, client_id=self.client_id, clean_session=False)

        self.mqtt.init_app(app=self.app)

        @self.mqtt.on_connect()
        def connect(client: Client, flags: int, rc: int, properties: dict):
            """成功连结回调"""
            sys_log.debug(
                f"connect: {client._client_id}[{client.protocol_version}], {flags}, {rc}, {properties}")
            """开启订阅操作"""
            for topic in self.topics:
                self.mqtt.client.subscribe(topic, qos=self.qos)
            """如果配置了链接 topic,通知服务端进行发路网指令"""
            if self.online_topic and self.online_body:
                """配置"""
                self.publish(topic=self.online_topic, msg=self.online_body, qos=1)  # type: ignore

        @self.mqtt.on_disconnect()
        def disconnect(client: Client, packet, exc=None):
            """断连回调"""
            sys_log.debug(f"disconnect: {client._client_id}, {packet} {exc}")

        @self.mqtt.on_subscribe()
        def subscribe(client: Client, mid: int, qos: tuple, properties: dict):
            """订阅回调"""
            sys_log.debug(f"subscribe: {client._client_id}, {mid}, {qos}, {properties}")

        @self.mqtt.on_message()
        async def message(client: Client, topic: str, payload: bytes, qos: int, properties: dict):
            """消息回调"""
            body = payload.decode()
            sys_log.debug(f"message: {client._client_id}, topic: {topic}, 消息内容: {body}, {qos}, {properties}")
            if iscoroutinefunction(self.invoke):
                await self.invoke(client, client._client_id, topic, body, qos, properties)
            else:
                self.invoke(client, client._client_id, topic, body, qos, properties)

    def publish(self, topic: str, msg: str, qos: int = 0) -> None:
        """发布消息"""
        if self.mqtt.client.is_connected and self.mqtt.client._is_active:
            self.mqtt.publish(topic, msg, qos)
        else:
            sys_log.error(f'MQTT已经断链.....重连ING')
            
