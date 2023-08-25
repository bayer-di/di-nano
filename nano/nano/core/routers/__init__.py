#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

from fastapi import FastAPI

from ..config import Settings
from ..routers import healthz
from ..routers import msg


def init_router(app: FastAPI, conf: Settings):
    """初始化路由"""
    app.include_router(
        router=healthz.router,
        prefix=conf.API_V1,
        tags=['healthz']
    )
    app.include_router(
        router=msg.router,
        prefix=conf.API_V1,
        tags=['msg']
    )
