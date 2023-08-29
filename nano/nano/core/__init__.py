#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

from fastapi import FastAPI

from .caches.global_cache import nano_setting_cache
from .exception import init_exception_handler
from .logs import sys_log, init_log
from .middleware import init_middleware
from .mqtt import init_mqtt
from .routers import init_router


def create_app() -> FastAPI:
    """
    该方法用于初始化和创建 FASTAPI
    """
    conf = nano_setting_cache['obj']

    # 初始化日志
    init_log(conf=conf)
    
    app = FastAPI(title=conf.name, version=conf.version) if conf.environment == 'production' else FastAPI(title=conf.name, version=conf.version, redoc_url=None, docs_url=None, debug=False)

    # 初始化异常处理
    init_exception_handler(app=app)

    # 初始化中间件
    init_middleware(app=app)

    # 初始化路由
    init_router(app=app, conf=conf)

    # 初始化MQTT
    init_mqtt(app=app, conf=conf)

    print(f"Startup application with {conf.environment}!")
    sys_log.info(f"Startup application with {conf.environment}!")

    return app
