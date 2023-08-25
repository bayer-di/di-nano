#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

from asgi_correlation_id import CorrelationIdMiddleware, middleware
from fastapi import FastAPI
from starlette.middleware.cors import CORSMiddleware
from uuid import uuid4

from .logging_middleware import LoggingMiddleware

origins = {
    "*"
}


def init_middleware(app: FastAPI):
    """初始化中间件"""
    app.add_middleware(
        CorrelationIdMiddleware,
        header_name='X-Request-ID',
        update_request_header=True,
        generator=lambda: uuid4().hex,
        validator=middleware.is_valid_uuid4,
        transformer=lambda a: a,
    )

    app.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        allow_credentials=True,
        allow_methods=['*'],
        allow_headers=['*'],
        expose_headers=['X-Request-ID']
    )

    app.add_middleware(LoggingMiddleware)
