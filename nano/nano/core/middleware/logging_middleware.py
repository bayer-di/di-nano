#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

import time
from starlette.middleware.base import BaseHTTPMiddleware

from ..logs import access_log


class LoggingMiddleware(BaseHTTPMiddleware):
    """访问日志中间件"""

    async def dispatch(self, request, call_next):
        """记录日志"""
        start_time = time.time()
        response = await call_next(request)
        elapsed_time = time.time() - start_time
        access_log.info(
            f"{elapsed_time} - [{request.method}]{request.url} - {response.status_code}"
        )
        return response
