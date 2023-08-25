#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

from pydantic import BaseModel
from typing import Union


class Response(BaseModel):
    """通用响应"""
    code: int = 200
    message: str = "ok"
    data: Union[dict, None] = None
    detail: Union[str, None] = None

    @staticmethod
    def success(message: str, data: Union[dict, None] = None):
        """成功响应"""
        return Response(message=message, data=data).dict(exclude_none=True)

    @staticmethod
    def error(code: int = -1, message: str = 'failed', detail: Union[str, None] = None):
        """错误响应"""
        return Response(code=code, message=message, detail=detail).dict(exclude_none=True)
