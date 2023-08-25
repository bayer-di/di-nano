#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

from fastapi import APIRouter

from ..schemas.response import Response

router = APIRouter()


@router.get(path='/health', response_model_exclude_none=True)
async def healthz() -> Response:
    """健康检查"""
    return Response.success(message='your are good luck') # type: ignore
