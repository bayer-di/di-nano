#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse

from ..schemas.nano_exception import NanoException
from ..schemas.response import Response


def init_exception_handler(app: FastAPI):
    @app.exception_handler(HTTPException)
    async def http_exception_handler(request, exc):
        """http异常处理"""
        return JSONResponse(
            status_code=exc.status_code,
            content=Response.error(
                code=exec.status_code,
                message='Internal Server Error',
                detail=str(exc)
            ).dict(exclude_none=True) # type: ignore
        )

    @app.exception_handler(NanoException)
    async def nano_exception_handler(request, exc):
        """Nano异常处理"""
        return JSONResponse(
            status_code=500,
            content=Response.error(
                code=500,
                message=exc.detail,
                detail=str(exc)
            ).dict(exclude_none=True) # type: ignore
        )

    @app.exception_handler(Exception)
    async def exception_handler(request, exc):
        """普通异常处理"""
        return JSONResponse(
            status_code=500,
            content=Response.error(
                code=500,
                message='Internal Server Error',
                detail=str(exc)
            ).dict(exclude_none=True) # type: ignore
        )
