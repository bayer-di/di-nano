#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

"""
本地开发的时候启用
"""
import os
import json

from core.caches.global_cache import nano_node_cache
from core.utils import load_params_yaml


class NanoServer:

    def __init__(self):
        super().__init__()

        self.conf_file = os.path.join(os.getcwd(), 'nano/params/nano.yaml')

        self.conf_data = load_params_yaml(self.conf_file)
        self.port = self.conf_data['port']
        print(f"配置文件路径: {self.conf_file}")
        print(f"FastAPI端口: {self.port}")
        print(f"启动参数: {json.dumps(self.conf_data, indent=4)}")


def uvicorn_run(port: int):
    """运行方法"""
    from core import create_app
    import uvicorn
    app = create_app
    uvicorn.run(
        app=app,
        host='0.0.0.0',
        port=port,
        factory=True
    )


def main(args=None):
    nano_node = NanoServer()
    nano_node_cache['node'] = nano_node

    uvicorn_run(nano_node.port)


if __name__ == '__main__':
    main()
