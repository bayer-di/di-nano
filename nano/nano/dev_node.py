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

from core.caches.global_cache import nano_node_cache, nano_setting_cache
from core.utils import load_params_yaml
from core import create_app
from core.config import get_configs

class NanoServer:

    def __init__(self):
        super().__init__()

        self.conf_file = os.path.join(os.getcwd(), 'nano/params/nano.yaml')

        self.conf_data = load_params_yaml(self.conf_file)
        
        self.conf = get_configs(f=self.conf_file)   
        nano_setting_cache['obj'] = self.conf
        print(f"配置文件路径: {self.conf_file}")
        print(f"启动参数: {json.dumps(self.conf_data, indent=4)}")


def main(args=None):
    nano_node = NanoServer()
    nano_node_cache['node'] = nano_node

    create_app()


if __name__ == '__main__':
    main()
