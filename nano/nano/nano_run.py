#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

"""
机器人环境使用
"""

import rclpy
import threading
import uvicorn

from .core import create_app
from .core.caches.global_cache import nano_node_cache, nano_conf_cache, nano_setting_cache
from .nano_node import NanoNode

app = create_app


def uvicorn_run(port: int):
    """运行方法"""
    uvicorn.run(
        app=app,
        host='0.0.0.0',
        port=port,
        factory=True
    )


def main(args=None):
    """运行脚本"""
    rclpy.init(args=args)

    nano_node = NanoNode()
    nano_node_cache['node'] = nano_node
    nano_conf_cache['file'] = nano_node.conf_file
    nano_conf_cache['dict_data'] = nano_node.conf_data
    from .core.config import get_configs
    conf = get_configs()
    nano_setting_cache['obj'] = conf

    nano_node.node_start()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(nano_node,))
    spin_thread.start()
    uvicorn_run(nano_node.port)
  

    # from rclpy.executors import MultiThreadedExecutor
    # executor = MultiThreadedExecutor()
    # executor.add_node(nano_node)
    # thread = threading.Thread(target=web_server_run)
    # thread.start()
    # 在主线程中运行事件循环
    # executor.spin()
    # thread.join()

    rclpy.shutdown()


if __name__ == '__main__':
    """启动脚本"""
    main()
