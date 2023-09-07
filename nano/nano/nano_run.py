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
import signal
import threading
import uvicorn

from .core import create_app
from .core.caches.global_cache import nano_node_cache
from .nano_node import NanoNode

app = create_app





def uvicorn_run(port: int):

    try:
        """运行方法"""
        uvicorn.run(
            app=app,
            host='0.0.0.0',
            port=port,
            factory=True
        )

    except Exception as e:
        print("FastAPI启动发生异常:", str(e))



def main(args=None):
    """运行脚本"""
    rclpy.init(args=args)

    nano_node = NanoNode()
    nano_node_cache['node'] = nano_node

  
    # spin_thread = threading.Thread(target=rclpy.spin, args=(nano_node,))
    # spin_thread.start()
    # rclpy.spin(nano_node)

    import concurrent.futures
    with concurrent.futures.ThreadPoolExecutor() as executor:
        executor.submit(uvicorn_run, nano_node.port)
        executor.submit(rclpy.spin(nano_node))

    

    # my_thread = threading.Thread(target=uvicorn_run, args=(nano_node.port,), daemon=True)
    # my_thread.start()

    # rclpy.spin(nano_node)
    # uvicorn_run(nano_node.port)
    

    # my_thread.join()
    rclpy.shutdown()


if __name__ == '__main__':
    """启动脚本"""
    main()
