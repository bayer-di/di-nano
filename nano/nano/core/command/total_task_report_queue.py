#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""
from collections import OrderedDict
from threading import RLock, Thread, Timer

from ..common.logger import Logger


class ReportItem:
    def __init__(self, report: any, max_count: int):
        self.report = report
        self.max_count = max_count
        self.counter = 0



class ReportOrderedDict():
    
    def __init__(self, logger: Logger):
        self.logger = logger
        self.cache = OrderedDict()
        self.lock = RLock()

    def add(self, key:str, max_count: int, value: any):
        with self.lock:
            if key not in self.cache:
                self.cache[key] = ReportItem(value, max_count)


    def remove(self, key:str):
        self._remove(key=key, trigger='ack')

    def _remove(self, key:str, trigger: str):
        with self.lock:
            if key in self.cache:
                self.logger.sys_log.info(f'{trigger} remove-> {key}')
                del self.cache[key]


    def get(self, key:str):
        with self.lock:
            if key in self.cache:
                return self.cache[key]
    
    def process(self, callback):
        with self.lock:
            items = list(self.cache.items())
            for key, obj in items:
                self.logger.sys_log.info(str(obj.counter) + " " + key)
                if obj.counter >= obj.max_count:
                    self._remove(key=key, trigger='max')
                else:
                    obj.counter += 1
                    callback(obj.report)



# def _add(report_dict: ReportOrderedDict):

#     while True:

#         import time
#         time.sleep(3)

#         import uuid
#         k = f"{uuid.uuid4()}"
#         report_dict.add(k, k)

# def _remove(report_dict: ReportOrderedDict):
#     import time
#     time.sleep(10)

#     print("====> 手动移除 x2")
#     report_dict.remove("x2")


# def _callback(arg):
#     print(f"====> 打印 callback ={arg}")

# def _print(report_dict: ReportOrderedDict):
#     while True:
#         import time
#         time.sleep(3)
#         report_dict.process(max=10, callback=_callback)
    

# if __name__ == '__main__':
#     report_dict = ReportOrderedDict()
#     report_dict.add("x1", "x1")
#     report_dict.add("x2", "x2")
#     report_dict.add("x3", "x3")

#     thread_add = Thread(target=_add, args={report_dict,})
#     thread_remove = Thread(target=_remove, args={report_dict,})
    
#     thread_add.start()
#     thread_remove.start()





#     thread_print = Thread(target=_print, args={report_dict,})
#     thread_print.start()

#     thread_print.join()
#     thread_add.join()
#     thread_remove.join()

