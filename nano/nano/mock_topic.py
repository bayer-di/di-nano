#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

"""
Mock Ros Topic
"""

import random
import json
import time
from uuid import uuid4
from std_msgs.msg import String
from agv_msgs.msg import BatteryInfoMsg
from task_msgs.msg import Point
from device_msgs.msg import RoboStatus, ErrorStatus, ErrorMessages, ErrorMessage


class Mocker:
    def __init__(self):
        pass

    def gen_battery(self):
        battery = BatteryInfoMsg()
        battery.voltage = 10.0
        battery.current = 10.0
        battery.capacity = 10.0
        battery.capacity_remain = 10.0
        battery.temperature = 40.0
        battery.cycle_counts = 11
        battery.charging_connect = False
        battery.charging_status = False
        battery.percentage = 80
        battery.battery_error_code = '0110'
        return battery
    
    def gen_robo_status(self):
        robo_status = RoboStatus()
        robo_status.robo_status = 1
        robo_status.automatic_status = 0
        robo_status.manual_status = 0
        robo_status.fault_status = 0
        robo_status.obstacle_avoidance_status = 0
        robo_status.task_manage_status = 1
        robo_status.current_task_id = "0"
        robo_status.moving_status = 1
        robo_status.operation_status = 2
        # robo_status.current_vel = Twist()
        robo_status.current_pos = self.gen_point()
        return robo_status


    def gen_point(self):
        p_a = self.gen_points()
        idx = random.randint(0, len(p_a) - 1)
        return p_a[idx]

    def gen_points(self):
        p = []
        for a in [[39.62, 35.73], [40.62, 35.73], [39.62, 40.73], [42.62, 33.73], [33.33, 33.73]]:
            px = Point()
            px.node_id= 'G1003'
            px.x = a[0]
            px.y = a[1]
            px.yaw = random.randint(0, 360) * 1.1
            px.yaw_degree = random.randint(0, 360)
            px.description = ''
            p.append(px)
        return p


    def gen_err(self):
        err_status = ErrorStatus()
        err_status.bool_agv_error = True
        err_status.info = self.gen_err_info()
        return err_status

    def gen_ping(self):
        s = String()
        s.data = ""
        return s

    def gen_err_info(self):
        ems = ErrorMessages()
        ems.error_messages = self.gen_err_arr()
        return ems


    def gen_err_arr(self):
        error_msgs = []
        # em = self.gen_err_item()
        # error_msgs.append(em)
        return error_msgs


    def gen_err_item(self):
        em = ErrorMessage()
        em.error_code = 11001
        em.error_level = 0
        em.error_description = 'test descri'
        return em

    def consume(self, msg_data):
        # {"event_type":1,"agv_no":"AMR00001","task_id":"21f1b0ab3772d43593cc4bf2b52ae019","task_type":2,"results":[]}
        # {"event_type":0,"agv_no":"AMR00001","task_id":"21f1b0ab3772d43593cc4bf2b52ae019","task_type":2,"results":[[["G6-G5",["G6",59.97,11.5],["G5",61.1,2.13],[1,60.0],0.8,1.3,1],["G5-G4",["G5",61.1,2.13],["G4",64.17,-11.45],[1,60.0],0.8,1.3,1],["3-G4",["G4",64.17,-11.45],["3",58.67,-12.25],[0],0.8,1.3,1],["2-3",["3",58.67,-12.25],["2",27.97,-14.68],[0],0.8,1.3,1],["1-2",["2",27.97,-14.68],["1",2.59,-16.29],[0],0.8,1.3,1],["P7-1",["1",2.59,-16.29],["P7",-4.84,-13.22],[3,60.0],0.8,1.3,1]]]}
        task = json.loads(msg_data)
        event_type = task['event_type']
        task_id = task['task_id']
        task_type = task['task_type']
        mock_resp = {
            "task_id": task_id,
            "agv_no": task['agv_no'],
            "event_type": event_type,
            "task_type": task_type,
            "task_finish": 1 if event_type == 0 else 2,
            "results":[]
        }
        if event_type == 0 and task_type == 2:
            return
        
        if event_type == 0:
            time.sleep(60)
        
        s = String()
        s.data = json.dumps(mock_resp)
        return s
