import json
import math
import random

from paho.mqtt.client import Client
from django.shortcuts import render
from threading import Thread
from apscheduler.schedulers.background import BackgroundScheduler
import logging
from rcs.common.enum import CMDEnum
import time
from rcs.common.types import euler_from_quaternion, euler_to_quaternion


class SimulatorPool:
    def __init__(self):
        pass


class Simulator:
    def __init__(self, name):
        self._name = name
        self._logger = logging.getLogger('simulator')
        self._heartbeat_scd = BackgroundScheduler()
        self._heartbeat_scd.add_job(self.heartbeat, 'interval', seconds=1)
        self._localization_scd = BackgroundScheduler()
        self._localization_scd.add_job(self.report_localization, 'interval', seconds=0.1)
        self._battery_scd = BackgroundScheduler()
        self._battery_scd.add_job(self.report_battery, 'interval', seconds=1)
        self._battery_scd.add_job(self.report_system_usage, 'interval', seconds=1)
        self._move_dt = 0.1
        self._move_scd = BackgroundScheduler()
        self._move_scd.add_job(self.update_pos, 'interval', seconds=self._move_dt)
        self._client = Client('simulator' + name, clean_session=True)
        self._client.on_connect = lambda client, userdata, flags, rc: self._logger.info(rc)
        self._client.reconnect_delay_set(1, 10)
        self._is_localization = False
        self._pose = []
        self._linear = 0
        self._angular = 0

    def enable(self):
        try:
            self._client.connect('localhost', 1883)
            self._heartbeat_scd.start()
            self._battery_scd.start()
            self._client.loop_start()
            self._client.subscribe('/root/+/cmd/#')
            self._client.message_callback_add('/root/{0}/cmd/+/set'.format(self._name), self.dispatch_cmd)
        except Exception as e:
            self._logger.error(e.__str__())

    def disable(self):
        try:
            self._client.loop_stop()
            self._heartbeat_scd.shutdown()
            self._battery_scd.shutdown()
            self._localization_scd.shutdown()
        except Exception as e:
            self._logger.error(e.__str__())

    def heartbeat(self):
        self._client.publish('/root/{0}/heartbeat'.format(self._name), json.dumps({'time': '111'}))

    def report_localization(self):
        if self._is_localization:
            self._client.publish('/root/{0}/report/navigation/localization'.format(self._name), json.dumps({
                "timestamp": "",
                "data": {
                    "pose": self._pose
                }
            }))

    def report_battery(self):
        battery_info = {
            'charged': False,
            'percentage': round(random.uniform(30, 60), 2),
            'voltage': round(random.uniform(50, 60), 2),
            'current': round(random.uniform(5, 6), 2),
            'temperature': round(random.uniform(30, 45), 2)
        }
        self._client.publish('/root/{0}/report/chassis/battery'.format(self._name), json.dumps(battery_info))

    def report_system_usage(self):
        system_usage = {
            'cpu': round(random.uniform(30, 60), 2),
            'memory': round(random.uniform(50, 60), 2),
        }
        self._client.publish('/root/{0}/report/navigation/usage'.format(self._name), json.dumps(system_usage))

    def on_init_position(self, msg):
        if self._is_localization:
            self._pose = msg['data']
            return
        self._is_localization = True
        self._pose = msg['data']
        self._localization_scd.start()

    def on_move(self, msg):
        speed = msg['data']
        self._linear = speed['linear']
        self._angular = speed['angular']
        if not self._is_localization:
            return

        if self._move_scd.state == 1:
            return

        if self._move_scd.state == 2:
            self._move_scd.resume()
            return

        if self._linear != 0 or self._angular != 0:
            self._move_scd.start()
        else:
            self._move_scd.shutdown()

    def update_pos(self):
        if self._linear == 0 and self._angular == 0:
            self._move_scd.pause()
        position = self._pose[0]
        quaternion = self._pose[1]
        _, _, yaw = euler_from_quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        # yaw += self._angular * self._move_dt
        # yaw -= 2 * math.pi * math.floor((yaw + math.pi) / (2 * math.pi))
        # position[0] += math.cos(yaw) * self._linear * self._move_dt - math.sin(yaw) * self._angular * self._move_dt
        # position[1] -= math.cos(yaw) * self._angular * self._move_dt + math.sin(yaw) * self._linear * self._move_dt

        # yaw += self._angular
        # position[0] += math.cos(yaw) * self._linear
        # position[1] += - math.sin(yaw) * self._linear

        yaw = yaw + -self._angular * self._move_dt
        # yaw -= 2 * math.pi * math.floor((yaw + math.pi) / (2 * math.pi))
        position[0] += math.cos(yaw) * self._linear * self._move_dt - math.sin(yaw) * -self._angular * self._move_dt
        position[1] -= math.cos(yaw) * -self._angular * self._move_dt + math.sin(yaw) * self._linear * self._move_dt

        q, _ = euler_to_quaternion(0, 0, yaw)
        self._pose[0] = position
        self._pose[1] = q

    def dispatch_cmd(self, client, obj, msg):
        data = json.loads(msg.payload)
        if data['messageType'] == CMDEnum.INIT_POSITION.value:
            self.on_init_position(data)

        if data['messageType'] == CMDEnum.DRIVE.value:
            self.on_move(data)
