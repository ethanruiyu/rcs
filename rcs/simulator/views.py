import json
from paho.mqtt.client import Client
from django.shortcuts import render
from threading import Thread
from apscheduler.schedulers.background import BackgroundScheduler
import logging


class SimulatorPool:
    def __init__(self):
        pass


class Simulator:
    def __init__(self, name):
        self._name = name
        self._logger = logging.getLogger('simulator')
        self._scheduler = BackgroundScheduler()
        self._scheduler.add_job(self.pub_heartbeat, 'interval', seconds=1)
        self._client = Client(name, clean_session=True)
        self._client.on_connect = lambda client, userdata, flags, rc: self._logger.info(rc)
        self._client.reconnect_delay_set(1, 10)

    def enable(self):
        try:
            self._client.connect('localhost', 1883)
            self._scheduler.start()
            self._client.loop_start()
        except Exception as e:
            self._logger.error(e.__str__())

    def disable(self):
        try:
            self._client.loop_stop()
        except Exception as e:
            self._logger.error(e.__str__())

    def pub_heartbeat(self):
        self._client.publish('/vehicle/{0}/heartbeat'.format(self._name), json.dumps({'time': '111'}))
