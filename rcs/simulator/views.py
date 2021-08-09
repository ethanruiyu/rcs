import json
from paho.mqtt.client import Client
from django.shortcuts import render
from threading import Thread
from apscheduler.schedulers.background import BackgroundScheduler


class SimulatorPool:
    def __init__(self):
        pass


class Simulator:
    def __init__(self, name):
        self.name = name
        self._scheduler = BackgroundScheduler()
        self._scheduler.add_job(self.pub_heartbeat, 'interval', seconds=1)
        self._client = Client(name, clean_session=True)

    def enable(self):
        self._client.connect('localhost', 1883)
        self._scheduler.start()
        self._client.loop_start()

    def disable(self):
        self._client.loop_stop()

    def pub_heartbeat(self):
        self._client.publish('/vehicle/{0}/heartbeat'.format(self.name), json.dumps({'time': '111'}))
