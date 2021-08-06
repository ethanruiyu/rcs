import json

from django.shortcuts import render
from threading import Thread
from rcs.common.mqtt import MqttClient
from apscheduler.schedulers.background import BackgroundScheduler


class SimulatorPool:
    def __init__(self):
        pass


class Simulator(Thread):
    def __init__(self, name):
        super(Simulator, self).__init__()
        self.name = name
        self.scheduler = BackgroundScheduler()
        self.scheduler.add_job(self.pub_heartbeat, 'interval', seconds=1)
        self.client = MqttClient(client_name=name)

    def run(self) -> None:
        self.scheduler.start()
        self.client.setDaemon(True)
        self.client.start()

    def pub_heartbeat(self):
        self.client.publish('/vehicle/{0}/heartbeat'.format(self.name), json.dumps({'time': '111'}))
