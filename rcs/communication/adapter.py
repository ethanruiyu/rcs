from typing import List, Any
import paho.mqtt.client as mqtt
from threading import Event
from ..common.commands import Heartbeat
from queue import Queue

"""
new access vehicle
"""
SCAN_VEHICLES = []

"""
accessed vehicle adapters
"""
VEHICLE_ADAPTERS = {}


class VehicleAdapter(mqtt.Client):
    event = Event()
    command_list = List[Any]

    def run(self):
        self.connect('192.168.1.8')
        self.subscribe('/#')
        self.message_callback_add(
            '/root/{0}/report/#'.format(self._client_id), self.on_report)
        self.message_callback_add(
            '/root/{0}/cmd/+/ack'.format(self._client_id), self.on_ack
        )
        self.loop_start()

    def sync_publish(self, command):
        if self.event.is_set():
            self.event.clear()
        self.publish(topic=command.topic.format(
            self._client_id.decode("utf-8")), payload=command.__json__())
        self.event.wait(timeout=5)

    def async_publish(self, command):
        self.publish(topic=command.topic.format(
            self._client_id.decode("utf-8")), payload=command.__json__())

    def enqueue(self, command):
        self.command_queue.put(command)

    def dequeue(self, command):
        self.command_queue.get()

    def heartbeat(self):
        self.async_publish(Heartbeat())

    def on_ack(self, client, obj, msg):
        pass

    def on_connect(self, client, obj, flags, rc):
        pass

    def on_report(self, client, obj, msg):
        """vehicle report message callback"""
        pass
