import json
import logging

import paho.mqtt.client as mqtt
from paho.mqtt import MQTTException

from rcs.core.models import VehicleModel
from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer
from rcs.common.types import *
from threading import Timer
from rcs.common.enum import *


"""
new access vehicle
"""
SCAN_VEHICLES = []

"""
accessed vehicle adapters
"""
VEHICLE_ADAPTERS = {}


def on_offline():
    """
    disable vehicle adapter
    """
    pass


def on_online():
    """
    enable vehicle adapter
    """
    pass


def on_heartbeat(client, obj, msg):
    vehicle_name: str = msg.topic.split('/')[2]
    # if in database, update vehicle state and enable adapter
    if VehicleModel.objects.filter(name=vehicle_name).exists():
        obj = VehicleModel.objects.get(name=vehicle_name)
        obj.state = VehicleState.IDLE
        obj.save()
        if vehicle_name in VEHICLE_ADAPTERS.keys():
            if isinstance(VEHICLE_ADAPTERS[vehicle_name], VehicleAdapter) \
                    and not VEHICLE_ADAPTERS[vehicle_name].is_alive():
                VEHICLE_ADAPTERS[vehicle_name].restart()
            else:
                VEHICLE_ADAPTERS[vehicle_name].keepalive()
        else:
            vehicle_adapter = VehicleAdapter(vehicle_name)
            vehicle_adapter.enable()
            VEHICLE_ADAPTERS[vehicle_name] = vehicle_adapter

    # if not in database, add to SCAN_VEHICLES list
    else:
        if vehicle_name not in SCAN_VEHICLES:
            SCAN_VEHICLES.append(vehicle_name)


class MasterMqttAdapter:
    def __init__(self):
        self._client = mqtt.Client(client_id='master', clean_session=True)
        self._enabled = False
        self._logger = logging.getLogger()
        self.interval = 0

    def enable(self):
        try:
            self._client.connect('localhost', 1883)
            self._client.reconnect_delay_set(1, 10)
            self._client.loop_start()
            self._client.subscribe('/root/#')
            self._client.message_callback_add('/root/+/heartbeat', on_heartbeat)
        except MQTTException as e:
            self._logger.error(e.__context__)

        self._enabled = True

    def disable(self):
        try:
            self._client.loop_stop()
        except MQTTException as e:
            self._logger.error(e.__context__)

        self._enabled = False

    def is_alive(self):
        return self._client.is_connected()

    def restart(self):
        return self._client.reconnect()

    # def on_location(self, client, obj, msg):
    #     try:
    #         data = json.loads(msg.payload)
    #         pose = Pose(position=Point(data[0]), orientation=Quaternion(data[1])).nav2vis()
    #         # local_path = Path(data['local_path'])
    #
    #         if self.interval == 1:
    #             channel_layer = get_channel_layer()
    #             async_to_sync(channel_layer.group_send)('Robot-1', {
    #                 'type': 'location',
    #                 'message': pose
    #             })
    #             self.interval = 0
    #         self.interval += 1
    #     except Exception as e:
    #         pass


class VehicleAdapter:
    def __init__(self, name):
        self._client = mqtt.Client(client_id=name, clean_session=True)
        self._enabled = False
        self._logger = logging.getLogger()
        self._vehicle = VehicleModel.objects.get(name=name)
        self._vehicle_online = False
        self._alive_toggle = None

    def enable(self):
        try:
            self._client.connect('localhost', 1883)
            self._client.reconnect_delay_set(1, 10)
            self._client.loop_start()
            self._client.subscribe('/root/{0}'.format(self._vehicle.name))
        except MQTTException as e:
            self._logger.error(e.__context__)

        self._enabled = True

    def disable(self):
        try:
            self._client.loop_stop()
        except MQTTException as e:
            self._logger.error(e.__context__)

        self._enabled = False

    def is_alive(self):
        return self._client.is_connected()

    def restart(self):
        return self._client.reconnect()

    def keepalive(self):
        self._vehicle_online = True
        if self._alive_toggle:
            self._alive_toggle.cancel()
        self._alive_toggle = Timer(10, self.set_vehicle_offline)
        self._alive_toggle.setDaemon(True)
        self._alive_toggle.start()

    def set_vehicle_offline(self):
        self._vehicle_online = False

    def __str__(self):
        return self._vehicle.name
