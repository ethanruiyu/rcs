import json
import logging

import paho.mqtt.client as mqtt
from paho.mqtt import MQTTException

from rcs.core.models import VehicleModel
from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer
from rcs.common.types import *
from rcs.common.commands import *
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


def on_heartbeat(client, obj, msg):
    vehicle_name: str = msg.topic.split('/')[2]
    # if in database, update vehicle state and enable adapter
    if VehicleModel.objects.filter(name=vehicle_name).exists():
        obj = VehicleModel.objects.get(name=vehicle_name)
        obj.state = VehicleState.IDLE.value
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
            self._client.subscribe('/root/{0}/report/#'.format(self._vehicle.name))
            self._client.message_callback_add('/root/{0}/report/navigation/localization'.format(self._vehicle.name),
                                              self.on_localization)
            self._client.message_callback_add('/root/{0}/report/chassis/battery'.format(self._vehicle.name),
                                              self.on_battery)
            self._client.message_callback_add('/root/{0}/report/navigation/usage'.format(self._vehicle.name),
                                              self.on_usage)
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

    def cmd_init_position(self, pose):
        data = InitPosition(pose).to_json()
        self._client.publish('/root/{0}/cmd/navigation/set'.format(self._vehicle.name), data)

    def cmd_drive(self, speed):
        data = Drive(speed).to_json()
        self._client.publish('/root/{0}/cmd/chassis/set'.format(self._vehicle.name), data)

    def on_localization(self, client, obj, msg):
        try:
            payload = json.loads(msg.payload)
            pose = Pose(position=Point(payload['data']['pose'][0]),
                        orientation=Quaternion(payload['data']['pose'][1])).nav2vis()
            # local_path = Path(data['local_path'])

            channel_layer = get_channel_layer()
            async_to_sync(channel_layer.group_send)(self._vehicle.name, {
                'type': 'localization',
                'message': pose
            })
        except Exception as e:
            pass

    def on_battery(self, client, obj, msg):
        try:
            payload = json.loads(msg.payload)
            channel_layer = get_channel_layer()
            async_to_sync(channel_layer.group_send)(self._vehicle.name, {
                'type': 'battery',
                'message': payload
            })
        except Exception as e:
            pass

    def on_usage(self, client, obj, msg):
        try:
            payload = json.loads(msg.payload)
            channel_layer = get_channel_layer()
            async_to_sync(channel_layer.group_send)(self._vehicle.name, {
                'type': 'usage',
                'message': payload
            })
        except Exception as e:
            pass

    def __str__(self):
        return self._vehicle.name
