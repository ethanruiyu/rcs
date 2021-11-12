import json
from os import set_blocking
import paho.mqtt.client as mqtt
import logging
from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer
from apscheduler.schedulers.background import BackgroundScheduler
from threading import Timer

from rcs.common.enum import CommandEnum
from rcs.vehicle.models import VehicleModel
from ..common.commands import Heartbeat
from ..common.types import MissionState, Vehicle
from ..common.types import VehicleState
from datetime import datetime


class VehicleAdapter:
    def __init__(self, vehicle_name: str) -> None:
        self._client = mqtt.Client(
            client_id=vehicle_name + '-adapter', clean_session=True)
        self._vehicle = VehicleModel.objects.get(name=vehicle_name)
        self._mission = None
        self._heartbeat = BackgroundScheduler()
        self._heartbeat.add_job(self.heartbeat_job, 'interval', seconds=5)
        self._vehicle_general = BackgroundScheduler()
        self._vehicle_general.add_job(self.vehicle_general_job, 'interval', seconds=1)
        self._offline_toggle = None
        self._logger = logging.getLogger('django')

    def enable(self):
        self._client.connect('192.168.1.5', 1883)
        self._client.subscribe('/root/{0}/report/#'.format(self._vehicle.name))
        self._client.subscribe(
            '/root/{0}/cmd/+/ack'.format(self._vehicle.name))
        self._client.subscribe(
            '/root/{0}/cmd/+/notify'.format(self._vehicle.name))
        self._client.subscribe(
            '/root/{0}/setting/+/ack'.format(self._vehicle.name))
        self._client.subscribe(
            '/root/{0}/heartbeat/ack'.format(self._vehicle.name))
        self._client.message_callback_add(
            '/root/{0}/heartbeat/ack'.format(self._vehicle.name),
            self.on_heartbeat)
        self._client.message_callback_add(
            '/root/{0}/cmd/+/ack'.format(self._vehicle.name),
            self.on_cmd_ack)
        self._client.message_callback_add(
            '/root/{0}/cmd/+/notify'.format(self._vehicle.name),
            self.on_cmd_notify)
        self._client.message_callback_add(
            '/root/{0}/report/navigation/general'.format(self._vehicle.name),
            self.on_report_navigation_general)
        self._client.loop_start()

        self._vehicle_general.start()
        self._heartbeat.start()

    def disable(self):
        self._heartbeat.shutdown()

        self._client.disconnect()
        self._client.loop_stop()

    def send_command(self, command):
        self._logger.info('{0}/n{1}'.format(command.topic, str(command)))
        self._client.publish(topic=command.topic.format(
            self._vehicle.name), payload=str(command))

    def heartbeat_job(self):
        message = Heartbeat()
        self._client.publish(message.topic.format(
            self._vehicle.name), str(message))

    def on_cmd_ack(self, client, obj, msg):
        data = json.loads(msg.payload)
        if data['messageType'] == CommandEnum.MISSION.value:
            self._mission.state = MissionState.PROCESSED
            self._mission.beginTime = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            self._vehicle.state = VehicleState.BUSY
            self._vehicle.save()
            self._mission.save()


    def on_cmd_notify(self, client, obj, msg):
        data = json.loads(msg.payload)
        if data['messageType'] == CommandEnum.MISSION.value:
            if data['isFinal'] == True:
                self._vehicle.state = VehicleState.IDLE
                self._mission.state = MissionState.FINISHED
                self._mission.finishTime = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                self._mission.save()
                self._vehicle.save()
                channel_layer = get_channel_layer()
                async_to_sync(channel_layer.group_send)('master', {
                        'type': 'notification',
                        'message': 'Mission : {} complete'.format(self._mission.name)
                    })
                self._mission = None
        
        if data['messageType'] == CommandEnum.ABORT_MISSION.value:
            self.abort_mission()

    def on_heartbeat(self, client, obj, msg):
        print(msg.payload)
        if self._vehicle.state == VehicleState.OFFLINE:
            self._vehicle.state = VehicleState.IDLE
            self._vehicle.save()
        if self._offline_toggle:
            self._offline_toggle.cancel()
        self._offline_toggle = Timer(6, self.set_offline)
        self._offline_toggle.setDaemon(True)
        self._offline_toggle.start()

    def set_offline(self):
        self._vehicle.state = VehicleState.OFFLINE
        self._vehicle.save()

    def on_report_navigation_general(self, client, obj, msg):
        payload = json.loads(msg.payload)
        data = payload['data']
        self._vehicle.set_position(payload['data']['currentPosition'])
        channel_layer = get_channel_layer()
        async_to_sync(channel_layer.group_send)(self._vehicle.name, {
            'type': 'navigation',
            'message': data
        })

    def get_position(self):
        return (self._vehicle.x, self._vehicle.y)

    def can_proceed(self):
        return self._vehicle.state == VehicleState.IDLE

    def set_mission(self, mission):
        self._mission = mission

    def vehicle_general_job(self):
        print('general')
        path = []
        if self._mission:
            path = self._mission.path

        message = {
                'state': self._vehicle.state,
                'path': path
        }
        channel_layer = get_channel_layer()
        async_to_sync(channel_layer.group_send)(self._vehicle.name, {
            'type': 'general',
            'message': message
        })

    def abort_mission(self):
        if self._mission:
            self._mission.state = MissionState.ABORT
            self._mission.save()
            self._vehicle.state = VehicleState.IDLE


"""
new access vehicle
"""
SCAN_VEHICLES = []

"""
accessed vehicle adapters
"""
VEHICLE_ADAPTERS = {}
