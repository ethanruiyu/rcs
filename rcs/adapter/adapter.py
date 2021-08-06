from rcs.core.models import VehicleModel
import paho.mqtt.client as mqtt
from paho.mqtt import MQTTException
import logging

scan_vehicles = []


class MasterMqttAdapter:
    def __init__(self):
        self._client = mqtt.Client(client_id='master', clean_session=True)
        self._enabled = False
        self._logger = logging.getLogger()

    def enable(self):
        try:
            self._client.connect('localhost', 1883)
            self._client.reconnect_delay_set(1, 10)
            self._client.loop_start()
            self._client.subscribe('/vehicle/#')
            self._client.message_callback_add('/vehicle/+/heartbeat', self.on_heartbeat)
        except MQTTException as e:
            self._logger.error(e.__context__)

        self._enabled = True

    def disable(self):
        try:
            self._client.loop_stop()
        except MQTTException as e:
            self._logger.error(e.__context__)

        self._enabled = False

    def on_heartbeat(self, client, obj, msg):
        vehicle_name: str = msg.topic.split('/')[2]
        if VehicleModel.objects.filter(name=vehicle_name).exists():
            obj = VehicleModel.objects.get(name=vehicle_name)
            obj.state = 1
            obj.save()
        else:
            if vehicle_name not in scan_vehicles:
                scan_vehicles.append(vehicle_name)