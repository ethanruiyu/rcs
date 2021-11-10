import paho.mqtt.client as mqtt
from .adapter import SCAN_VEHICLES, VEHICLE_ADAPTERS, VehicleAdapter
from ..vehicle.models import VehicleModel
from ..common.types import VehicleState


class Master:
    def __init__(self) -> None:
        self._client = mqtt.Client(client_id='master', clean_session=True)
        self._client.reconnect_delay_set(1, 10)

    def enable(self):
        self.init_vehicle_adapter()

        self._client.connect('192.168.1.5', 1883)
        self._client.subscribe('/root/+/heartbeat/ack')
        self._client.message_callback_add('/root/+/heartbeat/ack', self.on_heartbeat)
        self._client.loop_start()

    def disable(self):
        self._client.disconnect()
        self._client.loop_stop()

    def on_heartbeat(self, client, obj, msg):
        vehicle_name: str = msg.topic.split('/')[2]
        if not VehicleModel.objects.filter(name=vehicle_name).exists():
            SCAN_VEHICLES.append(vehicle_name)

    def init_vehicle_adapter(self):
        all_vehicle = VehicleModel.objects.all()
        for vehicle in all_vehicle:
            if vehicle.name not in VEHICLE_ADAPTERS.keys():
                adapter = VehicleAdapter(vehicle.name)
                VEHICLE_ADAPTERS[vehicle.name] = adapter
                adapter.enable()
            