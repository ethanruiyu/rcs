import paho.mqtt.client as mqtt
from .adapter import SCAN_VEHICLES, VEHICLE_ADAPTERS, VehicleAdapter
from ..vehicle.models import VehicleModel
from ..common.types import VehicleState


class Master(mqtt.Client):
    def on_connect(self, client, obj, flags, rc):
        pass

    def run(self):
        self.connect('192.168.1.8')
        self.subscribe('/#')
        self.message_callback_add('/root/+/heartbeat/ack', self.on_heartbeat)
        self.message_callback_add('/root/+/log', self.on_vehicle_log)

        self.loop_start()

    def on_heartbeat(self, client, obj, msg):
        """all vehicle heartbeat callback"""
        vehicle_name: str = msg.topic.split('/')[2]
        if VehicleModel.objects.filter(name=vehicle_name).exists():
            obj = VehicleModel.objects.get(name=vehicle_name)
            if obj.state != VehicleState.IDLE:
                obj.state = VehicleState.IDLE
                obj.save()
            if vehicle_name in VEHICLE_ADAPTERS.keys():
                if isinstance(VEHICLE_ADAPTERS[vehicle_name], VehicleAdapter) \
                        and not VEHICLE_ADAPTERS[vehicle_name].is_connected():
                    VEHICLE_ADAPTERS[vehicle_name].restart()
                else:
                    VEHICLE_ADAPTERS[vehicle_name].heartbeat()
            else:
                vehicle_adapter = VehicleAdapter(vehicle_name)
                vehicle_adapter.run()
                VEHICLE_ADAPTERS[vehicle_name] = vehicle_adapter

        # if not in database, add to SCAN_VEHICLES list
        else:
            if vehicle_name not in SCAN_VEHICLES:
                SCAN_VEHICLES.append(vehicle_name)


    def on_vehicle_log(self, client, obj, msg):
        """all vehicle log callback"""
        pass
