from paho.mqtt import client as mqtt
from apscheduler.schedulers.background import BackgroundScheduler
from ..vehicle.models import VehicleModel
from ..common.commands import Command
from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer
from ..common.types import VehicleState


class VehicleAdapter2:
    def __init__(self, vehicle_name) -> None:
        self._client = mqtt.Client(
            id='rcs-{}'.format(vehicle_name),
            clean_session=True)
        self._client.reconnect_delay_set(1, 10)

        self._mission_model_obj = None
        self._vehicle_model_obj = VehicleModel.objects.get(name=vehicle_name)

        self._channel_layer = get_channel_layer()

        self._heartbeat = BackgroundScheduler()
        self._heartbeat.add_job(self.heartbeat_job, 'interval', seconds=5)

    def enable(self):
        """ Enable Adapter
            connect mqtt broker and subscript topic
            start background scheduler job
        """
        # start mqtt client
        self._client.connect('localhost')
        self._client.loop_start()

        # start background job
        self._heartbeat.start()

    def disable(self):
        """ Disable Adapter
        """
        self._client.disconnect()
        self._client.loop_stop()

    def can_proceed(self):
        return self._vehicle.state == VehicleState.IDLE

    def send_command(self, command: Command):
        pass

    def heartbeat_job(self):
        pass
