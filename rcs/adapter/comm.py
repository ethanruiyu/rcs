from rcs.common.mqtt import MqttClient
from rcs.core.models import VehicleModel
from rcs.common.base.basics import BasicVehicleCommAdapter
from threading import ThreadError


class MqttVehicleCommAdapter(BasicVehicleCommAdapter):
    def __init__(self, name):
        super(MqttVehicleCommAdapter, self).__init__()
        self._enabled = False
        self._client = MqttClient(name)

    def is_enabled(self):
        return self._enabled

    def enable(self):
        try:
            self._client.setDaemon(True)
            self._client.start()
        except ThreadError as e:
            self.logger.error(e.__context__)
        self._enabled = True

    def disable(self):
        try:
            self._client.join(5)
        except ThreadError as e:
            self.logger.error(e.__context__)
        self._enabled = False

    def restart(self):
        pass

    def enqueue(self, value):
        pass

    def dequeue(self, value):
        pass
