from paho.mqtt import client as mqtt


class VehicleAdapter2:
    def __init__(self, vehicle_name) -> None:
        self._client = mqtt.Client(
            id='rcs-{}'.format(vehicle_name),
            clean_session=True)
        self._client.reconnect_delay_set(1, 10)

    def enable(self):
        """ Enable Adapter
            connect mqtt broker and subscript topic
            start background scheduler job
        """
        # start mqtt client
        self._client.connect('localhost')
        self._client.loop_start()

    def disable(self):
        """ Disable Adapter
        """
        self._client.disconnect()
        self._client.loop_stop()