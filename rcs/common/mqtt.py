import logging
import threading

import paho.mqtt.client as mqtt
from django.conf import settings

LOG = logging.getLogger('django')


class MqttClient(threading.Thread):
    def __init__(self, client_name):
        threading.Thread.__init__(self)
        try:
            self._client_name = client_name
            self._client = mqtt.Client(client_id='rcs_{0}'.format(client_name), clean_session=True)
            self._client.on_connect = self.on_connect
            self._client.on_disconnect = self.on_disconnect

        except ConnectionRefusedError as e:
            LOG.error(e.__context__)

    def on_connect(self, client, userdata, flags, rc):
        del userdata, flags, client

        if rc != 0:
            LOG.error('MQTT server connect fail, error code with {0}'.format(rc))
        else:
            LOG.info('MQTT connected with {0}'.format(self._client_name))

    def on_disconnect(self, userdata, rc):
        pass

    def run(self) -> None:
        try:
            LOG.info('{0} mqtt thread start.'.format(self._client_name))
            self._client.connect(host='localhost', port=1883)
            self._client.loop_forever()
        except ConnectionRefusedError as e:
            LOG.error(e.__context__)

    def publish(self, topic, payload, qos=0):
        return self._client.publish(topic=topic, payload=payload, qos=qos)

    def subscribe(self, topic, qos=0):
        return self._client.subscribe(topic=topic, qos=qos)

    def disconnect(self):
        self._client.disconnect(5)
