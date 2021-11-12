import json

from channels.generic.websocket import AsyncWebsocketConsumer


class MasterConsumer(AsyncWebsocketConsumer):
    def __init__(self):
        super(MasterConsumer, self).__init__()
        self.group_name = ''

    async def connect(self):
        self.group_name = 'master'
        await self.channel_layer.group_add(self.group_name, self.channel_name)
        await self.accept()

    async def disconnect(self, code):
        await self.channel_layer.group_discard(
            self.group_name,
            self.channel_name
        )

    async def receive(self, text_data=None, bytes_data=None):
        """
        Called with a decoded WebSocket frame.
        """
        pass

    async def notification(self, event):
        await self.send(text_data=event['message'])


class VehicleConsumer(AsyncWebsocketConsumer):
    def __init__(self):
        super(VehicleConsumer, self).__init__()
        self.group_name = ''

    async def connect(self):
        self.group_name = self.scope['url_route']['kwargs']['name']
        await self.channel_layer.group_add(self.group_name, self.channel_name)
        await self.accept()

    async def disconnect(self, code):
        await self.channel_layer.group_discard(
            self.group_name,
            self.channel_name
        )

    def receive(self, text_data=None, bytes_data=None):
        pass

    async def navigation(self, event):
        await self.send(text_data=json.dumps({
            'navigation': event['message']
        }))
        
    async def general(self, event):
        await self.send(text_data=json.dumps({
            'general': event['message']
        }))
