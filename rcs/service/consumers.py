from channels.generic.websocket import AsyncWebsocketConsumer


class RCSConsumer(AsyncWebsocketConsumer):
    def __init__(self):
        super(RCSConsumer, self).__init__()
        self.group_name = ''

    async def connect(self):
        self.group_name = self.scope['url_route']['kwargs']

    async def disconnect(self, code):
        await self.channel_layer.group_discard(
            self.group_name,
            self.channel_name
        )

    def receive(self, text_data=None, bytes_data=None):
        pass
