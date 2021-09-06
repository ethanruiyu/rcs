from django.urls import path
from .consumers import RCSConsumer

websocket_urlpatterns = [
    path('ws/<str:name>', RCSConsumer.as_asgi(), name='ws'),
]
