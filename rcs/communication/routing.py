from django.urls import path
from .consumers import MasterConsumer, VehicleConsumer

websocket_urlpatterns = [
    path('notification', MasterConsumer.as_asgi(), name='master'),
    path('vehicle/<str:name>', VehicleConsumer.as_asgi(), name='vehicle')
]
