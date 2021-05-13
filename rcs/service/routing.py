from django.urls import path
from .consumers import RCSConsumer

websocket_urlpatterns = [
    path('ws/', RCSConsumer),
]
