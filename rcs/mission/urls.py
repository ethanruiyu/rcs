from rest_framework.routers import DefaultRouter
from django.urls import path
from .views import MissionViewset
from .dispatcher import DISPATCHER


mission_router = DefaultRouter(trailing_slash=False)

mission_router.register(prefix='missions', viewset=MissionViewset)

urlpatterns = [
]

urlpatterns += mission_router.urls

DISPATCHER.setDaemon(True)
DISPATCHER.start()