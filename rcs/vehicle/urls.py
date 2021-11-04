from rest_framework.routers import DefaultRouter
from django.urls import path
from .views import VehicleViewSet

vehicle_router = DefaultRouter(trailing_slash=False)

vehicle_router.register(prefix='vehicles', viewset=VehicleViewSet)

urlpatterns = [
]

urlpatterns += vehicle_router.urls
