from rest_framework.routers import DefaultRouter
from django.urls import path
from .views import SystemSettingViewSet


system_router = DefaultRouter(trailing_slash=False)

system_router.register(prefix='system_settings', viewset=SystemSettingViewSet)

urlpatterns = [
]

urlpatterns += system_router.urls
