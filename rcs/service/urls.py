from rest_framework.routers import DefaultRouter
from django.urls import path
from rcs.service.views import *

service_router = DefaultRouter(trailing_slash=False)
service_router.register(basename='', prefix='map', viewset=MapViewSet)
service_router.register(basename='', prefix='point', viewset=PointViewSet)
service_router.register(basename='', prefix='area', viewset=AreaViewSet)
service_router.register(basename='', prefix='block', viewset=BlockViewSet)
service_router.register(basename='', prefix='action', viewset=ActionViewSet)
service_router.register(basename='', prefix='vehicle', viewset=VehicleViewSet)
service_router.register(basename='', prefix='vehicle-type', viewset=VehicleTypeViewSet)
service_router.register(basename='', prefix='vehicle-setting', viewset=VehicleSettingViewSet)
service_router.register(basename='', prefix='mission', viewset=MissionViewSet)
service_router.register(basename='', prefix='system-setting', viewset=SystemSettingViewSet)


urlpatterns = [
    path('system-setting/update/', SystemSettingViewSet.as_view({'put': 'settings_update'})),

    # path('vehicle/online-info/', VehicleViewSet.as_view({'get': 'online_info'}))
]

urlpatterns += service_router.urls
