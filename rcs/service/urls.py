from rest_framework.routers import DefaultRouter
from django.urls import path
from rcs.service.views import *

service_router = DefaultRouter(trailing_slash=False)
service_router.register(basename='', prefix='map', viewset=MapViewSet)
service_router.register(basename='', prefix='point', viewset=PointViewSet)
service_router.register(basename='', prefix='area', viewset=AreaViewSet)
service_router.register(basename='', prefix='block', viewset=BlockViewSet)
service_router.register(basename='', prefix='vehicle', viewset=VehicleViewSet)
service_router.register(basename='', prefix='vehicle-setting', viewset=VehicleSettingViewSet)
service_router.register(basename='', prefix='path', viewset=PathViewSet)
service_router.register(basename='', prefix='global-setting', viewset=GlobalSettingViewSet)


urlpatterns = [
    # path('test/multi_delete/', MapViewSet.as_view({'delete': 'multi_delete'}))
]

urlpatterns += service_router.urls
