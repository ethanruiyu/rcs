from rest_framework.routers import DefaultRouter

from rcs.service.views import MapViewSet, PointViewSet, AreaViewSet, ActionViewSet, VehicleViewSet, MissionViewSet

core_router = DefaultRouter(trailing_slash=False)
core_router.register(basename='', prefix='maps', viewset=MapViewSet)
core_router.register(basename='', prefix='points', viewset=PointViewSet)
core_router.register(basename='', prefix='areas', viewset=AreaViewSet)
core_router.register(basename='', prefix='actions', viewset=ActionViewSet)
core_router.register(basename='', prefix='vehicles', viewset=VehicleViewSet)
core_router.register(basename='', prefix='missions', viewset=MissionViewSet)

urlpatterns = [
]

urlpatterns += core_router.urls
