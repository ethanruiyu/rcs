from rest_framework.routers import DefaultRouter
from django.urls import path
from .views import AreaTypeViewSet, MapViewSet, PointTypeViewSet

map_router = DefaultRouter(trailing_slash=False)

map_router.register(prefix='maps', viewset=MapViewSet)
map_router.register(prefix='point_types', viewset=PointTypeViewSet)
map_router.register(prefix='area_types', viewset=AreaTypeViewSet)

urlpatterns = [
]

urlpatterns += map_router.urls
