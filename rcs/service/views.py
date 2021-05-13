from rest_framework.viewsets import ModelViewSet

from .serializers import *


class MapViewSet(ModelViewSet):
    serializer_class = MapSerializer
