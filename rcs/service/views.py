from rest_framework.viewsets import ModelViewSet

from .serializers import *


class MapViewSet(ModelViewSet):
    serializer_class = MapSerializer
    queryset = MapModel.objects.all()


class PointViewSet(ModelViewSet):
    serializer_class = PointSerializer
    queryset = PointModel.objects.all()


class PointTypeViewSet(ModelViewSet):
    serializer_class = PointTypeSerializer
    queryset = PointTypeModel.objects.all()


class AreaViewSet(ModelViewSet):
    serializer_class = AreaSerializer
    queryset = AreaModel.objects.all()


class BlockViewSet(ModelViewSet):
    serializer_class = BlockSerializer
    queryset = BlockModel.objects.all()


class PathViewSet(ModelViewSet):
    serializer_class = PathSerializer
    queryset = PathModel.objects.all()


class VehicleViewSet(ModelViewSet):
    serializer_class = VehicleSerializer
    queryset = VehicleModel.objects.all()


class VehicleSettingViewSet(ModelViewSet):
    serializer_class = VehicleSettingSerializer
    queryset = VehicleSettingModel.objects.all()


class GlobalSettingViewSet(ModelViewSet):
    serializer_class = GlobalSettingSerializer
    queryset = GlobalSettingModel.objects.all()
