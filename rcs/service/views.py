import zipfile

from django.shortcuts import get_object_or_404
from rest_framework import status
from rest_framework.decorators import action
from rest_framework.viewsets import ModelViewSet

from .paginations import *
from .serializers import *
from PIL import Image


class MapViewSet(ModelViewSet):
    serializer_class = MapSerializer
    queryset = MapModel.objects.all()
    pagination_class = MapPagination

    def create(self, request, *args, **kwargs):
        serializer = self.get_serializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        self.perform_create(serializer)
        headers = self.get_success_headers(serializer.data)

        name = request.data.get('name')
        # read .zip file
        zp = request.data.get('file', None)
        if zp:
            zip_obj = zipfile.ZipFile(zp)
            for file in zip_obj.namelist():
                zip_obj.extract(file, 'media/maps/{0}'.format(name))

        im = Image.open('media/maps/{0}/map.png'.format(name))
        width, height = im.size

        if self.queryset.filter(name=name).exists():
            obj = self.queryset.get(name=name)
            obj.config = {
                'width': width,
                'height': height
            }
            obj.save()
        return Response(serializer.data, status=status.HTTP_201_CREATED, headers=headers)

    @action(methods=['delete'], detail=False)
    def multi_delete(self, request, *args, **kwargs):
        delete_id = request.query_params.get('deleteid', None)
        if not delete_id:
            return Response(status=status.HTTP_404_NOT_FOUND)

        for i in delete_id.split(','):
            get_object_or_404(MapModel, pk=int(i)).delete()
        return Response(status=status.HTTP_204_NO_CONTENT)


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
