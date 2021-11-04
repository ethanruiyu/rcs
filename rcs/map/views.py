import zipfile
import shutil
import cv2
import numpy as np
import yaml
from PIL import Image
from django.db import models
from rest_framework.viewsets import ModelViewSet
from django_filters.rest_framework import DjangoFilterBackend

from .serializers import MapSerializer, PointSerializer, PointTypeSerializer, AreaSerializer, AreaTypeSerializer
from .models import MapModel, PointModel, PointTypeModel, AreaModel, AreaTypeModel
from .paginations import MapPagination
from .filters import MapFilter
from ..common.utils.conversion import navigation2image
from ..common.utils.response import success_response, error_response


class MapViewSet(ModelViewSet):
    serializer_class = MapSerializer
    queryset = MapModel.objects.all()
    pagination_class = MapPagination
    filter_class = MapFilter
    filter_backends = [DjangoFilterBackend]

    def list(self, request, *args, **kwargs):
        queryset = self.filter_queryset(self.get_queryset())

        page = self.paginate_queryset(queryset)
        if page is not None:
            serializer = self.get_serializer(page, many=True)
            return self.get_paginated_response(serializer.data)

        serializer = self.get_serializer(queryset, many=True)
        return success_response(data=serializer.data)

    def create(self, request, *args, **kwargs):
        serializer = self.get_serializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        self.perform_create(serializer)
        headers = self.get_success_headers(serializer.data)
        try:
            name = request.data['name']
            zip_file = request.data['file']
            zip_obj = zipfile.ZipFile(zip_file)
            for file in zip_obj.namelist():
                zip_obj.extract(file, 'media/maps/{0}'.format(name))

            # copy to plan png image
            shutil.copyfile('media/maps/{0}/map.png'.format(name),
                            'media/maps/{0}/plan.png'.format(name))
            img_rgb = cv2.imread('media/maps/{0}/map.png'.format(name))
            img_rgb[np.where((img_rgb == [0, 0, 0]).all(axis=2))] = [
                147, 97, 94]
            img_rgb[np.where((img_rgb == [127, 127, 127]).all(axis=2))] = [
                36, 23, 21]
            img_rgb[np.where((img_rgb == [255, 255, 255]).all(axis=2))] = [
                82, 54, 49]

            # generate ui png image
            cv2.imwrite('media/maps/{0}/ui.png'.format(name), img_rgb)
            im = Image.open('media/maps/{0}/map.png'.format(name))
            width, height = im.size

            # decode yaml file
            yaml_file = open(
                'media/maps/{0}/map.yaml'.format(name), 'r', encoding='utf-8')
            content = yaml_file.read()
            yaml_file.close()
            config = yaml.load(content)
            if self.queryset.filter(name=name).exists():
                obj = self.queryset.get(name=name)
                obj.width = width
                obj.height = height
                obj.resolution = config['resolution']
                obj.originX = config['origin'][0]
                obj.originY = config['origin'][1]
                obj.save()
        except Exception as e:
            return error_response(data=e.__str__)

        return success_response(data=serializer.data, status=201, headers=headers)

    def update(self, request, *args, **kwargs):
        partial = kwargs.pop('partial', False)
        instance = self.get_object()
        serializer = self.get_serializer(
            instance, data=request.data, partial=partial)
        serializer.is_valid(raise_exception=True)
        self.perform_update(serializer)

        PointModel.objects.filter(map=instance).delete()
        AreaModel.objects.filter(map=instance).delete()

        # save shape objects
        image = cv2.imread('media/maps/{0}/map.png'.format(instance.name))
        mask = np.zeros(image.shape, dtype='uint8')
        obstacles = []
        for i in instance.raw:
            if i['shape'] == 'Circle':
                PointModel.objects.create(
                    map=instance,
                    x=i['x'],
                    y=i['y'],
                    name=i['id'],
                    rotation=i['rotation'],
                    type=PointTypeModel.objects.get(name=i['type']))

            if i['shape'] == 'Polygon':
                AreaModel.objects.create(
                    name=i['id'],
                    map=instance,
                    points=i['points'],
                    type=AreaTypeModel.objects.get(name=i['type'])
                )
                if i['type'] == 'Wall' or i['type'] == 'Forbidden':
                    points = []
                    for j in range(0, len(i['points']), 2):
                        image_point = navigation2image(
                            i['points'][j] + i['x'], i['points'][j + 1] + i['y'], instance)
                        points.append([image_point.x, image_point.y])
                    obstacles.append(points)

        arr = np.array(obstacles, dtype=np.int32)
        channel_count = image.shape[2]
        ignore_mask_color = (255,) * channel_count
        cv2.fillPoly(mask, arr, ignore_mask_color)
        final = image.copy()
        final[mask > 0] = 0
        cv2.imwrite('media/maps/{0}/plan.png'.format(instance.name), final)
        if getattr(instance, '_prefetched_objects_cache', None):
            # If 'prefetch_related' has been applied to a queryset, we need to
            # forcibly invalidate the prefetch cache on the instance.
            instance._prefetched_objects_cache = {}

        return success_response(data=serializer.data)


class PointViewSet(ModelViewSet):
    serializer_class = PointSerializer
    queryset = PointModel.objects.all()


class PointTypeViewSet(ModelViewSet):
    serializer_class = PointTypeSerializer
    queryset = PointTypeModel.objects.all()

    def list(self, request, *args, **kwargs):
        queryset = self.filter_queryset(self.get_queryset())

        page = self.paginate_queryset(queryset)
        if page is not None:
            serializer = self.get_serializer(page, many=True)
            return self.get_paginated_response(serializer.data)

        serializer = self.get_serializer(queryset, many=True)
        return success_response(data=serializer.data)


class AreaViewSet(ModelViewSet):
    serializer_class = AreaSerializer
    queryset = AreaModel.objects.all()


class AreaTypeViewSet(ModelViewSet):
    serializer_class = AreaTypeSerializer
    queryset = AreaTypeModel.objects.all()

    def list(self, request, *args, **kwargs):
        queryset = self.filter_queryset(self.get_queryset())

        page = self.paginate_queryset(queryset)
        if page is not None:
            serializer = self.get_serializer(page, many=True)
            return self.get_paginated_response(serializer.data)

        serializer = self.get_serializer(queryset, many=True)
        return success_response(data=serializer.data)
