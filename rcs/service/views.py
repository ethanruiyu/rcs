import shutil
import zipfile

import cv2
import numpy as np
import yaml
from PIL import Image
from django.shortcuts import get_object_or_404
from django_filters import rest_framework
from rest_framework import status
from rest_framework.decorators import action
from rest_framework.viewsets import ModelViewSet

from rcs.adapter.adapter import SCAN_VEHICLES
from rcs.adapter.adapter import VEHICLE_ADAPTERS
from .filters import *
from .paginations import *
from .serializers import *

DEFAULT_VEHICLE_SETTINGS = [
    {
        'key': 'Diameter',
        'value': 1,
        'description': 'vehicle max diameter'
    }
]


class MapViewSet(ModelViewSet):
    serializer_class = MapSerializer
    queryset = MapModel.objects.all()
    pagination_class = MapPagination
    filter_class = MapFilter
    filter_backends = [rest_framework.DjangoFilterBackend]

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
        shutil.copyfile('media/maps/{0}/map.png'.format(name),
                        'media/maps/{0}/plan.png'.format(name))
        im = Image.open('media/maps/{0}/map.png'.format(name))
        width, height = im.size

        yaml_file = open(
            'media/maps/{0}/map.yaml'.format(name), 'r', encoding='utf-8')
        content = yaml_file.read()
        yaml_file.close()
        config = yaml.load(content)
        # generate plan image
        if self.queryset.filter(name=name).exists():
            obj = self.queryset.get(name=name)
            obj.config = {
                'width': width,
                'height': height,
                'origin': config['origin'],
                'resolution': config['resolution']
            }
            obj.save()
        return Response(serializer.data, status=status.HTTP_201_CREATED, headers=headers)

    def update(self, request, *args, **kwargs):
        partial = kwargs.pop('partial', False)
        instance = self.get_object()
        serializer = self.get_serializer(
            instance, data=request.data, partial=partial)
        serializer.is_valid(raise_exception=True)
        self.perform_update(serializer)

        PointModel.objects.all().delete()
        AreaModel.objects.all().delete()
        BlockModel.objects.all().delete()

        image = cv2.imread('media/maps/{0}/map.png'.format(instance.name))
        mask = np.zeros(image.shape, dtype='uint8')
        for item in instance.raw:
            if item['type'] == 'Route':
                PointModel.objects.create(position={'x': item['x'], 'y': item['y']}, name=item['id'], type='Route',
                                          map=instance)
            if item['type'] == 'Charge':
                PointModel.objects.create(position={'x': item['x'], 'y': item['y']}, name=item['id'], type='Charge',
                                          map=instance)
            if item['type'] == 'Park':
                PointModel.objects.create(position={'x': item['x'], 'y': item['y']}, name=item['id'], type='Parking',
                                          map=instance)
            if item['type'] == 'Area':
                AreaModel.objects.create(
                    map=instance, name=item['id'], vertices=str(item['points']))

            if item['type'] == 'Block':
                block_masks = []
                BlockModel.objects.create(
                    map=instance, name=item['id'], vertices=str(item['points']))
                points = []
                for i in range(0, len(item['points']), 2):
                    points.append([item['points'][i] + instance.config['width'] / 2 + item['x'],
                                   item['points'][i + 1] + instance.config['height'] / 2 + item['y']])
                block_masks.append(points)

                arr = np.array(block_masks, dtype=np.int32)
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

        return Response(serializer.data)

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


# class PointTypeViewSet(ModelViewSet):
#     serializer_class = PointTypeSerializer
#     queryset = PointTypeModel.objects.all()


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
    filter_class = VehicleFilter
    filter_backends = [rest_framework.DjangoFilterBackend]

    @action(methods=['get'], detail=False)
    def scan(self, request, *args, **kwargs):
        return Response(data=SCAN_VEHICLES)

    @action(methods=['post'], detail=False)
    def register(self, request, *args, **kwargs):
        vehicle_name = request.data.get('name', None)
        if vehicle_name is not None and not VehicleModel.objects.filter(name=vehicle_name).exists():
            obj = VehicleModel.objects.create(name=vehicle_name)
            # init vehicle default settings when vehicle registering
            for i in DEFAULT_VEHICLE_SETTINGS:
                VehicleSettingModel.objects.create(key=i['key'], value=i['value'], description=i['description'],
                                                   vehicle=obj)
            SCAN_VEHICLES.remove(vehicle_name)

        return Response(data='')

    @action(methods=['get'], detail=False)
    def online_info(self, request, *args, **kwargs):
        total = VehicleModel.objects.all().count()
        offline = VehicleModel.objects.filter(state__in=[0, 4]).count()
        online = total - offline
        data = {
            'total': total,
            'offline': offline,
            'online': online
        }
        return Response(data=data, status=200)

    @action(methods=['post'], detail=False)
    def init_position(self, request, *args, **kwargs):
        try:
            data = request.data
            adapter = VEHICLE_ADAPTERS.get(data['name'])
            adapter.cmd_init_position(data['pose'])
        except Exception as e:
            return Response(data='', status=500)
        return Response(data='', status=200)

    @action(methods=['post'], detail=False)
    def drive(self, request, *args, **kwargs):
        try:
            data = request.data
            linear = data['linear']
            angular = data['angular']

            adapter = VEHICLE_ADAPTERS.get(data['name'])
            adapter.cmd_drive({
                'linear': linear,
                'angular': angular
            })
        except Exception as e:
            return Response(data='', status=500)
        return Response(data='', status=200)


class VehicleSettingViewSet(ModelViewSet):
    serializer_class = VehicleSettingSerializer
    queryset = VehicleSettingModel.objects.all()


class VehicleTypeViewSet(ModelViewSet):
    serializer_class = VehicleTypeSerializer
    queryset = VehicleTypeModel.objects.all()


class SystemSettingViewSet(ModelViewSet):
    serializer_class = SystemSettingSerializer
    queryset = SystemSettingModel.objects.all()

    def settings_update(self, request, *args, **kwargs):
        data = request.data

        for i in data.keys():
            obj = SystemSettingModel.objects.filter(key=i).first()
            obj.value = data[i]
            obj.save()

        return Response(status=200)


class ActionViewSet(ModelViewSet):
    serializer_class = ActionSerializer
    queryset = ActionModel.objects.all()


class MissionViewSet(ModelViewSet):
    """
    {
        "vehicle": "automatch" / "Robot-01",
        "name": "mission name",
        "mission": [
            {
                "action": "MOVE_TO_POSITION",
                "parameters": {
                    "position": [x, y, z],
                    "orientation": [0, 0, 0, 1]
                }
            },
            {
                "action": "TURN_ON_CUTTER",
                "parameters": {}
            },
            {
                "action": "COVERAGE",
                "parameters": {
                    "polygon": [x, y, x, y, x, y]
                }
            },
            {
                "action": "WAIT",
                "parameters": {
                    "seconds": 300
                }
            },
            {
                "action": "TURN_OFF_CUTTER",
                "parameters": {}
            },
            {
                "action": "CHARGING",
                "parameters": {}
            }
        ]
    }
    """
    serializer_class = MissionSerializer
    queryset = MissionModel.objects.all()

    def create(self, request, *args, **kwargs):
        serializer = self.get_serializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        self.perform_create(serializer)
        headers = self.get_success_headers(serializer.data)
        return Response(serializer.data, status=status.HTTP_201_CREATED, headers=headers)

    @action(methods=['post'], detail=False)
    def preview(request, *args, **kwargs):
        return Response(status=200)

    @action(methods=['post'], detail=False)
    def abort(request, *args, **kwargs):
        return Response(status=200)

    @action(methods=['post'], detail=False)
    def pause(request, *args, **kwargs):
        return Response(status=200)

    @action(methods=['post'], detail=False)
    def continued(request, *args, **kwargs):
        return Response(status=200)
