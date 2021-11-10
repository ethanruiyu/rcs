from rest_framework import serializers
from rest_framework.viewsets import ModelViewSet
from rest_framework.decorators import action
from .models import VehicleModel
from .serializers import VehicleSerializer
from ..communication.adapter import SCAN_VEHICLES, VEHICLE_ADAPTERS
from ..common.utils.response import error_response, success_response
from ..common.commands import Drive, InitPosition, SwitchMap


class VehicleViewSet(ModelViewSet):
    serializer_class = VehicleSerializer
    queryset = VehicleModel.objects.all()

    def list(self, request, *args, **kwargs):
        queryset = self.filter_queryset(self.get_queryset())

        page = self.paginate_queryset(queryset)
        if page is not None:
            serializer = self.get_serializer(page, many=True)
            return self.get_paginated_response(serializer.data)

        serializer = self.get_serializer(queryset, many=True)
        return success_response(data=serializer.data)

    @action(detail=False, methods=['get'])
    def scan(self, request):
        return success_response(data=SCAN_VEHICLES)

    @action(detail=False, methods=['post'])
    def register(self, request):
        vehicle_name = request.data.get('name', None)
        if vehicle_name is not None and not VehicleModel.objects.filter(name=vehicle_name).exists():
            obj = VehicleModel.objects.create(name=vehicle_name)
            SCAN_VEHICLES.remove(vehicle_name)

        return success_response(data='')

    @action(detail=True, methods=['post'])
    def init_position(self, request, pk):
        position = request.data.get('pose')
        vehicle = self.get_object()
        command = InitPosition(position)
        adapter = VEHICLE_ADAPTERS.get(vehicle.name)
        if adapter is None:
            return error_response(detail='vehicle not online')
        adapter.send_command(command)
        return success_response(data='', detail='init position set success')

    @action(detail=True, methods=['post'])
    def drive(self, request, pk):
        try:
            vehicle = self.get_object()
            data = request.data
            linear = data['linear']
            angular = data['angular']

            adapter = VEHICLE_ADAPTERS.get(vehicle.name)
            adapter.send_command(Drive({
                'linear': linear,
                'angular': angular
            }))
        except Exception as e:
            return error_response(data=e.__str__, status=500)
        return success_response(data='', status=200)

    @action(detail=True, methods=['post'])
    def pause(self, request, pk):
        return success_response(data='')

    def update(self, request, *args, **kwargs):
        partial = kwargs.pop('partial', False)
        instance = self.get_object()
        serializer = self.get_serializer(instance, data=request.data, partial=partial)
        serializer.is_valid(raise_exception=True)
        self.perform_update(serializer)

        adapter = VEHICLE_ADAPTERS.get(instance.name)
        if adapter is None:
            return error_response(detail='vehicle not online')
        adapter.send_command(SwitchMap(map_name=instance.map))

        if getattr(instance, '_prefetched_objects_cache', None):
            # If 'prefetch_related' has been applied to a queryset, we need to
            # forcibly invalidate the prefetch cache on the instance.
            instance._prefetched_objects_cache = {}

        return success_response(data=serializer.data, detail='settings update success', code=1)


    def handle_exception(self, exc):
        print(exc)
        return super().handle_exception(exc)