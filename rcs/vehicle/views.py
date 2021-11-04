from rest_framework.viewsets import ModelViewSet
from rest_framework.decorators import action
from .models import VehicleModel
from .serializers import VehicleSerializer
from ..communication.adapter import SCAN_VEHICLES, VEHICLE_ADAPTERS
from ..common.utils.response import error_response, success_response
from ..common.commands import Drive, InitPosition


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
        adapter.sync_publish(command)
        return success_response(data='')

    @action(detail=True, methods=['post'])
    def drive(self, request, pk):
        try:
            vehicle = self.get_object()
            data = request.data
            linear = data['linear']
            angular = data['angular']

            adapter = VEHICLE_ADAPTERS.get(vehicle.name)
            adapter.async_publish(Drive({
                'linear': linear,
                'angular': angular
            }))
        except Exception as e:
            return error_response(data='', status=500)
        return success_response(data='', status=200)