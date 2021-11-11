from django_filters.rest_framework import DjangoFilterBackend
from rest_framework.filters import OrderingFilter
from rest_framework.viewsets import ModelViewSet
from rest_framework import status
from rest_framework.decorators import action
from .serializers import MissionSerializer
from .models import MissionModel
from .paginations import MissionPagination
from .filters import MissionFilter
from ..common.utils.response import success_response, error_response
from ..communication.adapter import VEHICLE_ADAPTERS
from ..plugins.planner.module import move_to_position
from ..common.types import Point
from ..vehicle.models import VehicleModel
from ..common.commands import Pause, Abort

class MissionViewset(ModelViewSet):
    """Mission viewset"""
    serializer_class = MissionSerializer
    queryset = MissionModel.objects.all()
    pagination_class = MissionPagination
    filter_class = MissionFilter
    filter_backends = (DjangoFilterBackend, OrderingFilter)
    ordering_fields = ('createTime', 'id', 'state')

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
        return success_response(serializer.data, status=status.HTTP_201_CREATED)

    @action(methods=['post'], detail=False)
    def preview(self, request, *args, **kwargs):
        try:
            data = request.data
            vehicle = data['vehicle']
            vehicle_obj = VehicleModel.objects.filter(id=vehicle).first()
            raw = data['raw']
            adapter = VEHICLE_ADAPTERS.get(vehicle_obj.name)
            if adapter is None:
                return error_response(detail='vehicle not online')

            start_position = Point(
                x=adapter.get_position()[0],
                y=adapter.get_position()[1])

            # start_position  = Point()
            all_path = []
            for i in raw:
                if i['action'] == 'MOVE_TO_POSITION':
                    target_position = Point(
                        x=i['parameters']['position'][0],
                        y=i['parameters']['position'][1])
                    step_path = move_to_position(
                        start_position,
                        target_position,
                        vehicle_obj.width
                        )
                    start_position = target_position
                    all_path.append(step_path)
        except Exception as e:
            return error_response(detail=e.__str__)

        return success_response(data=all_path)

    @action(methods=['post'], detail=True)
    def pause(self, request, pk):
        value = request.data['value']
        mission = self.get_object()
        vehicle = mission.vehicle
        adapter = VEHICLE_ADAPTERS.get(vehicle.name)
        adapter.send_command(Pause(value))
        return success_response(data='')

    @action(methods=['post'], detail=True)
    def abort(self, request, pk):
        mission = self.get_object()
        vehicle = mission.vehicle
        adapter = VEHICLE_ADAPTERS.get(vehicle.name)
        adapter.send_command(Abort())
        return success_response(data='')
