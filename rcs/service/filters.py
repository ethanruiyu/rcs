import django_filters
from rcs.core.models import MapModel, VehicleModel


class MapFilter(django_filters.FilterSet):
    active = django_filters.BooleanFilter()

    class Meta:
        model = MapModel
        fields = ['active']


class VehicleFilter(django_filters.FilterSet):
    id = django_filters.NumberFilter()

    class Meta:
        model = VehicleModel
        fields = ['id']
