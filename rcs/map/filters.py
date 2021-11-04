import django_filters
from .models import MapModel


class MapFilter(django_filters.FilterSet):
    active = django_filters.BooleanFilter()

    class Meta:
        model = MapModel
        fields = ['active']