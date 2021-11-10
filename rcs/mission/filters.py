import django
import django_filters
from .models import MissionModel


class MissionFilter(django_filters.FilterSet):
    createTimeLte = django_filters.DateFilter(field_name='createTime', lookup_expr='lte')
    createTimeGte = django_filters.DateFilter(field_name='createTime', lookup_expr='gte')

    class Meta:
        model = MissionModel
        fields = {
            'name': ['exact'],
            'state': ['exact'],
            'createTime': ['exact']
        }