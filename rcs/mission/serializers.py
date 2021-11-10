from rest_framework.serializers import ModelSerializer, DateTimeField
from .models import MissionModel


class MissionSerializer(ModelSerializer):
    """Mission serializer"""
    
    createTime = DateTimeField(format='%Y-%m-%d %H:%M:%S', read_only=True)
    beginTime = DateTimeField(format='%Y-%m-%d %H:%M:%S', read_only=True)
    finishTime = DateTimeField(format='%Y-%m-%d %H:%M:%S', read_only=True)
    class Meta:
        model = MissionModel
        fields = '__all__'
