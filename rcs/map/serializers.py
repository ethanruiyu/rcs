from rest_framework.serializers import ModelSerializer, CharField, DateTimeField, BooleanField

from .models import MapModel, PointModel, PointTypeModel, AreaModel, AreaTypeModel


class MapSerializer(ModelSerializer):
    """Map model serializers"""
    name = CharField(required=False)
    createTime = DateTimeField(format='%Y-%m-%d %H:%M', read_only=True)
    updateTime = DateTimeField(format='%Y-%m-%d %H:%M', read_only=True)
    active = BooleanField(required=False)
    
    class Meta:
        model = MapModel
        fields = '__all__'

class PointSerializer(ModelSerializer):
    """Point model serializers"""
    class Meta:
        model = PointModel
        fields = '__all__'


class PointTypeSerializer(ModelSerializer):
    """Point type model serializers"""
    class Meta:
        model = PointTypeModel
        fields = '__all__'



class AreaSerializer(ModelSerializer):
    """Area model serializers"""
    class Meta:
        model = AreaModel
        fields = '__all__'



class AreaTypeSerializer(ModelSerializer):
    """Area type model serializers"""
    class Meta:
        model = AreaTypeModel
        fields = '__all__'