from rest_framework.serializers import ModelSerializer, DateTimeField

from rcs.core.models import *


class MapSerializer(ModelSerializer):
    """
    Map model serializer
    """
    createTime = DateTimeField(format='%Y-%m-%d %H:%M', read_only=True)
    updateTime = DateTimeField(format='%Y-%m-%d %H:%M', read_only=True)

    class Meta:
        model = MapModel
        fields = '__all__'


class PointSerializer(ModelSerializer):
    """
    Point model serializer
    """

    class Meta:
        model = PointModel
        fields = '__all__'


class PointTypeSerializer(ModelSerializer):
    """
    Point Type model serializer
    """

    class Meta:
        model = PointTypeModel
        fields = '__all__'


class BlockSerializer(ModelSerializer):
    """
    Block model serializer
    """

    class Meta:
        model = BlockModel
        fields = '__all__'


class AreaSerializer(ModelSerializer):
    """
    Area model serializer
    """

    class Meta:
        model = AreaModel
        fields = '__all__'


class PathSerializer(ModelSerializer):
    """
    Path model serializer
    """

    class Meta:
        model = PathModel
        fields = '__all__'


class VehicleSerializer(ModelSerializer):
    """
    Vehicle model serializer
    """

    class Meta:
        model = VehicleModel
        fields = '__all__'


class VehicleSettingSerializer(ModelSerializer):
    """
    Vehicle Setting model serializer
    """

    class Meta:
        model = VehicleSettingModel
        fields = '__all__'


class GlobalSettingSerializer(ModelSerializer):
    """
    Global Setting model serializer
    """

    class Meta:
        model = GlobalSettingModel
        fields = '__all__'
