from rest_framework import serializers
from rcs.core.models import *


class MapSerializer(serializers.ModelSerializer):
    """
    Map model serializer
    """
    name = serializers.CharField(required=False)
    createTime = serializers.DateTimeField(format='%Y-%m-%d %H:%M', read_only=True)
    updateTime = serializers.DateTimeField(format='%Y-%m-%d %H:%M', read_only=True)
    active = serializers.BooleanField()

    class Meta:
        model = MapModel
        fields = '__all__'


class PointSerializer(serializers.ModelSerializer):
    """
    Point model serializer
    """

    class Meta:
        model = PointModel
        fields = '__all__'


class PointTypeSerializer(serializers.ModelSerializer):
    """
    Point Type model serializer
    """

    class Meta:
        model = PointTypeModel
        fields = '__all__'


class BlockSerializer(serializers.ModelSerializer):
    """
    Block model serializer
    """

    class Meta:
        model = BlockModel
        fields = '__all__'


class AreaSerializer(serializers.ModelSerializer):
    """
    Area model serializer
    """

    class Meta:
        model = AreaModel
        fields = '__all__'


class PathSerializer(serializers.ModelSerializer):
    """
    Path model serializer
    """

    class Meta:
        model = PathModel
        fields = '__all__'


class VehicleSerializer(serializers.ModelSerializer):
    """
    Vehicle model serializer
    """
    avatar = serializers.SerializerMethodField(read_only=True)

    def get_avatar(self, obj):
        if VehicleGroupModel.objects.filter(name=obj.group.name).exists():
            return VehicleGroupModel.objects.get(name=obj.group.name).avatar
        else:
            return ''

    class Meta:
        model = VehicleModel
        fields = '__all__'


class VehicleGroupSerializer(serializers.ModelSerializer):
    """
    Vehicle Group model serializer
    """
    online_count = serializers.SerializerMethodField()

    def get_online_count(self, obj):
        total_count = VehicleModel.objects.filter(group_id=obj.id).count()
        online_vehicle_count = VehicleModel.objects.filter(group_id=obj.id).exclude(state__in=[0, 4]).count()

        return online_vehicle_count / total_count * 100

    class Meta:
        model = VehicleGroupModel
        fields = '__all__'


class VehicleSettingSerializer(serializers.ModelSerializer):
    """
    Vehicle Setting model serializer
    """

    class Meta:
        model = VehicleSettingModel
        fields = '__all__'


class GlobalSettingSerializer(serializers.ModelSerializer):
    """
    Global Setting model serializer
    """

    class Meta:
        model = GlobalSettingModel
        fields = '__all__'
