from rest_framework.serializers import ModelSerializer
from .models import VehicleModel


class VehicleSerializer(ModelSerializer):
    """Vehicle seriazlier"""

    class Meta:
        model = VehicleModel
        fields = '__all__'