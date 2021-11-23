from rest_framework.serializers import ModelSerializer
from .models import SystemSettingModel


class SystemSettingSerializer(ModelSerializer):
    """Serializer for System settings"""

    class Meta:
        model = SystemSettingModel
        fields = '__all__'