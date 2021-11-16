from rest_framework.serializers import ModelSerializer
from .models import SystemSetting


class SystemSettingSerializer(ModelSerializer):
    """Serializer for System settings"""

    class Meta:
        model = SystemSetting
        fields = '__all__'