from rest_framework.viewsets import ModelViewSet
from .models import SystemSettingModel
from .serializers import SystemSettingSerializer



class SystemSettingViewSet(ModelViewSet):
    queryset = SystemSettingModel.objects.all()
    serializer_class = SystemSettingSerializer