from django.contrib import admin
from .models import PointTypeModel, VehicleModel, VehicleSettingModel, SystemSettingModel

admin.site.register(PointTypeModel)
admin.site.register(VehicleModel)
admin.site.register(VehicleSettingModel)
admin.site.register(SystemSettingModel)
