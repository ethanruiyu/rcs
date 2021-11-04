from django.contrib import admin
from .models import MapModel, PointTypeModel, AreaTypeModel


admin.site.register(MapModel)
admin.site.register(PointTypeModel)
admin.site.register(AreaTypeModel)