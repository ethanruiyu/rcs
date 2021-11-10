from django.db import models
from colorfield.fields import ColorField
from django.conf import settings


class MapModel(models.Model):
    name = models.CharField(max_length=64)
    description = models.TextField(null=True)
    createTime = models.DateTimeField(
        auto_now_add=True, db_column='createTime')
    updateTime = models.DateTimeField(auto_now=True, db_column='updateTime')
    createBy = models.ForeignKey(
        'account.User', on_delete=models.DO_NOTHING, db_column='createBy', null=True)
    raw = models.JSONField(default=list)
    originX = models.FloatField(null=True, db_column='originX')
    originY = models.FloatField(null=True, db_column='originY')
    resolution = models.FloatField(null=True)
    height = models.IntegerField(null=True)
    width = models.IntegerField(null=True)
    active = models.BooleanField(default=False)

    class Meta:
        db_table = 'rcs_map'
        verbose_name = 'Map'

    def save(self, *args, **kwargs):
        if self.active:
            try:
                temp = MapModel.objects.get(active=True)
                # settings.ACTIVE_MAP_CONFIG = temp.config
                if self != temp:
                    temp.active = False
                    temp.save()
            except MapModel.DoesNotExist:
                pass

        super(MapModel, self).save(*args, **kwargs)


class AreaTypeModel(models.Model):
    name = models.CharField(max_length=256)
    description = models.TextField(null=True, blank=True)
    fillColor = ColorField(null=True, db_column='fillColor')
    strokeColor = ColorField(null=True, db_column='strokeColor')
    height = models.FloatField(default=0)
    action = models.ForeignKey(
        'common.ActionModel', on_delete=models.SET_NULL, null=True, blank=True)
    fields = models.JSONField(default=list)

    class Meta:
        db_table = 'rcs_area_type'


class PointTypeModel(models.Model):
    name = models.CharField(max_length=256)
    description = models.TextField(null=True)
    fillColor = ColorField(null=True, db_column='fillColor')
    strokeColor = ColorField(null=True, db_column='strokeColor')
    action = models.ForeignKey(
        'common.ActionModel', on_delete=models.SET_NULL, null=True)
    fields = models.JSONField(null=True)

    class Meta:
        db_table = 'rcs_point_type'


class PointModel(models.Model):
    name = models.CharField(max_length=256)
    description = models.TextField(null=True)
    type = models.ForeignKey('map.PointTypeModel',
                             on_delete=models.CASCADE)
    x = models.FloatField(default=0)
    y = models.FloatField(default=0)
    rotation = models.FloatField(default=0)
    map = models.ForeignKey('map.MapModel', on_delete=models.CASCADE)
    active = models.BooleanField(default=True)

    class Meta:
        db_table = 'rcs_point'


class AreaModel(models.Model):
    name = models.CharField(max_length=256)
    description = models.TextField(null=True)
    type = models.ForeignKey('map.AreaTypeModel',
                             on_delete=models.CASCADE)
    points = models.JSONField(default=list)
    map = models.ForeignKey('map.MapModel', on_delete=models.CASCADE)
    active = models.BooleanField(default=True)

    class Meta:
        db_table = 'rcs_area'