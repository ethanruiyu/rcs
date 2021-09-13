import os
import shutil

from django.conf import settings
from django.db import models
from django.dispatch import receiver
from rcs.common.enum import *


def map_dir_name(instance, filename):
    """
    FileField prefix dir
    """
    return 'maps/{0}/{1}'.format(instance.name, filename)


def vehicle_dir_name(instance, filename):
    """
    FileField prefix dir
    """
    return 'vehicles/{0}/{1}'.format(instance.name, filename)


class SiteModel(models.Model):
    name = models.CharField(max_length=64)
    description = models.TextField(null=True)

    class Meta:
        db_table = 'rcs_site'


class MapModel(models.Model):
    name = models.CharField(max_length=64)
    description = models.TextField(null=True)
    createTime = models.DateTimeField(
        auto_now_add=True, db_column='createTime')
    updateTime = models.DateTimeField(auto_now=True, db_column='updateTime')
    raw = models.JSONField(default=list)
    config = models.JSONField(default=dict)
    active = models.BooleanField(default=False)
    file = models.FileField(upload_to=map_dir_name, blank=True)
    # site = models.ForeignKey('core.SiteModel', on_delete=models.CASCADE)

    class Meta:
        db_table = 'rcs_map'

    def save(self, *args, **kwargs):
        if self.active:
            try:
                temp = MapModel.objects.get(active=True)
                if self != temp:
                    temp.active = False
                    temp.save()
            except MapModel.DoesNotExist:
                pass

        super(MapModel, self).save(*args, **kwargs)


class PointTypeModel(models.Model):
    name = models.CharField(max_length=64)
    description = models.TextField(null=True)

    class Meta:
        db_table = 'rcs_point_type'

    def __str__(self):
        return self.name


class PointModel(models.Model):
    name = models.CharField(max_length=64)
    type = models.ForeignKey('core.PointTypeModel', on_delete=models.CASCADE, null=True)
    position = models.JSONField(default=dict)
    orientation = models.JSONField(default=dict)
    active = models.BooleanField(default=True)
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')
    disabled = models.BooleanField(default=False)

    class Meta:
        db_table = 'rcs_point'


class BlockModel(models.Model):
    name = models.CharField(max_length=64)
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')

    class Meta:
        db_table = 'rcs_block'


class AreaModel(models.Model):
    name = models.CharField(max_length=64)
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')

    class Meta:
        db_table = 'rcs_area'


class AreaTypeModel(models.Model):
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')

    class Meta:
        db_table = 'rcs_area_type'


class VehicleTypeModel(models.Model):
    name = models.CharField(max_length=64)
    avatar = models.URLField(null=True)
    action = models.CharField(null=True, max_length=64)

    class Meta:
        db_table = 'rcs_vehicle_type'


class PathModel(models.Model):
    name = models.CharField(max_length=64)
    sourcePoint = models.ForeignKey('core.PointModel', on_delete=models.CASCADE, related_name='sourcePoint',
                                    db_column='sourcePoint')
    destinationPoint = models.ForeignKey('core.PointModel', on_delete=models.CASCADE, related_name='destinationPoint',
                                         db_column='destinationPoint')
    length = models.FloatField()
    maxVelocity = models.FloatField(default=1.0, db_column='maxVelocity')
    maxReverseVelocity = models.FloatField(
        default=1.0, db_column='maxReverseVelocity')
    locked = models.BooleanField(default=False)
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')

    class Meta:
        db_table = 'rcs_path'


class VehicleModel(models.Model):
    VEHICLE_STATE = (
        (0, 'OFFLINE'),
        (1, 'IDLE'),
        (2, 'BUSY'),
        (3, 'PAUSE'),
        (4, 'ERROR'),
        (5, 'CHARGING')
    )
    name = models.CharField(max_length=64)
    state = models.IntegerField(choices=VEHICLE_STATE, default=0)
    position = models.JSONField(default=dict)
    type = models.ForeignKey(
        'core.VehicleTypeModel', on_delete=models.CASCADE, null=True, blank=True, default=1, db_column='typeId')
    image = models.FileField(upload_to=vehicle_dir_name, blank=True)
    active = models.BooleanField(default=True)
    # site = models.ForeignKey('core.SiteModel', on_delete=models.CASCADE, db_column='siteId')
    battery = models.IntegerField(default=0)
    ip = models.GenericIPAddressField(null=True)

    class Meta:
        db_table = 'rcs_vehicle'
        verbose_name = 'Vehicle'


class MissionModel(models.Model):
    MISSION_STATE = (
        (0, 'RAW'),
        (1, 'DISPATCH'),
        (2, 'PROCESSED'),
        (3, 'FINISHED'),
        (4, 'PAUSED'),
        (5, 'ABORT')
    )
    sn = models.CharField(max_length=128)
    name = models.CharField(max_length=128, null=True, blank=True)
    state = models.IntegerField(choices=MISSION_STATE, default=0)
    create_time = models.DateTimeField(auto_now_add=True)
    begin_time = models.DateTimeField(null=True)
    finish_time = models.DateTimeField(null=True)
    reason = models.TextField(null=True)
    track = models.JSONField(default=list)
    plan = models.JSONField(default=list)
    vehicle = models.ForeignKey('core.VehicleModel', on_delete=models.CASCADE, null=True)
    isTemplate = models.BooleanField(default=False, db_column='isTemplate')

    class Meta:
        db_table = 'rcs_mission'


class MissionGroup(models.Model):
    name = models.CharField(max_length=128)

    class Meta:
        db_table = 'rcs_mission_group'


class VehicleSettingModel(models.Model):
    vehicle = models.ForeignKey(
        'core.VehicleModel', on_delete=models.CASCADE, null=True)
    key = models.CharField(max_length=64)
    value = models.JSONField(default=dict)
    description = models.TextField(null=True)

    def __str__(self):
        return self.key

    class Meta:
        db_table = 'rcs_vehicle_setting'
        verbose_name = 'Vehicle Setting'


class SystemSettingModel(models.Model):
    key = models.CharField(max_length=64, unique=True)
    value = models.CharField(max_length=64)
    description = models.TextField(null=True)

    def __str__(self):
        return self.key

    class Meta:
        db_table = 'rcs_system_setting'
        verbose_name = 'System Setting'


@receiver(models.signals.post_delete, sender=MapModel)
def auto_delete_file_on_delete(sender, instance, **kwargs):
    """
    delete media files, When delete map obj
    :param sender:
    :param instance:
    :param kwargs:
    :return:
    """
    if instance.name:
        map_dir_path = settings.MEDIA_ROOT + 'maps/' + instance.name
        if os.path.isdir(map_dir_path):
            shutil.rmtree(map_dir_path)
