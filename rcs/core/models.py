import os
import shutil

from django.conf import settings
from django.db import models
from django.dispatch import receiver
from rcs.common.types import VehicleState, MissionState


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
                settings.ACTIVE_MAP_CONFIG = temp.config
                if self != temp:
                    temp.active = False
                    temp.save()
            except MapModel.DoesNotExist:
                pass

        super(MapModel, self).save(*args, **kwargs)


# class PointTypeModel(models.Model):
#     name = models.CharField(max_length=64)
#     description = models.TextField(null=True)
#
#     class Meta:
#         db_table = 'rcs_point_type'
#
#     def __str__(self):
#         return self.name


class PointModel(models.Model):
    POINT_TYPE = (
        ('Route', 'route point'),
        ('Charge', 'charge point'),
        ('Parking', 'parking point')
    )
    name = models.CharField(max_length=64)
    type = models.CharField(choices=POINT_TYPE, max_length=64, null=True)
    position = models.JSONField(default=dict)
    orientation = models.JSONField(default=dict)
    active = models.BooleanField(default=True)
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')

    class Meta:
        db_table = 'rcs_point'


class BlockModel(models.Model):
    name = models.CharField(max_length=64)
    vertices = models.TextField(null=True)
    active = models.BooleanField(default=True)
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')

    class Meta:
        db_table = 'rcs_block'


class WallModel(models.Model):
    name = models.CharField(max_length=64)
    vertices = models.TextField(null=True)
    active = models.BooleanField(default=True)
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')

    class Meta:
        db_table = 'rcs_wall'


class ForbiddenModel(models.Model):
    name = models.CharField(max_length=64)
    vertices = models.TextField(null=True)
    active = models.BooleanField(default=True)
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')

    class Meta:
        db_table = 'rcs_forbidden'


class AreaModel(models.Model):
    name = models.CharField(max_length=64)
    vertices = models.TextField(null=True)
    active = models.BooleanField(default=True)
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


class VehicleModel(models.Model):
    name = models.CharField(max_length=64)
    state = models.CharField(choices=VehicleState.CHOICES, default=VehicleState.OFFLINE, max_length=32)
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
    sn = models.CharField(max_length=128)
    name = models.CharField(max_length=128, null=True, blank=True)
    state = models.IntegerField(choices=MissionState.CHOICES, default=MissionState.RAW)
    createTime = models.DateTimeField(auto_now_add=True, db_column='createTime')
    beginTime = models.DateTimeField(null=True, db_column='beginTime')
    finishTime = models.DateTimeField(null=True, db_column='finishTime')
    reason = models.TextField(null=True)
    footprint = models.JSONField(default=list)
    path = models.JSONField(default=list)
    vehicle = models.ForeignKey('core.VehicleModel', on_delete=models.DO_NOTHING)
    isTemplate = models.BooleanField(default=False, db_column='isTemplate')
    raw = models.JSONField(default=dict)

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


class ActionModel(models.Model):
    name = models.CharField(max_length=64)
    action_type = models.CharField(max_length=64)
    description = models.CharField(max_length=128, null=True)
    help = models.TextField(null=True)
    parameters = models.JSONField(null=True)
    active = models.BooleanField(default=True)

    class Meta:
        db_table = 'rcs_action'
        verbose_name = 'Actions'


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
