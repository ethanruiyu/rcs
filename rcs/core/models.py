import os
import shutil

from django.conf import settings
from django.db import models
from django.dispatch import receiver


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


class VehicleState(models.TextChoices):
    OFFLINE = 'offline'
    IDLE = 'idle'
    BUSY = 'busy'
    PAUSE = 'pause'
    ERROR = 'error'
    CHARGING = 'charging'


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

    class Meta:
        db_table = 'rcs_map'


class PointModel(models.Model):
    name = models.CharField(max_length=64)
    type = models.ForeignKey('core.PointTypeModel', on_delete=models.CASCADE)
    position = models.JSONField(default=dict)
    orientation = models.JSONField(default=dict)
    group = models.ForeignKey(
        'core.GroupModel', on_delete=models.CASCADE, null=True, db_column='groupId')
    active = models.BooleanField(default=True)
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')

    class Meta:
        db_table = 'rcs_point'


class PointTypeModel(models.Model):
    name = models.CharField(max_length=64)
    description = models.TextField(null=True)

    class Meta:
        db_table = 'rcs_point_type'
        verbose_name = 'Point Type'

    def __str__(self):
        return self.name


class BlockModel(models.Model):
    name = models.CharField(max_length=64)
    group = models.ForeignKey(
        'core.GroupModel', on_delete=models.CASCADE, null=True, db_column='groupId')
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')

    class Meta:
        db_table = 'rcs_block'


class AreaModel(models.Model):
    name = models.CharField(max_length=64)
    group = models.ForeignKey(
        'core.GroupModel', on_delete=models.CASCADE, null=True, db_column='groupId')
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')

    class Meta:
        db_table = 'rcs_area'


class AreaTypeModel(models.Model):
    group = models.ForeignKey(
        'core.GroupModel', on_delete=models.CASCADE, null=True, db_column='groupId')
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')

    class Meta:
        db_table = 'rcs_area_type'


class GroupModel(models.Model):
    name = models.CharField(max_length=64)
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')

    class Meta:
        db_table = 'rcs_group'


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
    group = models.ForeignKey(
        'core.GroupModel', on_delete=models.CASCADE, null=True, db_column='groupId')
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')

    class Meta:
        db_table = 'rcs_path'


class VehicleModel(models.Model):
    name = models.CharField(max_length=64)
    state = models.IntegerField(choices=VehicleState.choices, default=0)
    position = models.JSONField(default=dict)
    nextPosition = models.JSONField(default=dict, db_column='nextPosition')
    group = models.ForeignKey(
        'core.GroupModel', on_delete=models.CASCADE, null=True, db_column='groupId')
    map = models.ForeignKey(
        'core.MapModel', on_delete=models.CASCADE, db_column='mapId')
    image = models.FileField(upload_to=vehicle_dir_name, blank=True)

    class Meta:
        db_table = 'rcs_vehicle'
        verbose_name = 'Vehicle'


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


class GlobalSettingModel(models.Model):
    key = models.CharField(max_length=64)
    value = models.CharField(max_length=64)
    description = models.TextField(null=True)

    def __str__(self):
        return self.key

    class Meta:
        db_table = 'rcs_global_setting'
        verbose_name = 'Global Setting'


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
