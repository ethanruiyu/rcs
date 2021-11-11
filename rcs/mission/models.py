from django.db import models
from ..common.types import MissionState
from django.dispatch.dispatcher import receiver
from django.db.models.signals import post_save
from .dispatcher import DISPATCHER


class MissionModel(models.Model):
    sn = models.CharField(max_length=256)
    name = models.CharField(max_length=256, null=True, blank=True)
    state = models.CharField(max_length=64, choices=MissionState.CHOICES, default=MissionState.RAW)
    createTime = models.DateTimeField(auto_now_add=True, db_column='createTime')
    beginTime = models.DateTimeField(null=True, db_column='beginTime')
    finishTime = models.DateTimeField(null=True, db_column='finishTime')
    reason = models.TextField(null=True)
    footprint = models.JSONField(default=list)
    path = models.JSONField(default=list)
    vehicle = models.ForeignKey('vehicle.VehicleModel', on_delete=models.DO_NOTHING, null=True, related_name='mission_vehicle')
    isTemplate = models.BooleanField(default=False, db_column='isTemplate')
    raw = models.JSONField(default=dict)

    class Meta:
        db_table = 'rcs_mission'


@receiver(post_save, sender=MissionModel)
def mission_save_receiver(sender, instance, **kwargs):
    if instance.state == 'raw':
        DISPATCHER.put(instance)