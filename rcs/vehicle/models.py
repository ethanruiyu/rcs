from django.db import models
from ..common.types import VehicleState
from django.db.models.signals import pre_save
from django.dispatch import receiver

class VehicleModel(models.Model):

    name = models.CharField(max_length=64)
    state = models.CharField(choices=VehicleState.CHOICES,
                             default=VehicleState.OFFLINE, max_length=32)
    x = models.FloatField(default=0)
    y = models.FloatField(default=0)
    rotation = models.FloatField(default=0)
    battery = models.IntegerField(default=0)
    ip = models.GenericIPAddressField(null=True)
    width = models.FloatField(default=1)
    map = models.CharField(max_length=256, null=True)

    class Meta:
        db_table = 'rcs_vehicle'
        verbose_name = 'Vehicle'
