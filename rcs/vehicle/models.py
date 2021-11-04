from django.db import models
from ..common.types import VehicleState

class VehicleModel(models.Model):

    name = models.CharField(max_length=64)
    state = models.CharField(choices=VehicleState.CHOICES,
                             default=VehicleState.OFFLINE, max_length=32)
    x = models.FloatField(default=0)
    y = models.FloatField(default=0)
    rotation = models.FloatField(default=0)
    battery = models.IntegerField(default=0)
    ip = models.GenericIPAddressField(null=True)

    class Meta:
        db_table = 'rcs_vehicle'
        verbose_name = 'Vehicle'
