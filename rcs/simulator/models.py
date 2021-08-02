from django.db import models


class SimulatorModel(models.Model):
    vehicle = models.ForeignKey('core.VehicleModel', on_delete=models.CASCADE)
    enable = models.BooleanField(default=False)

    class Meta:
        db_table = 'rcs_simulator'
