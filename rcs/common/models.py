from django.db import models

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