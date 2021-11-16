from django.db import models

class SystemSettingModel(models.Model):
    FIELD_TYPE_CHOICES = (
        ('input', 'input'),
        ('select', 'select'),
    )
    key = models.CharField(max_length=256)
    value = models.CharField(max_length=256)
    description = models.TextField(blank=True)
    fieldType = models.CharField(max_length=64, choices=FIELD_TYPE_CHOICES, db_column='fieldType')
    options = models.JsonField(blank=True)

    class Meta:
        db_table = 'rcs_system'
        verbose_name = 'System'