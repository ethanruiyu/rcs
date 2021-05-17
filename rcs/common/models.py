from django.db import models
from rcs.core.models import VehicleState


class VehicleProcess:
    def __init__(self):
        super(VehicleProcess, self).__init__()
        self._status = VehicleState.OFFLINE

    @property
    def status(self):
        return self._status

    @status.setter
    def status(self, status):
        self._status = status
