from typing import Dict, Any
import threading
import time

from rcs.common.types import MissionState
from ..communication.adapter import VEHICLE_ADAPTERS
from ..common.commands import Mission


class Dispatcher(threading.Thread):
    mission_list = list()

    def run(self) -> None:
        while True:
            if self.count() > 0:
                mission_model_obj = self.peek()
                if mission_model_obj.vehicle:
                    vehicle_adapter = VEHICLE_ADAPTERS[mission_model_obj.vehicle.name]
                    mission_cmd = Mission(vehicle_adapter.get_position(), mission_model_obj.vehicle.width, mission_model_obj.raw)
                    mission_model_obj.path = mission_cmd.get_path()
                    mission_model_obj.state = MissionState.DISPATCH
                    mission_model_obj.save()
                    vehicle_adapter.send_command(mission_cmd)
                    vehicle_adapter.set_mission(mission_model_obj)
                    self.pop()
                else:
                    for vehicle_adapter in VEHICLE_ADAPTERS.values():
                        if vehicle_adapter.can_proceed():
                            mission_cmd = Mission(vehicle_adapter.get_position(), mission_model_obj.vehicle.width, mission_model_obj.raw)
                            mission_model_obj.path = mission_cmd.get_path()
                            mission_model_obj.state = MissionState.DISPATCH
                            mission_model_obj.save()
                            vehicle_adapter.send_command(mission_cmd)
                            vehicle_adapter.set_mission(mission_model_obj)
                            self.pop()
                            break

            print(self.count())

            time.sleep(1)

    def count(self):
        return self.mission_list.__len__()

    def put(self, mission):
        self.mission_list.append(mission)

    def pop(self):
        self.mission_list.pop(0)

    def peek(self):
        return self.mission_list[0]


DISPATCHER = Dispatcher()
