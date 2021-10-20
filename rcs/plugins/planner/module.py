from build import planner_module
from rcs.core.models import MapModel


def move_to_position(start, end):
  try:
    map_obj = MapModel.objects.filter(active=True).first()
    if map_obj is None:
      return -1

    
  except Exception as e:
    pass
