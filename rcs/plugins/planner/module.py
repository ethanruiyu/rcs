from django.conf import settings
from rcs.plugins.planner import planner_module
from rcs.map.models import MapModel
from rcs.common.error_codes import CommonErrorCode
from rcs.common.utils.conversion import navigation2image, image2navigation
from rcs.common.types import Point


def move_to_position(init: Point, goal: Point, vehicle_width):
    if not MapModel.objects.filter(active=True).exists():
        return CommonErrorCode

    active_map = MapModel.objects.filter(active=True).first()
    name = active_map.name

    image_init_point = navigation2image(init.x, init.y, active_map)
    image_goal_point = navigation2image(goal.x, goal.y, active_map)

    map_path = settings.MEDIA_ROOT + 'maps/' + name + '/plan.png'
    image_coordinate_path = planner_module.plan(map_path, image_init_point.__tuple__(), image_goal_point.__tuple__(), vehicle_width * 0.5)

    path = []
    for i in image_coordinate_path:
        x, y = i
        navigation_point = image2navigation(x, y, active_map)
        path.append(navigation_point.__list__())

    return path


