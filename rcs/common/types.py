from rcs.core.models import MapModel
from django.db import connection
import math


class Types:
    def __init__(self):
        # cursor = connection.cursor()
        # table_list = connection.introspection.get_table_list(cursor)
        # names = []
        # for i in table_list:
        #     names.append(i.name)
        # if 'rcs_map' in names:
        self.config = MapModel.objects.filter(active=True).first().config if MapModel.objects.filter(
            active=True).exists() else None


class Point(Types):
    x: float = 0
    y: float = 0
    z: float = 0

    def __init__(self, point: list):
        super(Point, self).__init__()
        self.x = point[0]
        self.y = point[1]
        self.z = point[2]

    def nav2vis(self):
        """
        navigation coordinates to visualization coordinates
        """
        ipx = int((self.x - self.config['origin'][0]) / self.config['resolution'])
        ipy = int(self.config['height'] - (self.y - self.config['origin'][1]) / self.config['resolution'] - 1)

        vpx = ipx - self.config['width'] / 2
        vpy = ipy - self.config['height'] / 2

        return [vpx, vpy, 0]

    def vis2nav(self):
        """
        visualization coordinates to navigation coordinates
        """
        if self.config:
            pass
        else:
            return None

    def __str__(self):
        return [self.x, self.y, self.z]

    def __dict__(self):
        return {
            'x', self.x,
            'y', self.y,
            'z', self.z,
        }


class Angle(Types):
    angle: float = 0

    def __init__(self, **kwargs):
        super(Angle, self).__init__()
        self.angle = kwargs['angle']

    def __str__(self):
        return self.angle


class Quaternion(Types):
    x: float = 0
    y: float = 0
    z: float = 0
    w: float = 0

    def __init__(self, orientation: list):
        super(Quaternion, self).__init__()
        self.x = orientation[0]
        self.y = orientation[1]
        self.z = orientation[2]
        self.w = orientation[3]

    def to_angle(self):
        # t0 = +2.0 * (self.w * self.x + self.y * self.z)
        # t1 = +1.0 - 2.0 * (self.x * self.x + self.y * self.y)
        # roll_x = math.atan2(t0, t1)
        #
        # t2 = +2.0 * (self.w * self.y - self.z * self.x)
        # t2 = +1.0 if t2 > +1.0 else t2
        # t2 = -1.0 if t2 < -1.0 else t2
        # pitch_y = math.asin(t2)

        t3 = +2.0 * (self.w * self.z + self.x * self.y)
        t4 = +1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        yaw_z = math.atan2(t3, t4)

        return -(180 / math.pi * yaw_z) - 90  # in radians

    def __str__(self):
        return [self.x, self.y, self.z, self.w]

    def __dict__(self):
        return {
            'x', self.x,
            'y', self.y,
            'z', self.z,
            'w', self.w
        }


class Pose(Types):
    position: Point = Point([0, 0, 0])
    orientation: Quaternion = Quaternion([0, 0, 0, 0])

    def __init__(self, **kwargs):
        super(Pose, self).__init__()
        self.position = kwargs['position']
        self.orientation = kwargs['orientation']

    def nav2vis(self):
        nav_position = self.position.nav2vis()
        nav_orientation = self.orientation.to_angle()

        return [nav_position, [nav_orientation]]

    def vis2nav(self):
        pass

    def __str__(self):
        return [self.position.__str__(), self.orientation.__str__()]

    def __dict__(self):
        return {
            'position': {
                'x': self.position.x,
                'y': self.position.y,
                'z': self.position.z
            },
            'orientation': {
                'x': self.orientation.x,
                'y': self.orientation.y,
                'z': self.orientation.z,
                'w': self.orientation.w
            }
        }


class Path:
    path: list = []

    def __init__(self, path: list):
        self.path = path

    def __str__(self):
        return self.path
