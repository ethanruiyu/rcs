class PointMessage:
    def __init__(self, name, position):
        self._name = name
        self._position = position
    

class PathMessage:
    def __init__(self):
        self._source_point = None
        self._destination_point = None

    @property
    def source_point(self):
        """
        {
            "name": "Point-01",
            "position": {
                "x": 0,
                "y": 0
            }
        }
        """
        return self._source_point

    @property
    def destination_point(self):
        """
        {
            "name": "Point-01",
            "position": {
                "x": 0,
                "y": 0
            }
        }
        """
        return self._destination_point

    def __repr__(self):
        return repr((self._source_point, self._destination_point))


class MovementMessage:
    def __init__(self):
        self._properties = dict
        self._path = None
        self._action = None
        self._final_movement = False

    @property
    def path(self):
        return self._path

    @path.setter
    def path(self, value):
        self._path = value
    
    @property
    def action(self):
        return self._action

    @action.setter
    def action(self, value):
        self._action = value

    @property
    def properties(self):
        return self._properties

    @properties.setter
    def properties(self, value):
        self._properties = value

    def is_without_action(self) -> bool:
        return self._action is None

    def is_final_movement(self) -> bool:
        return self._final_movement is True

    def __repr__(self):
        return repr((self._action, self._properties))
