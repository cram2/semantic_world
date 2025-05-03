
from ..spatial_types import Vector3
from sqlalchemy import types
from collections import namedtuple
from sqlalchemy.types import TypeDecorator, String

Vector3Tuple = namedtuple('Vector3Tuple', ['x', 'y', 'z'])


class Vector3Type(TypeDecorator):
    """
    Type for vector serialization from casadi vector3.
    """
    impl = String
    cache_ok = True

    def process_bind_param(self, value, dialect):
        if value is None:
            return None
        return f"{value.x},{value.y},{value.z}"

    def process_result_value(self, value, dialect):
        if value is None:
            return None
        x, y, z = map(float, value.split(','))
        return Vector3Tuple(x, y, z)



