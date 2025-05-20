from dataclasses import dataclass
from typing import Type

from ormatic.utils import classproperty

from ..prefixed_name import PrefixedName
from ..spatial_types import RotationMatrix, Vector3, Point3, TransformationMatrix, ReferenceFrameMixin
from sqlalchemy import types, event
from collections import namedtuple
from sqlalchemy.types import TypeDecorator, String
from ormatic.ormatic import ORMaticExplicitMapping

from ..spatial_types.spatial_types import Quaternion


class Vector3Type(TypeDecorator):
    impl = types.JSON

    def process_bind_param(self, value: Vector3, dialect):
        return {"reference_frame": (value.reference_frame.name, value.reference_frame.prefix),
                "position": value.to_np().tolist()}

    def process_result_value(self, value, dialect) -> Vector3:
        reference_frame = PrefixedName(*value["reference_frame"])
        return Vector3.from_xyz(*value["position"], reference_frame=reference_frame)



class Point3Type(TypeDecorator):
    impl = types.JSON

    def process_bind_param(self, value: Point3, dialect):
        return {"reference_frame": (value.reference_frame.name, value.reference_frame.prefix),
                "position": value.to_np().tolist()}

    def process_result_value(self, value, dialect) -> Point3:
        reference_frame = PrefixedName(*value["reference_frame"])
        return Point3.from_xyz(*value["position"], reference_frame=reference_frame)


class RotationMatrixType(TypeDecorator):
    impl = types.JSON

    def process_bind_param(self, value: RotationMatrix, dialect):
        return {"reference_frame":(value.reference_frame.name, value.reference_frame.prefix),
                "rotation": value.to_quaternion().to_np().tolist()}

    def process_result_value(self, value, dialect) -> RotationMatrix:
        reference_frame = PrefixedName(*value["reference_frame"])
        rotation = Quaternion.from_xyzw(*value["rotation"], reference_frame=reference_frame)
        rotation = RotationMatrix.from_quaternion(rotation)
        return rotation


class TransformationMatrixType(TypeDecorator):
    impl = types.JSON

    def process_bind_param(self, value: TransformationMatrix, dialect):
        return {"reference_frame":(value.reference_frame.name, value.reference_frame.prefix),
                "child_frame":(value.child_frame.name, value.child_frame.prefix),
                "position": value.to_position().to_np().tolist(),
                "rotation": value.to_quaternion().to_np().tolist()}

    def process_result_value(self, value, dialect) -> TransformationMatrix:
        reference_frame = PrefixedName(*value["reference_frame"])
        child_frame = PrefixedName(*value["child_frame"])
        position = Point3.from_xyz(*value["position"])
        rotation = Quaternion.from_xyzw(*value["rotation"], reference_frame=reference_frame)

        rotation = RotationMatrix.from_quaternion(rotation)

        return TransformationMatrix.from_point_rotation_matrix(position, rotation, reference_frame=reference_frame,
                                                               child_frame=child_frame)


custom_types = {Vector3: Vector3Type(),
                Point3: Point3Type(),
                TransformationMatrix: TransformationMatrixType(),
                RotationMatrix: RotationMatrixType()}

