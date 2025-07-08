from dataclasses import dataclass
from typing import List, Dict, Any

from ormatic.dao import AlternativeMapping

from ..prefixed_name import PrefixedName
from ..spatial_types import RotationMatrix, Vector3, Point3, TransformationMatrix
from ..spatial_types.spatial_types import Quaternion
from ..world import World, Body
from ..world_entity import Connection


@dataclass
class WorldMapping(AlternativeMapping[World]):
    bodies: List[Body]
    connections: List[Connection]

    @classmethod
    def to_dao(cls, obj: World, memo: Dict[int, Any] = None):
        return cls(obj.bodies, obj.connections)


@dataclass
class Vector3Mapping(AlternativeMapping[Vector3]):
    x: float
    y: float
    z: float

    @classmethod
    def to_dao(cls, obj: Vector3, memo: Dict[int, Any] = None):
        x, y, z = obj.to_np().tolist()
        return cls(x=x, y=y, z=z)


@dataclass
class Point3Mapping(AlternativeMapping[Point3]):
    x: float
    y: float
    z: float

    @classmethod
    def to_dao(cls, obj: Point3, memo: Dict[int, Any] = None):
        x, y, z = obj.to_np().tolist()[:3]
        return cls(x=x, y=y, z=z)


@dataclass
class QuaternionMapping(AlternativeMapping[Quaternion]):
    x: float
    y: float
    z: float
    w: float

    @classmethod
    def to_dao(cls, obj: Quaternion, memo: Dict[int, Any] = None):
        x, y, z, w = obj.to_np().tolist()
        return cls(x=x, y=y, z=z, w=w)


@dataclass
class RotationMatrixMapping(AlternativeMapping[RotationMatrix]):
    reference_frame: PrefixedName
    rotation: Quaternion

    @classmethod
    def to_dao(cls, obj: RotationMatrix, memo: Dict[int, Any] = None):
        return cls(reference_frame=obj.reference_frame, rotation=obj.to_quaternion())


@dataclass
class TransformationMatrixMapping(AlternativeMapping[TransformationMatrix]):
    reference_frame: PrefixedName
    child_frame: PrefixedName
    position: Point3
    rotation: Quaternion

    @classmethod
    def to_dao(cls, obj: TransformationMatrix, memo: Dict[int, Any] = None):
        position = obj.to_position()
        rotation = obj.to_quaternion()
        result = cls(reference_frame=obj.reference_frame, child_frame=obj.child_frame,
                     position=position,
                     rotation=rotation)

        return result
