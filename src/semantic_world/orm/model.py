from dataclasses import dataclass
from typing import List, Dict, Any

from ormatic.dao import AlternativeMapping

from ..prefixed_name import PrefixedName
from ..spatial_types import RotationMatrix, Vector3, Point3, TransformationMatrix
from ..spatial_types.spatial_types import Quaternion
from ..spatial_types.symbol_manager import symbol_manager
from ..world import World, Body
from ..world_entity import Connection


@dataclass
class WorldMapping(AlternativeMapping[World]):
    bodies: List[Body]
    connections: List[Connection]

    @classmethod
    def create_instance(cls, obj: World):
        return cls(obj.bodies, obj.connections)


@dataclass
class Vector3Mapping(AlternativeMapping[Vector3]):
    reference_frame: PrefixedName

    x: float
    y: float
    z: float

    @classmethod
    def create_instance(cls, obj: Vector3):
        x, y, z, _ = symbol_manager.evaluate_expr(obj).tolist()
        return cls(x=x, y=y, z=z, reference_frame=obj.reference_frame)


@dataclass
class Point3Mapping(AlternativeMapping[Point3]):
    reference_frame: PrefixedName

    x: float
    y: float
    z: float

    @classmethod
    def create_instance(cls, obj: Point3):
        x, y, z, _ = symbol_manager.evaluate_expr(obj).tolist()
        result = cls(x=x, y=y, z=z, reference_frame=obj.reference_frame)
        return result


@dataclass
class QuaternionMapping(AlternativeMapping[Quaternion]):
    reference_frame: PrefixedName
    x: float
    y: float
    z: float
    w: float

    @classmethod
    def create_instance(cls, obj: Quaternion):
        x, y, z, w = symbol_manager.evaluate_expr(obj).tolist()
        result = cls(x=x, y=y, z=z, w=w, reference_frame=obj.reference_frame)
        return result

@dataclass
class RotationMatrixMapping(AlternativeMapping[RotationMatrix]):
    reference_frame: PrefixedName
    rotation: Quaternion

    @classmethod
    def create_instance(cls, obj: RotationMatrix):
        return cls(reference_frame=obj.reference_frame, rotation=obj.to_quaternion())


@dataclass
class TransformationMatrixMapping(AlternativeMapping[TransformationMatrix]):
    reference_frame: PrefixedName
    child_frame: PrefixedName
    position: Point3
    rotation: Quaternion

    @classmethod
    def create_instance(cls, obj: TransformationMatrix):
        position = obj.to_position()
        rotation = obj.to_quaternion()
        result = cls(reference_frame=obj.reference_frame, child_frame=obj.child_frame, position=position,
                     rotation=rotation)
        return result
