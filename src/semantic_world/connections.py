from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, TYPE_CHECKING, Tuple

import semantic_world.spatial_types.spatial_types as cas
from .free_variable import FreeVariable
from .spatial_types.derivatives import Derivatives
from .world_entity import Connection


@dataclass
class FixedConnection(Connection):
    """
    A connection that has 0 degrees of freedom.
    """


@dataclass
class MoveableConnection(Connection):
    """
    Base class for moveable connections.
    """
    free_variables: List[FreeVariable] = field(default_factory=list)


@dataclass
class PrismaticConnection(MoveableConnection):
    axis: Tuple[float, float, float] = field(default=None)
    multiplier: float = 1.0
    offset: float = 0.0
    free_variable: FreeVariable = field(default=None)

    def __post_init__(self):
        super().__post_init__()
        if self.multiplier is None:
            self.multiplier = 1
        else:
            self.multiplier = self.multiplier
        if self.offset is None:
            self.offset = 0
        else:
            self.offset = self.offset
        self.axis = self.axis
        self.free_variables = [self.free_variable]

        motor_expression = self.free_variable.get_symbol(Derivatives.position) * self.multiplier + self.offset
        translation_axis = cas.Vector3(self.axis) * motor_expression
        parent_T_child = cas.TransformationMatrix.from_xyz_rpy(x=translation_axis[0],
                                                               y=translation_axis[1],
                                                               z=translation_axis[2])
        self.origin = self.origin.dot(parent_T_child)


@dataclass
class RevoluteConnection(MoveableConnection):
    axis: Tuple[float, float, float] = field(default=None)
    multiplier: float = 1.0
    offset: float = 0.0
    free_variable: FreeVariable = field(default=None)

    def __post_init__(self):
        super().__post_init__()
        if self.multiplier is None:
            self.multiplier = 1
        else:
            self.multiplier = self.multiplier
        if self.offset is None:
            self.offset = 0
        else:
            self.offset = self.offset
        self.axis = self.axis
        self.free_variables = [self.free_variable]

        motor_expression = self.free_variable.get_symbol(Derivatives.position) * self.multiplier + self.offset
        rotation_axis = cas.Vector3(self.axis)
        parent_R_child = cas.RotationMatrix.from_axis_angle(rotation_axis, motor_expression)
        self.origin = self.origin.dot(cas.TransformationMatrix(parent_R_child))
