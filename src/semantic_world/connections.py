from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, TYPE_CHECKING, Tuple

import numpy as np

import semantic_world.spatial_types.spatial_types as cas
from .free_variable import FreeVariable
from .prefixed_name import PrefixedName
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
class VirtualConnection(Connection):
    """
    Base class for virtual connections.
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


@dataclass
class Connection6DoF(VirtualConnection):
    x: FreeVariable = field(default=None)
    y: FreeVariable = field(default=None)
    z: FreeVariable = field(default=None)

    qx: FreeVariable = field(default=None)
    qy: FreeVariable = field(default=None)
    qz: FreeVariable = field(default=None)
    qw: FreeVariable = field(default=None)

    def __post_init__(self):
        if self.x is None:
            self.x = self._world.create_virtual_free_variable(name=PrefixedName('x', self.name))
        if self.y is None:
            self.y = self._world.create_virtual_free_variable(name=PrefixedName('y', self.name))
        if self.z is None:
            self.z = self._world.create_virtual_free_variable(name=PrefixedName('z', self.name))
        if self.qx is None:
            self.qx = self._world.create_virtual_free_variable(name=PrefixedName('qx', self.name))
        if self.qy is None:
            self.qy = self._world.create_virtual_free_variable(name=PrefixedName('qy', self.name))
        if self.qz is None:
            self.qz = self._world.create_virtual_free_variable(name=PrefixedName('qz', self.name))
        if self.qw is None:
            self.qw = self._world.create_virtual_free_variable(name=PrefixedName('qw', self.name))

        self._world._state[Derivatives.position][self.qw.state_idx] = 1.
        parent_P_child = cas.Point3((self.x.get_symbol(Derivatives.position),
                                     self.y.get_symbol(Derivatives.position),
                                     self.z.get_symbol(Derivatives.position)))
        parent_R_child = cas.Quaternion((self.qx.get_symbol(Derivatives.position),
                                         self.qy.get_symbol(Derivatives.position),
                                         self.qz.get_symbol(Derivatives.position),
                                         self.qw.get_symbol(Derivatives.position))).to_rotation_matrix()
        self.origin = cas.TransformationMatrix.from_point_rotation_matrix(parent_P_child, parent_R_child)

    def update_transform(self, position: np.ndarray, orientation: np.ndarray) -> None:
        self._world._state[Derivatives.position][self.x.state_idx] = position[0]
        self._world._state[Derivatives.position][self.y.state_idx] = position[1]
        self._world._state[Derivatives.position][self.z.state_idx] = position[2]
        self._world._state[Derivatives.position][self.qx.state_idx] = orientation[0]
        self._world._state[Derivatives.position][self.qy.state_idx] = orientation[1]
        self._world._state[Derivatives.position][self.qz.state_idx] = orientation[2]
        self._world._state[Derivatives.position][self.qw.state_idx] = orientation[3]
