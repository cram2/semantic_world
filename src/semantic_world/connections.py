from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, TYPE_CHECKING, Tuple

import numpy as np

import semantic_world.spatial_types.spatial_types as cas
from .free_variable import FreeVariable
from .prefixed_name import PrefixedName
from .spatial_types.derivatives import Derivatives
from .spatial_types.math import rpy_from_quaternion
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
    free_variables: List[FreeVariable] = field(default_factory=list, init=False)


@dataclass
class VirtualConnection(Connection):
    """
    Base class for virtual connections.
    """
    free_variables: List[FreeVariable] = field(default_factory=list, init=False)


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


@dataclass
class OmniDrive(MoveableConnection):
    x: FreeVariable = field(default=None)
    y: FreeVariable = field(default=None)
    z: FreeVariable = field(default=None)
    roll: FreeVariable = field(default=None)
    pitch: FreeVariable = field(default=None)
    yaw: FreeVariable = field(default=None)
    x_vel: FreeVariable = field(default=None)
    y_vel: FreeVariable = field(default=None)

    translation_velocity_limits: float = field(default=0.6)
    rotation_velocity_limits: float = field(default=0.5)

    def __post_init__(self):
        self.x = self.x or self._world.create_virtual_free_variable(name=PrefixedName('x', self.name))
        self.y = self.y or self._world.create_virtual_free_variable(name=PrefixedName('y', self.name))
        self.z = self.z or self._world.create_virtual_free_variable(name=PrefixedName('z', self.name))

        self.roll = self.roll or self._world.create_virtual_free_variable(name=PrefixedName('roll', self.name))
        self.pitch = self.pitch or self._world.create_virtual_free_variable(name=PrefixedName('pitch', self.name))
        self.yaw = self.yaw or self._world.create_free_variable(
            name=PrefixedName('yaw_vel', self.name),
            lower_limits={Derivatives.velocity: -self.rotation_velocity_limits},
            upper_limits={Derivatives.velocity: self.rotation_velocity_limits})

        self.x_vel = self.x_vel or self._world.create_free_variable(
            name=PrefixedName('x_vel', self.name),
            lower_limits={Derivatives.velocity: -self.translation_velocity_limits},
            upper_limits={Derivatives.velocity: self.translation_velocity_limits})
        self.y_vel = self.y_vel or self._world.create_free_variable(
            name=PrefixedName('y_vel', self.name),
            lower_limits={Derivatives.velocity: -self.translation_velocity_limits},
            upper_limits={Derivatives.velocity: self.translation_velocity_limits})
        self.free_variables = [self.x_vel, self.y_vel, self.yaw]

        odom_T_bf = cas.TransformationMatrix.from_xyz_rpy(x=self.x.get_symbol(Derivatives.position),
                                                          y=self.y.get_symbol(Derivatives.position),
                                                          yaw=self.yaw.get_symbol(Derivatives.position))
        bf_T_bf_vel = cas.TransformationMatrix.from_xyz_rpy(x=self.x_vel.get_symbol(Derivatives.position),
                                                            y=self.y_vel.get_symbol(Derivatives.position))
        bf_vel_T_bf = cas.TransformationMatrix.from_xyz_rpy(x=0,
                                                            y=0,
                                                            z=self.z.get_symbol(Derivatives.position),
                                                            roll=self.roll.get_symbol(Derivatives.position),
                                                            pitch=self.pitch.get_symbol(Derivatives.position),
                                                            yaw=0)
        self.origin = odom_T_bf.dot(bf_T_bf_vel).dot(bf_vel_T_bf)

    def update_transform(self, position: np.ndarray, orientation: np.ndarray) -> None:
        roll, pitch, yaw = rpy_from_quaternion(*orientation)
        self._world._state[Derivatives.position, self.x.state_idx] = position[0]
        self._world._state[Derivatives.position, self.y.state_idx] = position[1]
        self._world._state[Derivatives.position, self.z.state_idx] = position[2]
        self._world._state[Derivatives.position, self.roll.state_idx] = roll
        self._world._state[Derivatives.position, self.pitch.state_idx] = pitch
        self._world._state[Derivatives.position, self.yaw.state_idx] = yaw

    def update_state(self, dt: float) -> None:
        state = self._world._state
        state[Derivatives.position, self.x_vel.state_idx] = 0
        state[Derivatives.position, self.y_vel.state_idx] = 0

        x_vel = state[Derivatives.velocity, self.x_vel.state_idx]
        y_vel = state[Derivatives.velocity, self.y_vel.state_idx]
        delta = state[Derivatives.position, self.yaw.name]
        state[Derivatives.velocity, self.x.state_idx] = (np.cos(delta) * x_vel - np.sin(delta) * y_vel)
        state[Derivatives.position, self.x.state_idx] += state[Derivatives.velocity, self.x.state_idx] * dt
        state[Derivatives.velocity, self.y.state_idx] = (np.sin(delta) * x_vel + np.cos(delta) * y_vel)
        state[Derivatives.position, self.y.state_idx] += state[Derivatives.velocity, self.y.state_idx] * dt

    def get_free_variable_names(self) -> List[PrefixedName]:
        return [self.x.name, self.y.name, self.yaw.name]
