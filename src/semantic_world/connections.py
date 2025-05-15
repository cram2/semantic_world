from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

import semantic_world.spatial_types.spatial_types as cas
from .degree_of_freedom import DegreeOfFreedom
from .prefixed_name import PrefixedName
from .spatial_types.derivatives import Derivatives
from .spatial_types.math import rpy_from_quaternion
from .world_entity import Connection


@dataclass
class FixedConnection(Connection):
    """
    Has 0 degrees of freedom.
    """


@dataclass
class ActiveConnection(Connection):
    """
    Has active degrees of freedom.
    """
    active_dofs: List[DegreeOfFreedom] = field(default_factory=list, init=False)


@dataclass
class PassiveConnection(Connection):
    """
    Has passive degrees of freedom.
    """
    passive_dofs: List[DegreeOfFreedom] = field(default_factory=list, init=False)


@dataclass
class PrismaticConnection(ActiveConnection):
    axis: Tuple[float, float, float] = field(kw_only=True)
    multiplier: float = 1.0
    offset: float = 0.0
    dof: DegreeOfFreedom = field(default=None)

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
        self.dof = self.dof or self._world.create_degree_of_freedom(name=PrefixedName(self.name))
        self.active_dofs = [self.dof]

        motor_expression = self.dof.get_symbol(Derivatives.position) * self.multiplier + self.offset
        translation_axis = cas.Vector3(self.axis) * motor_expression
        parent_T_child = cas.TransformationMatrix.from_xyz_rpy(x=translation_axis[0],
                                                               y=translation_axis[1],
                                                               z=translation_axis[2])
        self.origin = self.origin.dot(parent_T_child)


@dataclass
class RevoluteConnection(ActiveConnection):
    axis: Tuple[float, float, float] = field(kw_only=True)
    multiplier: float = 1.0
    offset: float = 0.0
    dof: DegreeOfFreedom = field(default=None)

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
        self.dof = self.dof or self._world.create_degree_of_freedom(name=PrefixedName(self.name))
        self.active_dofs = [self.dof]

        motor_expression = self.dof.get_symbol(Derivatives.position) * self.multiplier + self.offset
        rotation_axis = cas.Vector3(self.axis)
        parent_R_child = cas.RotationMatrix.from_axis_angle(rotation_axis, motor_expression)
        self.origin = self.origin.dot(cas.TransformationMatrix(parent_R_child))


@dataclass
class Connection6DoF(PassiveConnection):
    x: DegreeOfFreedom = field(default=None)
    y: DegreeOfFreedom = field(default=None)
    z: DegreeOfFreedom = field(default=None)

    qx: DegreeOfFreedom = field(default=None)
    qy: DegreeOfFreedom = field(default=None)
    qz: DegreeOfFreedom = field(default=None)
    qw: DegreeOfFreedom = field(default=None)

    def __post_init__(self):
        self.x = self.x or self._world.create_degree_of_freedom(name=PrefixedName('x', self.name))
        self.y = self.y or self._world.create_degree_of_freedom(name=PrefixedName('y', self.name))
        self.z = self.z or self._world.create_degree_of_freedom(name=PrefixedName('z', self.name))
        self.qx = self.qx or self._world.create_degree_of_freedom(name=PrefixedName('qx', self.name))
        self.qy = self.qy or self._world.create_degree_of_freedom(name=PrefixedName('qy', self.name))
        self.qz = self.qz or self._world.create_degree_of_freedom(name=PrefixedName('qz', self.name))
        self.qw = self.qw or self._world.create_degree_of_freedom(name=PrefixedName('qw', self.name))
        self.passive_dofs = [self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw]

        self._world.state[Derivatives.position][self.qw.state_idx] = 1.
        parent_P_child = cas.Point3((self.x.get_symbol(Derivatives.position),
                                     self.y.get_symbol(Derivatives.position),
                                     self.z.get_symbol(Derivatives.position)))
        parent_R_child = cas.Quaternion((self.qx.get_symbol(Derivatives.position),
                                         self.qy.get_symbol(Derivatives.position),
                                         self.qz.get_symbol(Derivatives.position),
                                         self.qw.get_symbol(Derivatives.position))).to_rotation_matrix()
        self.origin = cas.TransformationMatrix.from_point_rotation_matrix(parent_P_child, parent_R_child)

    def update_transform(self, position: np.ndarray, orientation: np.ndarray) -> None:
        self._world.state[Derivatives.position][self.x.state_idx] = position[0]
        self._world.state[Derivatives.position][self.y.state_idx] = position[1]
        self._world.state[Derivatives.position][self.z.state_idx] = position[2]
        self._world.state[Derivatives.position][self.qx.state_idx] = orientation[0]
        self._world.state[Derivatives.position][self.qy.state_idx] = orientation[1]
        self._world.state[Derivatives.position][self.qz.state_idx] = orientation[2]
        self._world.state[Derivatives.position][self.qw.state_idx] = orientation[3]


class HasUpdateState(ABC):
    @abstractmethod
    def update_state(self, dt: float) -> None:
        pass


@dataclass
class OmniDrive(ActiveConnection, PassiveConnection, HasUpdateState):
    x: DegreeOfFreedom = field(default=None)
    y: DegreeOfFreedom = field(default=None)
    z: DegreeOfFreedom = field(default=None)
    roll: DegreeOfFreedom = field(default=None)
    pitch: DegreeOfFreedom = field(default=None)
    yaw: DegreeOfFreedom = field(default=None)
    x_vel: DegreeOfFreedom = field(default=None)
    y_vel: DegreeOfFreedom = field(default=None)

    translation_velocity_limits: float = field(default=0.6)
    rotation_velocity_limits: float = field(default=0.5)

    def __post_init__(self):
        self.x = self.x or self._world.create_degree_of_freedom(name=PrefixedName('x', self.name))
        self.y = self.y or self._world.create_degree_of_freedom(name=PrefixedName('y', self.name))
        self.z = self.z or self._world.create_degree_of_freedom(name=PrefixedName('z', self.name))

        self.roll = self.roll or self._world.create_degree_of_freedom(name=PrefixedName('roll', self.name))
        self.pitch = self.pitch or self._world.create_degree_of_freedom(name=PrefixedName('pitch', self.name))
        self.yaw = self.yaw or self._world.create_degree_of_freedom(
            name=PrefixedName('yaw', self.name),
            lower_limits={Derivatives.velocity: -self.rotation_velocity_limits},
            upper_limits={Derivatives.velocity: self.rotation_velocity_limits})

        self.x_vel = self.x_vel or self._world.create_degree_of_freedom(
            name=PrefixedName('x_vel', self.name),
            lower_limits={Derivatives.velocity: -self.translation_velocity_limits},
            upper_limits={Derivatives.velocity: self.translation_velocity_limits})
        self.y_vel = self.y_vel or self._world.create_degree_of_freedom(
            name=PrefixedName('y_vel', self.name),
            lower_limits={Derivatives.velocity: -self.translation_velocity_limits},
            upper_limits={Derivatives.velocity: self.translation_velocity_limits})
        self.active_dofs = [self.x_vel, self.y_vel, self.yaw]
        self.passive_dofs = [self.x, self.y, self.z, self.roll, self.pitch]

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
        self._world.state[Derivatives.position, self.x.state_idx] = position[0]
        self._world.state[Derivatives.position, self.y.state_idx] = position[1]
        self._world.state[Derivatives.position, self.z.state_idx] = position[2]
        self._world.state[Derivatives.position, self.roll.state_idx] = roll
        self._world.state[Derivatives.position, self.pitch.state_idx] = pitch
        self._world.state[Derivatives.position, self.yaw.state_idx] = yaw

    def update_state(self, dt: float) -> None:
        state = self._world.state
        state[Derivatives.position, self.x_vel.state_idx] = 0
        state[Derivatives.position, self.y_vel.state_idx] = 0

        x_vel = state[Derivatives.velocity, self.x_vel.state_idx]
        y_vel = state[Derivatives.velocity, self.y_vel.state_idx]
        delta = state[Derivatives.position, self.yaw.state_idx]
        state[Derivatives.velocity, self.x.state_idx] = (np.cos(delta) * x_vel - np.sin(delta) * y_vel)
        state[Derivatives.position, self.x.state_idx] += state[Derivatives.velocity, self.x.state_idx] * dt
        state[Derivatives.velocity, self.y.state_idx] = (np.sin(delta) * x_vel + np.cos(delta) * y_vel)
        state[Derivatives.position, self.y.state_idx] += state[Derivatives.velocity, self.y.state_idx] * dt

    def get_free_variable_names(self) -> List[PrefixedName]:
        return [self.x.name, self.y.name, self.yaw.name]
