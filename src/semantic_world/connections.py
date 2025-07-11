from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

from . import spatial_types as cas
from .degree_of_freedom import DegreeOfFreedom
from .prefixed_name import PrefixedName
from .spatial_types.derivatives import Derivatives
from .spatial_types.math import quaternion_from_rotation_matrix
from .world_entity import Connection


class Has1DOFState:
    """
    Mixin class that implements state access for connections with 1 degree of freedom.
    """

    @property
    def position(self) -> float:
        return self._world.state[Derivatives.position, self.dof.state_idx]

    @position.setter
    def position(self, value: float) -> None:
        self._world.state[Derivatives.position, self.dof.state_idx] = value
        self._world.notify_state_change()

    @property
    def velocity(self) -> float:
        return self._world.state[Derivatives.velocity, self.dof.state_idx]

    @velocity.setter
    def velocity(self, value: float) -> None:
        self._world.state[Derivatives.velocity, self.dof.state_idx] = value
        self._world.notify_state_change()

    @property
    def acceleration(self) -> float:
        return self._world.state[Derivatives.acceleration, self.dof.state_idx]

    @acceleration.setter
    def acceleration(self, value: float) -> None:
        self._world.state[Derivatives.acceleration, self.dof.state_idx] = value
        self._world.notify_state_change()

    @property
    def jerk(self) -> float:
        return self._world.state[Derivatives.jerk, self.dof.state_idx]

    @jerk.setter
    def jerk(self, value: float) -> None:
        self._world.state[Derivatives.jerk, self.dof.state_idx] = value
        self._world.notify_state_change()


@dataclass
class FixedConnection(Connection):
    """
    Has 0 degrees of freedom.
    """


@dataclass
class ActiveConnection(Connection):
    """
    Has one or more degrees of freedom that can be actively controlled, e.g., robot joints.
    """
    active_dofs: List[DegreeOfFreedom] = field(default_factory=list, init=False)


@dataclass
class PassiveConnection(Connection):
    """
    Has one or more degrees of freedom that cannot be actively controlled.
    Useful if a transformation is only tracked, e.g., the robot's localization.
    """
    passive_dofs: List[DegreeOfFreedom] = field(default_factory=list, init=False)


@dataclass
class UnitVector:
    """
    Represents a unit vector which is always of size 1.
    """

    x: float
    y: float
    z: float

    def __post_init__(self):
        self.normalize()

    def normalize(self):
        length = self.length
        self.x /= length
        self.y /= length
        self.z /= length

    @property
    def length(self):
        return np.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def __getitem__(self, item: int) -> float:
        if item == 0:
            return self.x
        if item == 1:
            return self.y
        if item == 2:
            return self.z
        raise IndexError

    def as_tuple(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.z

    @classmethod
    def X(cls):
        return cls(1, 0, 0)

    @classmethod
    def Y(cls):
        return cls(0, 1, 0)

    @classmethod
    def Z(cls):
        return cls(0, 0, 1)


@dataclass
class PrismaticConnection(ActiveConnection, Has1DOFState):
    """
    Allows the movement along an axis.
    """

    axis: UnitVector = field(kw_only=True)
    """
    Connection moves along this axis, should be a unit vector.
    The axis is defined relative to the local reference frame of the parent body.
    """

    multiplier: float = 1.0
    """
    Movement along the axis is multiplied by this value. Useful if Connections share DoFs.
    """

    offset: float = 0.0
    """
    Movement along the axis is offset by this value. Useful if Connections share DoFs.
    """

    dof: DegreeOfFreedom = field(default=None)
    """
    Degree of freedom to control movement along the axis.
    """

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
        self.origin_expression = self.origin_expression.dot(parent_T_child)

    def __hash__(self):
        return hash((self.parent, self.child))


@dataclass
class RevoluteConnection(ActiveConnection, Has1DOFState):
    """
    Allows rotation about an axis.
    """

    axis: UnitVector = field(kw_only=True)
    """
    Connection rotates about this axis, should be a unit vector.
    The axis is defined relative to the local reference frame of the parent body.
    """

    multiplier: float = 1.0
    """
    Rotation about the axis is multiplied by this value. Useful if Connections share DoFs.
    """

    offset: float = 0.0
    """
    Rotation about the axis is offset by this value. Useful if Connections share DoFs.
    """

    dof: DegreeOfFreedom = field(default=None)
    """
    Degree of freedom to control rotation about the axis.
    """

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
        self.origin_expression = self.origin_expression.dot(cas.TransformationMatrix(parent_R_child))

    def __hash__(self):
        return hash((self.parent, self.child))


@dataclass
class Connection6DoF(PassiveConnection):
    """
    Has full 6 degrees of freedom, that cannot be actively controlled.
    Useful for synchronizing with transformations from external providers.
    """

    x: DegreeOfFreedom = field(default=None)
    """
    Displacement of child body with respect to parent body along the x-axis.
    """
    y: DegreeOfFreedom = field(default=None)
    """
    Displacement of child body with respect to parent body along the y-axis.
    """
    z: DegreeOfFreedom = field(default=None)
    """
    Displacement of child body with respect to parent body along the z-axis.
    """

    qx: DegreeOfFreedom = field(default=None)
    qy: DegreeOfFreedom = field(default=None)
    qz: DegreeOfFreedom = field(default=None)
    qw: DegreeOfFreedom = field(default=None)
    """
    Rotation of child body with respect to parent body represented as a quaternion.
    """

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
        self.origin_expression = cas.TransformationMatrix.from_point_rotation_matrix(point=parent_P_child,
                                                                          rotation_matrix=parent_R_child,
                                                                          reference_frame=self.parent.name,
                                                                          child_frame=self.child.name)

    @property
    def origin(self) -> np.ndarray:
        return super().origin

    @origin.setter
    def origin(self, transformation: np.ndarray) -> None:
        orientation = quaternion_from_rotation_matrix(transformation)
        self._world.state[Derivatives.position][self.x.state_idx] = transformation[0, 3]
        self._world.state[Derivatives.position][self.y.state_idx] = transformation[1, 3]
        self._world.state[Derivatives.position][self.z.state_idx] = transformation[2, 3]
        self._world.state[Derivatives.position][self.qx.state_idx] = orientation[0]
        self._world.state[Derivatives.position][self.qy.state_idx] = orientation[1]
        self._world.state[Derivatives.position][self.qz.state_idx] = orientation[2]
        self._world.state[Derivatives.position][self.qw.state_idx] = orientation[3]
        self._world.notify_state_change()


class HasUpdateState(ABC):
    """
    Mixin class for connections that need state updated which are not trivial integrations.
    Typically needed for connections that use active and passive degrees of freedom.
    Look at OmniDrive for an example usage.
    """

    @abstractmethod
    def update_state(self, dt: float) -> None:
        """
        Allows the connection to update the state of its dofs.
        An integration update for active dofs will have happened before this method is called.
        Write directly into self._world.state, but don't touch dofs that don't belong to this connection.
        :param dt: Time passed since last update.
        """
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
        self.origin_expression = odom_T_bf.dot(bf_T_bf_vel).dot(bf_vel_T_bf)

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
