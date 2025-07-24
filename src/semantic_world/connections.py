from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Tuple, TYPE_CHECKING

import numpy as np

from . import spatial_types as cas
from .degree_of_freedom import DegreeOfFreedom
from .prefixed_name import PrefixedName
from .spatial_types.derivatives import Derivatives, DerivativeMap
from .spatial_types.math import quaternion_from_rotation_matrix
from .types import NpMatrix4x4
from .world_entity import Connection

if TYPE_CHECKING:
    from .world import World


class Has1DOFState:
    """
    Mixin class that implements state access for connections with 1 degree of freedom.
    """

    dof: DegreeOfFreedom
    _world: World

    @property
    def position(self) -> float:
        return self._world.state[self.dof.name].position

    @position.setter
    def position(self, value: float) -> None:
        self._world.state[self.dof.name].position = value
        self._world.notify_state_change()

    @property
    def velocity(self) -> float:
        return self._world.state[self.dof.name].velocity

    @velocity.setter
    def velocity(self, value: float) -> None:
        self._world.state[self.dof.name].velocity = value
        self._world.notify_state_change()

    @property
    def acceleration(self) -> float:
        return self._world.state[self.dof.name].acceleration

    @acceleration.setter
    def acceleration(self, value: float) -> None:
        self._world.state[self.dof.name].acceleration = value
        self._world.notify_state_change()

    @property
    def jerk(self) -> float:
        return self._world.state[self.dof.name].jerk

    @jerk.setter
    def jerk(self, value: float) -> None:
        self._world.state[self.dof.name].jerk = value
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
class FixedConnection(Connection):
    """
    Has 0 degrees of freedom.
    """


@dataclass
class ActiveConnection(Connection):
    """
    Has one or more degrees of freedom that can be actively controlled, e.g., robot joints.
    """

    @property
    def active_dofs(self) -> List[DegreeOfFreedom]:
        return []


@dataclass
class PassiveConnection(Connection):
    """
    Has one or more degrees of freedom that cannot be actively controlled.
    Useful if a transformation is only tracked, e.g., the robot's localization.
    """

    @property
    def passive_dofs(self) -> List[DegreeOfFreedom]:
        return []


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
        self._post_init_world_part()

        motor_expression = self.dof.symbols.position * self.multiplier + self.offset
        translation_axis = cas.Vector3(self.axis) * motor_expression
        parent_T_child = cas.TransformationMatrix.from_xyz_rpy(x=translation_axis[0],
                                                               y=translation_axis[1],
                                                               z=translation_axis[2])
        self.origin_expression = self.origin_expression.dot(parent_T_child)
        self.origin_expression.reference_frame = self.parent
        self.origin_expression.child_frame = self.child

    def _post_init_with_world(self):
        self.dof = self.dof or self._world.create_degree_of_freedom(name=PrefixedName(str(self.name)))

    def _post_init_without_world(self):
        if self.dof is None:
            raise ValueError("PrismaticConnection cannot be created without a world "
                             "if the dof is not provided.")

    @property
    def active_dofs(self) -> List[DegreeOfFreedom]:
        return [self.dof]

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
        self._post_init_world_part()

        motor_expression = self.dof.symbols.position * self.multiplier + self.offset
        rotation_axis = cas.Vector3(self.axis)
        parent_R_child = cas.RotationMatrix.from_axis_angle(rotation_axis, motor_expression)
        self.origin_expression = self.origin_expression.dot(cas.TransformationMatrix(parent_R_child))
        self.origin_expression.reference_frame = self.parent
        self.origin_expression.child_frame = self.child

    def _post_init_with_world(self):
        self.dof = self.dof or self._world.create_degree_of_freedom(name=PrefixedName(str(self.name)))

    def _post_init_without_world(self):
        if self.dof is None:
            raise ValueError("RevoluteConnection cannot be created without a world "
                             "if the dof is not provided.")

    @property
    def active_dofs(self) -> List[DegreeOfFreedom]:
        return [self.dof]

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
        super().__post_init__()
        self._post_init_world_part()
        parent_P_child = cas.Point3((self.x.symbols.position,
                                     self.y.symbols.position,
                                     self.z.symbols.position))
        parent_R_child = cas.Quaternion((self.qx.symbols.position,
                                         self.qy.symbols.position,
                                         self.qz.symbols.position,
                                         self.qw.symbols.position)).to_rotation_matrix()
        self.origin_expression = cas.TransformationMatrix.from_point_rotation_matrix(point=parent_P_child,
                                                                                     rotation_matrix=parent_R_child,
                                                                                     reference_frame=self.parent,
                                                                                     child_frame=self.child)

    def _post_init_with_world(self):
        if all(dof is None for dof in self.passive_dofs):
            self.x = self._world.create_degree_of_freedom(name=PrefixedName('x', str(self.name)))
            self.y = self._world.create_degree_of_freedom(name=PrefixedName('y', str(self.name)))
            self.z = self._world.create_degree_of_freedom(name=PrefixedName('z', str(self.name)))
            self.qx = self._world.create_degree_of_freedom(name=PrefixedName('qx', str(self.name)))
            self.qy = self._world.create_degree_of_freedom(name=PrefixedName('qy', str(self.name)))
            self.qz = self._world.create_degree_of_freedom(name=PrefixedName('qz', str(self.name)))
            self.qw = self._world.create_degree_of_freedom(name=PrefixedName('qw', str(self.name)))
            self._world.state[self.qw.name].position = 1.
        elif any(dof is None for dof in self.passive_dofs):
            raise ValueError("Connection6DoF can only be created "
                             "if you provide all or none of the passive degrees of freedom")

    def _post_init_without_world(self):
        if any(dof is None for dof in self.passive_dofs):
            raise ValueError("Connection6DoF cannot be created without a world "
                             "if some passive degrees of freedom are not provided.")

    @property
    def passive_dofs(self) -> List[DegreeOfFreedom]:
        return [self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw]

    @property
    def origin(self) -> NpMatrix4x4:
        return super().origin

    @origin.setter
    def origin(self, transformation: NpMatrix4x4) -> None:
        orientation = quaternion_from_rotation_matrix(transformation)
        self._world.state[self.x.name].position = transformation[0, 3]
        self._world.state[self.y.name].position = transformation[1, 3]
        self._world.state[self.z.name].position = transformation[2, 3]
        self._world.state[self.qx.name].position = orientation[0]
        self._world.state[self.qy.name].position = orientation[1]
        self._world.state[self.qz.name].position = orientation[2]
        self._world.state[self.qw.name].position = orientation[3]
        self._world.notify_state_change()


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
        super().__post_init__()
        self._post_init_world_part()
        odom_T_bf = cas.TransformationMatrix.from_xyz_rpy(x=self.x.symbols.position,
                                                          y=self.y.symbols.position,
                                                          yaw=self.yaw.symbols.position)
        bf_T_bf_vel = cas.TransformationMatrix.from_xyz_rpy(x=self.x_vel.symbols.position,
                                                            y=self.y_vel.symbols.position)
        bf_vel_T_bf = cas.TransformationMatrix.from_xyz_rpy(x=0,
                                                            y=0,
                                                            z=self.z.symbols.position,
                                                            roll=self.roll.symbols.position,
                                                            pitch=self.pitch.symbols.position,
                                                            yaw=0)
        self.origin_expression = odom_T_bf.dot(bf_T_bf_vel).dot(bf_vel_T_bf)
        self.origin_expression.reference_frame = self.parent
        self.origin_expression.child_frame = self.child

    def _post_init_with_world(self):
        if all(dof is None for dof in self.dofs):
            stringified_name = str(self.name)
            lower_translation_limits = DerivativeMap()
            lower_translation_limits.velocity = -self.translation_velocity_limits
            upper_translation_limits = DerivativeMap()
            upper_translation_limits.velocity = self.translation_velocity_limits
            lower_rotation_limits = DerivativeMap()
            lower_rotation_limits.velocity = -self.rotation_velocity_limits
            upper_rotation_limits = DerivativeMap()
            upper_rotation_limits.velocity = self.rotation_velocity_limits

            self.x = self._world.create_degree_of_freedom(name=PrefixedName('x', stringified_name))
            self.y = self._world.create_degree_of_freedom(name=PrefixedName('y', stringified_name))
            self.z = self._world.create_degree_of_freedom(name=PrefixedName('z', stringified_name))

            self.roll = self._world.create_degree_of_freedom(name=PrefixedName('roll', stringified_name))
            self.pitch = self._world.create_degree_of_freedom(name=PrefixedName('pitch', stringified_name))
            self.yaw = self._world.create_degree_of_freedom(
                name=PrefixedName('yaw', stringified_name),
                lower_limits=lower_rotation_limits,
                upper_limits=upper_rotation_limits)

            self.x_vel = self._world.create_degree_of_freedom(
                name=PrefixedName('x_vel', stringified_name),
                lower_limits=lower_translation_limits,
                upper_limits=upper_translation_limits)
            self.y_vel = self._world.create_degree_of_freedom(
                name=PrefixedName('y_vel', stringified_name),
                lower_limits=lower_translation_limits,
                upper_limits=upper_translation_limits)
        elif any(dof is None for dof in self.passive_dofs):
            raise ValueError("OmniDrive can only be created "
                             "if you provide all or none of the passive degrees of freedom")

    def _post_init_without_world(self):
        if any(dof is None for dof in self.dofs):
            raise ValueError("OmniDrive cannot be created without a world "
                             "if some passive degrees of freedom are not provided.")

    @property
    def active_dofs(self) -> List[DegreeOfFreedom]:
        return [self.x_vel, self.y_vel, self.yaw]

    @property
    def passive_dofs(self) -> List[DegreeOfFreedom]:
        return [self.x, self.y, self.z, self.roll, self.pitch]

    @property
    def dofs(self) -> List[DegreeOfFreedom]:
        return self.active_dofs + self.passive_dofs

    def update_state(self, dt: float) -> None:
        state = self._world.state
        state[self.x_vel.name].position = 0
        state[self.y_vel.name].position = 0

        x_vel = state[self.x_vel.name].velocity
        y_vel = state[self.y_vel.name].velocity
        delta = state[self.yaw.name].position
        state[self.x.name].velocity = (np.cos(delta) * x_vel - np.sin(delta) * y_vel)
        state[self.x.name].position += state[self.x.name].velocity * dt
        state[self.y.name].velocity = (np.sin(delta) * x_vel + np.cos(delta) * y_vel)
        state[self.y.name].position += state[self.y.name].velocity * dt

    def get_free_variable_names(self) -> List[PrefixedName]:
        return [self.x.name, self.y.name, self.yaw.name]
