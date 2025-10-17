from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from ormatic.dao import HasGeneric
from random_events.utils import SubclassJSONSerializer, recursive_subclasses
from typing_extensions import Self, Dict, Any, TypeVar, TYPE_CHECKING

from .connections import (
    FixedConnection,
    PrismaticConnection,
    RevoluteConnection,
    Connection6DoF,
    OmniDrive,
)
from .geometry import transformation_from_json, transformation_to_json
from .world_entity import Connection
from .. import spatial_types as cas
from ..datastructures.prefixed_name import PrefixedName
from ..spatial_types.spatial_types import TransformationMatrix
from ..spatial_types.symbol_manager import symbol_manager

if TYPE_CHECKING:
    from ..world import World

T = TypeVar("T")


@dataclass
class ConnectionFactory(HasGeneric[T], SubclassJSONSerializer, ABC):
    """
    Factory for creating connections.
    This class can be used to serialize connections indirectly.

    This class and its subclasses are needed for communication.
    These classes are serializable, alternative representations of connections such that other processes are able
    to semantically repeat the creation of a connection.

    Check the documentation of the original class for more information and the fields.
    """

    name: PrefixedName
    parent_name: PrefixedName
    child_name: PrefixedName
    parent_T_connection_expression: TransformationMatrix

    @classmethod
    def from_connection(cls, connection: Connection) -> Self:
        for factory in recursive_subclasses(cls) + [cls]:
            if factory.original_class() == connection.__class__:
                return factory._from_connection(connection)
        raise ValueError(f"Unknown connection type: {connection.name}")

    @classmethod
    @abstractmethod
    def _from_connection(cls, connection: Connection) -> Self:
        """
        Create a connection factory from a connection.

        :param connection: The connection to create the factory from.
        :return: The created connection factory.
        """
        raise NotImplementedError

    @abstractmethod
    def create(self, world: World) -> T:
        """
        Create the connection in a given world.

        :param world: The world in which to create the connection.
        :return: The created connection.
        """
        raise NotImplementedError

    def to_json(self) -> Dict[str, Any]:
        return {
            **super().to_json(),
            "name": self.name.to_json(),
            "parent_name": self.parent_name.to_json(),
            "child_name": self.child_name.to_json(),
            "parent_T_connection_expression": transformation_to_json(
                self.parent_T_connection_expression
            ),
        }


@dataclass
class FixedConnectionFactory(ConnectionFactory[FixedConnection]):

    @classmethod
    def _from_connection(cls, connection: Connection) -> Self:
        return cls(
            name=connection.name,
            parent_name=connection.parent.name,
            child_name=connection.child.name,
            parent_T_connection_expression=connection.parent_T_connection_expression,
        )

    def create(self, world: World) -> None:
        parent = world.get_kinematic_structure_entity_by_name(self.parent_name)
        child = world.get_kinematic_structure_entity_by_name(self.child_name)
        self.parent_T_connection_expression.reference_frame = parent
        connection = self.original_class()(
            parent=parent,
            child=child,
            name=self.name,
            parent_T_connection_expression=self.parent_T_connection_expression,
            _world=world,
        )
        world.add_connection(connection)

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(
            name=PrefixedName.from_json(data["name"]),
            parent_name=PrefixedName.from_json(data["parent_name"]),
            child_name=PrefixedName.from_json(data["child_name"]),
            parent_T_connection_expression=transformation_from_json(
                data["parent_T_connection_expression"]
            ),
        )


@dataclass
class ActiveConnection1DOFFactory(ConnectionFactory[T]):
    axis: cas.Vector3
    multiplier: float
    offset: float
    dof_name: PrefixedName

    @classmethod
    def _from_connection(cls, connection: PrismaticConnection) -> Self:
        return cls(
            name=connection.name,
            parent_name=connection.parent.name,
            child_name=connection.child.name,
            axis=connection.axis,
            multiplier=connection.multiplier,
            offset=connection.offset,
            dof_name=connection.dof.name,
            parent_T_connection_expression=connection.parent_T_connection_expression,
        )

    def create(self, world: World) -> None:
        parent = world.get_kinematic_structure_entity_by_name(self.parent_name)
        child = world.get_kinematic_structure_entity_by_name(self.child_name)
        self.parent_T_connection_expression.reference_frame = parent

        connection = self.original_class()(
            parent=parent,
            child=child,
            name=self.name,
            axis=self.axis,
            multiplier=self.multiplier,
            offset=self.offset,
            dof_name=self.dof_name,
            parent_T_connection_expression=self.parent_T_connection_expression,
            _world=world,
        )
        world.add_connection(connection)
        # The init of the connection adds a new transformation to the origin expression but since this is already done \
        # to this origin we just use it as is
        connection.parent_T_connection_expression = self.parent_T_connection_expression

    def to_json(self) -> Dict[str, Any]:
        return {
            **super().to_json(),
            "axis": symbol_manager.evaluate_expr(self.axis).tolist(),
            "multiplier": self.multiplier,
            "offset": self.offset,
            "dof": self.dof_name.to_json(),
        }

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(
            name=PrefixedName.from_json(data["name"]),
            parent_name=PrefixedName.from_json(data["parent_name"]),
            child_name=PrefixedName.from_json(data["child_name"]),
            axis=cas.Vector3.from_iterable(data["axis"]),
            multiplier=data["multiplier"],
            offset=data["offset"],
            dof_name=PrefixedName.from_json(data["dof"]),
            parent_T_connection_expression=transformation_from_json(
                data["parent_T_connection_expression"]
            ),
        )


@dataclass
class RevoluteConnectionFactory(ActiveConnection1DOFFactory[RevoluteConnection]): ...


@dataclass
class PrismaticConnectionFactory(ActiveConnection1DOFFactory[PrismaticConnection]): ...


@dataclass
class Connection6DoFFactory(ConnectionFactory[Connection6DoF]):
    x_name: PrefixedName
    y_name: PrefixedName
    z_name: PrefixedName
    qx_name: PrefixedName
    qy_name: PrefixedName
    qz_name: PrefixedName
    qw_name: PrefixedName

    @classmethod
    def _from_connection(cls, connection: Connection6DoF) -> Self:
        return cls(
            name=connection.name,
            parent_name=connection.parent.name,
            child_name=connection.child.name,
            x_name=connection.x.name,
            y_name=connection.y.name,
            z_name=connection.z.name,
            qx_name=connection.qx.name,
            qy_name=connection.qy.name,
            qz_name=connection.qz.name,
            qw_name=connection.qw.name,
            parent_T_connection_expression=connection.parent_T_connection_expression,
        )

    def create(self, world: World) -> None:
        parent = world.get_kinematic_structure_entity_by_name(self.parent_name)
        child = world.get_kinematic_structure_entity_by_name(self.child_name)
        self.parent_T_connection_expression.reference_frame = parent
        connection = self.original_class()(
            parent=parent,
            child=child,
            name=self.name,
            x=world.get_degree_of_freedom_by_name(self.x_name),
            y=world.get_degree_of_freedom_by_name(self.y_name),
            z=world.get_degree_of_freedom_by_name(self.z_name),
            qx=world.get_degree_of_freedom_by_name(self.qx_name),
            qy=world.get_degree_of_freedom_by_name(self.qy_name),
            qz=world.get_degree_of_freedom_by_name(self.qz_name),
            qw=world.get_degree_of_freedom_by_name(self.qw_name),
            parent_T_connection_expression=self.parent_T_connection_expression,
            _world=world,
        )
        world.add_connection(connection)
        # The init of the  connection adds a new transformation to the origin expression but since this is already done \
        # to this origin we just use it as is
        connection.parent_T_connection_expression = self.parent_T_connection_expression

    def to_json(self) -> Dict[str, Any]:
        return {
            **super().to_json(),
            "x": self.x_name.to_json(),
            "y": self.y_name.to_json(),
            "z": self.z_name.to_json(),
            "qx": self.qx_name.to_json(),
            "qy": self.qy_name.to_json(),
            "qz": self.qz_name.to_json(),
            "qw": self.qw_name.to_json(),
        }

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(
            name=PrefixedName.from_json(data["name"]),
            parent_name=PrefixedName.from_json(data["parent_name"]),
            child_name=PrefixedName.from_json(data["child_name"]),
            x_name=PrefixedName.from_json(data["x"]),
            y_name=PrefixedName.from_json(data["y"]),
            z_name=PrefixedName.from_json(data["z"]),
            qx_name=PrefixedName.from_json(data["qx"]),
            qy_name=PrefixedName.from_json(data["qy"]),
            qz_name=PrefixedName.from_json(data["qz"]),
            qw_name=PrefixedName.from_json(data["qw"]),
            parent_T_connection_expression=transformation_from_json(
                data["parent_T_connection_expression"]
            ),
        )


@dataclass
class OmniDriveFactory(ConnectionFactory[OmniDrive]):

    x_name: PrefixedName
    y_name: PrefixedName
    z_name: PrefixedName
    roll_name: PrefixedName
    pitch_name: PrefixedName
    yaw_name: PrefixedName
    x_velocity_name: PrefixedName
    y_velocity_name: PrefixedName
    translation_velocity_limits: float = field(default=0.6)
    rotation_velocity_limits: float = field(default=0.5)

    @classmethod
    def _from_connection(cls, connection: OmniDrive) -> Self:
        return cls(
            name=connection.name,
            parent_name=connection.parent.name,
            child_name=connection.child.name,
            x_name=connection.x.name,
            y_name=connection.y.name,
            z_name=connection.z.name,
            roll_name=connection.roll.name,
            pitch_name=connection.pitch.name,
            yaw_name=connection.yaw.name,
            x_velocity_name=connection.x_vel.name,
            y_velocity_name=connection.y_vel.name,
            translation_velocity_limits=connection.translation_velocity_limits,
            rotation_velocity_limits=connection.rotation_velocity_limits,
            parent_T_connection_expression=connection.parent_T_connection_expression,
        )

    def create(self, world: World) -> None:
        parent = world.get_kinematic_structure_entity_by_name(self.parent_name)
        child = world.get_kinematic_structure_entity_by_name(self.child_name)
        self.parent_T_connection_expression.reference_frame = parent
        connection = self.original_class()(
            parent=parent,
            child=child,
            name=self.name,
            x=world.get_degree_of_freedom_by_name(self.x_name),
            y=world.get_degree_of_freedom_by_name(self.y_name),
            z=world.get_degree_of_freedom_by_name(self.z_name),
            roll=world.get_degree_of_freedom_by_name(self.roll_name),
            pitch=world.get_degree_of_freedom_by_name(self.pitch_name),
            yaw=world.get_degree_of_freedom_by_name(self.yaw_name),
            x_vel=world.get_degree_of_freedom_by_name(self.x_velocity_name),
            y_vel=world.get_degree_of_freedom_by_name(self.y_velocity_name),
            translation_velocity_limits=self.translation_velocity_limits,
            rotation_velocity_limits=self.rotation_velocity_limits,
            parent_T_connection_expression=self.parent_T_connection_expression,
            _world=world,
        )
        world.add_connection(connection)
        # The init of the  connection adds a new transformation to the origin expression but since this is already done \
        # to this origin we just use it as is
        connection.parent_T_connection_expression = self.parent_T_connection_expression

    def to_json(self) -> Dict[str, Any]:
        return {
            **super().to_json(),
            "x": self.x_name.to_json(),
            "y": self.y_name.to_json(),
            "z": self.z_name.to_json(),
            "roll": self.roll_name.to_json(),
            "pitch": self.pitch_name.to_json(),
            "yaw": self.yaw_name.to_json(),
            "x_velocity": self.x_velocity_name.to_json(),
            "y_velocity": self.y_velocity_name.to_json(),
            "translation_velocity_limits": self.translation_velocity_limits,
            "rotation_velocity_limits": self.rotation_velocity_limits,
        }

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(
            name=PrefixedName.from_json(data["name"]),
            parent_name=PrefixedName.from_json(data["parent_name"]),
            child_name=PrefixedName.from_json(data["child_name"]),
            x_name=PrefixedName.from_json(data["x"]),
            y_name=PrefixedName.from_json(data["y"]),
            z_name=PrefixedName.from_json(data["z"]),
            roll_name=PrefixedName.from_json(data["roll"]),
            pitch_name=PrefixedName.from_json(data["pitch"]),
            yaw_name=PrefixedName.from_json(data["yaw"]),
            x_velocity_name=PrefixedName.from_json(data["x_velocity"]),
            y_velocity_name=PrefixedName.from_json(data["y_velocity"]),
            translation_velocity_limits=data["translation_velocity_limits"],
            rotation_velocity_limits=data["rotation_velocity_limits"],
            parent_T_connection_expression=transformation_from_json(
                data["parent_T_connection_expression"]
            ),
        )
