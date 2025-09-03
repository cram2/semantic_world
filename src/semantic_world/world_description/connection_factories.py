from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing_extensions import Self, Dict, Any, TypeVar

from ormatic.dao import HasGeneric
from random_events.utils import SubclassJSONSerializer, recursive_subclasses

from .connections import (
    FixedConnection,
    PrismaticConnection,
    RevoluteConnection,
    Connection6DoF,
)
from .geometry import transformation_from_json, transformation_to_json
from ..datastructures.prefixed_name import PrefixedName
from ..spatial_types.spatial_types import TransformationMatrix
from ..spatial_types.symbol_manager import symbol_manager
from ..world import World
from .world_entity import Connection
from .. import spatial_types as cas

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
    origin_expression: TransformationMatrix

    @classmethod
    def from_connection(cls, connection: Connection) -> Self:
        for factory in recursive_subclasses(cls):
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
            "origin_expression": transformation_to_json(self.origin_expression),
        }


@dataclass
class FixedConnectionFactory(ConnectionFactory[FixedConnection]):

    @classmethod
    def _from_connection(cls, connection: Connection) -> Self:
        return cls(
            name=connection.name,
            parent_name=connection.parent.name,
            child_name=connection.child.name,
            origin_expression=connection.origin_expression,
        )

    def create(self, world: World) -> Connection:
        parent = world.get_kinematic_structure_entity_by_name(self.parent_name)
        child = world.get_kinematic_structure_entity_by_name(self.child_name)
        return self.original_class()(
            parent=parent,
            child=child,
            name=self.name,
            origin_expression=self.origin_expression,
            _world=world,
        )

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(
            name=PrefixedName.from_json(data["name"]),
            parent_name=PrefixedName.from_json(data["parent_name"]),
            child_name=PrefixedName.from_json(data["child_name"]),
            origin_expression=transformation_from_json(data["origin_expression"]),
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
            origin_expression=connection.origin_expression,
        )

    def create(self, world: World) -> Connection:
        parent = world.get_kinematic_structure_entity_by_name(self.parent_name)
        child = world.get_kinematic_structure_entity_by_name(self.child_name)
        # print(self.dof)
        # world.add_degree_of_freedom(self.dof)
        return self.original_class()(
            parent=parent,
            child=child,
            name=self.name,
            axis=self.axis,
            multiplier=self.multiplier,
            offset=self.offset,
            dof=world.get_degree_of_freedom_by_name(self.dof_name.name),
            origin_expression=self.origin_expression,
            _world=world,
        )

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
            origin_expression=transformation_from_json(data["origin_expression"]),
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
            origin_expression=connection.origin_expression,
        )

    def create(self, world: World) -> Connection6DoF:
        parent = world.get_kinematic_structure_entity_by_name(self.parent_name)
        child = world.get_kinematic_structure_entity_by_name(self.child_name)
        return self.original_class()(
            parent=parent,
            child=child,
            name=self.name,
            x=world.get_degree_of_freedom_by_name(self.x_name.name),
            y=world.get_degree_of_freedom_by_name(self.y_name.name),
            z=world.get_degree_of_freedom_by_name(self.z_name.name),
            qx=world.get_degree_of_freedom_by_name(self.qx_name.name),
            qy=world.get_degree_of_freedom_by_name(self.qy_name.name),
            qz=world.get_degree_of_freedom_by_name(self.qz_name.name),
            qw=world.get_degree_of_freedom_by_name(self.qw_name.name),
            origin_expression=self.origin_expression,
            _world=world,
        )

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
            origin_expression=transformation_from_json(data["origin_expression"]),
        )
