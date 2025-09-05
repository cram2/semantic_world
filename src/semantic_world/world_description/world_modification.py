from __future__ import annotations

from abc import abstractmethod, ABC
from dataclasses import dataclass
from typing_extensions import List, Dict, Any, Self, Optional, Callable, ClassVar

from random_events.utils import SubclassJSONSerializer, recursive_subclasses

from semantic_world.world_description.connection_factories import ConnectionFactory
from semantic_world.world_description.degree_of_freedom import DegreeOfFreedom
from semantic_world.datastructures.prefixed_name import PrefixedName
from semantic_world.world import World, FunctionStack
from semantic_world.world_description.world_entity import Body


@dataclass
class UnknownWorldModification(Exception):
    """
    Raised when an unknown world modification is attempted.
    """

    call: Callable
    kwargs: Dict[str, Any]

    def __post_init__(self):
        super().__init__(
            " Make sure that world modifications are atomic and that every atomic modification is "
            "represented by exactly one subclass of WorldModelModification."
            "This module might be incomplete, you can help by expanding it."
        )


@dataclass
class WorldModelModification(SubclassJSONSerializer, ABC):
    """
    A record of a modification to the model (structure) of the world.
    This includes add/remove body and add/remove connection.
    """

    function_name: ClassVar[Optional[str]] = None
    """
    The name of the function that this modifications records.
    """

    @abstractmethod
    def apply(self, world: World):
        """
        Apply this change to the given world.

        :param world: The world to modify.
        """

    @classmethod
    @abstractmethod
    def _from_kwargs(cls, kwargs: Dict[str, Any]) -> Self:
        """
        Factory to construct this change from the kwargs of a function call.

        :param kwargs: The kwargs of the function call.
        :return: A new instance.
        """
        raise NotImplementedError

    @classmethod
    def from_world_modification(cls, call: Callable, kwargs: Dict[str, Any]) -> Self:
        """
        Factory to construct this change from a function and its kwargs.
        These function - kwargs pairs are usually extracted from :attr:`World._atomic_modifications`.

        This method searches through all subclasses and compares their :attr:`WorldModelModification.function_name`
        with the given call.

        :param call: The called function.
        :param kwargs: The kwargs of the function call.
        :return: A new instance.
        """

        for subclass in recursive_subclasses(cls):
            if subclass.function_name == call.__name__:
                return subclass._from_kwargs(kwargs)
        raise UnknownWorldModification(call, kwargs)


@dataclass
class AddBodyModification(WorldModelModification):
    """
    Addition of a body to the world.
    """

    body: Body
    """
    The body that was added.
    """

    function_name = World._add_kinematic_structure_entity.__name__

    @classmethod
    def _from_kwargs(cls, kwargs: Dict[str, Any]):
        return cls(kwargs["kinematic_structure_entity"])

    def apply(self, world: World):
        world.add_kinematic_structure_entity(self.body)

    def to_json(self):
        return {**super().to_json(), "body": self.body.to_json()}

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(body=Body.from_json(data["body"]))


@dataclass
class RemoveBodyModification(WorldModelModification):
    """
    Removal of a body from the world.
    """

    body_name: PrefixedName
    """
    The name of the body that was removed.
    """

    function_name = World._remove_kinematic_structure_entity.__name__

    @classmethod
    def _from_kwargs(cls, kwargs: Dict[str, Any]):
        return cls(kwargs["kinematic_structure_entity"].name)

    def apply(self, world: World):
        world.remove_kinematic_structure_entity(
            world.get_kinematic_structure_entity_by_name(self.body_name)
        )

    def to_json(self) -> Dict[str, Any]:
        return {**super().to_json(), "body_name": self.body_name.to_json()}

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(body_name=PrefixedName.from_json(data["body_name"]))


@dataclass
class AddConnectionModification(WorldModelModification):
    """
    Addition of a connection to the world.
    """

    connection_factory: ConnectionFactory
    """
    The connection factory that can be used to create the added connection again.
    """

    function_name = World._add_connection.__name__

    @classmethod
    def _from_kwargs(cls, kwargs: Dict[str, Any]):
        return cls(ConnectionFactory.from_connection(kwargs["connection"]))

    def apply(self, world: World):
        connection = self.connection_factory.create(world)
        world.add_connection(connection)

    def to_json(self):
        return {
            **super().to_json(),
            "connection_factory": self.connection_factory.to_json(),
        }

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(
            connection_factory=ConnectionFactory.from_json(data["connection_factory"])
        )


@dataclass
class RemoveConnectionModification(WorldModelModification):
    """
    Removal of a connection from the world.
    """

    connection_name: PrefixedName
    """
    The name of the connection that was removed.
    """

    function_name = World._remove_connection.__name__

    @classmethod
    def _from_kwargs(cls, kwargs: Dict[str, Any]):
        return cls(kwargs["connection"].name)

    def apply(self, world: World):
        world._remove_connection(world.get_connection_by_name(self.connection_name))

    def to_json(self):
        return {
            **super().to_json(),
            "connection_name": self.connection_name.to_json(),
        }

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(connection_name=PrefixedName.from_json(data["connection_name"]))


@dataclass
class AddDegreeOfFreedomModification(WorldModelModification):
    """
    Addition of a degree of freedom to the world.
    """

    dof: DegreeOfFreedom
    """
    The degree of freedom that was added.
    """

    function_name = World._add_degree_of_freedom.__name__

    @classmethod
    def _from_kwargs(cls, kwargs: Dict[str, Any]):
        return cls(dof=kwargs["dof"])

    def apply(self, world: World):
        world.add_degree_of_freedom(self.dof)

    def to_json(self):
        return {
            **super().to_json(),
            "dof": self.dof.to_json(),
        }

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(dof=DegreeOfFreedom.from_json(data["dof"]))


@dataclass
class RemoveDegreeOfFreedomModification(WorldModelModification):
    dof_name: PrefixedName

    function_name = World.remove_degree_of_freedom.__name__

    @classmethod
    def _from_kwargs(cls, kwargs: Dict[str, Any]):
        return cls(dof_name=kwargs["dof"].name)

    def apply(self, world: World):
        world.remove_degree_of_freedom(
            world.get_degree_of_freedom_by_name(self.dof_name)
        )

    def to_json(self):
        return {
            **super().to_json(),
            "dof": self.dof_name.to_json(),
        }

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(dof_name=PrefixedName.from_json(data["dof"]))


@dataclass
class WorldModelModificationBlock(SubclassJSONSerializer):
    """
    A sequence of WorldModelModifications that were applied to the world within one `with world.modify_world()` context.
    """

    modifications: List[WorldModelModification]
    """
    The list of modifications to apply to the world.
    """

    def apply(self, world: World):
        with world.modify_world():
            for modification in self.modifications:
                modification.apply(world)

    @classmethod
    def from_modifications(cls, modifications: FunctionStack):
        return cls(
            [
                WorldModelModification.from_world_modification(call, kwargs)
                for call, kwargs in modifications
            ]
        )

    def to_json(self):
        return {
            **super().to_json(),
            "modifications": [m.to_json() for m in self.modifications],
        }

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls([WorldModelModification.from_json(d) for d in data["modifications"]])
