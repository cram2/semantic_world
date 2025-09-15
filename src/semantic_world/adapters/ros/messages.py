from abc import ABC
from dataclasses import dataclass
from functools import lru_cache
from typing import Dict, Any, Self, List

from random_events.utils import SubclassJSONSerializer

from semantic_world.datastructures.prefixed_name import PrefixedName
from semantic_world.world_description.world_modification import (
    WorldModelModificationBlock,
)


@dataclass
class MetaData(SubclassJSONSerializer):
    """
    Class for data describing the origin of a message.
    """

    node_name: str
    """
    The name of the node that published this message
    """

    process_id: int
    """
    The id of the process that published this message
    """

    object_id: int
    """
    The id of the object in the process that issues this publishing call
    """

    @lru_cache(maxsize=None)
    def to_json(self) -> Dict[str, Any]:
        return {
            **super().to_json(),
            "node_name": self.node_name,
            "process_id": self.process_id,
            "object_id": self.object_id,
        }

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(
            node_name=data["node_name"],
            process_id=data["process_id"],
            object_id=data["object_id"],
        )

    def __hash__(self):
        return hash((self.node_name, self.process_id, self.object_id))


@dataclass
class Message(SubclassJSONSerializer, ABC):

    meta_data: MetaData
    """
    Message origin meta data.
    """

    def to_json(self) -> Dict[str, Any]:
        return {
            **super().to_json(),
            "meta_data": self.meta_data.to_json(),
        }


@dataclass
class WorldStateUpdate(Message):
    """
    Class describing the updates to the free variables of a world state.
    """

    prefixed_names: List[PrefixedName]
    """
    The prefixed names of the changed free variables.
    """

    states: List[float]
    """
    The states of the changed free variables.
    """

    def to_json(self) -> Dict[str, Any]:
        return {
            **super().to_json(),
            "prefixed_names": [n.to_json() for n in self.prefixed_names],
            "states": list(self.states),
        }

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(
            meta_data=MetaData.from_json(data["meta_data"]),
            prefixed_names=[PrefixedName.from_json(n) for n in data["prefixed_names"]],
            states=data["states"],
        )


@dataclass
class ModificationBlock(Message):
    """
    Message describing the modifications done to a world.
    """

    modifications: WorldModelModificationBlock
    """
    The modifications done to a world.
    """

    def to_json(self) -> Dict[str, Any]:
        return {
            **super().to_json(),
            "modifications": self.modifications.to_json(),
        }

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(
            meta_data=MetaData.from_json(data["meta_data"]),
            modifications=WorldModelModificationBlock.from_json(data["modifications"]),
        )


@dataclass
class LoadModel(Message):
    """
    Message for requesting the loading of a model identified by its primary key.
    """

    primary_key: int
    """
    The primary key identifying the model to be loaded.
    """

    def to_json(self) -> Dict[str, Any]:
        return {
            **super().to_json(),
            "primary_key": self.primary_key,
        }

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(
            meta_data=MetaData.from_json(data["meta_data"]),
            primary_key=data["primary_key"],
        )
