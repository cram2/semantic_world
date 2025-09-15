from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from typing_extensions import TYPE_CHECKING, Callable

if TYPE_CHECKING:
    from ..world import World

logger = logging.getLogger(__name__)


@dataclass
class Callback(ABC):
    """
    Callback is an abstract base class (ABC)
    reacting to changes in the associated `world`.
    It provides a flexible  mechanism for subclasses to implement custom behaviors to be triggered
    whenever a change occurs.

    The primary purpose of this class is to encapsulate logic that needs to be
    executed as a response to certain events or changes within the `world` object.
    """

    world: World
    """
    The world this callback is listening on.
    """

    @abstractmethod
    def notify(self):
        """
        Notify the callback of a change in the world.
        """
        raise NotImplementedError


@dataclass
class StateChangeCallback(Callback, ABC):
    """
    Callback for handling state changes.
    """

    def __post_init__(self):
        self.world.state_change_callbacks.append(self)


@dataclass
class ModelChangeCallback(Callback, ABC):
    """
    Callback for handling model changes.
    """

    def __post_init__(self):
        self.world.model_change_callbacks.append(self)
