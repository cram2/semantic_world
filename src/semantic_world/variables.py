from __future__ import annotations

from enum import Enum
from functools import cached_property

from ormatic.utils import classproperty
from random_events.variable import Continuous
from sortedcontainers import SortedSet


class SpatialVariables(Enum):
    """
    Enum for spatial variables used in the semantic world. Used in the context of random events.
    """
    x = Continuous("x")
    y = Continuous("y")
    z = Continuous("z")

    @classproperty
    def xy(cls):
        return SortedSet([cls.x.value, cls.y.value])

    @classproperty
    def yz(cls):
        return SortedSet([cls.y.value, cls.z.value])